/**
 * @file fcb_accelerometer_magnetometer.c
 *
 * Implements fcb_accelerometer_magnetometer.h API
 *
 * This file does most of the GPIO & NVIC setup to receive the interrupts
 * from LSM303DLHC.
 *
 * The lsm303dlhc.c file does most of the configuration of the acc/magneto-meter
 * itself.
 *
 * This file also handles coordinate transform to translate magnetometer &
 * accelerometer data to the quadcopter x y z axes.
 *
 * @see fcb_accelerometer.h
 */
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_sensor_calibration.h"
#include "sphere_calibration.h"
#include "fcb_sensors.h"
#include "fcb_error.h"
#include "lsm303dlhc.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"
#include "trace.h"
#include "flash.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include <stdint.h>
#include <stdio.h>

#define FCB_ACCMAG_DEBUG

enum {
    ACCMAG_AXES_N = 3
};

enum {
    ACCMAG_SAMPLING_MAX_STRING_SIZE = 128
};
/* print-to-usb com port sampling */
static uint32_t nbrOfSamplesForCalibration;

static float32_t sXYZDotDot[] = { 0, 0, 0 }; /* not volatile - only print thread reads */
static float32_t sXYZMagVector[] = { 0, 0, 0 }; /* not volatile - only print thread reads */

static xSemaphoreHandle mutexAcc;
static xSemaphoreHandle mutexMag;

static float32_t sAccSamplePeriod = 0.0f;
static float32_t sMagSamplePeriod = 0.0f;

/**
 * Magnetometer calibration offset & scaling coefficients
 *
 * @see FcbSensorCalibrationParmIndex for what the numbers mean.
 */
static float32_t sXYZMagCalPrm[CALIB_IDX_MAX] = { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
static float32_t sXYZAccCalPrm[CALIB_IDX_MAX] = { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };


static enum FcbAccMagMode accMagMode = ACCMAGMTR_UNINITIALISED;

/* static fcn declarations */

static SendCorrectionUpdateCallback_TypeDef SendCorrectionUpdateCallback = NULL;

/* public fcn definitions */

uint8_t FcbInitialiseAccMagSensor(void) {
    uint8_t retVal = FCB_OK;
    FlashErrorStatus flash_status = FLASH_OK;
    float32_t sXYZMagCalPrmTemp[CALIB_IDX_MAX];

    if (accMagMode != ACCMAGMTR_UNINITIALISED) {
        /* they are already initialised - this is a logical error. */
        return FCB_ERR_INIT;
    }

    if (NULL == (mutexAcc = xSemaphoreCreateMutex())) {
        return FCB_ERR_INIT;
    }

    if (NULL == (mutexMag = xSemaphoreCreateMutex())) {
        return FCB_ERR_INIT;
    }

    /* configure STM32 interrupts & GPIO */
    ACCELERO_DRDY_GPIO_CLK_ENABLE(); /* GPIOE clock */

    /* STM32F3 doc UM1570 page 27/36. Accelerometer interrupt */
    FcbSensorsInitGpioPinForInterrupt(GPIOE, GPIO_PIN_4);

    /* STM32F3 doc UM1570 page 27/36. Magnetometer interrupt */
    FcbSensorsInitGpioPinForInterrupt(GPIOE, GPIO_PIN_2);

    /* set up interrupt DRDY for accelerometer - see HAL_GPIO_EXTI_Callback fcn */
    HAL_NVIC_SetPriority(EXTI4_IRQn,
    configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    /* set up interrupt DRDY for magnetometer - see HAL_GPIO_EXTI_Callback fcn */
    HAL_NVIC_SetPriority(EXTI2_TSC_IRQn,
    configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

    /* ISSUE1_TODO - fetch accelerometer calib from flash */
    /* ISSUE2_TODO - fetch magnetometer calib from flash */

    LSM303DLHC_AccConfig();
    sAccSamplePeriod = 1 / (float) LSM303DLHC_AccDataRateHz();

    LSM303DLHC_MagInit();
    sMagSamplePeriod = 1 / (float) LSM303DLHC_MagDataRateHz();

    /* do a pre-read to get the DRDY interrupts going. Since we trig on
     * rising flank and the sensor has data from power-on, by the time we get
     * here the interrupt is already high. Reading the data trigs the
     * sensor to load a new set of values into its registers.
     */
    float32_t dummyData[3];
    LSM303DLHC_AccReadXYZ(sXYZDotDot);
    LSM303DLHC_MagReadXYZ(dummyData);

    flash_status = ReadMagCalibrationValuesFromFlash(sXYZMagCalPrmTemp);
    if(flash_status == FLASH_OK && CheckMagCalParams(sXYZMagCalPrmTemp) == FCB_OK) {
    	// If previously saved calibration params found, copy these to used params
    	memcpy(sXYZMagCalPrm, sXYZMagCalPrmTemp, sizeof(sXYZMagCalPrm));
    }

    accMagMode = ACCMAGMTR_FETCHING;
    return retVal;
}

uint8_t SensorRegisterAccClientCallback(SendCorrectionUpdateCallback_TypeDef cbk) {
  if (NULL != SendCorrectionUpdateCallback) {
    return FCB_ERR;
  }

  SendCorrectionUpdateCallback = cbk;

  return FCB_OK;
}

void FetchDataFromAccelerometer(void) {
    float32_t acceleroMeterData[3] = { 0.0f, 0.0f, 0.0f };
    HAL_StatusTypeDef status = HAL_OK;

    if ((ACCMAGMTR_FETCHING == accMagMode) || (ACCMAGMTR_CALIBRATING == accMagMode)) {
        status = LSM303DLHC_AccReadXYZ(acceleroMeterData);
        if (status != HAL_OK) { // Handle accelerometer read timeout error
#ifdef FCB_ACCMAG_DEBUG
            USBComSendString("ERROR: LSM303DLHC_AccReadXYZ\n");
#endif
            FcbSendSensorMessage(FCB_SENSOR_ACC_DATA_READY);
            return;
        }
    }

    /* from accelerometer to quadcopter coordinate axes
     * see "Sensors" page in Wiki.
     *
     * NOTE: X axis already aligned
     */
    acceleroMeterData[Y_IDX] = -acceleroMeterData[Y_IDX];
    acceleroMeterData[Z_IDX] = -acceleroMeterData[Z_IDX];

    // TODO IS BELOW NEEDED? HOW TO CALIBRATE ACCELEROMETER?
    if (ACCMAGMTR_FETCHING == accMagMode) {
        /* only apply calibration tune when in normal fetch mode - in calibrate
         * mode raw data is desired.
         */
        acceleroMeterData[X_IDX] = (acceleroMeterData[X_IDX] - sXYZAccCalPrm[X_OFFSET_CALIB_IDX])
                * sXYZAccCalPrm[X_SCALING_CALIB_IDX];
        acceleroMeterData[Y_IDX] = (acceleroMeterData[Y_IDX] - sXYZAccCalPrm[Y_OFFSET_CALIB_IDX])
                * sXYZAccCalPrm[Y_SCALING_CALIB_IDX];
        acceleroMeterData[Z_IDX] = (acceleroMeterData[Z_IDX] - sXYZAccCalPrm[Z_OFFSET_CALIB_IDX])
                * sXYZAccCalPrm[Z_SCALING_CALIB_IDX];
    }

    if (pdTRUE != xSemaphoreTake(mutexAcc, portMAX_DELAY /* wait forever */)) {
        ErrorHandler();
        return;
    }

    /* all 3 updated simultaneously */
    sXYZDotDot[X_IDX] = acceleroMeterData[X_IDX];
    sXYZDotDot[Y_IDX] = acceleroMeterData[Y_IDX];
    sXYZDotDot[Z_IDX] = acceleroMeterData[Z_IDX];

    if (pdTRUE != xSemaphoreGive(mutexAcc)) {
        ErrorHandler();
        return;
    }

    if (SendCorrectionUpdateCallback != NULL) {
        SendCorrectionUpdateCallback(ACC_IDX, 1 /* dummy not used for acc */, sXYZDotDot);
    }
}

void StartAccMagMtrCalibration(uint32_t samples) {
    nbrOfSamplesForCalibration = samples;
    accMagMode = ACCMAGMTR_CALIBRATING;
}

void StopAccMagMtrCalibration(uint8_t samples) {
    (void) samples;
    accMagMode = ACCMAGMTR_FETCHING;
}

void FetchDataFromMagnetometer(void) {
    HAL_StatusTypeDef status = HAL_OK;
    float32_t magnetoMeterData[3] = { 0.0f, 0.0f, 0.0f };

    if ((ACCMAGMTR_FETCHING == accMagMode) || (ACCMAGMTR_CALIBRATING == accMagMode)) {
        status = LSM303DLHC_MagReadXYZ(magnetoMeterData);
        if (status != HAL_OK) {
#ifdef FCB_ACCMAG_DEBUG
            USBComSendString("ERROR: LSM303DLHC_MagReadXYZ\n");
#endif
            FcbSendSensorMessage(FCB_SENSOR_MAGNETO_DATA_READY);
            return;
        }
    }

    /* TODO ... and then copy them into a mutex-protected sXYZMagVector here?? */

    /* adjust magnetometer axes to the axes of the quadcopter fuselage
     * see "Sensors" page in Wiki.
     *
     * NOTE: X axis is already aligned
     */
    magnetoMeterData[Y_IDX] = -magnetoMeterData[Y_IDX];
    magnetoMeterData[Z_IDX] = -magnetoMeterData[Z_IDX];

    if (ACCMAGMTR_FETCHING == accMagMode) {
        magnetoMeterData[X_IDX] = (magnetoMeterData[X_IDX] - sXYZMagCalPrm[X_OFFSET_CALIB_IDX])
                / sXYZMagCalPrm[X_SCALING_CALIB_IDX];
        magnetoMeterData[Y_IDX] = (magnetoMeterData[Y_IDX] - sXYZMagCalPrm[Y_OFFSET_CALIB_IDX])
                / sXYZMagCalPrm[Y_SCALING_CALIB_IDX];
        magnetoMeterData[Z_IDX] = (magnetoMeterData[Z_IDX] - sXYZMagCalPrm[Z_OFFSET_CALIB_IDX])
                / sXYZMagCalPrm[Z_SCALING_CALIB_IDX];

		if (pdTRUE != xSemaphoreTake(mutexMag, portMAX_DELAY /* wait forever */)) {
			ErrorHandler();
			return;
		}

		sXYZMagVector[X_IDX] = (float32_t)magnetoMeterData[X_IDX];
		sXYZMagVector[Y_IDX] = (float32_t)magnetoMeterData[Y_IDX];
		sXYZMagVector[Z_IDX] = (float32_t)magnetoMeterData[Z_IDX];

		if (pdTRUE != xSemaphoreGive(mutexMag)) {
			ErrorHandler();
			return;
		}

		if (SendCorrectionUpdateCallback != NULL) {
			SendCorrectionUpdateCallback(MAG_IDX, 1 /* dummy - not used for mag */, sXYZMagVector);
		}
    }
    else if (ACCMAGMTR_CALIBRATING == accMagMode) {
    	static uint32_t sampleIndex = 0;
        if (sampleIndex < nbrOfSamplesForCalibration) {
        	addNewSample(magnetoMeterData);
        	sampleIndex++;
        }
        else {
        	calibrate(sXYZMagCalPrm);

        	WriteMagCalibrationValuesToFlash(sXYZMagCalPrm);

        	char string[100];
        	snprintf(string, 100, "Calib value: %f\t: %f\t: %f: %f\t: %f\t: %f\n",
        			sXYZMagCalPrm[0], sXYZMagCalPrm[1], sXYZMagCalPrm[2], sXYZMagCalPrm[3], sXYZMagCalPrm[4], sXYZMagCalPrm[5]);
        	USBComSendString(string);

        	/* calibration done */
            accMagMode = ACCMAGMTR_FETCHING;
            sampleIndex = 0;
        }
    }
}

void GetAcceleration(float32_t * xDotDot, float32_t * yDotDot, float32_t * zDotDot) {
    if (pdTRUE != xSemaphoreTake(mutexAcc, portMAX_DELAY /* wait forever */)) {
        ErrorHandler();
        return;
    }

    *xDotDot = sXYZDotDot[X_IDX];
    *yDotDot = sXYZDotDot[Y_IDX];
    *zDotDot = sXYZDotDot[Z_IDX];

    if (pdTRUE != xSemaphoreGive(mutexAcc)) {
        ErrorHandler();
        return;
    }
}

void GetAccelerationNoMutex(float32_t * xDotDot, float32_t * yDotDot, float32_t * zDotDot) {
    *xDotDot = sXYZDotDot[X_IDX];
    *yDotDot = sXYZDotDot[Y_IDX];
    *zDotDot = sXYZDotDot[Z_IDX];
}

void GetMagVector(float32_t * x, float32_t * y, float32_t * z) {
    if (pdTRUE != xSemaphoreTake(mutexMag, portMAX_DELAY /* wait forever */)) {
        ErrorHandler();
        return;
    }

    *x = sXYZMagVector[X_IDX];
    *y = sXYZMagVector[Y_IDX];
    *z = sXYZMagVector[Z_IDX];

    if (pdTRUE != xSemaphoreGive(mutexMag)) {
        ErrorHandler();
        return;
    }
}

void GetMagVectorNoMutex(float32_t * x, float32_t * y, float32_t * z) {
    *x = sXYZMagVector[X_IDX];
    *y = sXYZMagVector[Y_IDX];
    *z = sXYZMagVector[Z_IDX];
}

void PrintAccelerometerValues(void) {
    static char sampleString[ACCMAG_SAMPLING_MAX_STRING_SIZE];

    // TODO use mutex
    snprintf((char*) sampleString, ACCMAG_SAMPLING_MAX_STRING_SIZE,
            "Accelerometer readings [m/(s * s)]:\nAccX: %f\nAccY: %f\nAccZ: %f\n\r\n", sXYZDotDot[X_IDX],
            sXYZDotDot[Y_IDX], sXYZDotDot[Z_IDX]);

    USBComSendString(sampleString);
}

void SetAccMagMeasuredSamplePeriod(float32_t accMeasuredPeriod, float32_t magMeasuredPeriod) {
    sAccSamplePeriod = accMeasuredPeriod;
    sMagSamplePeriod = magMeasuredPeriod;
}

void GetAccMagMeasuredSamplePeriod(float32_t *accMeasuredPeriod, float32_t *magMeasuredPeriod) {
    *accMeasuredPeriod = sAccSamplePeriod;
    *magMeasuredPeriod = sMagSamplePeriod;
}

uint8_t CheckMagCalParams(float32_t* magCalPrms) {
	uint8_t status = FCB_OK;

	if (magCalPrms[X_SCALING_CALIB_IDX] < 0.1) {
		status = FCB_ERR;
	} else if (magCalPrms[Y_SCALING_CALIB_IDX] < 0.1) {
		status = FCB_ERR;
	} else if (magCalPrms[Z_SCALING_CALIB_IDX] < 0.1) {
		status = FCB_ERR;
	}

	return status;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
