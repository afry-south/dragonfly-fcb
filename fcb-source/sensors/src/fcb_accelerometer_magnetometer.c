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

enum { NBR_OF_SAMPLES_IN_EACH_POSITION  = 10 };
enum { NBR_SAMPLE_POSITIONS = 6 };
enum { NBR_OF_SAMPLES_BETWEEN_EACH_POSITION = 500 };

typedef enum {
	MOVING,
	IN_POSITION
} AccCalibSamplePosition_t;

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
static void setXYZVector(float32_t *srcVector, float32_t *dstVector);
void adjustAxesOrientation(float32_t *xyzValues);
static void applayCalibrationPrmToRawData(float32_t *calPrmVector, float32_t *xyzValues);
bool handleAccSampling(float32_t *acceleroMeterData);
uint8_t CheckCalParams(float32_t* magCalPrms);

/* public fcn definitions */

uint8_t FcbInitialiseAccMagSensor(void) {
    uint8_t retVal = FCB_OK;
    FlashErrorStatus flash_status = FLASH_OK;
    float32_t sXYZCalPrmTemp[CALIB_IDX_MAX];

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

    flash_status = ReadMagCalibrationValuesFromFlash(sXYZCalPrmTemp);
    if(flash_status == FLASH_OK && CheckCalParams(sXYZCalPrmTemp) == FCB_OK) {
        // If previously saved calibration params found, copy these to used params
        memcpy(sXYZMagCalPrm, sXYZCalPrmTemp, sizeof(sXYZMagCalPrm));
    }

    flash_status = ReadAccCalibrationValuesFromFlash(sXYZCalPrmTemp);
    if(flash_status == FLASH_OK && CheckCalParams(sXYZCalPrmTemp) == FCB_OK) {
        // If previously saved calibration params found, copy these to used params
        memcpy(sXYZAccCalPrm, sXYZCalPrmTemp, sizeof(sXYZAccCalPrm));
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

void setXYZVector(float32_t *srcVector, float32_t *dstVector) {
	if (pdTRUE != xSemaphoreTake(mutexMag, portMAX_DELAY /* wait forever */)) {
		ErrorHandler();
		return;
	}

	dstVector[X_IDX] = srcVector[X_IDX];
	dstVector[Y_IDX] = srcVector[Y_IDX];
	dstVector[Z_IDX] = srcVector[Z_IDX];

	if (pdTRUE != xSemaphoreGive(mutexMag)) {
		ErrorHandler();
		return;
	}
}

void adjustAxesOrientation(float32_t *xyzValues) {
	/* adjust sensor axes to the axes of the quadcopter fuselage
	 * see "Sensors" page in Wiki.
	 *
	 * NOTE: X axis is already aligned
	 */
	xyzValues[Y_IDX] = -xyzValues[Y_IDX];
	xyzValues[Z_IDX] = -xyzValues[Z_IDX];
}

void applayCalibrationPrmToRawData(float32_t *calPrmVector, float32_t *xyzValues) {
	xyzValues[X_IDX] = (xyzValues[X_IDX] - calPrmVector[X_OFFSET_CALIB_IDX])
					/ calPrmVector[X_SCALING_CALIB_IDX];
	xyzValues[Y_IDX] = (xyzValues[Y_IDX] - calPrmVector[Y_OFFSET_CALIB_IDX])
					/ calPrmVector[Y_SCALING_CALIB_IDX];
	xyzValues[Z_IDX] = (xyzValues[Z_IDX] - calPrmVector[Z_OFFSET_CALIB_IDX])
					/ calPrmVector[Z_SCALING_CALIB_IDX];
}

bool handleAccSampling(float32_t *acceleroMeterData) {
	static uint32_t sampleIndex = 0;
	static AccCalibSamplePosition_t sampleInPosition = MOVING;
	static uint32_t samplePosition = 0;
	bool calibrationDone = false;

	if (samplePosition < NBR_SAMPLE_POSITIONS) {
		// We will take samples from a number of different positions.
		if (sampleInPosition == IN_POSITION) {
			// Device is in new position
			if (sampleIndex < NBR_OF_SAMPLES_IN_EACH_POSITION) {
				// Add the sample to the calibration algorithm.
				adjustAxesOrientation(acceleroMeterData);
				addNewSample(acceleroMeterData);
				sampleIndex++;
			} else {
				// All samples for this position is done. Inform the user to put the device in a new position.
				sampleIndex = 0;
				sampleInPosition = MOVING;
				samplePosition++;

				USBComSendString("Move device to next position\n");
			}
		} else {
			// Wait some time for the device to be put in a new position.
			if (sampleIndex < NBR_OF_SAMPLES_BETWEEN_EACH_POSITION) {
				// Still more time to wait.
				sampleIndex++;
			} else {
				// Time to start to take some samples. Also inform the user about this.
				sampleIndex = 0;
				sampleInPosition = IN_POSITION;

				USBComSendString("Start sampling for this position\n");
			}
		}
	} else {
		// Calibration done, return true and reset internal variables.
		calibrationDone = true;
		sampleIndex = 0;
		sampleInPosition = MOVING;
		samplePosition = 0;
	}

	return calibrationDone;
}

void FetchDataFromAccelerometer(void) {
    float32_t acceleroMeterData[3] = { 0.0f, 0.0f, 0.0f };
    HAL_StatusTypeDef status = HAL_OK;

    if (ACCMAGMTR_UNINITIALISED != accMagMode) {
        status = LSM303DLHC_AccReadXYZ(acceleroMeterData);
        if (status != HAL_OK) { // Handle accelerometer read timeout error
#ifdef FCB_ACCMAG_DEBUG
            USBComSendString("ERROR: LSM303DLHC_AccReadXYZ\n");
#endif
            FcbSendSensorMessage(FCB_SENSOR_ACC_DATA_READY);
            return;
        }
    }

	if (ACCMAGMTR_FETCHING == accMagMode) {
		adjustAxesOrientation(acceleroMeterData);
		applayCalibrationPrmToRawData(sXYZAccCalPrm, acceleroMeterData);
		setXYZVector(acceleroMeterData, sXYZDotDot);

	    if (SendCorrectionUpdateCallback != NULL) {
            SendCorrectionUpdateCallback(ACC_IDX, 1 /* dummy not used for acc */, sXYZDotDot);
	    }
	} else if (ACCMTR_CALIBRATING == accMagMode) {
		if (handleAccSampling(acceleroMeterData)) {
			calibrate(sXYZAccCalPrm);
			WriteAccCalibrationValuesToFlash(sXYZAccCalPrm);

			char string[100];
			snprintf(string, 100,
					"Calib acc value: %f\t: %f\t: %f: %f\t: %f\t: %f\n",
					sXYZAccCalPrm[0], sXYZAccCalPrm[1], sXYZAccCalPrm[2],
					sXYZAccCalPrm[3], sXYZAccCalPrm[4], sXYZAccCalPrm[5]);
			USBComSendString(string);

			/* calibration done */
			accMagMode = ACCMAGMTR_FETCHING;
		}
	}
}

void StartAccMagMtrCalibration(uint32_t samples) {
    nbrOfSamplesForCalibration = samples;
    accMagMode = MAGMTR_CALIBRATING;
}

void FetchDataFromMagnetometer(void) {
	HAL_StatusTypeDef status = HAL_OK;
	float32_t magnetoMeterData[3] = { 0.0f, 0.0f, 0.0f };

	if (ACCMAGMTR_UNINITIALISED != accMagMode) {
		status = LSM303DLHC_MagReadXYZ(magnetoMeterData);
		if (status != HAL_OK) {
#ifdef FCB_ACCMAG_DEBUG
			USBComSendString("ERROR: LSM303DLHC_MagReadXYZ\n");
#endif
			FcbSendSensorMessage(FCB_SENSOR_MAGNETO_DATA_READY);
			return;
		}
	}

	if (ACCMAGMTR_FETCHING == accMagMode) {
		adjustAxesOrientation(magnetoMeterData);
		applayCalibrationPrmToRawData(sXYZMagCalPrm, magnetoMeterData);
		setXYZVector(magnetoMeterData, sXYZMagVector);

		if (SendCorrectionUpdateCallback != NULL) {
			SendCorrectionUpdateCallback(MAG_IDX, 1 /* dummy - not used for mag */, sXYZMagVector);
		}
	} else if (MAGMTR_CALIBRATING == accMagMode) {
		static uint32_t sampleIndex = 0;
		if (sampleIndex < nbrOfSamplesForCalibration) {
			adjustAxesOrientation(magnetoMeterData);
			addNewSample(magnetoMeterData);
			sampleIndex++;
		} else {
			calibrate(sXYZMagCalPrm);

			WriteMagCalibrationValuesToFlash(sXYZMagCalPrm);

			char string[100];
			snprintf(string, 100,
					"Calib mag value: %f\t: %f\t: %f: %f\t: %f\t: %f\n",
					sXYZMagCalPrm[0], sXYZMagCalPrm[1], sXYZMagCalPrm[2],
					sXYZMagCalPrm[3], sXYZMagCalPrm[4], sXYZMagCalPrm[5]);
			USBComSendString(string);
			USBComSendString("\nMove device to first position for Accelerometer calibration.\n");

			/* calibration done */
			accMagMode = ACCMTR_CALIBRATING;
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

uint8_t CheckCalParams(float32_t* calPrms) {
	uint8_t status = FCB_OK;

	if (calPrms[X_SCALING_CALIB_IDX] < 0.1) {
		status = FCB_ERR;
	} else if (calPrms[Y_SCALING_CALIB_IDX] < 0.1) {
		status = FCB_ERR;
	} else if (calPrms[Z_SCALING_CALIB_IDX] < 0.1) {
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
