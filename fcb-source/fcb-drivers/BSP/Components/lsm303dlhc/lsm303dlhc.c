/**
 ******************************************************************************
 * @file    lsm303dlhc.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    18-February-2014
 * @brief   This file provides a set of functions needed to manage the lsm303dlhc
 *          MEMS accelerometer available on STM32F401-Discovery Kit.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "lsm303dlhc.h"

#include "fcb_error.h"
/** @addtogroup BSP
 * @{
 */

/** @addtogroup LSM303DLHC
 * @{
 */

/** @addtogroup LSM303DLHC
 * @{
 */


/** @defgroup LSM303DLHC_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @defgroup LSM303DLHC_Private_Defines
 *
 * @todo create one for accelerometer config.
 *
 * @{
 */

struct MagnetometerConfig {
  int16_t xySensitivity;
  int16_t zSensitivity;
  uint8_t fullScale;
  uint8_t dataRate;
  uint8_t temperatureSensor;
};

struct AccelerometerConfig {
  int16_t sensitivity; /* all three axes */
};

/**
 * @}
 */

/** @defgroup LSM303DLHC_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup LSM303DLHC_Private_Variables
 * @{
 */

static struct MagnetometerConfig magConfig; /* initialised in LSM303DLHC_MagInit */
static struct AccelerometerConfig accConfig; /* initialised in LSM303DLHC_AccInit */

uint8_t tmpreg2A[8] = {0x00};
uint8_t tmpreg3A = 0x00;
uint8_t tmpregcfgA = 0x00;

/**
 * @}
 */

/** @defgroup LSM303DLHC_Private_Functions
 * @{
 */

/**
 * Set LSM303DLHC Initialization.
 *
 * It also configures IO for the sensor, therefore it has to
 * be called before LSM303DLHC_MagInit
 *
 * @see LSM303DLHC_MagInit
 */
void LSM303DLHC_AccConfig(void) {
  /*  Low level init */
  COMPASSACCELERO_IO_Init();

  /* set up accelerometer */
  uint8_t ctrlReg1 = 0x00 |
      LSM303DLHC_NORMAL_MODE |
      LSM303DLHC_ODR_50_HZ |
      LSM303DLHC_AXES_ENABLE;

  uint8_t ctrlReg3 = 0x00 | LSM303DLHC_IT1_DRY1 ;

  uint8_t ctrlReg4 = 0x00 |
      LSM303DLHC_FULLSCALE_2G |
      LSM303DLHC_BlockUpdate_Single |
      LSM303DLHC_BLE_LSB |
      LSM303DLHC_HR_ENABLE ;

  if (I_AM_LMS303DLHC != LSM303DLHC_AccReadID()) {
    ErrorHandler();
  }

  LSM303DLHC_AccRebootCmd();
  LSM303DLHC_AccInit(ctrlReg1, ctrlReg3, ctrlReg4);

  LSM303DLHC_AccFilterConfig(LSM303DLHC_HPM_NORMAL_MODE |
      LSM303DLHC_HPFCF_16 |
      LSM303DLHC_HPF_AOI1_DISABLE |
      LSM303DLHC_HPF_AOI2_DISABLE);
}


/**
 * section 7.1.3 in LSM303DLHC data sheet doc DocID018771 Rev 2
 *
 * @param  ctrlReg1 sets power mode, output data rate and enable xyz axes
 * @param  ctrlReg3 sets DRDY to INT1 interrupt
 * @param  ctrlReg4 full scale, block update mode, MSB/LSB, high resolution mode.
 *
 * @retval   None
 */
void LSM303DLHC_AccInit(uint8_t ctrlReg1, uint8_t ctrlReg3, uint8_t ctrlReg4)
{
  switch (ctrlReg4 & 0x30) {
  case LSM303DLHC_FULLSCALE_2G:
    accConfig.sensitivity = LSM303DLHC_ACC_SENSITIVITY_2G;
    break;
  case LSM303DLHC_FULLSCALE_4G:
    accConfig.sensitivity = LSM303DLHC_ACC_SENSITIVITY_4G;
    break;
  case LSM303DLHC_FULLSCALE_8G:
    accConfig.sensitivity = LSM303DLHC_ACC_SENSITIVITY_8G;
    break;
  case LSM303DLHC_FULLSCALE_16G:
    accConfig.sensitivity = LSM303DLHC_ACC_SENSITIVITY_16G;
    break;
  }

#ifdef FCB_ACCMAG_DEBUG
  uint8_t tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A);
#endif
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, 0x00);

  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, ctrlReg1);
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, ctrlReg3);
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlReg4);
}

/**
 * @brief     Read LSM303DLHC ID.
 * @retval   ID
 */
uint8_t LSM303DLHC_AccReadID(void)
{
  uint8_t ctrl = 0x00;

  /*  Low level init */
  COMPASSACCELERO_IO_Init();

  /* Read value at Who am I register address*/
  ctrl = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_WHO_AM_I_ADDR);

  return ctrl;
}

/**
 * @brief     Reboot memory content of LSM303DLHC
 * @retval   None
 */
void LSM303DLHC_AccRebootCmd(void)
{
  uint8_t tmpreg;

  /* Read CTRL_REG5 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A);

  /* Enable or Disable the reboot memory */
  tmpreg |= LSM303DLHC_BOOT_REBOOTMEMORY;

  /* Write value to ACC MEMS CTRL_REG5 regsister */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, tmpreg);
}

/**
 * @brief     Set High Pass Filter Modality
 * @param  FilterStruct: contains data for filter config
 * @retval   None
 */
void LSM303DLHC_AccFilterConfig(uint8_t FilterStruct)
{
  uint8_t tmpreg;

  /* Read CTRL_REG2 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A);

  tmpreg &= 0x0C;
  tmpreg |= FilterStruct;

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, tmpreg);
}

/**
 * @brief  Enable or Disable High Pass Filter
 * @param  HighPassFilterState: new state of the High Pass Filter feature.
 *      This parameter can be:
 *         @arg: LSM303DLHC_HighPassFilter_DISABLE
 *         @arg: LSM303DLHC_HighPassFilter_ENABLE
 * @retval None
 */
void LSM303DLHC_AccFilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;

  /* Read CTRL_REG2 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A);

  tmpreg &= 0xF7;

  tmpreg |= HighPassFilterState;

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, tmpreg);
}

/**
 * @brief  Read X, Y & Z Accelration values
 * @param  pfData : Data out pointer
 * @retval None
 */
void LSM303DLHC_AccReadXYZ(float * pData)
{

  uint8_t buffer[6];
  uint8_t i = 0;

  /* Read output register X, Y & Z acceleration */
  buffer[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A);
  buffer[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_H_A);
  buffer[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_A);
  buffer[3] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_A);
  buffer[4] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_A);
  buffer[5] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_A);

  /* check in the control register4 the data alignment
   *
   * We use LSM303DLHC_BLE_LSB convention.
   */
  for (i=0; i<3; i++)
  {
    int16_t rawData = (int16_t)((int16_t)(buffer[2*i + 1] << 8) + buffer[2*i]); /* convert to int16_t */
    float asFloat = (float)rawData;            /* convert to float (int16_t & float are two's complement) */
    asFloat = asFloat / 16;                    /* handle 12-bit value alignment ("shift 4 right") */
    asFloat = asFloat * accConfig.sensitivity; /* apply sensitivity convert from LSB to milli-G */
    asFloat = asFloat * 9.82 / 1000;           /* convert from milli-G to m/(s * s)       */
    pData[i] = asFloat;   /* store output */
  }
}

/**
 * @brief  Enable or Disable High Pass Filter on CLick
 * @param  HighPassFilterState: new state of the High Pass Filter feature.
 *      This parameter can be:
 *         @arg: LSM303DLHC_HPF_CLICK_DISABLE
 *         @arg: LSM303DLHC_HPF_CLICK_ENABLE
 * @retval None
 */
void LSM303DLHC_AccFilterClickCmd(uint8_t HighPassFilterClickState)
{
  uint8_t tmpreg = 0x00;

  /* Read CTRL_REG2 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A);

  tmpreg &= ~(LSM303DLHC_HPF_CLICK_ENABLE);

  tmpreg |= HighPassFilterClickState;

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, tmpreg);
}

/**
 * @brief Enable LSM303DLHC Interrupt1
 * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
 *           This parameter can be any combination of the following values:
 *         @arg   LSM303DLHC_IT1_CLICK
 *         @arg   LSM303DLHC_IT1_AOI1
 *         @arg   LSM303DLHC_IT1_AOI2
 *         @arg   LSM303DLHC_IT1_DRY1
 *         @arg   LSM303DLHC_IT1_WTM
 *         @arg   LSM303DLHC_IT1_OVERRUN
 * @retval None
 */
void LSM303DLHC_AccIT1Enable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;

  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A);

  /* Enable IT1 */
  tmpval |= LSM303DLHC_IT;

  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, tmpval);
}

/**
 * @brief Disable LSM303DLHC Interrupt1
 * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
 *           This parameter can be any combination of the following values:
 *         @arg   LSM303DLHC_IT1_CLICK
 *         @arg   LSM303DLHC_IT1_AOI1
 *         @arg   LSM303DLHC_IT1_AOI2
 *         @arg   LSM303DLHC_IT1_DRY1
 *         @arg   LSM303DLHC_IT1_WTM
 *         @arg   LSM303DLHC_IT1_OVERRUN.
 * @retval None
 */
void LSM303DLHC_AccIT1Disable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;

  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A);

  /* Disable IT1 */
  tmpval &= ~LSM303DLHC_IT;

  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, tmpval);
}

/**
 * @brief Enable LSM303DLHC Interrupt2
 * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
 *           This parameter can be any combination of the following values:
 *         @arg   LSM303DLHC_IT2_CLICK2
 *         @arg   LSM303DLHC_IT2_INT1
 *         @arg   LSM303DLHC_IT2_INT2
 *         @arg   LSM303DLHC_IT2_BOOT
 *         @arg   LSM303DLHC_IT2_ACT
 *         @arg   LSM303DLHC_IT2_HLACTIVE
 * @retval None
 */
void LSM303DLHC_AccIT2Enable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;

  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A);

  /* Enable IT2 */
  tmpval |= LSM303DLHC_IT;

  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, tmpval);
}

/**
 * @brief Disable LSM303DLHC Interrupt2
 * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
 *           This parameter can be any combination of the following values:
 *         @arg   LSM303DLHC_IT2_CLICK2
 *         @arg   LSM303DLHC_IT2_INT1
 *         @arg   LSM303DLHC_IT2_INT2
 *         @arg   LSM303DLHC_IT2_BOOT
 *         @arg   LSM303DLHC_IT2_ACT
 *         @arg   LSM303DLHC_IT2_HLACTIVE
 * @retval None
 */
void LSM303DLHC_AccIT2Disable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;

  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A);

  /* Disable IT2 */
  tmpval &= ~LSM303DLHC_IT;

  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, tmpval);
}

/**
 * @brief  INT1 interrupt enable
 * @param  ITCombination: Or or And combination
 *         ITAxes: axes to be enabled
 * @retval None
 */
void LSM303DLHC_AccINT1InterruptEnable(uint8_t ITCombination, uint8_t ITAxes)
{
  uint8_t tmpval = 0x00;

  /* Read INT1_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A);

  /* Enable the selected interrupt */
  tmpval |= (ITAxes | ITCombination);

  /* Write value to MEMS INT1_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, tmpval);
}

/**
 * @brief  INT1 interrupt disable
 * @param  ITCombination: Or or And combination
 *         ITAxes: axes to be enabled
 * @retval None
 */
void LSM303DLHC_AccINT1InterruptDisable(uint8_t ITCombination, uint8_t ITAxes)
{
  uint8_t tmpval = 0x00;

  /* Read INT1_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A);

  /* Disable the selected interrupt */
  tmpval &= ~(ITAxes | ITCombination);

  /* Write value to MEMS INT1_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, tmpval);
}

/**
 * @brief  INT2 interrupt enable
 * @param  ITCombination: Or or And combination
 *         ITAxes: axes to be enabled
 *         NewState: Enable or Disable
 * @retval None
 */
void LSM303DLHC_AccINT2InterruptEnable(uint8_t ITCombination, uint8_t ITAxes)
{
  uint8_t tmpval = 0x00;

  /* Read INT2_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A);

  /* Enable the selected interrupt */
  tmpval |= (ITAxes | ITCombination);

  /* Write value to MEMS INT2_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, tmpval);
}

/**
 * @brief  INT2 interrupt config
 * @param  ITCombination: Or or And combination
 *         ITAxes: axes to be enabled
 *         NewState: Enable or Disable
 * @retval None
 */
void LSM303DLHC_AccINT2InterruptDisable(uint8_t ITCombination, uint8_t ITAxes)
{
  uint8_t tmpval = 0x00;

  /* Read INT2_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A);

  /* Disable the selected interrupt */
  tmpval &= ~(ITAxes | ITCombination);

  /* Write value to MEMS INT2_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, tmpval);
}

/**
 * @brief  click interrupt enable
 * @param  ITCombination: Or or And combination
 *         ITAxes: axes to be enabled
 * @retval None
 */
void LSM303DLHC_AccClickITEnable(uint8_t ITClick)
{
  uint8_t tmpval = 0x00;

  /* Read CLICK_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A);

  /* Enable the selected interrupt */
  tmpval |= ITClick;

  /* Write value to MEMS CLICK CFG register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, tmpval);

  /* Configure Click Threshold on Z axis */
  tmpval = 0x0A;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_THS_A, tmpval);

  /* Configure Time Limit */
  tmpval = 0x05;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_TIME_LIMIT_A, tmpval);

  /* Configure Latency */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_TIME_LATENCY_A, tmpval);

  /* Configure Click Window */
  tmpval = 0x32;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_TIME_WINDOW_A, tmpval);

}

/**
 * @brief  click interrupt disable
 * @param  ITCombination: Or or And combination
 *         ITAxes: axes to be enabled
 * @retval None
 */
void LSM303DLHC_AccClickITDisable(uint8_t ITClick)
{
  uint8_t tmpval = 0x00;

  /* Read CLICK_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A);

  /* Disable the selected interrupt */
  tmpval &= ~ITClick;

  /* Write value to MEMS CLICK_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, tmpval);
}

/**
 * @brief  click on Z axis interrupt config
 * @param  None
 * @retval None
 */
void LSM303DLHC_AccZClickITConfig(void)
{
  /* configure low level IT config */
  COMPASSACCELERO_IO_ITConfig();

  /* select click IT as INT1 interrupt */
  LSM303DLHC_AccIT1Enable(LSM303DLHC_IT1_CLICK);

  /* Enable High pass filter for click IT */
  LSM303DLHC_AccFilterClickCmd(LSM303DLHC_HPF_CLICK_ENABLE);

  /* Enable simple click IT on Z axis, */
  LSM303DLHC_AccClickITEnable(LSM303DLHC_Z_SINGLE_CLICK);
}

#define MAGNET

#ifdef MAGNET
/**
 * @brief  Set LSM303DLHC Mag Initialization.
 * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_MagInitTypeDef structure
 *         that contains the configuration setting for the LSM303DLHC.
 * @retval None
 */
void LSM303DLHC_MagInit(void)
{
  uint8_t mr_regm = 0x00, cra_regm = 0x00, crb_regm = 0x00;
  magConfig.fullScale = LSM303DLHC_FS_1_3_GA; /* earth's magnetic field vector is .5 Gauss */
  magConfig.xySensitivity = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
  magConfig.zSensitivity = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
  magConfig.dataRate = LSM303DLHC_ODR_75_HZ;
  magConfig.temperatureSensor = LSM303DLHC_TEMPSENSOR_DISABLE;

  /* Configure MEMS: temp and Data rate */
  cra_regm |= (uint8_t) (magConfig.temperatureSensor | magConfig.dataRate);

  /* Configure MEMS: full Scale */
  crb_regm |= (uint8_t) (magConfig.fullScale);

  /* Configure MEMS: working mode */
  mr_regm = (uint8_t) LSM303DLHC_CONTINUOS_CONVERSION;

  /* Write value to Mag MEMS CRA_REG regsister */
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, cra_regm);

  /* Write value to Mag MEMS CRB_REG regsister */
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, crb_regm);

  /* Write value to Mag MEMS MR_REG regsister */
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, mr_regm);
}

/**
 * @brief  Get status for Mag LSM303DLHC data
 * @param  None
 * @retval Data status in a LSM303DLHC Data register
 */
uint8_t LSM303DLHC_MagGetDataStatus(void)
{
  uint8_t tmpreg;

  /* Read Mag STATUS register */
  tmpreg = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_SR_REG_M);

  return tmpreg;
}


/**
 * @brief  Read X, Y & Z Magnetometer  values
 * @param  pfData : Data out pointer
 * @retval None
 */
void LSM303DLHC_MagReadXYZ(float* pfData)
{
  float pnRawData[3];
  uint8_t buffer[6];
  uint8_t i = 0;
  uint8_t addr = MAG_I2C_ADDRESS + 1;

  /* Read output register X, Y & Z acceleration */
  buffer[1] = COMPASSACCELERO_IO_Read(addr, LSM303DLHC_OUT_X_H_M);
  buffer[0] = COMPASSACCELERO_IO_Read(addr, LSM303DLHC_OUT_X_L_M);
  buffer[5] = COMPASSACCELERO_IO_Read(addr, LSM303DLHC_OUT_Z_H_M);
  buffer[4] = COMPASSACCELERO_IO_Read(addr, LSM303DLHC_OUT_Z_L_M);
  buffer[3] = COMPASSACCELERO_IO_Read(addr, LSM303DLHC_OUT_Y_H_M);
  buffer[2] = COMPASSACCELERO_IO_Read(addr, LSM303DLHC_OUT_Y_L_M);

  /* check in the control register4 the data alignment -
   * assume little endian (we never change it on the fly)
   */
  for(i=0; i<3; i++)
  {
    pnRawData[i]=(float)((int16_t)(buffer[2*i+1] << 8) + (int16_t)buffer[2*i]);
  }

  /* Obtain the Gauss value for the three axis */
  pfData[0] = (float) pnRawData[0]/magConfig.xySensitivity;
  pfData[1] = (float) pnRawData[1]/magConfig.xySensitivity;
  pfData[2] = (float) pnRawData[2]/magConfig.zSensitivity;
}
#endif
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
