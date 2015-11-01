/**
  ******************************************************************************
  * @file    l3gd20.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    10-June-2014
  * @brief   This file provides a set of functions needed to manage the L3GD20,
  *          ST MEMS motion sensor, 3-axis digital output gyroscope.  
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
#include "l3gd20.h"
#include <math.h>

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32F401_DISCOVERY
  * @{
  */ 

/** @addtogroup L3GD20
  * @{
  */


/** @defgroup L3GD20_Private_TypesDefinitions
  * @{
  */
  
/**
  * @}
  */

/** @defgroup L3GD20_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup L3GD20_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup L3GD20_Private_Variables
  * @{
  */ 
GYRO_DrvTypeDef L3gd20Drv =
{
  L3GD20_Init,
  L3GD20_ReadID,
  L3GD20_RebootCmd,
  L3GD20_INT1InterruptConfig,
  L3GD20_EnableIT,
  L3GD20_DisableIT,
  0,
  0,
  L3GD20_FilterConfig,
  L3GD20_FilterCmd,
  L3GD20_ReadXYZAngRate
};

/**
  * @}
  */

/** @defgroup L3GD20_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup L3GD20_Private_Functions
  * @{
  */

/**
 * Configure the L3GD20 for FCB usage, bypassing BSP ...
 *
 * enables SPI as well
 *
 * @returns zero upon success, nonzero upon error
 */
uint8_t L3GD20_Config(void) {
  uint8_t gyroId = 0;
  uint8_t ctrlReg1 = 0;
  uint8_t ctrlReg2 = 0;
  uint8_t ctrlReg3 = 0;
  uint8_t ctrlReg4 = 0;

  GYRO_InitTypeDef L3GD20_InitStructure;
  GYRO_FilterConfigTypeDef L3GD20_FilterStructure;

  GYRO_IO_Init(); /* Configure the low level IO interface in this CPU */

  L3GD20_RebootCmd();

  gyroId = L3GD20_ReadID();

  if ((I_AM_L3GD20 != gyroId) && (I_AM_L3GD20_TR != gyroId)) {
    return 1; // error
  }

  /* Configure Mems : data rate, power mode, full scale and axes */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1; /* 96 Hz according to data sheet, 94.5 Hz according to oscilloscope */
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB; /* if changed, modify L3GD20_ReadXYZAngRate as well */
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;

  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrlReg1 = (uint32_t) (L3GD20_InitStructure.Power_Mode | L3GD20_InitStructure.Output_DataRate | \
                    L3GD20_InitStructure.Axes_Enable | L3GD20_InitStructure.Band_Width);

  ctrlReg3 = L3GD20_INT2INTERRUPT_ENABLE;

  ctrlReg4 = (uint8_t) (L3GD20_InitStructure.BlockData_Update | L3GD20_InitStructure.Endianness |
                      L3GD20_InitStructure.Full_Scale);

  L3GD20_Init(ctrlReg1, ctrlReg3, ctrlReg4);

  L3GD20_FilterStructure.HighPassFilter_Mode_Selection = L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_9;

  ctrlReg2 = (uint8_t) ((L3GD20_FilterStructure.HighPassFilter_Mode_Selection |\
                     L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency));

  L3GD20_FilterConfig(ctrlReg2);
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_DISABLE);

  return 0;
}


/**
  * @brief  Sends initialisation parameters to L3GD20 gyroscope.
  *
  * @param ctrlReg 1, 3 and 4 see ST L3GD20 data sheet
  * @retval None
  */
void L3GD20_Init(uint8_t ctrlreg1, uint8_t ctrlreg3, uint8_t ctrlreg4)
{
  /* Write value to MEMS CTRL_REG1 regsister */
  GYRO_IO_Write(&ctrlreg1, L3GD20_CTRL_REG1_ADDR, 1);

  /* Write value to MEMS CTRL_REG3 regsister */
  GYRO_IO_Write(&ctrlreg3, L3GD20_CTRL_REG3_ADDR, 1);
  
  /* Write value to MEMS CTRL_REG4 regsister */
  GYRO_IO_Write(&ctrlreg4, L3GD20_CTRL_REG4_ADDR, 1);
}

/**
  * @brief  Read ID address of L3GD20
  * @param  Device ID address
  * @retval ID name
  */
uint8_t L3GD20_ReadID(void)
{
  uint8_t tmp;

  /* Configure the low level interface ---------------------------------------*/
  GYRO_IO_Init();
  
  /* Read WHO I AM register */
  GYRO_IO_Read(&tmp, L3GD20_WHO_AM_I_ADDR, 1);

  /* Return the ID */
  return (uint8_t)tmp;
}

/**
  * @brief  Reboot memory content of L3GD20
  * @param  None
  * @retval None
  */
void L3GD20_RebootCmd(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  
  /* Enable or Disable the reboot memory */
  tmpreg |= L3GD20_BOOT_REBOOTMEMORY;
  
  /* Write value to MEMS CTRL_REG5 regsister */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
}


/**
  * @brief Set L3GD20 Interrupt INT1 configuration
  * @param  L3GD20_InterruptConfig_TypeDef: pointer to a L3GD20_InterruptConfig_TypeDef 
  *         structure that contains the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */
void L3GD20_INT1InterruptConfig(uint16_t Int1Config)
{
  uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;
  
  /* Read INT1_CFG register */
  GYRO_IO_Read(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
  
  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((uint8_t) Int1Config >> 8);
  
  ctrl3 &= 0xDF;
  ctrl3 |= ((uint8_t) Int1Config);
  
  /* Configure latch Interrupt request and axe interrupts */                   
/*  ctrl_cfr |= (uint8_t)(L3GD20_IntConfigStruct->Latch_Request| \
                   L3GD20_IntConfigStruct->Interrupt_Axes);
                   
  ctrl3 |= (uint8_t)(L3GD20_IntConfigStruct->Interrupt_ActiveEdge);
*/  
  /* Write value to MEMS INT1_CFG register */
  GYRO_IO_Write(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Enable INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2   
  * @retval None
  */
void L3GD20_EnableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  
  if(IntSel == L3GD20_INT1)
  {
    tmpreg &= 0x7F;	
    tmpreg |= L3GD20_INT1INTERRUPT_ENABLE;
  }
  else if(IntSel == L3GD20_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_ENABLE;
  }
  
  /* Write value to MEMS CTRL_REG3 regsister */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Disable  INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2   
  * @retval None
  */
void L3GD20_DisableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  
  if(IntSel == L3GD20_INT1)
  {
    tmpreg &= 0x7F;	
    tmpreg |= L3GD20_INT1INTERRUPT_DISABLE;
  }
  else if(IntSel == L3GD20_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_DISABLE;
  }
  
  /* Write value to MEMS CTRL_REG3 regsister */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  FilterStruct: contains the configuration setting for the L3GD20.        
  * @retval None
  */
void L3GD20_FilterConfig(uint8_t FilterStruct) 
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
  
  tmpreg &= 0xC0;
  
  /* Configure MEMS: mode and cutoff frquency */
  tmpreg |= FilterStruct;

  /* Write value to MEMS CTRL_REG2 regsister */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: L3GD20_HIGHPASSFILTER_DISABLE 
  *         @arg: L3GD20_HIGHPASSFILTER_ENABLE          
  * @retval None
  */
void L3GD20_FilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  
  tmpreg &= 0xEF;
  
  tmpreg |= HighPassFilterState;
  
  /* Write value to MEMS CTRL_REG5 regsister */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
}

/**
  * @brief  Get status for L3GD20 data
  * @param  None         
  * @retval Data status in a L3GD20 Data
  */
uint8_t L3GD20_GetDataStatus(void)
{
  uint8_t tmpreg;
  
  /* Read STATUS_REG register */
  GYRO_IO_Read(&tmpreg, L3GD20_STATUS_REG_ADDR, 1);
                  
  return tmpreg;
}

/**
* @brief  Calculate the L3GD20 angular data.
* @param  pfData : Data out pointer
* @retval None
*/
void L3GD20_ReadXYZAngRate(float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;
  
  GYRO_IO_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  GYRO_IO_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/

  for(i=0; i<3; i++)
  {
    /* assume L3GD20_BLE_LSB endianness - this is configurable
     * see L3GD20 data sheet
     */
    RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & L3GD20_FULLSCALE_SELECTION)
  {
  case L3GD20_FULLSCALE_250:
    sensitivity=L3GD20_SENSITIVITY_250DPS;
    break;
    
  case L3GD20_FULLSCALE_500:
    sensitivity=L3GD20_SENSITIVITY_500DPS;
    break;
    
  case L3GD20_FULLSCALE_2000:
    sensitivity=L3GD20_SENSITIVITY_2000DPS;
    break;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
	/* translate data from milli degreees/sec to rad/sec */
    pfData[i]=(float)(RawData[i] * sensitivity * M_PI / 180 / 1000);
  }
}

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
