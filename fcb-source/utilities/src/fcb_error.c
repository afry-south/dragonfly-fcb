/******************************************************************************
 * @file    fcb_error.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-06-09
 * @brief   Module contains error handling functions
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "fcb_error.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define STACK_REGISTERS_STRING_SIZE		256

/* Exported functions --------------------------------------------------------*/

void ErrorHandler(void) {
  BSP_LED_On (LED3);
  BSP_LED_On (LED4);
  BSP_LED_On (LED5);
  BSP_LED_On (LED6);
  BSP_LED_On (LED7);
  BSP_LED_On (LED8);
  BSP_LED_On (LED9);
  BSP_LED_On (LED10);

  // TODO turn off motors!

  /* TODO in the future this function should accept a text
   * string that could be printed to USB, or elsewhere
   */
  while (1)
    {
    }
}

void GetRegistersFromStack(uint32_t* pulFaultStackAddress) {
	/* These are volatile to try and prevent the compiler/linker optimizing them
	 * away as the variables never actually get used.  If the debugger won't show the
	 * values of the variables, make them global my moving their declaration outside
	 * of this function. */
	volatile unsigned int sp_r0;
	volatile unsigned int r1;
	volatile unsigned int r2;
	volatile unsigned int r3;
	volatile unsigned int r12;
	volatile unsigned int lr; /* Link register. */
	volatile unsigned int pc; /* Program counter. */
	volatile unsigned int psr; /* Program status register. */
	volatile unsigned int _CFSR;
	volatile unsigned int _HFSR;
	volatile unsigned int _DFSR;
	volatile unsigned int _AFSR;
	volatile unsigned int _BFAR;
	volatile unsigned int _MMAR;
	char hardFaultString[STACK_REGISTERS_STRING_SIZE];

	sp_r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];
	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	/* Configurable Fault Status Register - Consists of MMSR, BFSR and UFSR */
	_CFSR = (*((volatile unsigned int*)(0xE000ED28)));

	/* Hard Fault Status Register */
	_HFSR = (*((volatile unsigned int*)(0xE000ED2C)));

	/* Debug Fault Status Register */
	_DFSR = (*((volatile unsigned int*)(0xE000ED30)));

	// Auxiliary Fault Status Register
	_AFSR = (*((volatile unsigned int*)(0xE000ED3C)));

	/* Read the Fault Address Registers. These may not contain valid values.
	 * Check BFARVALID/MMARVALID to see if they are valid values */

	/* MemManage Fault Address Register */
	_MMAR = (*((volatile unsigned int*)(0xE000ED34)));
	/* Bus Fault Address Register */
	_BFAR = (*((volatile unsigned int*)(0xE000ED38)));

	USBComSendString("Oh no, a Hard Fault Exception! Every byte for itself!\n");

	snprintf(hardFaultString, STACK_REGISTERS_STRING_SIZE,
			"SP_R0: 0x%x\nR1: 0x%x\nR2: 0x%x\nR3: 0x%x\nR12: 0x%x\nLR: 0x%x\nPC: 0x%x\nPSR: 0x%x\nCFSR: 0x%x\nHFSR: 0x%x\nDFSR: 0x%x\nAFSR: 0x%x\nMMAR: 0x%x\nBFAR: 0x%x\n\r\n",
			sp_r0, r1, r2, r3, r12, lr, pc, psr, _CFSR, _HFSR, _DFSR, _AFSR, _MMAR, _BFAR);

	USBComSendString(hardFaultString);
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/

