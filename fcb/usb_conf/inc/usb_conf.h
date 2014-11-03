/**
  ******************************************************************************
  * @file    usb_conf.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   demo configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CONF_H
#define __USB_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* External variables --------------------------------------------------------*/

/*-------------------------------------------------------------*/
/* EP_NUM */
/* defines how many endpoints are used by the device */
/*-------------------------------------------------------------*/

#define EP_NUM				(4)

/*-------------------------------------------------------------*/
/* --------------   Buffer Description Table  -----------------*/
/*-------------------------------------------------------------*/

/*


As for your other question: packet memory is also used to store buffer description table. Depending on number and type of used endpoints you need to reserve enough space, for example:

PMAAddr + BASEADDR_BTABLE + 0x00000000 : EP0_TX_ADDR
PMAAddr + BASEADDR_BTABLE + 0x00000002 : EP0_TX_COUNT
PMAAddr + BASEADDR_BTABLE + 0x00000004 : EP0_RX_ADDR
PMAAddr + BASEADDR_BTABLE + 0x00000006 : EP0_RX_COUNT
PMAAddr + BASEADDR_BTABLE + 0x00000008 : EP1_TX_ADDR
PMAAddr + BASEADDR_BTABLE + 0x0000000A : EP1_TX_COUNT
PMAAddr + BASEADDR_BTABLE + 0x0000000C : EP1_RX_ADDR
PMAAddr + BASEADDR_BTABLE + 0x0000000E : EP1_RX_COUNT
PMAAddr + BASEADDR_BTABLE + 0x00000010 : EP2_TX_ADDR
PMAAddr + BASEADDR_BTABLE + 0x00000012 : EP2_TX_COUNT
PMAAddr + BASEADDR_BTABLE + 0x00000014 : EP2_RX_ADDR
PMAAddr + BASEADDR_BTABLE + 0x00000016 : EP2_RX_COUNT

and in such case we get 0x18 address for endpoint 0 transmission buffer.
You can find more details in the reference manual


*/

/* buffer table base address */
#define BTABLE_ADDRESS      (0x00)



/*
 *
 * Every device must have endpoint zero configured as a control endpoint. There’s
rarely if ever a need for additional control endpoints.
 *
 */



/* EP0  */
/* rx/tx buffer base address */
#define ENDP0_RXADDR        (0x40)
#define ENDP0_TXADDR        (0x80)

/* EP1  */
/* tx buffer base address */
#define ENDP1_TXADDR        (0xC0)
#define ENDP2_TXADDR        (0x100)
#define ENDP3_RXADDR        (0x110)

/*-------------------------------------------------------------*/
/* -------------------   ISTR events  -------------------------*/
/*-------------------------------------------------------------*/
/* IMR_MSK */
/* mask defining which events has to be handled */
/* by the device application software */
#define IMR_MSK (CNTR_CTRM  | CNTR_WKUPM | CNTR_SUSPM | CNTR_ERRM  | CNTR_SOFM \
                 | CNTR_ESOFM | CNTR_RESETM )

/* CTR service routines */
/* associated to defined endpoints */
/* #define  EP1_IN_Callback   NOP_Process*/
#define  EP2_IN_Callback   NOP_Process
#define  EP3_IN_Callback   NOP_Process
#define  EP4_IN_Callback   NOP_Process
#define  EP5_IN_Callback   NOP_Process
#define  EP6_IN_Callback   NOP_Process
#define  EP7_IN_Callback   NOP_Process

#define  EP1_OUT_Callback   NOP_Process
#define  EP2_OUT_Callback   NOP_Process
//#define  EP3_OUT_Callback   NOP_Process
#define  EP4_OUT_Callback   NOP_Process
#define  EP5_OUT_Callback   NOP_Process
#define  EP6_OUT_Callback   NOP_Process
#define  EP7_OUT_Callback   NOP_Process

#endif /*__USB_CONF_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

