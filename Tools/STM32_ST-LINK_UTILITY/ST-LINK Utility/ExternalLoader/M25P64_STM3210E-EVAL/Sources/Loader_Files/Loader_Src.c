#include "stm32f10x.h"
#include "stm32_eval_spi_flash.h"
#include "stm3210e_eval.h"

/* Description :																			*/
/* Initilize the MCU Clock, the GPIO Pins corresponding to the 		*/
/* device and initilize the FSMC with the chosen configuration 		*/
/* Inputs 	:																				*/
/* 				 None 																		*/
/* outputs 	:																				*/
/* 				"1" 			: Operation succeeded								*/
/* 				"0" 			: Operation failure										*/
/* Note: Mandatory for all types of device 									*/

int Init (void)
{  
        SystemInit();
	sFLASH_Init();
	return 1;
}

/* Description :																			*/
/* Read data from the device 													    */
/* Inputs :																					*/
/* 				Address 	: Read location  										*/
/* 				Size 		: Length in bytes 										*/
/* 				buffer 		: Address where to write readed data			*/
/* outputs :																				*/
/* 				"1" 			: Operation succeeded								*/
/* 				"0" 			: Operation failure										*/
/* Info :																						*/
/* Align and memory size (32/16/8 bits) is handled in this function 	*/
/* Note : Not Mandatory for SRAM PSRAM and NOR_FLASH 		*/		

int Read (uint32_t Address, uint32_t Size, uint8_t* buffer)
{ 
	sFLASH_ReadBuffer(buffer, Address, Size);
	return 1;
} 

/* Description :																			*/
/* Write data to the device	 														*/
/* Inputs :																					*/
/* 				Address 	: Write location  										*/
/* 				Size 		: Length in bytes 										*/
/* 				buffer 		: Address where to get the data to write		*/
/* outputs :																				*/
/* 				"1" 			: Operation succeeded								*/
/* 				"0" 			: Operation failure										*/
/* Info :																						*/
/* Align and memory size (32/16/8 bits) is handled in this function 	*/
/* Note : Mandatory for all types except SRAM and PSRAM			*/	

int Write (uint32_t Address, uint32_t Size, uint8_t* buffer)
{
	sFLASH_WriteBuffer(buffer, Address, Size);
	return 1;
} 

/* Description :																			*/
/* Full erase of the device 															*/
/* Inputs :																					*/
/* 				None 																		*/
/* outputs :																				*/
/* 				"1" : Operation succeeded											*/
/* 				"0" : Operation failure													*/
/* Info :																						*/
/* Note : Not Mandatory for SRAM PSRAM and NOR_FLASH		*/	

int MassErase (void)
{  
	sFLASH_EraseBulk();
	return 1;	
}

/* Description :																			*/
/* Erase a full sector in the device 												*/
/* Inputs :																					*/
/* 				SectrorAddress	: Start of sector 								*/
/* outputs :																				*/
/* 				"1" : Operation succeeded											*/
/* 				"0" : Operation failure													*/
/* Note : Not Mandatory for SRAM PSRAM and NOR_FLASH		*/	


int SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress)
{      
	EraseStartAddress = EraseStartAddress -  EraseStartAddress%0x10000;
	while (EraseEndAddress>=EraseStartAddress)
	{
	 sFLASH_EraseSector(EraseStartAddress);
	 EraseStartAddress += 0x10000;
	}
 	return 1;	
}


/* Description :                                                    */                                                                              
/* Initilize the MCU Clock, the GPIO Pins corresponding to the            */
/* device and initilize the FSMC with the chosen configuration            */
/* Inputs    :                                                       */                                                                            
/*      FlashAddr   : Flash address                                */
/*      RAMBufferAddr : RAM buffer address                           */
/*      Size         : Size (in WORD)                        */
/* outputs   :                                              */                                                                                     
/*     "0"          : Operation succeeded                          */
/*     address of failure : Operation failed (address of failure)  */
/* Note: Optional for all types of device                          */

int Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size)
{ 
        uint32_t VerifiedData = 0;
        uint8_t TmpBuffer = 0x00;
        Size*=4;
        
        while (Size>VerifiedData)
        {
          sFLASH_ReadBuffer(&TmpBuffer, MemoryAddr+VerifiedData, 1);
          
          if (TmpBuffer != *((uint8_t*)RAMBufferAddr+VerifiedData))
            return MemoryAddr+VerifiedData;
          
          VerifiedData++;  
        }
        
        return 0;
}

