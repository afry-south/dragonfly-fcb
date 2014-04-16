#include "stm32f10x.h"
#include "stm3210e_eval_fsmc_sram.h"

#define StartAddresse 0x68000000

/* Description  :                                                                     */
/*   Initilize the MCU Clock, the GPIO Pins corresponding to the                      */
/*   device and initilize the FSMC with the chosen configuration                      */
/* Inputs       :                                                                     */
/*                 None                                                               */
/* outputs      :                                                                     */
/*                 "1" : Operation succeeded                                          */
/*                 "0" : Operation failure                                            */
/* Note: Mandatory for all types of device                                            */

int Init (void)
{  
	/* Set MCU Clock */
	SystemInit();
	
	/* Enable the FSMC Clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	
	/* Init the External SRAM */
	SRAM_Init();
	
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

int Read (uint32_t Address, uint32_t Size, uint32_t Buffer)
{ 
        uint32_t InternalAddr = Address - StartAddresse;
        uint32_t ReadData = 0;    
        uint16_t TmpBuffer = 0x0000;
        
        if (InternalAddr%2 != 0)
        {
          SRAM_ReadBuffer(&TmpBuffer, (InternalAddr - InternalAddr%2),1);
          
          *((uint8_t*)Buffer) = (uint8_t)(TmpBuffer>>8);
          
          ReadData++;
        }
        
        if (Size-ReadData >= 2)
        {
          SRAM_ReadBuffer((uint16_t*)((uint8_t*)Buffer+ReadData), InternalAddr+ReadData ,(Size-ReadData)/2);
          
          ReadData += (((Size-ReadData)/2)*2);
        }
        
        if (ReadData < Size)
        {  
          SRAM_ReadBuffer(&TmpBuffer, InternalAddr+ReadData ,1);
          
          *((uint8_t*)Buffer+ReadData) = (uint8_t)(TmpBuffer&0x00FF);
        }

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

int Write (uint32_t Address, uint32_t Size, uint32_t Buffer)
{
	uint32_t InternalAddr = Address - StartAddresse;
        uint32_t WritenData = 0;    
        uint16_t TmpBuffer = 0x0000;
        
        if (InternalAddr%2 != 0)
        {
          SRAM_ReadBuffer (&TmpBuffer, (InternalAddr - InternalAddr%2),1);
          
          *((uint8_t*)(&TmpBuffer)+1) = *(uint8_t*)(Buffer);
          
          SRAM_WriteBuffer (&TmpBuffer, (InternalAddr - InternalAddr%2),1);
          
          WritenData++;
        }
        
        if (Size-WritenData >= 2)
        {
          SRAM_WriteBuffer ((uint16_t*)((uint8_t*)Buffer+WritenData), InternalAddr+WritenData, ((Size-WritenData)/2));    

          WritenData += (((Size-WritenData)/2)*2);
        }
        
        if (WritenData < Size)
        {
          SRAM_ReadBuffer (&TmpBuffer, InternalAddr+WritenData,1);
          
          *((uint8_t*)(&TmpBuffer)) = *(uint8_t*)((uint8_t*)Buffer+WritenData);
          
          SRAM_WriteBuffer (&TmpBuffer, InternalAddr+WritenData,1);
        }

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

// int MassErase (void)
// {  
// 	return 1;
// }


/* Description :																			*/
/* Erase a full sector in the device 												*/
/* Inputs :																					*/
/* 				SectrorAddress	: Start of sector 								*/
/* outputs :																				*/
/* 				"1" : Operation succeeded											*/
/* 				"0" : Operation failure													*/
/* Note : Not Mandatory for SRAM PSRAM and NOR_FLASH		*/	

// int SectorErase (uint32_t SectrorAddress)
// {  
// 	return 1;
// }


/* Description :                                                    */                                                                              
/* Initilize the MCU Clock, the GPIO Pins corresponding to the            */
/* device and initilize the FSMC with the chosen configuration            */
/* Inputs    :                                                       */                                                                            
/*      FlashAddr   : Flash address                                */
/*      RAMBufferAddr : RAM buffer address                           */
/*     Size         : Size (in WORD)                        */
/* outputs   :                                              */                                                                                     
/*     "0"          : Operation succeeded                          */
/*     address of failure : Operation failed (address of failure)  */
/* Note: Optional for all types of device                          */

int Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size)
{ 
      uint32_t InternalAddr = MemoryAddr - StartAddresse;
      uint32_t VerifiedData = 0;
      uint16_t TmpBuffer = 0x0000;
      Size*=4;
      
      if (InternalAddr%2 != 0)
      {
        SRAM_ReadBuffer(&TmpBuffer, (InternalAddr - InternalAddr%2),1);
        
        if ((uint8_t)(TmpBuffer>>8) != (*(uint8_t*)RAMBufferAddr))
            return MemoryAddr;
        VerifiedData++;    
      }
      
      while ((Size-VerifiedData)>1)
      {
        SRAM_ReadBuffer(&TmpBuffer, InternalAddr+VerifiedData,1);
        
        if ((TmpBuffer&0x00FF) != (*((uint8_t*)RAMBufferAddr+VerifiedData)))
            return MemoryAddr+VerifiedData;
        
        VerifiedData++;
             
        if ((uint8_t)(TmpBuffer>>8) != (*((uint8_t*)RAMBufferAddr+VerifiedData)))
            return MemoryAddr+VerifiedData;
        
        VerifiedData++;
      }
      
      if ((Size-VerifiedData) != 0)
      {
        SRAM_ReadBuffer(&TmpBuffer, InternalAddr+VerifiedData,1);
        
        if ((uint8_t)(TmpBuffer&0x00FF) != (*((uint8_t*)RAMBufferAddr+VerifiedData)))
            return MemoryAddr+VerifiedData;
      }

      return 0;
}
