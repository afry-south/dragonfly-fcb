#include "stm32f10x.h"
#include "stm3210e_eval_fsmc_nor.h"

#define StartAddresse 0x64000000

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
	/* Set MCU Clock */
	SystemInit();
	
	/* Enable the FSMC Clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	
	/* Init the External SRAM */
	NOR_Init();
	
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

int Read (uint32_t Address, uint32_t Size, uint16_t* Buffer)
{      
      uint32_t InternalAddr = Address - StartAddresse;
      uint32_t ReadedData = 0;
      uint32_t Counter = 0;    
      uint16_t TmpBuffer = 0x00000000;
      
      if (InternalAddr%2 != 0)
      {
        NOR_ReadBuffer(&TmpBuffer, (InternalAddr - InternalAddr%2), 1);
        
        for (Counter =0; (Counter<(InternalAddr%2))&&(Counter<Size); Counter++)
          *((uint8_t*)Buffer+Counter) = *((uint8_t*)(&TmpBuffer)+InternalAddr%4+Counter);
        
        ReadedData += Counter;
      }
      
      if (Size-ReadedData >= 2)
      {
        NOR_ReadBuffer((uint16_t*)((uint8_t*)Buffer+ReadedData), InternalAddr+ReadedData ,(Size-ReadedData)/2);
        
        ReadedData += (((Size-ReadedData)/2)*2);
      }
      
      if (ReadedData < Size)
      {  
        NOR_ReadBuffer(&TmpBuffer, InternalAddr+ReadedData ,1);
        
        for (Counter =0; Counter<(Size-ReadedData); Counter++)
          *((uint8_t*)Buffer+ReadedData+Counter) = *((uint8_t*)(&TmpBuffer)+Counter);
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

int Write (uint32_t Address, uint32_t Size, uint16_t* Buffer)
{  
        uint32_t InternalAddr = Address - StartAddresse;
        uint32_t WritedData = 0;
        uint32_t Counter = 0;    
        uint16_t TmpBuffer = 0x00000000;
        
        if (InternalAddr%2 != 0)
        {
          NOR_ReadBuffer (&TmpBuffer, (InternalAddr - InternalAddr%2),1);
          
          for (Counter =0; (Counter<(2-InternalAddr%2))&&(Counter<Size); Counter++)
            *((uint8_t*)(&TmpBuffer)+InternalAddr%2+Counter) = * ((uint8_t*)Buffer+Counter);
          
          if (NOR_WriteBuffer (&TmpBuffer, (InternalAddr - InternalAddr%2),1) != 0)
            return 0;
          
          WritedData += Counter;
        }
        
        if (Size-WritedData >= 2)
        {
          if (NOR_WriteBuffer ((uint16_t*)((uint8_t*)Buffer+WritedData), InternalAddr+WritedData, ((Size-WritedData)/2))!=0)
            return 0;

          WritedData += (((Size-WritedData)/2)*2);
        }
        
        if (WritedData < Size)
        {
          NOR_ReadBuffer (&TmpBuffer, InternalAddr+WritedData,1);

          for (Counter =0; Counter<(Size-WritedData); Counter++)
                  *((uint8_t*)(&TmpBuffer)+Counter) = *((uint8_t*)Buffer+WritedData+Counter);
          
          if (NOR_WriteBuffer (&TmpBuffer, InternalAddr+WritedData,1)!=0)
            return 0;
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

int MassErase (void)
{  
	if (NOR_EraseChip()==0)
		return 1;
	else
		return 0;
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
	uint32_t BlockAddr;
	EraseStartAddress = EraseStartAddress -  EraseStartAddress%0x20000;
	while (EraseEndAddress>=EraseStartAddress)
	{
         BlockAddr = EraseStartAddress - StartAddresse;
         if (NOR_EraseBlock(BlockAddr)!=0)
          return 0;
         EraseStartAddress+=0x20000;
	}
 	return 1;	
}



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
        NOR_ReadBuffer(&TmpBuffer, (InternalAddr - InternalAddr%2),1);
        
        if ((uint8_t)(TmpBuffer>>8) != (*(uint8_t*)RAMBufferAddr))
            return MemoryAddr;
        VerifiedData++;    
      }
      
      while ((Size-VerifiedData)>1)
      {
        NOR_ReadBuffer(&TmpBuffer, InternalAddr+VerifiedData,1);
        
        if ((TmpBuffer&0x00FF) != (*((uint8_t*)RAMBufferAddr+VerifiedData)))
            return MemoryAddr+VerifiedData;
        
        VerifiedData++;
             
        if ((uint8_t)(TmpBuffer>>8) != (*((uint8_t*)RAMBufferAddr+VerifiedData)))
            return MemoryAddr+VerifiedData;
        
        VerifiedData++;
      }
      
      if ((Size-VerifiedData) != 0)
      {
        NOR_ReadBuffer(&TmpBuffer, InternalAddr+VerifiedData,1);
        
        if ((uint8_t)(TmpBuffer&0x00FF) != (*((uint8_t*)RAMBufferAddr+VerifiedData)))
            return MemoryAddr+VerifiedData;
      }

      return 0;
}

