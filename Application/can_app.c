/**
  ******************************************************************************
  * @file    can_app.c
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   ����CAN���ߵ�Bootloader����APP���Գ���.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can_app.h"
#include "crc16.h"
/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);

/* Private define ------------------------------------------------------------*/

/* Base address of the Flash sectors */
#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x80000)  /* 512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else 
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"    
#endif  
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  ����һ֡CAN����
  * @param  CANx CANͨ����
  * @param  TxMessage CAN��Ϣָ��
  * @retval None
  */
uint8_t CAN_WriteData(CanTxMsg *TxMessage)
{
  uint8_t TransmitMailbox;
  uint32_t  TimeOut=0;

  TransmitMailbox = CAN_Transmit(CAN1, TxMessage);
  while (CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok) {
    TimeOut++;
    if (TimeOut > 100) {
      return 1;
    }
  }
  return 0;
}

/**
  * @brief  ��������д��ָ����ַ��Flash�� ��
  * @param  Address Flash��ʼ��ַ��
  * @param  Data ���ݴ洢����ʼ��ַ��
  * @param  DataNum �����ֽ�����
  * @retval ������д״̬��
  */
FLASH_Status CAN_BOOT_ProgramDatatoFlash(uint32_t StartAddress,uint8_t *pData,uint32_t DataNum) 
{
	FLASH_Status FLASHStatus = FLASH_COMPLETE;

	uint32_t i;

	if(StartAddress<APP_EXE_FLAG_ADDR){
		return FLASH_ERROR_PG;
	}
   /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	
  for(i=0;i<(DataNum>>2);i++)
	{
    FLASHStatus = FLASH_ProgramWord(StartAddress, *((uint32_t*)pData));
		if (FLASHStatus == FLASH_COMPLETE){
      StartAddress += 4;
      pData += 4;
    }else{ 
			return FLASHStatus;
    }
  }
	return	FLASHStatus;
}
/**
  * @brief  ����ָ�����������Flash���� ��
  * @param  StartPage ��ʼ����
  * @param  EndPage ��������
  * @retval ��������״̬  
  */
FLASH_Status CAN_BOOT_ErasePage(uint32_t StartPageAddr,uint32_t EndPageAddr)
{
  uint32_t i;
  FLASH_Status FLASHStatus=FLASH_COMPLETE;

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  for(i=StartPageAddr;i<=EndPageAddr;i+=PAGE_SIZE){
    FLASHStatus = FLASH_ErasePage(i);
    if(FLASHStatus!=FLASH_COMPLETE){
      FLASH_Lock();
      return	FLASHStatus;	
    }
  }
  FLASH_Lock();
  return FLASHStatus;
}


/**
  * @brief  ��ȡCAN�ڵ��ַ���ú��������Լ���ʵ����������޸�
  * @param  None
  * @retval None
  */
uint8_t GetNAD(void)
{
  uint8_t NAD=0x12;
	//����оƬΨһ������ϳ�һ��NAD
  uint32_t sn = *(__IO uint32_t*)(0x1FFFF7E8)+*(__IO uint32_t*)(0x1FFFF7EC)+*(__IO uint32_t*)(0x1FFFF7F0);
  NAD = (sn>>24)+(sn>>16)+(sn>>8)+sn;
	
  return NAD&0x7F;
}

/**
  * @brief  ���Ƴ�����ת��ָ��λ�ÿ�ʼִ�� ��
  * @param  Addr ����ִ�е�ַ��
  * @retval ������ת״̬��
  */
void CAN_BOOT_JumpToApplication(__IO uint32_t Addr)
{
	pFunction Jump_To_Application;
	__IO uint32_t JumpAddress; 
	/* Test if user code is programmed starting from address "ApplicationAddress" */
	if (((*(__IO uint32_t*)Addr) & 0x2FFE0000 ) == 0x20000000)
	{ 
	  /* Jump to user application */
	  JumpAddress = *(__IO uint32_t*) (Addr + 4);
	  Jump_To_Application = (pFunction) JumpAddress;
		__set_PRIMASK(1);//�ر������ж�
	  /* Initialize user application's Stack Pointer */
	  __set_MSP(*(__IO uint32_t*)Addr);
	  Jump_To_Application();
	}
}


/**
  * @brief  ִ�������·�������
  * @param  pRxMessage CAN������Ϣ
  * @retval ��
  */
void CAN_BOOT_ExecutiveCommand(CanRxMsg *pRxMessage)
{
  CanTxMsg TxMessage;
	uint8_t PCI = pRxMessage->Data[1];
	uint8_t SID=0xFF;
	uint8_t CMD=0xFF;
	//������Ϣ��ȡSID��CMD
	if((PCI&0xF0)==0x00){
		SID = pRxMessage->Data[2];
		CMD = pRxMessage->Data[3];
	}else if((PCI&0xF0)==0x10){
		SID = pRxMessage->Data[3];
		CMD = pRxMessage->Data[4];
	}
	//׼�����͵�����
  TxMessage.DLC = 8;
	if(MSG_ID_TYPE == CAN_Id_Standard){
		TxMessage.StdId = MSG_SEND_ID;
		TxMessage.IDE = CAN_Id_Standard;
	}else{
		TxMessage.ExtId = MSG_SEND_ID;
		TxMessage.IDE = CAN_Id_Extended;
	}
  TxMessage.RTR = CAN_RTR_Data;
	TxMessage.Data[0] = GetNAD();//���NAD
	//��������
	switch(CMD)
	{
		case CMD_GET_FW_INFO:
			TxMessage.Data[1] = 0x06;
			TxMessage.Data[2] = SID+0x40;
			TxMessage.Data[3] = FW_TYPE;
			TxMessage.Data[4] = 0;//�̼��汾�� Major
			TxMessage.Data[5] = 0;//�̼��汾�� Minor
			TxMessage.Data[6] = 0;//�̼��汾�� Revision
			TxMessage.Data[7] = 2;//�̼��汾�� Build
			CAN_WriteData(&TxMessage);
			break;
		case CMD_ENTER_BOOT:
			//��BOOT�����в���Ҫִ���κβ�������APP����������Ҫ����APP���б�־����ת��BOOTִ�л��������λ����
			FLASH_Unlock();
			CAN_BOOT_ErasePage(APP_EXE_FLAG_ADDR,APP_EXE_FLAG_ADDR);//����д�뵽Flash�е�APPִ�б�־����λ���к󣬼���ִ��Bootloader����
			FLASH_Lock();
			__set_PRIMASK(1);//�ر������ж�
			NVIC_SystemReset();
			break;
		case CMD_ERASE_APP:
			//APP�в���ִ�в���APP�Ĳ���
			TxMessage.Data[1] = 0x06;
			TxMessage.Data[2] = SID+0x40;
			TxMessage.Data[3] = CAN_BOOT_ERR_ERASE_IN_APP;
			TxMessage.Data[4] = 0xFF;TxMessage.Data[5] = 0xFF;TxMessage.Data[6] = 0xFF;TxMessage.Data[7] = 0xFF;
			CAN_WriteData(&TxMessage);
			break;
		case CMD_SET_ADDR_OFFSET:
			TxMessage.Data[1] = 0x06;
			TxMessage.Data[2] = SID+0x40;
			TxMessage.Data[3] = CAN_BOOT_ERR_WRITE_IN_APP;
			TxMessage.Data[4] = 0xFF;TxMessage.Data[5] = 0xFF;TxMessage.Data[6] = 0xFF;TxMessage.Data[7] = 0xFF;
			CAN_WriteData(&TxMessage);
			break;
		case CMD_TRAN_DATA:
			break;
		case CMD_WRITE_DATA:
			TxMessage.Data[1] = 0x06;
			TxMessage.Data[2] = SID+0x40;
			TxMessage.Data[3] = CAN_BOOT_ERR_WRITE_IN_APP;
			TxMessage.Data[4] = 0xFF;TxMessage.Data[5] = 0xFF;TxMessage.Data[6] = 0xFF;TxMessage.Data[7] = 0xFF;
			CAN_WriteData(&TxMessage);
			break;
		case CMD_ENTER_APP:
			break;
		default:
			break;
	}	
}
/*********************************END OF FILE**********************************/

