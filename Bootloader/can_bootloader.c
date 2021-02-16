/**
  ******************************************************************************
  * @file    can_bootloader.c
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   ����CAN���ߵ�Bootloader����.
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
#include "can_bootloader.h"
#include "crc16.h"
#ifdef ENCRYPT
#include "aes.h"
#endif

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);
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
// #error "Please select first the STM32 device to be used (in stm32f10x.h)"
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
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
  * @brief  ��ȡCAN�ڵ��ַ���ú��������Լ���ʵ����������޸�
  * @param  None
  * @retval �ڵ��ַ
  */
uint8_t GetNAD(void)
{
  uint8_t NAD=0x12;
  //����оƬΨһ������ϳ�һ��NAD
  uint32_t sn = *(__IO uint32_t *)(0x1FFFF7E8) + *(__IO uint32_t *)(0x1FFFF7EC) + *(__IO uint32_t *)(0x1FFFF7F0);
  NAD = (sn>>24) + (sn>>16) + (sn>>8) + sn;

  return NAD&0x7F;
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
  uint32_t i;
  FLASH_Status FLASHStatus = FLASH_COMPLETE;

  if (StartAddress < APP_EXE_FLAG_ADDR) {
    return FLASH_ERROR_PG;
  }
   /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  for (i = 0; i < (DataNum>>2); i++) {
    FLASHStatus = FLASH_ProgramWord(StartAddress, *((uint32_t*)pData));
    if (FLASHStatus == FLASH_COMPLETE) {
      StartAddress += 4;
      pData += 4;
    } else {
      return FLASHStatus;
    }
  }
  return FLASHStatus;
}
/**
  * @brief  ����ָ�����������Flash���� ��
  * @param  StartPage ��ʼ������ַ
  * @param  EndPage ����������ַ
  * @retval ��������״̬
  */
FLASH_Status CAN_BOOT_ErasePage(uint32_t StartPageAddr,uint32_t EndPageAddr)
{
  uint32_t i;
  FLASH_Status FLASHStatus = FLASH_COMPLETE;

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  for (i = StartPageAddr; i <= EndPageAddr; i += PAGE_SIZE) {
    FLASHStatus = FLASH_ErasePage(i);
    if (FLASHStatus != FLASH_COMPLETE) {
      FLASH_Lock();
      return FLASHStatus;
    }
  }
  FLASH_Lock();
  return FLASHStatus;
}

/**
  * @brief  ���Ƴ�����ת��ָ��λ�ÿ�ʼִ�� ��
  * @param  Addr ����ִ�е�ַ��
  * @retval ������ת״̬��
  */
void CAN_BOOT_JumpToApplication(uint32_t Addr)
{
  static pFunction Jump_To_Application;
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
  uint8_t ret,i;
  uint8_t *pData;
  uint8_t PCI = pRxMessage->Data[1];
  uint8_t SID = 0xFF;
  uint8_t CMD = 0xFF;
  static uint8_t OLD_CMD = 0xFF;
  uint16_t crc_data;
  static uint8_t CF_Index=1;
  static uint32_t start_addr = APP_START_ADDR;
  static uint32_t data_size = 0;
  static uint32_t data_index = 0;
  static uint32_t addr_offset = 0;
  __align(4) static uint8_t data_temp[1024];
#if ENCRYPT
  const char *pKey="123456789abcdefggdfrthfgdfgefsse";
  static uint8_t chainCipherBlock[16]={0};
  static uint8_t encrypt_init_flag = 0;
#endif
  //������Ϣ��ȡSID��CMD
  if ((PCI&0xF0) == 0x00) {
    SID = pRxMessage->Data[2];
    CMD = pRxMessage->Data[3];
  }else if ((PCI&0xF0) == 0x10) {
    SID = pRxMessage->Data[3];
    CMD = pRxMessage->Data[4];
  }
  //׼�����͵�����
  TxMessage.DLC = 8;
  if (MSG_ID_TYPE == CAN_Id_Standard) {
    TxMessage.StdId = MSG_SEND_ID;
    TxMessage.IDE = CAN_Id_Standard;
  } else {
    TxMessage.ExtId = MSG_SEND_ID;
    TxMessage.IDE = CAN_Id_Extended;
  }
  TxMessage.RTR = CAN_RTR_Data;
  TxMessage.Data[0] = GetNAD();//���NAD
  //��������
  switch (CMD) {
    case CMD_GET_FW_INFO:
      OLD_CMD = CMD_GET_FW_INFO;
      TxMessage.Data[1] = 0x06;
      TxMessage.Data[2] = SID + 0x40;
      TxMessage.Data[3] = FW_TYPE;
      TxMessage.Data[4] = 0;//�̼��汾�� Major
      TxMessage.Data[5] = 0;//�̼��汾�� Minor
      TxMessage.Data[6] = 0;//�̼��汾�� Revision
      TxMessage.Data[7] = 1;//�̼��汾�� Build
      CAN_WriteData(&TxMessage);
      break;
    case CMD_ENTER_BOOT:
      OLD_CMD = CMD_ENTER_BOOT;
      //��BOOT�����в���Ҫִ���κβ�������APP����������Ҫ����APP���б�־����ת��BOOTִ�л��������λ����
      if (FW_TYPE == FW_TYPE_APP) {
        FLASH_Unlock();
        CAN_BOOT_ErasePage(APP_EXE_FLAG_ADDR,APP_EXE_FLAG_ADDR);//����д�뵽Flash�е�APPִ�б�־����λ���к󣬼���ִ��Bootloader����
        FLASH_Lock();
        __set_PRIMASK(1);//�ر������ж�
        NVIC_SystemReset();
      }
      break;
    case CMD_ERASE_APP:
      OLD_CMD = CMD_ERASE_APP;
      if (FW_TYPE == FW_TYPE_APP) {//APP�в���ִ�в���APP�Ĳ���
        TxMessage.Data[1] = 0x06;
        TxMessage.Data[2] = SID + 0x40;
        TxMessage.Data[3] = CAN_BOOT_ERR_ERASE_IN_APP;
      } else {
        TxMessage.Data[1] = 0x06;
        TxMessage.Data[2] = SID + 0x40;
        uint32_t EraseSize = (pRxMessage->Data[4]<<24)|(pRxMessage->Data[5]<<16)|(pRxMessage->Data[6]<<8)|(pRxMessage->Data[7]<<0);
        __set_PRIMASK(1);
        FLASH_Unlock();
        ret = CAN_BOOT_ErasePage(APP_EXE_FLAG_ADDR,APP_START_ADDR+EraseSize);
        FLASH_Lock();
        __set_PRIMASK(0);
        if (ret == FLASH_COMPLETE) {
          TxMessage.Data[3] = CAN_BOOT_ERR_SUCCESS;
        } else {
          TxMessage.Data[3] = CAN_BOOT_ERR_ERASE;
        }
      }
      TxMessage.Data[4] = 0xFF;
      TxMessage.Data[5] = 0xFF;
      TxMessage.Data[6] = 0xFF;
      TxMessage.Data[7] = 0xFF;
      CAN_WriteData(&TxMessage);
      break;
    case CMD_SET_ADDR_OFFSET:
      OLD_CMD = CMD_SET_ADDR_OFFSET;
      TxMessage.Data[1] = 0x06;
      TxMessage.Data[2] = SID + 0x40;
      if (FW_TYPE == FW_TYPE_APP) {
        TxMessage.Data[3] = CAN_BOOT_ERR_WRITE_IN_APP;
        TxMessage.Data[4] = 0xFF;
        TxMessage.Data[5] = 0xFF;
        TxMessage.Data[6] = 0xFF;
        TxMessage.Data[7] = 0xFF;
      } else {
        data_size = 0;
        data_index = 0;
        addr_offset = (pRxMessage->Data[4]<<24) | (pRxMessage->Data[5]<<16) | (pRxMessage->Data[6]<<8) | (pRxMessage->Data[7]<<0);
        start_addr = addr_offset+APP_START_ADDR;
        if (start_addr < APP_START_ADDR) {
          TxMessage.Data[3] = CAN_BOOT_ERR_WRITE_OUTRANGE;
        } else {
          uint16_t buffer_len=sizeof(data_temp);
          TxMessage.Data[3] = CAN_BOOT_ERR_SUCCESS;
          TxMessage.Data[4] = buffer_len>>8;
          TxMessage.Data[5] = buffer_len&0xFF;
          TxMessage.Data[6] = 0xFF;TxMessage.Data[7] = 0xFF;
        }
      }
      CAN_WriteData(&TxMessage);
      break;
    case CMD_TRAN_DATA:
      OLD_CMD = CMD_TRAN_DATA;
      if ((OLD_CMD == CMD_TRAN_DATA) || (OLD_CMD == CMD_SET_ADDR_OFFSET)) {
        if((PCI&0xF0) == 0x00){
          data_size = (PCI&0xF)-2;
          pData = &pRxMessage->Data[4];
          while (data_index < data_size) {
            data_temp[data_index++] = *pData++;
          }
        }else if ((PCI&0xF0) == 0x10) {
          CF_Index = 1;
          data_size = (((PCI&0xF)<<8)|pRxMessage->Data[2])-2;
          pData = &pRxMessage->Data[5];
          for (i = 0; i < 3; i++) {
            data_temp[data_index++] = *pData++;
          }
        }
      }
      break;
    case CMD_WRITE_DATA:
      OLD_CMD = CMD_WRITE_DATA;
      TxMessage.Data[1] = 0x06;
      TxMessage.Data[2] = SID + 0x40;
      if (FW_TYPE == FW_TYPE_APP) {
        TxMessage.Data[3] = CAN_BOOT_ERR_WRITE_IN_APP;
        TxMessage.Data[4] = 0xFF;
        TxMessage.Data[5] = 0xFF;
        TxMessage.Data[6] = 0xFF;
        TxMessage.Data[7] = 0xFF;
      }else{
        crc_data = (pRxMessage->Data[4]<<8)|(pRxMessage->Data[5]<<0);
        if ((data_size != data_index) || (crc_data != crc16_ccitt(data_temp, data_index))) {
          TxMessage.Data[3] = CAN_BOOT_ERR_TRAN_CRC;
        }else{
#if ENCRYPT
          if (!encrypt_init_flag) {
            for (i=0; i < sizeof(AES_Key_Table); i++){
              if (pKey[i] != '\0'){
                AES_Key_Table[i]=pKey[i];
              }else{
                break;
              }
            }
            aesDecInit();//��ִ�н��ܳ�ʼ��֮ǰ����ΪAES_Key_Table��ֵ��Ч����������
            encrypt_init_flag = 1;
          }
          for (int i=0; i < (data_size/16); i++){
            aesDecrypt(&data_temp[16*i], chainCipherBlock);//AES���ܣ��������ݴ����dat���棬�����ܾ��ܵõ�֮ǰ�����ġ�
          }
#endif
          __set_PRIMASK(1);
          FLASH_Unlock();
          ret = CAN_BOOT_ProgramDatatoFlash(start_addr,data_temp, data_size);
          FLASH_Lock();
          __set_PRIMASK(0);
          if (ret == FLASH_COMPLETE) {
            crc_data = crc16_ccitt((const unsigned char*)(data_temp), data_size);
            //�ٴζ�д��Flash�е����ݽ���CRCУ�飬ȷ��д��Flash����������
            if (crc_data != crc16_ccitt((const unsigned char*)(start_addr), data_size)) {
              TxMessage.Data[3] = CAN_BOOT_ERR_WRITE_CRC;
            }else{
              TxMessage.Data[3] = CAN_BOOT_ERR_SUCCESS;
            }
          } else {
            TxMessage.Data[3] = CAN_BOOT_ERR_WRITE;
          }
        }
      }
      TxMessage.Data[4] = 0xFF;
      TxMessage.Data[5] = 0xFF;
      TxMessage.Data[6] = 0xFF;
      TxMessage.Data[7] = 0xFF;
      CAN_WriteData(&TxMessage);
      break;
    case CMD_ENTER_APP:
      OLD_CMD = CMD_ENTER_APP;
      if (FW_TYPE == FW_TYPE_BOOT) {
        if ((*((uint32_t *)APP_START_ADDR) != 0xFFFFFFFF)) {
          CAN_BOOT_JumpToApplication(APP_START_ADDR);
        }
      }
      break;
    case 0xFF:
      if (OLD_CMD == CMD_TRAN_DATA) {
        if (((PCI&0xF0) == 0x20) && ((CF_Index&0xF) == (PCI&0xF))) {
          CF_Index++;
          pData = &pRxMessage->Data[2];
          if ((data_index+6) <= data_size) {
            for (i = 0; i < 6; i++) {
              data_temp[data_index++] = *pData++;
            }
          } else {
            while (data_index < data_size) {
              data_temp[data_index++] = *pData++;
            }
          }
        }
      }
      break;
    default:
      break;
  }
}
/*********************************END OF FILE**********************************/
