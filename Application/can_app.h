/**
  ******************************************************************************
  * @file    can_bootloader.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_APP_H
#define __CAN_APP_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define APP_START_ADDR             ((uint32_t)0x08008000)//APP������ʼ��ַ
#define APP_EXE_FLAG_ADDR          ((uint32_t)0x08004000)//APP�������б�־�洢��ַ���ñ�־��APP������д��Ͳ���
//�̼�����ֵ����
#define FW_TYPE_BOOT     0x55
#define FW_TYPE_APP      0xAA
//���嵱ǰ�̼�����ΪBOOT
//#define FW_TYPE         FW_TYPE_APP
//���������շ���֡ID���������λ���������һ�£������޷�����������
//����CAN���ߣ������շ�ID���Զ���Ϊһ��ID��Ҳ���Զ���Ϊ��ͬ��ID
#define	MSG_RECEIVE_ID	0x3C
#define	MSG_SEND_ID			0x3D
//���������շ�֡ID����,0-��׼֡��1-��չ֡
#define	MSG_ID_TYPE			0
//����ڵ�㲥��ַ
#define NAD_BROADCAST		0x7F
//BOOT�����
#define	CMD_GET_FW_INFO			0x80
#define	CMD_ENTER_BOOT			0xC1
#define	CMD_ERASE_APP				0x42
#define	CMD_SET_ADDR_OFFSET	0x03
#define	CMD_TRAN_DATA			  0xC4
#define	CMD_WRITE_DATA			0x85
#define	CMD_ENTER_APP				0x06
//BOOT������
#define CAN_BOOT_ERR_SUCCESS				0		//û�д���
#define CAN_BOOT_ERR_ERASE          1   //�̼���������
#define CAN_BOOT_ERR_ERASE_IN_APP   2   //��ǰģʽΪAPP�����ܲ����̼�
#define CAN_BOOT_ERR_WRITE_OUTRANGE 3   //��ǰ��ַ�����������ĵ�ַ��Χ
#define CAN_BOOT_ERR_WRITE_IN_APP   4   //��ǰģʽ����д��̼�����
#define CAN_BOOT_ERR_WRITE          5   //����д�����洢������
#define CAN_BOOT_ERR_WRITE_OUT_ADDR 6   //���ݳ��ȳ����˳���洢����Χ 
#define CAN_BOOT_ERR_TRAN_CRC       7   //���ݴ���CRCУ�����
#define CAN_BOOT_ERR_WRITE_CRC      8   //����д��оƬCRCУ�����
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint32_t GetSector(uint32_t Address);
FLASH_Status CAN_BOOT_ErasePage(uint32_t StartPageAddr,uint32_t EndPageAddr);
uint16_t CAN_BOOT_GetAddrData(void);
void CAN_BOOT_ExecutiveCommand(CanRxMsg *pRxMessage);
void CAN_BOOT_JumpToApplication(__IO uint32_t Addr);
FLASH_Status CAN_BOOT_ProgramDatatoFlash(uint32_t StartAddress,uint8_t *pData,uint32_t DataNum);
uint8_t GetNAD(void);
#endif
/*********************************END OF FILE**********************************/

