/**
  ******************************************************************************
  * @file    can_bootloader.h
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   基于CAN总线的Bootloader程序.
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
#define APP_START_ADDR             ((uint32_t)0x08008000)//APP程序起始地址
#define APP_EXE_FLAG_ADDR          ((uint32_t)0x08004000)//APP程序运行标志存储地址，该标志在APP程序中写入和擦除
//固件类型值定义
#define FW_TYPE_BOOT     0x55
#define FW_TYPE_APP      0xAA
//定义当前固件类型为BOOT
//#define FW_TYPE         FW_TYPE_APP
//定义数据收发的帧ID，必须跟上位机软件配置一致，否则无法正常工作。
//对于CAN总线，数据收发ID可以定义为一个ID，也可以定义为不同的ID
#define	MSG_RECEIVE_ID	0x3C
#define	MSG_SEND_ID			0x3D
//定义数据收发帧ID类型,0-标准帧，1-扩展帧
#define	MSG_ID_TYPE			0
//定义节点广播地址
#define NAD_BROADCAST		0x7F
//BOOT命令定义
#define	CMD_GET_FW_INFO			0x80
#define	CMD_ENTER_BOOT			0xC1
#define	CMD_ERASE_APP				0x42
#define	CMD_SET_ADDR_OFFSET	0x03
#define	CMD_TRAN_DATA			  0xC4
#define	CMD_WRITE_DATA			0x85
#define	CMD_ENTER_APP				0x06
//BOOT错误定义
#define CAN_BOOT_ERR_SUCCESS				0		//没有错误
#define CAN_BOOT_ERR_ERASE          1   //固件擦除出错
#define CAN_BOOT_ERR_ERASE_IN_APP   2   //当前模式为APP，不能擦除固件
#define CAN_BOOT_ERR_WRITE_OUTRANGE 3   //当前地址超出了正常的地址范围
#define CAN_BOOT_ERR_WRITE_IN_APP   4   //当前模式不能写入固件数据
#define CAN_BOOT_ERR_WRITE          5   //数据写入程序存储器出错
#define CAN_BOOT_ERR_WRITE_OUT_ADDR 6   //数据长度超出了程序存储器范围 
#define CAN_BOOT_ERR_TRAN_CRC       7   //数据传输CRC校验出错
#define CAN_BOOT_ERR_WRITE_CRC      8   //数据写入芯片CRC校验出错
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

