/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    11-April-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can_bootloader.h"

/** @addtogroup Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
extern volatile uint8_t CAN_MsgIndex;
extern CanRxMsg CAN1_RxMessage[2];
extern volatile uint8_t CAN1_CanRxMsgFlag;//接收到CAN数据后的标志
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_md.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */

  /* Add your application code here */
  STM_USART_Config();
  printf("EMMC bootloader v1.0.0\n");
  uint32_t cntrs = 1000000;
  while (cntrs--);

  if (*((uint32_t *)APP_EXE_FLAG_ADDR) == 0x78563412)
  {
    CAN_BOOT_JumpToApplication(APP_START_ADDR);
  }
  __set_PRIMASK(0);

  STM_CAN_Config();
  printf("NAD: 0x%02x\n", GetNAD());

//   /* Enable the Flash option control register access */
//   FLASH_OB_Unlock();
//   /* Enable FLASH_WRP_SECTORS write protection */
//   FLASH_OB_WRPConfig(OB_WRP_Sector_0, ENABLE);
//   FLASH_OB_Launch();
//   FLASH_OB_Lock();

//   if (FLASH_OB_GetRDP() != SET)
//   {
//     FLASH_OB_Unlock();
//     FLASH_OB_RDPConfig(OB_RDP_Level_1);
//     FLASH_OB_Launch();
//     FLASH_OB_Lock();
//   }

  /* Infinite loop */
  while (1)
  {
    if (CAN1_CanRxMsgFlag)
    {
      CAN_BOOT_ExecutiveCommand(&CAN1_RxMessage[CAN_MsgIndex&0x01]);
      CAN_MsgIndex++;
      CAN1_CanRxMsgFlag = 0;
    }
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
