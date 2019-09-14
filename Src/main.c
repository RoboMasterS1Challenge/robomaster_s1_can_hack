/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader1;
CAN_RxHeaderTypeDef RxHeader1;
uint8_t TxData1[8];
uint8_t RxData1[8];
uint32_t TxMailbox1;
uint32_t RxMailbox1;
CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData2[8];
uint8_t RxData2[8];
uint32_t TxMailbox2;
uint32_t RxMailbox2;

typedef struct CANRxMsg
{
  uint16_t can_id;
  uint8_t dlc;
  uint8_t data[8];
} CANRxMsg;

#define BUFFER_SIZE 1024
volatile CANRxMsg RxMsgBuffer1[BUFFER_SIZE];
volatile int buffer_rp1;
volatile int buffer_wp1;
volatile CANRxMsg RxMsgBuffer2[BUFFER_SIZE];
volatile int buffer_rp2;
volatile int buffer_wp2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_FilterConfig(void);
extern
void UdpSendData(uint8_t id, uint8_t in_data);
extern
void StartUdp( vBasicTFTPServer, pvParameters );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LWIP_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  /* Configure the CAN peripheral */
  CAN_FilterConfig();

  /*## Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*## Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

//  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL) != HAL_OK)
//  {
//    /* Notification Error */
//    Error_Handler();
//  }
//
//  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_FULL) != HAL_OK)
//  {
//    /* Notification Error */
//    Error_Handler();
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  buffer_rp1 = 0;
  buffer_wp1 = 0;
  buffer_rp2 = 0;
  buffer_wp2 = 0;

  StartUdp();
  CANRxMsg msg;
  int i = 0;
  while (1)
  {
	  if(buffer_rp1 != buffer_wp1){
		  msg = RxMsgBuffer1[buffer_rp1];
		  switch(msg.can_id){
			case 0x201:
				  for(i=0;i<msg.dlc;i++){
					  UdpSendData(0, msg.data[i]);
				  }
				  break;
		  }
		  buffer_rp1++;
		  buffer_rp1 %= BUFFER_SIZE;
	  }

	  if(buffer_rp2 != buffer_wp2){
		  msg = RxMsgBuffer2[buffer_rp2];
		  switch(msg.can_id){
			case 0x202:
			  for(i=0;i<msg.dlc;i++){
				  UdpSendData(1, msg.data[i]);
			  }
			  break;
	  	  }
		  buffer_rp2++;
		  buffer_rp2 %= BUFFER_SIZE;
	  }


    //HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
static void CAN_FilterConfig(void)
{
  CAN_FilterTypeDef sFilterConfig1;
  CAN_FilterTypeDef sFilterConfig2;

  /*## Configure the CAN Filter ###########################################*/
  sFilterConfig1.FilterBank = 0; //ID 0-13 for CAN1, 14+ is CAN2
  sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig1.FilterIdHigh = 0x0000;
  sFilterConfig1.FilterIdLow = 0x0000;
  sFilterConfig1.FilterMaskIdHigh = 0x0000;
  sFilterConfig1.FilterMaskIdLow = 0x0000;
  sFilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig1.FilterActivation = ENABLE;
  sFilterConfig1.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  sFilterConfig2.FilterBank = 14; //ID 0-13 for CAN1, 14+ is CAN2
  sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO1;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
}

/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, RxData1) == HAL_OK)
  {
    if (RxHeader1.StdId == 0x201)
    {
      TxHeader2.StdId = RxHeader1.StdId;
      TxHeader2.ExtId = RxHeader1.ExtId;
      TxHeader2.RTR = CAN_RTR_DATA;
      TxHeader2.IDE = CAN_ID_STD;
      TxHeader2.DLC = RxHeader1.DLC;
      TxHeader2.TransmitGlobalTime = DISABLE;
      memcpy(TxData2, RxData1, RxHeader1.DLC);

      /* Start the Transmission process */
      while ( HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2) != HAL_OK)
      {
    	  // wait
      }
      if (RxHeader1.StdId == 0x201){
		  CANRxMsg msg;
		  msg.can_id = RxHeader1.StdId;
		  msg.dlc = RxHeader1.DLC;
		  memcpy(msg.data, RxData1, RxHeader1.DLC);
		  RxMsgBuffer1[buffer_wp1] = msg;
		  buffer_wp1++;
		  buffer_wp1 %= BUFFER_SIZE;
      }
    }
  }
}
//void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
//{
//  /* Get RX message */
//  while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, RxData1) == HAL_OK)
//  {
//    if (RxHeader1.StdId == 0x201)
//    {
//      TxHeader2.StdId = RxHeader1.StdId;
//      TxHeader2.ExtId = RxHeader1.ExtId;
//      TxHeader2.RTR = CAN_RTR_DATA;
//      TxHeader2.IDE = CAN_ID_STD;
//      TxHeader2.DLC = RxHeader1.DLC;
//      TxHeader2.TransmitGlobalTime = DISABLE;
//      memcpy(TxData2, RxData1, RxHeader1.DLC);
//
//      /* Start the Transmission process */
//      while ( HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2) != HAL_OK)
//      {
//    	  // wait
//      }
//      if (RxHeader1.StdId == 0x201){
//		  CANRxMsg msg;
//		  msg.can_id = RxHeader1.StdId;
//		  msg.dlc = RxHeader1.DLC;
//		  memcpy(msg.data, RxData1, RxHeader1.DLC);
//		  RxMsgBuffer1[buffer_wp1] = msg;
//		  buffer_wp1++;
//		  buffer_wp1 %= BUFFER_SIZE;
//      }
//    }
//  }
//}

/**
  * @brief  Rx Fifo 1 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader2, RxData2) == HAL_OK)
  {
    if (RxHeader2.StdId != 0x201)
    {
      TxHeader1.StdId = RxHeader2.StdId;
      TxHeader1.ExtId = RxHeader2.ExtId;
      TxHeader1.RTR = CAN_RTR_DATA;
      TxHeader1.IDE = CAN_ID_STD;
      TxHeader1.DLC = RxHeader2.DLC;
      TxHeader1.TransmitGlobalTime = DISABLE;
      memcpy(TxData1, RxData2, RxHeader2.DLC);

      /* Start the Transmission process */
      while ( HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1) != HAL_OK)
      {
    	  // wait
      }
      if (RxHeader2.StdId == 0x202){
		  CANRxMsg msg;
		  msg.can_id = RxHeader2.StdId;
		  msg.dlc = RxHeader2.DLC;
		  memcpy(msg.data, RxData2, RxHeader2.DLC);
		  RxMsgBuffer2[buffer_wp2] = msg;
		  buffer_wp2++;
		  buffer_wp2 %= BUFFER_SIZE;
      }
    }
  }
}
//void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
//{
//  /* Get RX message */
//  while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader2, RxData2) == HAL_OK)
//  {
//    if (RxHeader2.StdId != 0x201)
//    {
//      TxHeader1.StdId = RxHeader2.StdId;
//      TxHeader1.ExtId = RxHeader2.ExtId;
//      TxHeader1.RTR = CAN_RTR_DATA;
//      TxHeader1.IDE = CAN_ID_STD;
//      TxHeader1.DLC = RxHeader2.DLC;
//      TxHeader1.TransmitGlobalTime = DISABLE;
//      memcpy(TxData1, RxData2, RxHeader2.DLC);
//
//      /* Start the Transmission process */
//      while ( HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1) != HAL_OK)
//      {
//    	  // wait
//      }
//     ;
//    }
//  }
//}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
