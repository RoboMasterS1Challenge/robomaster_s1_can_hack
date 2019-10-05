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
#include "robomaster_s1.h"

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

volatile CANRxMsg rx_msg_buffer1[BUFFER_SIZE];
volatile int buffer_rp1;
volatile int buffer_wp1;
volatile CANRxMsg rx_msg_buffer2[BUFFER_SIZE];
volatile int buffer_rp2;
volatile int buffer_wp2;

volatile uint8_t can_command_buffer[BUFFER_SIZE];
volatile int can_command_buffer_rp;
volatile int can_command_buffer_wp;

extern twist command_twist;
extern int command_blaster;

uint8_t time_stamp[2];
uint16_t blaster_counter[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_FilterConfig(void);
extern void UdpSendData(uint8_t id, uint8_t *send_data, uint8_t send_data_size);
extern void StartUdp(uint8_t *ip_addr);
extern int parseCanData(uint8_t id, uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size);
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

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_FULL) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  buffer_rp1 = 0;
  buffer_wp1 = 0;
  buffer_rp2 = 0;
  buffer_wp2 = 0;
  can_command_buffer_rp = 0;
  can_command_buffer_wp = 0;

  uint8_t send_ip_addr[] = {192, 168, 0, 255};
  StartUdp(send_ip_addr);

  CANRxMsg msg;
  uint8_t send_data[255];
  uint8_t send_data_size = 0;
  int i = 0;
  int ret = 0;
  uint64_t test_counter = 0;
  int blaster_cycle = 0;
  int blaster_timeout =0;
  blaster_counter[0] = 0;
  blaster_counter[1] = 0;
  while (1)
  {

    // 0x201 Intelligent Controller Command
    if (buffer_rp1 != buffer_wp1)
    {
      msg = rx_msg_buffer1[buffer_rp1];
      ret = parseCanData(msg.can_id, msg.data, msg.dlc, send_data, &send_data_size);
      buffer_rp1++;
      buffer_rp1 %= BUFFER_SIZE;
      if (ret)
      {
        if (command_blaster)
        {
          // kuratta oto 0x55,0x0F,0x04,0xA2,0x09,0x17,0x7C,0x09,0x00,0x3F,0x58,0xB0,0x01,0x10,0xE4

            if (blaster_cycle == 0)
            {
                uint8_t blaster_data[26];
                blaster_data[0] = 0x55;
                blaster_data[1] = 0x1A;
                blaster_data[2] = 0x04;
                blaster_data[3] = 0x00;
                appendCRC8CheckSum(blaster_data, 4);
                blaster_data[4] = 0x09;
                blaster_data[5] = 0x18;
                blaster_data[6] = blaster_counter[0] & 0xFF;
                blaster_data[7] = blaster_counter[0] >> 8;
                blaster_data[8] = 0x00;
                blaster_data[9] = 0x3F;
                blaster_data[10] = 0x32;
                blaster_data[11] = 0x01;
                blaster_data[12] = 0xFF;
                blaster_data[13] = 0x00;
                blaster_data[14] = 0x00;
                blaster_data[15] = 0x7F;
                blaster_data[16] = 0x46;
                blaster_data[17] = 0x00;
                blaster_data[18] = 0xC8;
                blaster_data[19] = 0x00;
                blaster_data[20] = 0xC8;
                blaster_data[21] = 0x00;
                blaster_data[22] = 0x0F;
                blaster_data[23] = 0x00;
                blaster_data[24] = 0x00;
                blaster_data[25] = 0x00;
                appendCRC16CheckSum(blaster_data, 26);
                blaster_counter[0]++;
                for (i = 0; i < 26; i++)
                {
                  can_command_buffer[can_command_buffer_wp] = blaster_data[i];
                  can_command_buffer_wp++;
                  can_command_buffer_wp %= BUFFER_SIZE;
                }
                UdpSendData(msg.can_id, blaster_data, 26);
                blaster_cycle++;
            }
            else if (blaster_cycle == 1)
            {
            uint8_t blaster_data[14];
            blaster_data[0] = 0x55;
            blaster_data[1] = 0x0E;
            blaster_data[2] = 0x04;
            blaster_data[3] = 0x00;
            appendCRC8CheckSum(blaster_data, 4);
            blaster_data[4] = 0x09;
            blaster_data[5] = 0x17;
            blaster_data[6] = blaster_counter[1] & 0xFF;
            blaster_data[7] = blaster_counter[1] >> 8;
            blaster_data[8] = 0x00;
            blaster_data[9] = 0x3F;
            blaster_data[10] = 0x51;
            blaster_data[11] = 0x11;
            blaster_data[12] = 0x00;
            blaster_data[13] = 0x00;
            appendCRC16CheckSum(blaster_data, 14);
            blaster_counter[1]++;
            for (i = 0; i < 14; i++)
            {
              can_command_buffer[can_command_buffer_wp] = blaster_data[i];
              can_command_buffer_wp++;
              can_command_buffer_wp %= BUFFER_SIZE;
            }
            UdpSendData(msg.can_id, blaster_data, 14);
            blaster_cycle++;
          }
          else if (blaster_cycle == 2)
          {
            uint8_t blaster_data[22];
            blaster_data[0] = 0x55;
            blaster_data[1] = 0x16;
            blaster_data[2] = 0x04;
            blaster_data[3] = 0x00;
            appendCRC8CheckSum(blaster_data, 4);
            blaster_data[4] = 0x09;
            blaster_data[5] = 0x17;
            blaster_data[6] = blaster_counter[1] & 0xFF;
            blaster_data[7] = blaster_counter[1] >> 8;
            blaster_data[8] = 0x00;
            blaster_data[9] = 0x3F;
            blaster_data[10] = 0x55;
            blaster_data[11] = 0x73;
            blaster_data[12] = 0x00;
            blaster_data[13] = 0xFF;
            blaster_data[14] = 0x00;
            blaster_data[15] = 0x01;
            blaster_data[16] = 0x28;
            blaster_data[17] = 0x00;
            blaster_data[18] = 0x00;
            blaster_data[19] = 0x00;
            blaster_data[20] = 0x00;
            blaster_data[21] = 0x00;
            appendCRC16CheckSum(blaster_data, 22);
            blaster_counter[1]++;
            for (i = 0; i < 22; i++)
            {
              can_command_buffer[can_command_buffer_wp] = blaster_data[i];
              can_command_buffer_wp++;
              can_command_buffer_wp %= BUFFER_SIZE;
            }
            UdpSendData(msg.can_id, blaster_data, 22);
            blaster_cycle++;
          }
          else if (blaster_cycle == 3)
          {
              uint8_t blaster_data[26];
              blaster_data[0] = 0x55;
              blaster_data[1] = 0x1A;
              blaster_data[2] = 0x04;
              blaster_data[3] = 0x00;
              appendCRC8CheckSum(blaster_data, 4);
              blaster_data[4] = 0x09;
              blaster_data[5] = 0x18;
              blaster_data[6] = blaster_counter[0] & 0xFF;
              blaster_data[7] = blaster_counter[0] >> 8;
              blaster_data[8] = 0x00;
              blaster_data[9] = 0x3F;
              blaster_data[10] = 0x32;
              blaster_data[11] = 0x05;
              blaster_data[12] = 0xFF;
              blaster_data[13] = 0x00;
              blaster_data[14] = 0x00;
              blaster_data[15] = 0x7F;
              blaster_data[16] = 0x46;
              blaster_data[17] = 0x00;
              blaster_data[18] = 0x64;
              blaster_data[19] = 0x00;
              blaster_data[20] = 0x64;
              blaster_data[21] = 0x00;
              blaster_data[22] = 0x30;
              blaster_data[23] = 0x00;
              blaster_data[24] = 0x00;
              blaster_data[25] = 0x00;
              appendCRC16CheckSum(blaster_data, 26);
              blaster_counter[0]++;
              for (i = 0; i < 26; i++)
              {
                can_command_buffer[can_command_buffer_wp] = blaster_data[i];
                can_command_buffer_wp++;
                can_command_buffer_wp %= BUFFER_SIZE;
              }
              UdpSendData(msg.can_id, blaster_data, 26);
              blaster_cycle = 0;
              command_blaster = 0;
          }

      }

      //            	if( send_data[4] == 0x09 && send_data[5] == 0x17){ // Block Blaster
      //            		send_data[1]  = 0;
      //            		//            		blaster_data[0] = 0x55;
      //            		//            		blaster_data[1] = 0x0E;
      //            		//            		blaster_data[2] = 0x04;
      //            		//            		blaster_data[3] = 0x00;
      //            		//            		appendCRC8CheckSum(blaster_data,4);
      //            		//            		blaster_data[4] = 0x09;
      //            		//            		blaster_data[5] = 0x17;
      //
      //            	}

      //            	if(send_data[4] == 0x09 && send_data[5] == 0x18){ // Block LED Command
      //            		send_data[1]  = 0;
      //
      //            	}
      //            	if(send_data[1] == 0x0D){
      //            		send_data[1]  = 0;
      //            	}

      // 脱力指示
      //            	if(send_data[1] == 0x0F){
      //            		send_data[1]  = 0;
      //            	}
      if (send_data[1] == 0x1B)
      {
        // Get time stamp
        time_stamp[0] = send_data[6];
        time_stamp[1] = send_data[7];

        // Linear X and Y
        uint16_t linear_x = 256 * command_twist.linear.x + 1024;
        uint16_t linear_y = 256 * command_twist.linear.y + 1024;
        int16_t angular_z = 256 * command_twist.angular.z + 1024;

        send_data[13] &= 0xC0;
        send_data[13] |= (linear_x >> 5) & 0x3F;
        send_data[12] = 0x00;
        send_data[12] |= linear_x << 3;
        send_data[12] |= (linear_y >> 8) & 0x07;
        send_data[11] = 0x00;
        send_data[11] |= linear_y & 0xFF;

        send_data[17] = (angular_z >> 4) & 0xFF; //0x40;
        send_data[16] = (angular_z << 4) | 0x08; //0x08;

        send_data[18] = 0x00;

        send_data[19] = 0x02 | ((angular_z << 2) & 0xFF);
        send_data[20] = (angular_z >> 6) & 0xFF; // 0x10;

        send_data[21] = 0x04;
        send_data[22] = 0x04; // Enable Flag 4:x-y 8:yaw
        send_data[23] = 0x00;
        send_data[24] = 0x04;

        send_data[send_data_size - 2] = 0;
        send_data[send_data_size - 1] = 0;
        appendCRC16CheckSum(send_data, send_data_size);
        test_counter++;
      }
      if (send_data[1] == 0x14 && send_data[5] == 0x04)
      {
        // Linear X and Y
        int16_t angular_y = -1024 * command_twist.angular.y;
        int16_t angular_z = -1024 * command_twist.angular.z;

        send_data[14] = (angular_y >> 8) & 0xFF;
        send_data[13] = angular_y & 0xFF;
        send_data[16] = (angular_z >> 8) & 0xFF;
        send_data[15] = angular_z & 0xFF;

        send_data[send_data_size - 2] = 0;
        send_data[send_data_size - 1] = 0;
        appendCRC16CheckSum(send_data, send_data_size);
      }

      // Smartphone touch event override
      if (send_data[0] == 0x55 &&
          send_data[1] == 0x0F &&
          send_data[2] == 0x04 &&
          send_data[3] == 0xA2 &&
          send_data[4] == 0x09 &&
          send_data[11] == 0x02)
      {
        send_data[11] = 0;
        // Recalcuration CRC16
        send_data[send_data_size - 2] = 0;
        send_data[send_data_size - 1] = 0;
        appendCRC16CheckSum(send_data, send_data_size);
      }
      UdpSendData(msg.can_id, send_data, send_data_size);
      for (i = 0; i < send_data_size; i++)
      {
        can_command_buffer[can_command_buffer_wp] = send_data[i];
        can_command_buffer_wp++;
        can_command_buffer_wp %= BUFFER_SIZE;
      }
    }
  }

  // others
  if (buffer_rp2 != buffer_wp2)
  {
    msg = rx_msg_buffer2[buffer_rp2];
    ret = parseCanData(msg.can_id, msg.data, msg.dlc, send_data, &send_data_size);
    buffer_rp2++;
    buffer_rp2 %= BUFFER_SIZE;
    if (ret)
    {
      UdpSendData(msg.can_id, send_data, send_data_size);
    }
  }

  // Transmit CAN Command
  if (can_command_buffer_rp != can_command_buffer_wp)
  {
    int can_command_buffer_size = (can_command_buffer_wp - can_command_buffer_rp + BUFFER_SIZE) % BUFFER_SIZE;
    if (can_command_buffer_size >= 8)
    {
      TxHeader2.DLC = 8;
    }
    else
    {
      TxHeader2.DLC = can_command_buffer_size;
    }
    TxHeader2.StdId = RxHeader1.StdId;
    TxHeader2.ExtId = RxHeader1.ExtId;
    TxHeader2.RTR = CAN_RTR_DATA;
    TxHeader2.IDE = CAN_ID_STD;
    TxHeader2.TransmitGlobalTime = DISABLE;
    for (i = 0; i < TxHeader2.DLC; i++)
    {
      TxData2[i] = can_command_buffer[(can_command_buffer_rp + i) % BUFFER_SIZE];
    }
    /* Start Transmission process */
    int ret = HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
    if (ret == HAL_OK)
    {
      can_command_buffer_rp += TxHeader2.DLC;
      can_command_buffer_rp %= BUFFER_SIZE;
    }
  }

  MX_LWIP_Process();
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin | STLK_TX_Pin;
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
  GPIO_InitStruct.Pin = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
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
      //            TxHeader2.StdId = RxHeader1.StdId;
      //            TxHeader2.ExtId = RxHeader1.ExtId;
      //            TxHeader2.RTR = CAN_RTR_DATA;
      //            TxHeader2.IDE = CAN_ID_STD;
      //            TxHeader2.DLC = RxHeader1.DLC;
      //            TxHeader2.TransmitGlobalTime = DISABLE;
      //            memcpy(TxData2, RxData1, RxHeader1.DLC);

      /* Start Transmission process */
      //int ret = HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
      //            while (ret != HAL_OK)
      //            {
      //            	ret = HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
      //            }
      CANRxMsg msg;
      msg.can_id = ID_0x201;
      msg.dlc = RxHeader1.DLC;
      memcpy(msg.data, RxData1, RxHeader1.DLC);
      rx_msg_buffer1[buffer_wp1] = msg;
      buffer_wp1++;
      buffer_wp1 %= BUFFER_SIZE;
    }
  }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, RxData1) == HAL_OK)
  {
    if (RxHeader1.StdId == 0x201)
    {
      //            TxHeader2.StdId = RxHeader1.StdId;
      //            TxHeader2.ExtId = RxHeader1.ExtId;
      //            TxHeader2.RTR = CAN_RTR_DATA;
      //            TxHeader2.IDE = CAN_ID_STD;
      //            TxHeader2.DLC = RxHeader1.DLC;
      //            TxHeader2.TransmitGlobalTime = DISABLE;
      //            memcpy(TxData2, RxData1, RxHeader1.DLC);

      /* Start Transmission process */
      //int ret = HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
      //            while (ret != HAL_OK)
      //            {
      //            	ret = HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
      //            }
      CANRxMsg msg;
      msg.can_id = ID_0x201;
      msg.dlc = RxHeader1.DLC;
      memcpy(msg.data, RxData1, RxHeader1.DLC);
      rx_msg_buffer1[buffer_wp1] = msg;
      buffer_wp1++;
      buffer_wp1 %= BUFFER_SIZE;
    }
  }
}

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

      /* Start Transmission process */
      int ret = HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1);
      //                        while (ret != HAL_OK)
      //                        {
      //                        	ret = HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1);
      //                        }

      CANRxMsg msg;
      switch (RxHeader2.StdId)
      {
      case 0x202:
        msg.can_id = ID_0x202;
        break;
      case 0x203:
        msg.can_id = ID_0x203;
        break;
      case 0x204:
        msg.can_id = ID_0x204;
        break;
      case 0x211:
        msg.can_id = ID_0x211;
        break;
      case 0x212:
        msg.can_id = ID_0x212;
        break;
      case 0x213:
        msg.can_id = ID_0x213;
        break;
      case 0x214:
        msg.can_id = ID_0x214;
        break;
      case 0x215:
        msg.can_id = ID_0x215;
        break;
      case 0x216:
        msg.can_id = ID_0x216;
        break;
      }
      msg.dlc = RxHeader2.DLC;
      memcpy(msg.data, RxData2, RxHeader2.DLC);
      rx_msg_buffer2[buffer_wp2] = msg;
      buffer_wp2++;
      buffer_wp2 %= BUFFER_SIZE;
    }
  }
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
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

      /* Start Transmission process */
      int ret = HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1);
      //                        while (ret != HAL_OK)
      //                        {
      //                        	ret = HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1);
      //                        }

      CANRxMsg msg;
      switch (RxHeader2.StdId)
      {
      case 0x202:
        msg.can_id = ID_0x202;
        break;
      case 0x203:
        msg.can_id = ID_0x203;
        break;
      case 0x204:
        msg.can_id = ID_0x204;
        break;
      case 0x211:
        msg.can_id = ID_0x211;
        break;
      case 0x212:
        msg.can_id = ID_0x212;
        break;
      case 0x213:
        msg.can_id = ID_0x213;
        break;
      case 0x214:
        msg.can_id = ID_0x214;
        break;
      case 0x215:
        msg.can_id = ID_0x215;
        break;
      case 0x216:
        msg.can_id = ID_0x216;
        break;
      }
      msg.dlc = RxHeader2.DLC;
      memcpy(msg.data, RxData2, RxHeader2.DLC);
      rx_msg_buffer2[buffer_wp2] = msg;
      buffer_wp2++;
      buffer_wp2 %= BUFFER_SIZE;
    }
  }
}

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
  if (htim->Instance == TIM10)
  {
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

#ifdef USE_FULL_ASSERT
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
