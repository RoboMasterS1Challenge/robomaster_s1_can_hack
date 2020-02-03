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
  * the "License"38400; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "robomaster_s1.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_LIST_SIZE 38
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef union float_uint8 {
  float float_data;
  uint8_t uint8_data[4];
} float_uint8;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
const uint8_t can_command_list[COMMAND_LIST_SIZE][0x4B] = {
#include "command_list.csv"
};
volatile uint16_t command_counter[COMMAND_LIST_SIZE];
volatile uint16_t counter_led;
volatile uint16_t counter_blaster;
volatile uint16_t counter_lose;
volatile uint16_t counter_mode;

volatile CANRxMsg rx_msg_buffer[BUFFER_SIZE];
volatile int buffer_rp;
volatile int buffer_wp;

volatile uint8_t can_command_buffer[BUFFER_SIZE];
volatile int can_command_buffer_rp;
volatile int can_command_buffer_wp;

extern twist command_twist;
extern int command_blaster;
extern int command_lose;
extern led command_led;

extern uint8_t usb_rBuf[];
extern int usb_rBuf_wp;
extern int usb_rBuf_rp;

// Timer
volatile uint8_t timer10sec_flag;
volatile uint32_t timer10sec_counter;
volatile uint8_t timer1sec_flag;
volatile uint32_t timer1sec_counter;
volatile uint8_t timer10msec_flag;
volatile uint32_t timer10msec_counter;
volatile uint8_t timer100msec_flag;
volatile uint32_t timer100msec_counter;

volatile int initial_task_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_FilterConfig(void);
extern void sendUdpData(uint8_t id, uint8_t *send_data, uint8_t send_data_size);
extern void startUdp(uint8_t *ip_addr);
extern int parseCanData(uint8_t id, uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size);
extern int parseUsbData(uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size);

extern void appendCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength);
extern void appendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_can_command(uint8_t command_no)
{
  uint8_t command_length = can_command_list[command_no][3];
  uint8_t header_command[0xFF];
  uint8_t idx = 0;
  for (int i = 2; i < command_length; i++)
  {
    header_command[idx] = can_command_list[command_no][i];

    if (i == 5 && can_command_list[command_no][5] == 0xFF)
    {
      appendCRC8CheckSum(header_command, 4);
    }
    if (i == 8 && can_command_list[command_no][8] == 0xFF)
    {
      header_command[idx] = command_counter[command_no] & 0xFF;
    }
    if (i == 9 && can_command_list[command_no][9] == 0xFF)
    {
      header_command[idx] = (command_counter[command_no] >> 8) & 0xFF;
    }
    command_counter[command_no]++;

    idx++;
  }
  appendCRC16CheckSum(header_command, command_length);
  for (int i = 0; i < command_length; i++)
  {
    can_command_buffer[can_command_buffer_wp] = header_command[i];
    can_command_buffer_wp++;
    can_command_buffer_wp %= BUFFER_SIZE;
  }
}
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Configure the CAN peripheral */
  CAN_FilterConfig();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t received_data[255];
  uint8_t received_data_size = 0;
  int i = 0;
  int ret = 0;
  uint8_t send_ip_addr[] = {192, 168, 0, 255};

  double base_odom_yaw;
  double gimbal_base_yaw_angle;
  double gimbal_map_yaw_angle;
  double gimbal_base_pitch_angle;
  double gimbal_map_pitch_angle;

  buffer_rp = 0;
  buffer_wp = 0;
  can_command_buffer_rp = 0;
  can_command_buffer_wp = 0;

  usb_rBuf_wp = 0;
  usb_rBuf_rp = 0;

  counter_blaster = 0;
  counter_led = 0;

  timer10sec_flag = 0;
  timer10sec_counter = 0;
  timer1sec_flag = 0;
  timer1sec_counter = 0;
  timer100msec_flag = 0;
  timer100msec_counter = 0;
  timer10msec_flag = 0;
  timer10msec_counter = 0;

  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
    // Initial Command 10sec after boot
    if (initial_task_flag == 0)
    {
      if (timer10sec_flag == 1)
      {
        initial_task_flag = 1;

        //boot command
        for (int command_no = 26; command_no < 35; command_no++)
        {
          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);
          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= BUFFER_SIZE;
          }
        }

        // LED on
        {
          uint8_t command_no = 11;
          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = counter_led & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (counter_led >> 8) & 0xFF;
            }
            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);

          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= BUFFER_SIZE;
          }

          counter_led++;
        }

        // FAST MODE
        {
          uint8_t command_no = 22;
          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = counter_mode & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (counter_mode >> 8) & 0xFF;
            }
            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);

          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= BUFFER_SIZE;
          }

          counter_mode++;
        }

        // Enable Odometry Output
        {
          uint8_t command_no = 35;
          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (i = 2; i < command_length - 2; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);

          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= BUFFER_SIZE;
          }
        }
      }
    }

    // After Initialize Command
    if (initial_task_flag)
    {

      // 10msec TASK
      if (timer10msec_flag)
      {
        timer10msec_flag = 0;

        if (command_lose == 1) // Lose
        {
          command_lose = 0;
          // Lose command
          int command_no = 36;

          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = counter_lose & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (counter_lose >> 8) & 0xFF;
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);

          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= BUFFER_SIZE;
          }

          counter_lose++;
        }

        if (command_lose == 2) // Recoover
        {
          command_lose = 0;
          // Recover command
          int command_no = 37;

          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = counter_lose & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (counter_lose >> 8) & 0xFF;
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);

          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= BUFFER_SIZE;
          }

          counter_lose++;
        }

        // LED Command
        if (command_led.enable)
        {
          command_led.enable = 0;
          // LED command
          for (int command_no = 9; command_no < 11; command_no++)
          {
            uint8_t header_command[0xFF];
            uint8_t idx = 0;
            uint8_t command_length = can_command_list[command_no][3];
            for (i = 2; i < command_length; i++)
            {
              header_command[idx] = can_command_list[command_no][i];

              if (i == 5 && can_command_list[command_no][5] == 0xFF)
              {
                appendCRC8CheckSum(header_command, 4);
              }
              if (i == 8 && can_command_list[command_no][8] == 0xFF)
              {
                header_command[idx] = counter_led & 0xFF;
              }
              if (i == 9 && can_command_list[command_no][9] == 0xFF)
              {
                header_command[idx] = (counter_led >> 8) & 0xFF;
              }

              // RED
              if (i == 16)
              {
                header_command[idx] = command_led.red;
              }

              // GREEN
              if (i == 17)
              {
                header_command[idx] = command_led.green;
              }

              // BLUE
              if (i == 18)
              {
                header_command[idx] = command_led.blue;
              }

              idx++;
            }
            appendCRC16CheckSum(header_command, command_length);

            for (int i = 0; i < command_length; i++)
            {
              can_command_buffer[can_command_buffer_wp] = header_command[i];
              can_command_buffer_wp++;
              can_command_buffer_wp %= BUFFER_SIZE;
            }

            counter_led++;
          }
        }

        // Blaster Command
        if (command_blaster)
        {
          command_blaster = 0;

          // Blaster
          for (int command_no = 7; command_no < 9; command_no++)
          {
            uint8_t header_command[0xFF];
            uint8_t idx = 0;
            uint8_t command_length = can_command_list[command_no][3];
            for (i = 2; i < command_length; i++)
            {
              header_command[idx] = can_command_list[command_no][i];

              if (i == 5 && can_command_list[command_no][5] == 0xFF)
              {
                appendCRC8CheckSum(header_command, 4);
              }
              if (i == 8 && can_command_list[command_no][8] == 0xFF)
              {
                header_command[idx] = counter_blaster & 0xFF;
              }
              if (i == 9 && can_command_list[command_no][9] == 0xFF)
              {
                header_command[idx] = (counter_blaster >> 8) & 0xFF;
              }

              idx++;
            }
            appendCRC16CheckSum(header_command, command_length);

            for (int i = 0; i < command_length; i++)
            {
              can_command_buffer[can_command_buffer_wp] = header_command[i];
              can_command_buffer_wp++;
              can_command_buffer_wp %= BUFFER_SIZE;
            }

            counter_blaster++;
          }
        }

        // Twist Command
        {
          uint8_t command_no = 5;
          uint8_t command_length = can_command_list[command_no][3];
          uint8_t header_command[0xFF];
          uint8_t idx = 0;

          // Linear X and Y
          uint16_t linear_x = 256 * command_twist.linear.x + 1024;
          uint16_t linear_y = 256 * command_twist.linear.y + 1024;
          int16_t angular_z = 256 * command_twist.angular.z + 1024;

          for (i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = command_counter[command_no] & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (command_counter[command_no] >> 8) & 0xFF;
            }
            command_counter[i]++;

            switch (i)
            {
            case 15:
              header_command[idx] = can_command_list[command_no][i] & 0xC0;
              header_command[idx] |= (linear_x >> 5) & 0x3F;
              break;
            case 14:
              header_command[idx] = linear_x << 3;
              header_command[idx] |= (linear_y >> 8) & 0x07;
              break;
            case 13:
              header_command[idx] = linear_y & 0xFF;
              break;
            case 19:
              header_command[idx] = (angular_z >> 4) & 0xFF; //0x40;
              break;
            case 18:
              header_command[idx] = (angular_z << 4) | 0x08; //0x08;
              break;
            case 20:
              header_command[idx] = 0x00;
              break;
            case 21:
              header_command[idx] = 0x02 | ((angular_z << 2) & 0xFF);
              break;
            case 22:
              header_command[idx] = (angular_z >> 6) & 0xFF; // 0x10;
              break;
            case 23:
              header_command[idx] = 0x04;
              break;
            case 24:
              header_command[idx] = 0x0C; // Enable Flag 4:x-y 8:yaw 0x0c
              break;
            case 25:
              header_command[idx] = 0x00;
              break;
            case 26:
              header_command[idx] = 0x04;
              break;
            default:
              break;
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);
          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= BUFFER_SIZE;
          }
        }

        // Gimbal Command
        {
          uint8_t command_no = 4;
          uint8_t command_length = can_command_list[command_no][3];
          uint8_t header_command[0xFF];
          uint8_t idx = 0;

          // Angular X and Y
          int16_t angular_y = -1024 * command_twist.angular.y;
          int16_t angular_z = -1024 * command_twist.angular.z;

          for (i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = command_counter[command_no] & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (command_counter[command_no] >> 8) & 0xFF;
            }
            command_counter[command_no]++;

            switch (i)
            {
            case 16:
              header_command[idx] = (angular_y >> 8) & 0xFF;
              break;
            case 15:
              header_command[idx] = angular_y & 0xFF;
              break;
            case 18:
              header_command[idx] = (angular_z >> 8) & 0xFF;
              break;
            case 17:
              header_command[idx] = angular_z & 0xFF;
              break;
            default:
              break;
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);
          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= BUFFER_SIZE;
          }
        }

        // 10msec Task
        for (i = 0; i < COMMAND_LIST_SIZE; i++)
        {
          if (can_command_list[i][1] == 1 && i != 5 && i != 4)
          {
            set_can_command((uint8_t)i);
          }
        }
      }

      // 100msec TASK
      if (timer100msec_flag)
      {
        timer100msec_flag = 0;

        for (i = 0; i < COMMAND_LIST_SIZE; i++)
        {
          if (can_command_list[i][1] == 10)
          {
            set_can_command((uint8_t)i);
          }
        }
      }

      // 1sec TASK
      if (timer1sec_flag)
      {
        timer1sec_flag = 0;

        for (i = 0; i < COMMAND_LIST_SIZE; i++)
        {
          if (can_command_list[i][1] == 100)
          {
            set_can_command((uint8_t)i);
          }
        }
      }

      // 10sec TASK
      if (timer10sec_flag)
      {
        timer10sec_flag = 0;
      }
    }

    // Receive from RoboMaster S1
    while (buffer_rp != buffer_wp)
    {
      CANRxMsg msg;
      msg = rx_msg_buffer[buffer_rp];
      ret = parseCanData(msg.can_id, msg.data, msg.dlc, received_data, &received_data_size);
      buffer_rp++;
      buffer_rp %= BUFFER_SIZE;
      if (ret)
      {
        //sendUdpData(msg.can_id, received_data, received_data_size);

        char *out_str;
        char *num_str;

        float_uint8 base_odom_yaw_raw[5];
        float_uint8 quaternion[5];
        uint32_t base_odom_yaw;

        switch (msg.can_id)
        {

        // Motion Controller Output Data
        case ID_0x202:
        {
          if (received_data[1] == 0x3D && received_data[4] == 0x03 && received_data[5] == 0x09)
          {
            int flag = (received_data[24] >> 7) & 0x01;
            base_odom_yaw = ((((uint16_t)received_data[24]) << 8) | (((uint16_t)received_data[23]) << 0));
            if (flag == 0)
            {
              int shift = (0x86 - ((received_data[24] << 1) | (received_data[23] >> 7)));
              base_odom_yaw = ((1 << 7) | received_data[23]) >> shift;
              base_odom_yaw *= -1;
            }
            else
            {
              int shift = (0x186 - ((received_data[24] << 1) | (received_data[23] >> 7)));
              base_odom_yaw = ((1 << 7) | received_data[23]) >> shift;
            }
            if (received_data[24] == 0 && received_data[23] == 0)
            {
              base_odom_yaw = 0;
            }

            // unknown angle filtered value
            base_odom_yaw_raw[0].uint8_data[0] = received_data[35];
            base_odom_yaw_raw[0].uint8_data[1] = received_data[36];
            base_odom_yaw_raw[0].uint8_data[2] = received_data[37];
            base_odom_yaw_raw[0].uint8_data[3] = received_data[38];
            base_odom_yaw_raw[1].uint8_data[0] = received_data[39];
            base_odom_yaw_raw[1].uint8_data[1] = received_data[40];
            base_odom_yaw_raw[1].uint8_data[2] = received_data[41];
            base_odom_yaw_raw[1].uint8_data[3] = received_data[42];
            base_odom_yaw_raw[2].uint8_data[0] = received_data[43];
            base_odom_yaw_raw[2].uint8_data[1] = received_data[44];
            base_odom_yaw_raw[2].uint8_data[2] = received_data[45];
            base_odom_yaw_raw[2].uint8_data[3] = received_data[46];
            base_odom_yaw_raw[3].uint8_data[0] = received_data[47];
            base_odom_yaw_raw[3].uint8_data[1] = received_data[48];
            base_odom_yaw_raw[3].uint8_data[2] = received_data[49];
            base_odom_yaw_raw[3].uint8_data[3] = received_data[50];
            base_odom_yaw_raw[4].uint8_data[0] = received_data[51];
            base_odom_yaw_raw[4].uint8_data[1] = received_data[52];
            base_odom_yaw_raw[4].uint8_data[2] = received_data[53];
            base_odom_yaw_raw[4].uint8_data[3] = received_data[54];
          }

          if (received_data[1] == 0x31 && received_data[4] == 0x03 && received_data[5] == 0x04)
          {
            // ~20 is unknown counter and state
            quaternion[0].uint8_data[0] = received_data[21];
            quaternion[0].uint8_data[1] = received_data[22];
            quaternion[0].uint8_data[2] = received_data[23];
            quaternion[0].uint8_data[3] = received_data[24];
            quaternion[1].uint8_data[0] = received_data[25];
            quaternion[1].uint8_data[1] = received_data[26];
            quaternion[1].uint8_data[2] = received_data[27];
            quaternion[1].uint8_data[3] = received_data[28];
            quaternion[2].uint8_data[0] = received_data[29];
            quaternion[2].uint8_data[1] = received_data[30];
            quaternion[2].uint8_data[2] = received_data[31];
            quaternion[2].uint8_data[3] = received_data[32];
            quaternion[3].uint8_data[0] = received_data[33];
            quaternion[3].uint8_data[1] = received_data[34];
            quaternion[3].uint8_data[2] = received_data[35];
            quaternion[3].uint8_data[3] = received_data[36];

            int32_t int_data1;
            uint32_t uint32_data = received_data[40];
            uint32_data = (uint32_data << 8) | received_data[39];
            uint32_data = (uint32_data << 8) | received_data[38];
            uint32_data = (uint32_data << 8) | received_data[37];
            int_data1 = (int32_t)uint32_data;

            int32_t int_data2;
            uint32_data = received_data[44];
            uint32_data = (uint32_data << 8) | received_data[43];
            uint32_data = (uint32_data << 8) | received_data[42];
            uint32_data = (uint32_data << 8) | received_data[41];
            int_data2 = (int32_t)uint32_data;

            uint8_t battery_soc = received_data[45];
          }

          // Counter
          if (received_data[1] == 0x39 && received_data[4] == 0x03 && received_data[5] == 0x0A)
          {
            uint32_t counter = 0;
            counter = (received_data[14] & 0x0F) * 10000;
            counter += (received_data[15] & 0x0F) * 1000;
            counter += (received_data[16] & 0x0F) * 100;
            counter += (received_data[17] & 0x0F) * 10;
            counter += (received_data[18] & 0x0F) * 1;
          }

          // Odometry Lab Mode Data
          if (received_data[1] == 0x6F && received_data[4] == 0x03 && received_data[5] == 0x09)
          {
            // Odom in Lab mode
            uint32_t data;
            int32_t int_data;
            uint16_t data16;
            int16_t int_data16;

            int16_t wheel_angular_velocity[4];

            data = received_data[16];
            data = (data << 8) | received_data[15];
            data = (data << 8) | received_data[14];
            data = (data << 8) | received_data[13];
            uint32_t odom_counter = (int32_t)data;

            data = received_data[20];
            data = (data << 8) | received_data[19];
            data = (data << 8) | received_data[18];
            data = (data << 8) | received_data[17];
            uint32_t odom_counter2 = (int32_t)data;

            // Wheel Angular Velocity
            // Unit is RPM
            data16 = received_data[22];
            data16 = (data16 << 8) | received_data[21];
            wheel_angular_velocity[0] = (int16_t)data16;

            data16 = received_data[24];
            data16 = (data16 << 8) | received_data[23];
            wheel_angular_velocity[1] = (int16_t)data16;

            data16 = received_data[26];
            data16 = (data16 << 8) | received_data[25];
            wheel_angular_velocity[2] = (int16_t)data16;

            data16 = received_data[28];
            data16 = (data16 << 8) | received_data[27];
            wheel_angular_velocity[3] = (int16_t)data16;

            float wheel_angle[4];
            // wheel position
            data16 = received_data[30];
            data16 = (data16 << 8) | received_data[29];
            data16 = data16 << 1;
            int_data16 = (int16_t)data16;
            wheel_angle[0] = int_data16 / 32767.0 * 180.0;

            data16 = received_data[32];
            data16 = (data16 << 8) | received_data[31];
            data16 = data16 << 1;
            int_data16 = (int16_t)data16;
            wheel_angle[1] = int_data16 / 32767.0 * 180.0;

            data16 = received_data[34];
            data16 = (data16 << 8) | received_data[33];
            data16 = data16 << 1;
            int_data16 = (int16_t)data16;
            wheel_angle[2] = int_data16 / 32767.0 * 180.0;

            data16 = received_data[36];
            data16 = (data16 << 8) | received_data[35];
            data16 = data16 << 1;
            int_data16 = (int16_t)data16;
            wheel_angle[3] = int_data16 / 32767.0 * 180.0;

            int32_t m_bus_update_count[4];
            // wheel update count
            data = received_data[40];
            data = (data << 8) | received_data[39];
            data = (data << 8) | received_data[38];
            data = (data << 8) | received_data[37];
            m_bus_update_count[0] = (int32_t)data;

            data = received_data[44];
            data = (data << 8) | received_data[43];
            data = (data << 8) | received_data[42];
            data = (data << 8) | received_data[41];
            m_bus_update_count[1] = (int32_t)data;

            data = received_data[48];
            data = (data << 8) | received_data[47];
            data = (data << 8) | received_data[46];
            data = (data << 8) | received_data[45];
            m_bus_update_count[2] = (int32_t)data;

            data = received_data[52];
            data = (data << 8) | received_data[51];
            data = (data << 8) | received_data[50];
            data = (data << 8) | received_data[49];
            m_bus_update_count[3] = (int32_t)data;

            // IMU Data
            float_uint8 union_data;

            float mag_x;
            float mag_y;
            float g_x;
            float g_y;
            float g_z;
            float gyro_x;
            float gyro_y;
            float gyro_z;
            float roll, pitch, yaw;

            union_data.uint8_data[0] = received_data[53 + 4];
            union_data.uint8_data[1] = received_data[54 + 4];
            union_data.uint8_data[2] = received_data[55 + 4];
            union_data.uint8_data[3] = received_data[56 + 4];
            mag_x = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 2];
            union_data.uint8_data[1] = received_data[54 + 4 * 2];
            union_data.uint8_data[2] = received_data[55 + 4 * 2];
            union_data.uint8_data[3] = received_data[56 + 4 * 2];
            mag_y = union_data.float_data;

            // mag_z is blank

            union_data.uint8_data[0] = received_data[53 + 4 * 4];
            union_data.uint8_data[1] = received_data[54 + 4 * 4];
            union_data.uint8_data[2] = received_data[55 + 4 * 4];
            union_data.uint8_data[3] = received_data[56 + 4 * 4];
            g_x = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 5];
            union_data.uint8_data[1] = received_data[54 + 4 * 5];
            union_data.uint8_data[2] = received_data[55 + 4 * 5];
            union_data.uint8_data[3] = received_data[56 + 4 * 5];
            g_y = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 6];
            union_data.uint8_data[1] = received_data[54 + 4 * 6];
            union_data.uint8_data[2] = received_data[55 + 4 * 6];
            union_data.uint8_data[3] = received_data[56 + 4 * 6];
            g_z = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 7];
            union_data.uint8_data[1] = received_data[54 + 4 * 7];
            union_data.uint8_data[2] = received_data[55 + 4 * 7];
            union_data.uint8_data[3] = received_data[56 + 4 * 7];
            gyro_x = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 8];
            union_data.uint8_data[1] = received_data[54 + 4 * 8];
            union_data.uint8_data[2] = received_data[55 + 4 * 8];
            union_data.uint8_data[3] = received_data[56 + 4 * 8];
            gyro_y = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 9];
            union_data.uint8_data[1] = received_data[54 + 4 * 9];
            union_data.uint8_data[2] = received_data[55 + 4 * 9];
            union_data.uint8_data[3] = received_data[56 + 4 * 9];
            gyro_z = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 11];
            union_data.uint8_data[1] = received_data[54 + 4 * 11];
            union_data.uint8_data[2] = received_data[55 + 4 * 11];
            union_data.uint8_data[3] = received_data[56 + 4 * 11];
            yaw = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 12];
            union_data.uint8_data[1] = received_data[54 + 4 * 12];
            union_data.uint8_data[2] = received_data[55 + 4 * 12];
            union_data.uint8_data[3] = received_data[56 + 4 * 12];
            pitch = union_data.float_data;

            union_data.uint8_data[0] = received_data[53 + 4 * 13];
            union_data.uint8_data[1] = received_data[54 + 4 * 13];
            union_data.uint8_data[2] = received_data[55 + 4 * 13];
            union_data.uint8_data[3] = received_data[56 + 4 * 13];
            roll = union_data.float_data;
          }

          break;
        }

        // Gimbal Output Data
        case ID_0x203: //0x203
        {
          // From Gimbal
          if (received_data[1] == 0x11 && received_data[4] == 0x04 && received_data[5] == 0x03)
          {
            // Yaw Angle
            uint16_t data = received_data[12];
            data = (data << 8) | received_data[11];
            gimbal_base_yaw_angle = -(int16_t)(data) / 10.0;
            data = received_data[14];
            data = (data << 8) | received_data[13];
            gimbal_map_yaw_angle = -(int16_t)(data) / 100.0;

            //printf("%lf, %lf\n",gimbal_base_yaw_angle_,gimbal_map_yaw_angle_);
          }
          if (received_data[1] == 0x16 && received_data[4] == 0x04 && received_data[5] == 0x09)
          {
            // Pitch Angle
            uint32_t data = received_data[14];
            data = (data << 8) | received_data[13];
            data = (data << 8) | received_data[12];
            data = (data << 8) | received_data[11];
            gimbal_map_pitch_angle = (int32_t)(data) / 20000000.0 * 30.0;
            data = received_data[18];
            data = (data << 8) | received_data[17];
            data = (data << 8) | received_data[16];
            data = (data << 8) | received_data[15];
            gimbal_base_pitch_angle = (int32_t)(data) / 20000000.0 * 30.0;
            //printf("%lf, %lf\n", gimbal_map_pitch_angle_, gimbal_base_pitch_angle_);
          }
          break;
        }
        case ID_0x204: //0x204
        {
          if (received_data[4] == 0x17 && received_data[5] == 0x09)
          {
          }
          break;
        }
        default:
          break;
        }

        // while (CDC_Transmit_FS((uint8_t *)received_data, received_data_size) == USBD_BUSY)
        // {
        // }
        break;
      }
    }

    // Receive Command from USB
    uint8_t usb_received_data[BUFFER_SIZE];
    uint16_t usb_received_data_size = 0;
    uint8_t usb_parsed_data[255];
    uint8_t usb_parsed_data_size = 0;
    while (usb_rBuf_rp != usb_rBuf_wp)
    {
      usb_received_data[usb_received_data_size] = usb_rBuf[usb_rBuf_rp];
      usb_received_data_size++;
      usb_rBuf_rp++;
      usb_rBuf_rp %= BUFFER_SIZE;
    }
    ret = parseUsbData(usb_received_data, usb_received_data_size, usb_parsed_data, &usb_parsed_data_size);
    if(ret && usb_parsed_data_size > 7)
    {
      if (usb_parsed_data[4] == 0x01) // Twist command No.
      {
        twist received_twist;
        int ret = parseTwistCommandData(usb_parsed_data, usb_parsed_data_size, &received_twist);
        if (ret)
        {
          command_twist = received_twist;
        }
      }
      if (usb_parsed_data[4] == 0x02) // Blaster command No.
      {
        int received_blaster;
        received_blaster = parseBlasterCommandData(usb_parsed_data, usb_parsed_data_size);
        if (received_blaster)
        {
          command_blaster = received_blaster;
        }
      }
      if (usb_parsed_data[4] == 0x03) // Lose command No.
      {
        int received_lose;
        received_lose = parseLoseCommandData(usb_parsed_data, usb_parsed_data_size);
        if (received_lose)
        {
          command_lose = received_lose;
        }
      }
      if (usb_parsed_data[4] == 0x04) // LED command No.
      {
        led received_led;
        int ret = parseLEDCommandData(usb_parsed_data, usb_parsed_data_size, &received_led);
        if (ret)
        {
          command_led = received_led;
        }
      }
    }

    // Transmit CAN Command
    while (can_command_buffer_rp != can_command_buffer_wp &&
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 2)
    {
      uint8_t TxData[8];
      CAN_TxHeaderTypeDef TxHeader;
      uint32_t TxMailbox;

      int can_command_buffer_size = (can_command_buffer_wp - can_command_buffer_rp + BUFFER_SIZE) % BUFFER_SIZE;
      if (can_command_buffer_size >= 8)
      {
        TxHeader.DLC = 8;
      }
      else
      {
        TxHeader.DLC = can_command_buffer_size;
      }
      TxHeader.StdId = 0x201; //RxHeader.StdId;
      TxHeader.ExtId = 0x00;  //RxHeader.ExtId;
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.IDE = CAN_ID_STD;
      TxHeader.TransmitGlobalTime = DISABLE;
      for (i = 0; i < TxHeader.DLC; i++)
      {
        TxData[i] = can_command_buffer[(can_command_buffer_rp + i) % BUFFER_SIZE];
      }
      // Start Transmission process
      int ret = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
      if (ret == HAL_OK)
      {
        can_command_buffer_rp += TxHeader.DLC;
        can_command_buffer_rp %= BUFFER_SIZE;
      }
    }

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
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
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD2_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
static void CAN_FilterConfig(void)
{
  CAN_FilterTypeDef sFilterConfig;

  /*## Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0; //ID 0-13 for CAN1, 14+ is CAN2
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
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
  uint8_t RxData[8];
  CAN_RxHeaderTypeDef RxHeader;

  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    if (RxHeader.StdId != 0x201)
    {
      CANRxMsg msg;
      switch (RxHeader.StdId)
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
      if (initial_task_flag)
      {
        msg.dlc = RxHeader.DLC;
        memcpy(msg.data, RxData, RxHeader.DLC);
        rx_msg_buffer[buffer_wp] = msg;
        buffer_wp++;
        buffer_wp %= BUFFER_SIZE;
      }
    }
  }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t RxData[8];
  CAN_RxHeaderTypeDef RxHeader;

  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    if (RxHeader.StdId != 0x201)
    {
      CANRxMsg msg;
      switch (RxHeader.StdId)
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
      if (initial_task_flag)
      {
        msg.dlc = RxHeader.DLC;
        memcpy(msg.data, RxData, RxHeader.DLC);
        rx_msg_buffer[buffer_wp] = msg;
        buffer_wp++;
        buffer_wp %= BUFFER_SIZE;
      }
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2)
  {
    // 10msec Timer
    timer10msec_counter++;
    timer10msec_flag = 1;

    // 10sec conter
    if (timer10msec_counter % 1000 == 0)
    {
      timer10sec_flag = 1;
    }

    // 1sec conter
    if (timer10msec_counter % 100 == 0)
    {
      timer1sec_flag = 1;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
    }

    // 100msec counter
    if (timer10msec_counter % 10 == 0)
    {
      timer100msec_flag = 1;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
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
