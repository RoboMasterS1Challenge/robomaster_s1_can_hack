#include <stdio.h>
#include <string.h>
#include <limits.h> // CHAR_BIT
#include <stdint.h>
#include <stdlib.h>

#include "robomaster_s1.h"
#include "robomaster_s1_crc.h"

twist command_twist;
int command_blaster;
int command_lose;
led command_led;

volatile uint8_t robomas_buffer[CAN_ID_NUM][BUFFER_SIZE];
volatile int robomas_buffer_rp[CAN_ID_NUM];
volatile int robomas_buffer_wp[CAN_ID_NUM];
volatile uint8_t command_buffer[BUFFER_SIZE];
volatile int command_buffer_rp;
volatile int command_buffer_wp;


int parseCanData(uint16_t id, uint8_t* in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t* out_data_size)
{
  int i;

  for(int i=0;i<in_data_size;i++){
    robomas_buffer[id][robomas_buffer_wp[id]] = in_data[i];
    robomas_buffer_wp[id]++;
    robomas_buffer_wp[id] %= BUFFER_SIZE;
  }

  int can_in_buffer_size = (robomas_buffer_wp[id] - robomas_buffer_rp[id] + BUFFER_SIZE) % BUFFER_SIZE;

  // Data size check
  if(can_in_buffer_size < 7){
    return 0;
  }

  // Search Header
  int find_flag = 0;
  for(i = 0; i < can_in_buffer_size - 7; i++){
    if(robomas_buffer[id][(robomas_buffer_rp[id]+i)%BUFFER_SIZE]==0x55 && robomas_buffer[id][(robomas_buffer_rp[id] +i +2)%BUFFER_SIZE]==0x04){
      find_flag = 1;
      break;
    }
    can_in_buffer_size--;
    robomas_buffer_rp[id]++;
    robomas_buffer_rp[id] %= BUFFER_SIZE;
  }

  if(find_flag == 0){
    return 0;
  }

  // Check data length
  int send_data_size = robomas_buffer[id][(robomas_buffer_rp[id]+1)%BUFFER_SIZE];
  if(send_data_size > can_in_buffer_size)
  {
    // Not enough data
    return 0;
  }

  // Prepare send data
  uint8_t* send_data;
  send_data = (uint8_t*)malloc(sizeof(uint8_t) * send_data_size);


  for(i = 0; i < send_data_size; i++){
    int buffer_p = (robomas_buffer_rp[id] + i) % BUFFER_SIZE;
    send_data[i] = robomas_buffer[id][buffer_p];
  }

  // Check header crc8
  if(!verifyCRC8CheckSum(send_data, 4))
  {
    // checksum error
    // skip header
    robomas_buffer_rp[id]++;
    robomas_buffer_rp[id] %= BUFFER_SIZE;
    free(send_data);
    return 0;
  }

  // Check crc16
  if(!verifyCRC16CheckSum(send_data, send_data_size))
  {
    // checksum error
    // skip header
    robomas_buffer_rp[id]++;
    robomas_buffer_rp[id] %= BUFFER_SIZE;
    free(send_data);
    return 0;
  }

  memcpy(out_data, send_data, send_data_size);
  *out_data_size = send_data_size;

  free(send_data);
  robomas_buffer_rp[id] += send_data_size;
  robomas_buffer_rp[id] %= BUFFER_SIZE;
  return 1;
}

int parseUsbData(uint8_t* in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t* out_data_size)
{
  int i;

  for(int i=0;i<in_data_size;i++){
    command_buffer[command_buffer_wp] = in_data[i];
    command_buffer_wp++;
    command_buffer_wp %= BUFFER_SIZE;
  }

  int usb_in_buffer_size = (command_buffer_wp - command_buffer_rp + BUFFER_SIZE) % BUFFER_SIZE;

  // Data size check
  if(usb_in_buffer_size < 7){
    return 0;
  }

  // Search Header
  int find_flag = 0;
  for(i = 0; i < usb_in_buffer_size - 7; i++){
    if(command_buffer[(command_buffer_rp+i)%BUFFER_SIZE]==0x55 && command_buffer[(command_buffer_rp +i +2)%BUFFER_SIZE]==0x04){
      find_flag = 1;
      break;
    }
    command_buffer_rp ++;
    command_buffer_rp %= BUFFER_SIZE;
    usb_in_buffer_size --;
  }

  if(find_flag == 0){
    return 0;
  }

  // Check data length
  int send_data_size = command_buffer[(command_buffer_rp+1)%BUFFER_SIZE];
  if(send_data_size > usb_in_buffer_size)
  {
    // Not enough data
    return 0;
  }

  // Prepare send data
  uint8_t* send_data;
  send_data = (uint8_t*)malloc(sizeof(uint8_t) * send_data_size);


  for(i = 0; i < send_data_size; i++){
    int buffer_p = (command_buffer_rp + i) % BUFFER_SIZE;
    send_data[i] = command_buffer[buffer_p];
  }

  // Check header crc8
  if(!verifyCRC8CheckSum(send_data, 4))
  {
    // checksum error
    // skip header
    command_buffer_rp++;
    command_buffer_rp %= BUFFER_SIZE;
    free(send_data);
    return 0;
  }

  // Check crc16
  if(!verifyCRC16CheckSum(send_data, send_data_size))
  {
    // checksum error
    // skip header
    command_buffer_rp++;
    command_buffer_rp %= BUFFER_SIZE;
    free(send_data);
    return 0;
  }

  memcpy(out_data, send_data, send_data_size);
  *out_data_size = send_data_size;

  free(send_data);
  command_buffer_rp += send_data_size;
  command_buffer_rp %= BUFFER_SIZE;
  return 1;
}

int parseTwistCommandData(uint8_t* in_data, uint8_t in_data_size, twist *command_twist)
{
  command_twist->enable = 0;
  command_twist->linear.x = 0;
  command_twist->linear.y = 0;
  command_twist->linear.z = 0;
  command_twist->angular.x = 0;
  command_twist->angular.y = 0;
  command_twist->angular.z = 0;

  // Check data length
  if(in_data_size != TWIST_COMMAND_SIZE)
  {
    // Not enough data
    return 0;
  }

  if(in_data[0] == 0x55 &&
    in_data[1] == TWIST_COMMAND_SIZE &&
    in_data[2] == 0x04 &&
    verifyCRC8CheckSum(in_data, 4) == 1 &&
    verifyCRC16CheckSum(in_data, TWIST_COMMAND_SIZE) == 1 &&
    in_data[4] == 0x01 // command id
    ){

    // make twist command
    command_twist->enable = 1;
    command_twist->linear.x = 0.01 * (int16_t)((((uint16_t)in_data[5])<<8)|(in_data[6]));
    command_twist->linear.y = 0.01 * (int16_t)((((uint16_t)in_data[7])<<8)|(in_data[8]));
    command_twist->linear.z = 0.01 * (int16_t)((((uint16_t)in_data[9])<<8)|(in_data[10]));
    command_twist->angular.x = 0.01 * (int16_t)((((uint16_t)in_data[11])<<8)|(in_data[12]));
    command_twist->angular.y = 0.01 * (int16_t)((((uint16_t)in_data[13])<<8)|(in_data[14]));
    command_twist->angular.z = 0.01 * (int16_t)((((uint16_t)in_data[15])<<8)|(in_data[16]));
  }

  return 1;
}

int parseBlasterCommandData(uint8_t* in_data, uint8_t in_data_size)
{
  int blaster = 0;
  // Check data length
  if(in_data_size != BLASTER_COMMAND_SIZE)
  {
    // Not enough data
    return 0;
  }

  if(in_data[0] == 0x55 &&
    in_data[1] == BLASTER_COMMAND_SIZE &&
    in_data[2] == 0x04 &&
    verifyCRC8CheckSum(in_data, 4) == 1 &&
    verifyCRC16CheckSum(in_data, BLASTER_COMMAND_SIZE) == 1 &&
    in_data[4] == 0x02 // command id
    ){

    // make blaster command
    blaster = in_data[5];
  }
  return blaster;
}

int parseLoseCommandData(uint8_t* in_data, uint8_t in_data_size)
{
  int lose = 0;
  // Check data length
  if(in_data_size != LOSE_COMMAND_SIZE)
  {
    // Not enough data
    return 0;
  }

  if(in_data[0] == 0x55 &&
    in_data[1] == LOSE_COMMAND_SIZE &&
    in_data[2] == 0x04 &&
    verifyCRC8CheckSum(in_data, 4) == 1 &&
    verifyCRC16CheckSum(in_data, LOSE_COMMAND_SIZE) == 1 &&
    in_data[4] == 0x03 // command id
    ){

    // make blaster command
    lose = in_data[5] + 1;
  }else{
    return 0;
  }
  return lose;
}


int parseLEDCommandData(uint8_t* in_data, uint8_t in_data_size, led *command_led)
{
  command_led->enable = 0;
  command_led->red = 0;
  command_led->green = 0;
  command_led->blue = 0;

  // Check data length
  if(in_data_size != LED_COMMAND_SIZE)
  {
    // Not enough data
    return 0;
  }

  if(in_data[0] == 0x55 &&
    in_data[1] == LED_COMMAND_SIZE &&
    in_data[2] == 0x04 &&
    verifyCRC8CheckSum(in_data, 4) == 1 &&
    verifyCRC16CheckSum(in_data, LED_COMMAND_SIZE) == 1 &&
    in_data[4] == 0x04 // command id
    ){

    // make twist command
    command_led->enable = 1;
    command_led->red = in_data[5];
    command_led->green = in_data[6];
    command_led->blue = in_data[7];
  }else{
    return 0;
  }

  return 1;
}
