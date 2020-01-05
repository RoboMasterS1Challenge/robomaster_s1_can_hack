#include <stdio.h>
#include <stdint.h>

#ifndef __ROBOMASTER_S1_H__
#define __ROBOMASTER_S1_H__

#define CAN_ID_NUM 10

#define BUFFER_SIZE 2048

#define PI 3.14159265

#define TWIST_COMMAND_SIZE 19
#define BLASTER_COMMAND_SIZE 8
#define LOSE_COMMAND_SIZE 8
#define LED_COMMAND_SIZE 10

typedef enum can_ids
{
    ID_0x201 = 0,
    ID_0x202 = 1,
    ID_0x203 = 2,
    ID_0x204 = 3,
    ID_0x211 = 4,
    ID_0x212 = 5,
    ID_0x213 = 6,
    ID_0x214 = 7,
    ID_0x215 = 8,
    ID_0x216 = 9
} can_ids;

typedef struct CANRxMsg
{
    can_ids can_id;
    uint8_t dlc;
    uint8_t data[8];
} CANRxMsg;

typedef struct linear
{
    double x;
    double y;
    double z;
} linear;
typedef struct angular
{
    double x;
    double y;
    double z;
} angular;
typedef struct twist
{
    uint8_t enable;
    linear linear;
    angular angular;
} twist;

typedef struct led
{
    uint8_t enable;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led;

#endif
