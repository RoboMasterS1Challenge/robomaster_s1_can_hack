#include <stdio.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/udp.h"
#include "robomaster_s1.h"

ip4_addr_t forward_ip;
struct udp_pcb *broadcast_pcb[CAN_ID_NUM];
struct udp_pcb *ptel_pcb;
extern int parseCanData(uint8_t id, uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size);
extern int parseUsbData(uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size);

twist command_twist;
int command_blaster;
int command_lose;
led command_led;

const uint16_t port_list[CAN_ID_NUM] = {
    0x201 + 10000,
    0x202 + 10000,
    0x203 + 10000,
    0x204 + 10000,
    0x211 + 10000,
    0x212 + 10000,
    0x213 + 10000,
    0x214 + 10000,
    0x215 + 10000,
    0x216 + 10000};

void sendUdpData(uint8_t id, uint8_t *send_data, uint8_t send_data_size)
{
  // Send UDP data
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, send_data_size * sizeof(uint8_t), PBUF_REF);
  p->payload = send_data;

  udp_sendto(broadcast_pcb[id], p, &forward_ip, port_list[id]); //

  pbuf_free(p);
}

void recvUdpDataCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
  uint8_t buf[255];
  int ret = 0;
  if (p != NULL)
  {
    if (p->len >= 8)
    {
      memcpy(buf, p->payload, p->len);

      if (buf[4] == 0x01) // Twist command No.
      {
        twist received_twist;
        int ret = parseTwistCommandData(buf, p->len, &received_twist);
        if (ret)
        {
          command_twist = received_twist;
        }
      }
      if (buf[4] == 0x02) // Blaster command No.
      {
        int received_blaster;
        received_blaster = parseBlasterCommandData(buf, p->len);
        if (received_blaster)
        {
          command_blaster = received_blaster;
        }
      }
      if (buf[4] == 0x03) // Lose command No.
      {
        int received_lose;
        received_lose = parseLoseCommandData(buf, p->len);
        if (received_lose)
        {
          command_lose = received_lose;
        }
      }
      if (buf[4] == 0x04) // LED command No.
      {
        led received_led;
        int ret = parseLEDCommandData(buf, p->len, &received_led);
        if (ret)
        {
          command_led = received_led;
        }
      }
    }
    pbuf_free(p);
  }
}

void startUdp(uint8_t *ip_addr)
{
  int i;
  err_t err;
  IP4_ADDR(&forward_ip, ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
  for (i = 0; i < CAN_ID_NUM; i++)
  {
    broadcast_pcb[i] = udp_new();
  }
  ptel_pcb = udp_new();

  int port = 0x200 + 10000;

  if (ptel_pcb)
  {
    err = udp_bind(ptel_pcb, IP_ADDR_ANY, port);

    if (err == ERR_OK)
    {
      /* Set a receive callback for the upcb */
      udp_recv(ptel_pcb, recvUdpDataCallback, NULL);
    }
  }
}
