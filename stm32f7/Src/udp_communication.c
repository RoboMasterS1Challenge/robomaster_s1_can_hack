#include <stdio.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/udp.h"
#include "robomaster_s1.h"

ip4_addr_t forward_ip;
struct udp_pcb *broadcast_pcb[CAN_ID_NUM];
struct udp_pcb *ptel_pcb;
extern twist parseTwistCommandData(uint8_t *in_data, uint8_t in_data_size);
extern int parseBlasterCommandData(uint8_t *in_data, uint8_t in_data_size);

twist command_twist;
int command_blaster;
extern uint8_t data_update_check;

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
	int i;
	uint8_t buf[TWIST_COMMAND_SIZE];
	if (p != NULL)
	{
		if (p->len == TWIST_COMMAND_SIZE)
		{
			memcpy(buf, p->payload, TWIST_COMMAND_SIZE);
			command_twist = parseTwistCommandData(buf, TWIST_COMMAND_SIZE);
			data_update_check = 0;
		}
		if (p->len == BLASTER_COMMAND_SIZE)
		{
			memcpy(buf, p->payload, BLASTER_COMMAND_SIZE);
			command_blaster = parseBlasterCommandData(buf, BLASTER_COMMAND_SIZE);
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
