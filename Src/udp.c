#include <stdio.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/udp.h"

extern
unsigned int VerifyCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength);
extern
uint32_t VerifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);

#define CAN_ID_NUM 10

ip4_addr_t forward_ip;
const uint16_t port_list[CAN_ID_NUM] = {
		0x201,
		0x202,
		0x203,
		0x204,
		0x211,
		0x212,
		0x213,
		0x214,
		0x215,
		0x216
};

struct udp_pcb *broadcast_pcb[CAN_ID_NUM];

#define BUFFER_SIZE 1024
volatile uint8_t can_buffer[CAN_ID_NUM][BUFFER_SIZE];
volatile int can_buffer_rp[CAN_ID_NUM];
volatile int can_buffer_wp[CAN_ID_NUM];

void UdpSendData(uint8_t id, uint8_t in_data)
{

	can_buffer[id][can_buffer_wp[id]] = in_data;
	can_buffer_wp[id]++;
	can_buffer_wp[id] %= BUFFER_SIZE;

	int can_in_buffer_size = (can_buffer_wp[id] - can_buffer_rp[id] + BUFFER_SIZE) % BUFFER_SIZE;

	// minimam data size
	if(can_in_buffer_size < 7){
		return;
	}

	// Search Header
	int i;
	int find_flag = 0;
	for(i = 0; i < can_in_buffer_size - 7; i++){
		if(can_buffer[id][can_buffer_rp[id]]==0x55 && can_buffer[id][can_buffer_rp[id]+2]==0x04){
			find_flag = 1;
			can_in_buffer_size -= i;
			break;
		}
		can_buffer_rp[id]++;
		can_buffer_rp[id] %= BUFFER_SIZE;
	}

	if(find_flag == 0){
		return;
	}

	// Check data length
	int send_data_size = can_buffer[id][can_buffer_rp[id]+1];
	if(send_data_size > can_in_buffer_size)
	{
		// Not enough data
		return;
	}

	// Prepare send data
	uint8_t* send_data;
	send_data = (uint8_t*)malloc(sizeof(uint8_t) * send_data_size);


	for(i = 0; i < send_data_size; i++){
		int buffer_p = (can_buffer_rp[id] + i) % BUFFER_SIZE;
		send_data[i] = can_buffer[id][buffer_p];
	}

	// Check header crc8
	if(!VerifyCRC8CheckSum(send_data, 4))
	{
		// checksum error
		// skip header
		can_buffer_rp[id]++;
		can_buffer_rp[id] %= BUFFER_SIZE;
		free(send_data);
		return;
	}

	// Check crc16
	if(!VerifyCRC16CheckSum(send_data, send_data_size))
	{
		// checksum error
		// skip header
		can_buffer_rp[id]++;
		can_buffer_rp[id] %= BUFFER_SIZE;
		free(send_data);
		return;
	}


	// Send UDP data
	struct pbuf * p = pbuf_alloc(PBUF_TRANSPORT, send_data_size * sizeof(uint8_t), PBUF_REF);
	p->payload = send_data;

	udp_sendto(broadcast_pcb[id], p, &forward_ip, port_list[id]); //

	pbuf_free(p);

	free(send_data);
	can_buffer_rp[id] += send_data_size;
	can_buffer_rp[id] %= BUFFER_SIZE;

}

void UdpRecvData(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
	int i;
      if (p != NULL) {
    	udp_sendto(pcb, p, &forward_ip, port); //dest port
            pbuf_free(p);
      }
}

void StartUdp(uint8_t* ip_addr)
{
	int i;
	for(i=0;i<CAN_ID_NUM;i++){
		struct udp_pcb *ptel_pcb = udp_new();
		int port = port_list[i];

		udp_bind(ptel_pcb, IP_ADDR_ANY, port);
		IP4_ADDR(&forward_ip, ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);

		//udp_recv(ptel_pcb, UdpRecvData, NULL);

		broadcast_pcb[i] = udp_new();
	}


}
