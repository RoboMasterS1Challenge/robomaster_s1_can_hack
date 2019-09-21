
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>

#include <stdio.h>
#include <cstring>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <mutex>
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include "robomaster_s1_crc.h"

#define BUFFER_MAX 255

#define CAN_ID_NUM 10

class broadcast_udp_receive
{
private:
  int sock_;
  struct sockaddr_in addr_;

public:
  broadcast_udp_receive(uint16_t port)
  {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    char broadcast = '1';
    setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_.sin_port = htons(port);
    printf("Create Socket %d\n", sock_);
  }
  ~broadcast_udp_receive()
  {
    close(sock_);
  }

  int udp_send(uint8_t *send_data, uint8_t send_data_size)
  {
    return sendto(sock_, send_data, send_data_size, 0, (struct sockaddr *)&addr_, sizeof(addr_));
  }

  int udp_bind()
  {
    int ret = bind(sock_, (const struct sockaddr *)&addr_, sizeof(addr_));
    printf("UDP bind %d\n", ret);
    return ret;
  }

  int udp_recv(uint8_t *buf, int size)
  {
    struct pollfd fd;
    int res;

    fd.fd = sock_;
    fd.events = POLLIN;
    res = poll(&fd, 1, 1000); // 1000 ms timeout

    if (res == 0)
    {
      // timeout
    }
    else if (res == -1)
    {
      // error
    }
    else
    {
      memset(buf, 0, size);
      int recv_len = recvfrom(sock_, buf, size, 0, NULL, 0);
      return recv_len;
    }
  }
};

class broadcast_udp_send
{
private:
  int sock_;
  struct sockaddr_in addr_;

public:
  broadcast_udp_send(std::string ip_addr, uint16_t port)
  {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(port);
    addr_.sin_addr.s_addr = inet_addr(ip_addr.c_str());
    int broadcast = 1;
    setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, (char *)&broadcast, sizeof(broadcast));
    printf("Create Socket %d\n", sock_);
  }
  ~broadcast_udp_send()
  {
    close(sock_);
  }

  int udp_send(uint8_t *send_data, uint8_t send_data_size)
  {
    return sendto(sock_, send_data, send_data_size, 0, (struct sockaddr *)&addr_, sizeof(addr_));
  }

  int udp_bind()
  {
    int ret = bind(sock_, (const struct sockaddr *)&addr_, sizeof(addr_));
    printf("UDP bind %d\n", ret);
    return ret;
  }

  int udp_recv(uint8_t *buf, uint16_t size)
  {
    memset(buf, 0, size);
    int recv_len = recvfrom(sock_, buf, size, 0, NULL, 0);
    return recv_len;
  }
};

class RoboMasterS1Bridge
{
private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // Publisher
  ros::Publisher robomaster_s1_bridge_rx_pub_;
  // Subscriber
  ros::Subscriber robomaster_s1_bridge_tx_sub_;

  std::vector<std::thread> receive_thread_list_;

  // UDP Port
  uint16_t receive_port_list_[CAN_ID_NUM];
  uint16_t send_port_;

  // UDP Send
  broadcast_udp_send *udp_send_;
  
  // UDP Receive
  std::vector<broadcast_udp_receive*> udp_receive_;

  int debug_print_;
  std::string ip_address_;

  // callback functions
  void twistCommandCallback(const geometry_msgs::Twist::ConstPtr &twist_command);

  // thread
  void udpReceiveThread(uint8_t can_id_num);

public:
  // Constructor
  RoboMasterS1Bridge();

  // Destructor
  ~RoboMasterS1Bridge();
};
