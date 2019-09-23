#include "robomaster_s1_bridge.h"

// RoboMasterS1Bridge()
// Constructor
RoboMasterS1Bridge::RoboMasterS1Bridge()
    : nh_(""), pnh_("~")
{
  // ROS
  robomaster_s1_bridge_tx_sub_ = nh_.subscribe("/robomaster_s1/cmd_vel", 10, &RoboMasterS1Bridge::twistCommandCallback, this);
  robomaster_s1_bridge_rx_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/robomaster_s1/can_info", 10);

  // Parameters
  pnh_.param("debug_print", debug_print_, (int)0);
  pnh_.param("ip_address", ip_address_, std::string("192.168.0.255"));

  receive_port_list_[0] = 0x201 + 10000;
  receive_port_list_[1] = 0x202 + 10000;
  receive_port_list_[2] = 0x203 + 10000;
  receive_port_list_[3] = 0x204 + 10000;
  receive_port_list_[4] = 0x211 + 10000;
  receive_port_list_[5] = 0x212 + 10000;
  receive_port_list_[6] = 0x213 + 10000;
  receive_port_list_[7] = 0x214 + 10000;
  receive_port_list_[8] = 0x215 + 10000;
  receive_port_list_[9] = 0x216 + 10000;
  send_port_ = 0x200 + 10000;

  udp_send_ = new broadcast_udp_send(ip_address_, send_port_);

  for (int32_t i = 0; i < CAN_ID_NUM; ++i)
  {
    // UDP Receive
    udp_receive_.emplace_back(new broadcast_udp_receive(receive_port_list_[i]));
    udp_receive_[i]->udp_bind();

    // Make receive thread
    receive_thread_list_.emplace_back(std::thread([this, i]() { this->udpReceiveThread(i); }));
    ROS_INFO("Start %d receive thread", i);
  }
} //RoboMasterS1Bridge()

// ~RoboMasterS1Bridge()
// Destructor
RoboMasterS1Bridge::~RoboMasterS1Bridge()
{
  for (int32_t i = 0; i < CAN_ID_NUM; ++i)
  {
    receive_thread_list_[i].join();
    delete udp_receive_[i];
  }
  delete udp_send_;
} //~RoboMasterS1Bridge()

void RoboMasterS1Bridge::twistCommandCallback(const geometry_msgs::Twist::ConstPtr &twist_command)
{
  // UDP Send
  uint8_t send_data[19];
  send_data[0] = 0x55;
  send_data[1] = 0x13;
  send_data[2] = 0x04;
  send_data[3] = 0x00;
  appendCRC8CheckSum(send_data, 4);
  send_data[4] = 0x01; // Command Number
  send_data[5] = uint8_t(int16_t(twist_command->linear.x * 100) >> 8);
  send_data[6] = uint8_t(int16_t(twist_command->linear.x * 100) & 0xFF);
  send_data[7] = uint8_t(int16_t(-twist_command->linear.y * 100) >> 8);
  send_data[8] = uint8_t(int16_t(-twist_command->linear.y * 100) & 0xFF);
  send_data[9] = uint8_t(int16_t(twist_command->linear.z * 100) >> 8);
  send_data[10] = uint8_t(int16_t(twist_command->linear.z * 100) & 0xFF);
  send_data[11] = uint8_t(int16_t(twist_command->angular.x * 100) >> 8);
  send_data[12] = uint8_t(int16_t(twist_command->angular.x * 100) & 0xFF);
  send_data[13] = uint8_t(int16_t(twist_command->angular.y * 100) >> 8);
  send_data[14] = uint8_t(int16_t(twist_command->angular.y * 100) & 0xFF);
  send_data[15] = uint8_t(int16_t(twist_command->angular.z * 100) >> 8);
  send_data[16] = uint8_t(int16_t(twist_command->angular.z * 100) & 0xFF);
  send_data[17] = 0x00;
  send_data[18] = 0x00;
  appendCRC16CheckSum(send_data, 19);
  if (udp_send_->udp_send(send_data, 19) == -1)
  {
    ROS_WARN("Cannot send packet");
  }
  else
  {
    ROS_WARN_DELAYED_THROTTLE(10, "Send packet");
  }
}

void RoboMasterS1Bridge::udpReceiveThread(uint8_t can_id_num)
{
  while (ros::ok())
  {
    uint8_t buf[BUFFER_MAX];
    memset(buf, 0, sizeof(buf));
    int recv_msglen = udp_receive_[can_id_num]->udp_recv(buf, sizeof(buf));

    switch (can_id_num)
    {
    case 0: //0x201
      std_msgs::UInt8MultiArray debug_data;
      if (buf[1] == 0x1B)
      {
        for (int i = 0; i < recv_msglen; i++)
        {
          debug_data.data.push_back(buf[i]);
          if (debug_print_)
          {
            printf("0x%02X,", buf[i]);
          }
        }
        if (debug_print_)
        {
          printf("\r");
          printf("\n");
        }
      }
      robomaster_s1_bridge_rx_pub_.publish(debug_data);
    }
    break;
  case 1: //0x202
    break;
  case 2: //0x203
    break;
  case 3: //0x204
    break;
  default:
    break;
  }
}
}

int32_t main(int32_t argc, char **argv)
{
  // ROS
  ros::init(argc, argv, "robomaster_s1_bridge_node");
  RoboMasterS1Bridge node;

  ros::spin();

  std::cerr << "\nrobomaster_s1_bridge_node: Exiting...\n";
  return (EXIT_SUCCESS);
} // end main()
