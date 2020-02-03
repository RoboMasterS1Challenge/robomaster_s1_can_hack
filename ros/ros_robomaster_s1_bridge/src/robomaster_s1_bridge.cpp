#include "robomaster_s1_bridge.h"
#include "std_msgs/String.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int fd;

int open_serial(const char *device_name)
{
  int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  struct termios options;
  tcgetattr(fd, &options);
  options.c_cflag = B1000000 /*B115200*/ | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR | ICRNL;
  options.c_oflag = 0;
  options.c_lflag = 0;
  cfmakeraw(&options);
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);
  return fd;
}

// RoboMasterS1Bridge()
// Constructor
RoboMasterS1Bridge::RoboMasterS1Bridge()
    : nh_(""), pnh_("~")
{
  // ROS
  robomaster_s1_bridge_rx_sub_ = nh_.subscribe("/robomaster_s1/cmd_vel", 10, &RoboMasterS1Bridge::twistCommandCallback, this);
  robomaster_s1_bridge_rx_joy_sub_ = nh_.subscribe("/joy", 10, &RoboMasterS1Bridge::joyCommandCallback, this);
  robomaster_s1_bridge_rx_led_sub_ = nh_.subscribe("/robomaster_s1/led", 10, &RoboMasterS1Bridge::ledCommandCallback, this);
  robomaster_s1_bridge_rx_lose_sub_ = nh_.subscribe("/robomaster_s1/lose", 10, &RoboMasterS1Bridge::loseCommandCallback, this);
  robomaster_s1_bridge_tx_debug_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/robomaster_s1/debug_info", 10);
  robomaster_s1_bridge_tx_info_pub_ = nh_.advertise<ros_robomaster_s1_bridge::RoboMasterS1Info>("/robomaster_s1/robomaster_s1_info", 10);

  char device_name[] = "/dev/ttyACM0";
  fd = open_serial(device_name);

  // Parameters
  pnh_.param("debug_print", debug_print_, (int)0);
  // pnh_.param("ip_address", ip_address_, std::string("192.168.0.255"));

  // receive_port_list_[0] = 0x201 + 10000;
  // receive_port_list_[1] = 0x202 + 10000;
  // receive_port_list_[2] = 0x203 + 10000;
  // receive_port_list_[3] = 0x204 + 10000;
  // receive_port_list_[4] = 0x211 + 10000;
  // receive_port_list_[5] = 0x212 + 10000;
  // receive_port_list_[6] = 0x213 + 10000;
  // receive_port_list_[7] = 0x214 + 10000;
  // receive_port_list_[8] = 0x215 + 10000;
  // receive_port_list_[9] = 0x216 + 10000;
  // send_port_ = 0x200 + 10000;

  // udp_send_ = new broadcast_udp_send(ip_address_, send_port_);

  // for (int32_t i = 0; i < CAN_ID_NUM; ++i)
  // {
  //   // UDP Receive
  //   udp_receive_.emplace_back(new broadcast_udp_receive(receive_port_list_[i]));
  //   udp_receive_[i]->udp_bind();

  //   // Make receive thread
  //   receive_thread_list_.emplace_back(std::thread([this, i]() { this->udpReceiveThread(i); }));
  //   ROS_INFO("Start %d receive thread", i);
  // }
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
  close(fd);
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
  // if (udp_send_->udp_send(send_data, 19) == -1)
  // {
  //   ROS_WARN("Cannot send packet");
  // }
  // else
  // {
  //   ROS_WARN_DELAYED_THROTTLE(10, "Send packet");
  // }

  int ret = write(fd, send_data, 19);
  if (ret < 0)
  {
    ROS_ERROR("Serial Fail: cound not write");
  }
}

void RoboMasterS1Bridge::joyCommandCallback(const sensor_msgs::Joy::ConstPtr &joy_command)
{
  if (joy_command->buttons[11] == 1)
  {
    // UDP Send
    uint8_t send_data[8];
    send_data[0] = 0x55;
    send_data[1] = 0x08;
    send_data[2] = 0x04;
    send_data[3] = 0x00;
    appendCRC8CheckSum(send_data, 4);
    send_data[4] = 0x02; // Command Number
    send_data[5] = 0x01; //Blaster
    send_data[6] = 0x00;
    send_data[7] = 0x00;
    appendCRC16CheckSum(send_data, 8);
    // if (udp_send_->udp_send(send_data, 8) == -1)
    // {
    //   ROS_WARN("Cannot send packet");
    // }
    // else
    // {
    //   ROS_WARN_DELAYED_THROTTLE(10, "Send packet");
    // }
    int ret = write(fd, send_data, 8);
    if (ret < 0)
    {
      ROS_ERROR("Serial Fail: cound not write");
    }
  }
}

void RoboMasterS1Bridge::ledCommandCallback(const std_msgs::ColorRGBA::ConstPtr &led_command)
{
  // UDP Send
  uint8_t send_data[10];
  send_data[0] = 0x55;
  send_data[1] = 0x0A;
  send_data[2] = 0x04;
  send_data[3] = 0x00;
  appendCRC8CheckSum(send_data, 4);
  send_data[4] = 0x04;                    // Command Number
  send_data[5] = (uint8_t)led_command->r; //LED
  send_data[6] = (uint8_t)led_command->g; //LED
  send_data[7] = (uint8_t)led_command->b; //LED
  send_data[8] = 0x00;
  send_data[9] = 0x00;
  appendCRC16CheckSum(send_data, 10);
  // if (udp_send_->udp_send(send_data, 10) == -1)
  // {
  //   ROS_WARN("Cannot send packet");
  // }
  // else
  // {
  //   ROS_WARN_DELAYED_THROTTLE(10, "Send packet");
  // }
  int ret = write(fd, send_data, 10);
  //ROS_INFO("0x%2X,0x%2X,0x%2X,0x%2X", send_data[0], send_data[1], send_data[2], send_data[3]);
  if (ret < 0)
  {
    ROS_ERROR("Serial Fail: cound not write");
  }
}
void RoboMasterS1Bridge::loseCommandCallback(const std_msgs::Bool::ConstPtr &lose_command)
{
  // UDP Send
  uint8_t send_data[8];
  send_data[0] = 0x55;
  send_data[1] = 0x08;
  send_data[2] = 0x04;
  send_data[3] = 0x00;
  appendCRC8CheckSum(send_data, 4);
  send_data[4] = 0x03;                        // Command Number
  send_data[5] = (uint8_t)lose_command->data; //LED
  send_data[6] = 0x00;
  send_data[7] = 0x00;
  appendCRC16CheckSum(send_data, 8);
  // if (udp_send_->udp_send(send_data, 8) == -1)
  // {
  //   ROS_WARN("Cannot send packet");
  // }
  // else
  // {
  //   ROS_WARN_DELAYED_THROTTLE(10, "Send packet");
  // }
  int ret = write(fd, send_data, 8);
  if (ret < 0)
  {
    ROS_ERROR("Serial Fail: cound not write");
  }
}

// void RoboMasterS1Bridge::udpReceiveThread(uint8_t can_id_num)
// {
//   while (ros::ok())
//   {
//     uint8_t buf[BUFFER_MAX];
//     memset(buf, 0, sizeof(buf));
//     int recv_msglen = udp_receive_[can_id_num]->udp_recv(buf, sizeof(buf));
//     std_msgs::UInt8MultiArray debug_data;

//     ros_robomaster_s1_bridge::RoboMasterS1Info robomaster_s1_info;
//     switch (can_id_num)
//     {
//     case 0: //0x201
//       if (buf[4] == 0x09 && (buf[5] == 0x17 || buf[5] == 0x18))
//       {
//         for (int i = 0; i < recv_msglen; i++)
//         {
//           debug_data.data.emplace_back(buf[i]);
//           if (debug_print_)
//           {
//             printf("0x%02X,", buf[i]);
//           }
//         }
//         if (debug_print_)
//         {
//           printf("\r");
//           printf("\n");
//           robomaster_s1_bridge_tx_debug_pub_.publish(debug_data);
//         }
//       }
//       break;
//     case 1: //0x202
//       // From Motion Controller
//       if (buf[1] == 0x3D && buf[4] == 0x03 && buf[5] == 0x09)
//       {
//         int flag = (buf[24] >> 7) & 0x01;
//         base_odom_yaw_ = ((((uint16_t)buf[24]) << 8) | (((uint16_t)buf[23]) << 0));
//         if (flag == 0)
//         {
//           int shift = (0x86 - ((buf[24] << 1) | (buf[23] >> 7)));
//           base_odom_yaw_ = ((1 << 7) | buf[23]) >> shift;
//           base_odom_yaw_ *= -1;
//         }
//         else
//         {
//           int shift = (0x186 - ((buf[24] << 1) | (buf[23] >> 7)));
//           base_odom_yaw_ = ((1 << 7) | buf[23]) >> shift;
//         }
//         if (buf[24] == 0 && buf[23] == 0)
//         {
//           base_odom_yaw_ = 0;
//         }

//         uint32_t data = buf[38];
//         data = (data << 8) | buf[37];
//         data = (data << 8) | buf[36];
//         data = (data << 8) | buf[35];
//         base_odom_yaw_raw_[0] = (int32_t)(data);
//         data = buf[42];
//         data = (data << 8) | buf[41];
//         data = (data << 8) | buf[40];
//         data = (data << 8) | buf[39];
//         base_odom_yaw_raw_[1] = (int32_t)(data);
//         data = buf[50];
//         data = (data << 8) | buf[49];
//         data = (data << 8) | buf[48];
//         data = (data << 8) | buf[47];
//         base_odom_yaw_raw_[2] = (int32_t)(data);
//         data = buf[54];
//         data = (data << 8) | buf[53];
//         data = (data << 8) | buf[52];
//         data = (data << 8) | buf[51];
//         base_odom_yaw_raw_[3] = (int32_t)(data);
//         //printf("%d, %d, %d, %d\n",base_odom_yaw_raw_[0],base_odom_yaw_raw_[1],base_odom_yaw_raw_[2],base_odom_yaw_raw_[3]);
//       }
//       if (buf[1] == 0x31 && buf[4] == 0x03 && buf[5] == 0x04)
//       {
//         uint32_t data = buf[24];
//         data = (data << 8) | buf[23];
//         data = (data << 8) | buf[22];
//         data = (data << 8) | buf[21];
//         wheel_odom_[0] = (int32_t)(data);
//         data = buf[28];
//         data = (data << 8) | buf[27];
//         data = (data << 8) | buf[26];
//         data = (data << 8) | buf[25];
//         wheel_odom_[1] = (int32_t)(data);
//         data = buf[32];
//         data = (data << 8) | buf[31];
//         data = (data << 8) | buf[30];
//         data = (data << 8) | buf[29];
//         wheel_odom_[2] = (int32_t)(data);
//         data = buf[36];
//         data = (data << 8) | buf[35];
//         data = (data << 8) | buf[34];
//         data = (data << 8) | buf[33];
//         wheel_odom_[3] = (int32_t)(data);
//         data = buf[40];
//         data = (data << 8) | buf[39];
//         data = (data << 8) | buf[38];
//         data = (data << 8) | buf[37];
//         wheel_odom_[4] = (int32_t)(data);
//         data = buf[44];
//         data = (data << 8) | buf[43];
//         data = (data << 8) | buf[42];
//         data = (data << 8) | buf[41];
//         wheel_odom_[5] = (int32_t)(data);
//         //printf("%d, %d, %d, %d, %d\n",wheel_odom_[0],wheel_odom_[1],wheel_odom_[2],wheel_odom_[3],wheel_odom_[4]);
//       }
//       break;
//     case 2: //0x203
//       // From Blaster Gimbal
//       if (buf[1] == 0x11 && buf[4] == 0x04 && buf[5] == 0x03)
//       {
//         // Yaw Angle
//         uint16_t data = buf[12];
//         data = (data << 8) | buf[11];
//         blaster_base_yaw_angle_ = -(int16_t)(data) / 10.0;
//         data = buf[14];
//         data = (data << 8) | buf[13];
//         blaster_map_yaw_angle_ = -(int16_t)(data) / 100.0;

//         //printf("%lf, %lf\n",blaster_base_yaw_angle_,blaster_map_yaw_angle_);
//       }
//       if (buf[1] == 0x16 && buf[4] == 0x04 && buf[5] == 0x09)
//       {
//         // Pitch Angle
//         uint32_t data = buf[14];
//         data = (data << 8) | buf[13];
//         data = (data << 8) | buf[12];
//         data = (data << 8) | buf[11];
//         blaster_map_pitch_angle_ = (int32_t)(data) / 20000000.0 * 30.0;
//         data = buf[18];
//         data = (data << 8) | buf[17];
//         data = (data << 8) | buf[16];
//         data = (data << 8) | buf[15];
//         blaster_base_pitch_angle_ = (int32_t)(data) / 20000000.0 * 30.0;
//         //printf("%lf, %lf\n", blaster_map_pitch_angle_, blaster_base_pitch_angle_);
//       }
//       robomaster_s1_info.blaster_base_yaw = blaster_base_yaw_angle_;
//       robomaster_s1_info.blaster_map_yaw = blaster_map_yaw_angle_;
//       robomaster_s1_info.blaster_base_pitch = blaster_base_pitch_angle_;
//       robomaster_s1_info.blaster_map_pitch = blaster_map_pitch_angle_;
//       robomaster_s1_bridge_tx_info_pub_.publish(robomaster_s1_info);
//       break;
//     case 3: //0x204
//       if (buf[4] == 0x17 && buf[5] == 0x09)
//       {
//         for (int i = 0; i < recv_msglen; i++)
//         {
//           debug_data.data.emplace_back(buf[i]);
//           if (debug_print_)
//           {
//             printf("0x%02X,", buf[i]);
//           }
//         }
//         if (debug_print_)
//         {
//           printf("\r");
//           printf("\n");
//         }
//       }
//       break;
//     default:
//       break;
//     }
//   }
// }

int32_t main(int32_t argc, char **argv)
{
  // ROS
  ros::init(argc, argv, "robomaster_s1_bridge_node");
  RoboMasterS1Bridge node;

  ros::spin();

  std::cerr << "\nrobomaster_s1_bridge_node: Exiting...\n";
  return (EXIT_SUCCESS);
} // end main()
