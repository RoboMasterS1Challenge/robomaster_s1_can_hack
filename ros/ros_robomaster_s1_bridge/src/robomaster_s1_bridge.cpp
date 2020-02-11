#include "robomaster_s1_bridge.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
typedef union float_uint8 {
  float float_data;
  uint8_t uint8_data[4];
} float_uint8;

int fd_;

int open_serial(const char *device_name)
{
  int fd_ = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  struct termios options;
  tcgetattr(fd_, &options);
  options.c_cflag = B1000000 /*B115200*/ | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR | ICRNL;
  options.c_oflag = 0;
  options.c_lflag = 0;
  cfmakeraw(&options);
  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &options);
  return fd_;
}

// RoboMasterS1Bridge()
// Constructor
RoboMasterS1Bridge::RoboMasterS1Bridge()
    : nh_(""), pnh_("~")
{
  // ROS
  robomaster_s1_bridge_rx_sub_ = nh_.subscribe("/robomaster_s1/cmd_vel", 1, &RoboMasterS1Bridge::twistCommandCallback, this);
  robomaster_s1_bridge_rx_joy_sub_ = nh_.subscribe("/joy", 1, &RoboMasterS1Bridge::joyCommandCallback, this);
  robomaster_s1_bridge_rx_led_sub_ = nh_.subscribe("/robomaster_s1/led", 1, &RoboMasterS1Bridge::ledCommandCallback, this);
  robomaster_s1_bridge_rx_lose_sub_ = nh_.subscribe("/robomaster_s1/lose", 1, &RoboMasterS1Bridge::loseCommandCallback, this);
  robomaster_s1_bridge_tx_debug_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/robomaster_s1/debug_info", 1);
  robomaster_s1_bridge_tx_info1_pub_ = nh_.advertise<ros_robomaster_s1_bridge::RoboMasterS1Info1>("/robomaster_s1/robomaster_s1_info1", 1);
  robomaster_s1_bridge_tx_info2_pub_ = nh_.advertise<ros_robomaster_s1_bridge::RoboMasterS1Info2>("/robomaster_s1/robomaster_s1_info2", 1);
  robomaster_s1_bridge_tx_info3_pub_ = nh_.advertise<ros_robomaster_s1_bridge::RoboMasterS1Info3>("/robomaster_s1/robomaster_s1_info3", 1);
  robomaster_s1_bridge_tx_info4_pub_ = nh_.advertise<ros_robomaster_s1_bridge::RoboMasterS1Info4>("/robomaster_s1/robomaster_s1_info4", 1);
  robomaster_s1_bridge_tx_info5_pub_ = nh_.advertise<ros_robomaster_s1_bridge::RoboMasterS1Info5>("/robomaster_s1/robomaster_s1_info5", 1);

  char device_name[] = "/dev/ttyACM0";
  fd_ = open_serial(device_name);

  // Parameters
  pnh_.param("debug_print", debug_print_, (int)0);

  command_buffer_rp_ = 0;
  command_buffer_wp_ = 0;

  timer = nh_.createTimer(ros::Duration(0.001), &RoboMasterS1Bridge::timer_callback, this);
} //RoboMasterS1Bridge()

// ~RoboMasterS1Bridge()
// Destructor
RoboMasterS1Bridge::~RoboMasterS1Bridge()
{
  close(fd_);
} //~RoboMasterS1Bridge()

void RoboMasterS1Bridge::twistCommandCallback(const geometry_msgs::Twist::ConstPtr &twist_command)
{
  // Send
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

  int ret = write(fd_, send_data, 19);
  if (ret < 0)
  {
    ROS_ERROR("Serial Fail: cound not write");
  }
}

void RoboMasterS1Bridge::joyCommandCallback(const sensor_msgs::Joy::ConstPtr &joy_command)
{
  if (joy_command->buttons[11] == 1)
  {
    // Send
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

    int ret = write(fd_, send_data, 8);
    if (ret < 0)
    {
      ROS_ERROR("Serial Fail: cound not write");
    }
  }
}

void RoboMasterS1Bridge::ledCommandCallback(const std_msgs::ColorRGBA::ConstPtr &led_command)
{
  // Send
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

  int ret = write(fd_, send_data, 10);
  //ROS_INFO("0x%2X,0x%2X,0x%2X,0x%2X", send_data[0], send_data[1], send_data[2], send_data[3]);
  if (ret < 0)
  {
    ROS_ERROR("Serial Fail: cound not write");
  }
}
void RoboMasterS1Bridge::loseCommandCallback(const std_msgs::Bool::ConstPtr &lose_command)
{
  // Send
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

  int ret = write(fd_, send_data, 8);
  if (ret < 0)
  {
    ROS_ERROR("Serial Fail: cound not write");
  }
}

int RoboMasterS1Bridge::parseUsbData(uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size)
{
  int i;

  for (int i = 0; i < in_data_size; i++)
  {
    command_buffer_[command_buffer_wp_] = in_data[i];
    command_buffer_wp_++;
    command_buffer_wp_ %= BUFFER_SIZE;
  }

  int usb_in_buffer_size = (command_buffer_wp_ - command_buffer_rp_ + BUFFER_SIZE) % BUFFER_SIZE;

  // Data size check
  if (usb_in_buffer_size < 7)
  {
    return 0;
  }

  // Search Header
  int find_flag = 0;
  for (i = 0; i < usb_in_buffer_size - 7; i++)
  {
    if (command_buffer_[(command_buffer_rp_) % BUFFER_SIZE] == 0x55 && command_buffer_[(command_buffer_rp_ + 2) % BUFFER_SIZE] == 0x04)
    {
      find_flag = 1;
      break;
    }
    command_buffer_rp_++;
    command_buffer_rp_ %= BUFFER_SIZE;
    usb_in_buffer_size--;
  }

  if (find_flag == 0)
  {
    return 0;
  }

  // Check data length
  int send_data_size = command_buffer_[(command_buffer_rp_ + 1) % BUFFER_SIZE];
  if (send_data_size > usb_in_buffer_size)
  {
    // Not enough data
    return 0;
  }

  // Prepare send data
  uint8_t *send_data;
  send_data = (uint8_t *)malloc(sizeof(uint8_t) * send_data_size);

  for (i = 0; i < send_data_size; i++)
  {
    int buffer_p = (command_buffer_rp_ + i) % BUFFER_SIZE;
    send_data[i] = command_buffer_[buffer_p];
  }

  // Check header crc8
  if (!verifyCRC8CheckSum(send_data, 4))
  {
    // checksum error
    // skip header
    command_buffer_rp_++;
    command_buffer_rp_ %= BUFFER_SIZE;
    free(send_data);
    return 0;
  }

  // Check crc16
  if (!verifyCRC16CheckSum(send_data, send_data_size))
  {
    // checksum error
    // skip header
    command_buffer_rp_++;
    command_buffer_rp_ %= BUFFER_SIZE;
    free(send_data);
    return 0;
  }

  memcpy(out_data, send_data, send_data_size);
  *out_data_size = send_data_size;

  free(send_data);
  command_buffer_rp_ += send_data_size;
  command_buffer_rp_ %= BUFFER_SIZE;
  return 1;
}

void RoboMasterS1Bridge::timer_callback(const ros::TimerEvent &)
{
  uint8_t buf[255];
  uint8_t parsed_command[255];
  uint8_t parsed_command_size;

  int recv_data_size = read(fd_, buf, sizeof(buf));
  if (recv_data_size > 0)
  {
    //for(int i=0;i<buf[1];i++){
    //  printf("0x%02X,",buf[i]);
    //}
    //printf(",%d - %d\n",command_buffer_rp_,command_buffer_wp_);
    while (parseUsbData(buf, recv_data_size, parsed_command, &parsed_command_size))
    {
      recv_data_size = 0;
      //ROS_INFO("0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02d", parsed_command[0], parsed_command[1], parsed_command[2], parsed_command[3], parsed_command[4], parsed_command[5]);

      if (verifyCRC8CheckSum(parsed_command, 4) &&
          verifyCRC16CheckSum(parsed_command, parsed_command[1]))
      {
        ros_robomaster_s1_bridge::RoboMasterS1Info1 info1;
        ros_robomaster_s1_bridge::RoboMasterS1Info2 info2;
        ros_robomaster_s1_bridge::RoboMasterS1Info3 info3;
        ros_robomaster_s1_bridge::RoboMasterS1Info4 info4;
        ros_robomaster_s1_bridge::RoboMasterS1Info5 info5;
        uint32_t uint32_data;
        switch (parsed_command[4])
        {
        case 1:
          info1.seq = parsed_command[5]; // Command Counter
          float_uint8 gimbal_yaw_float_uint8;
          gimbal_yaw_float_uint8.uint8_data[0] = parsed_command[6];
          gimbal_yaw_float_uint8.uint8_data[1] = parsed_command[7];
          gimbal_yaw_float_uint8.uint8_data[2] = parsed_command[8];
          gimbal_yaw_float_uint8.uint8_data[3] = parsed_command[9];
          info1.motion_gimbal_yaw = gimbal_yaw_float_uint8.float_data;

          float_uint8 unknown_data[5];

          for(int i=0; i < 5; i++){
            unknown_data[i].uint8_data[0] = parsed_command[10 + i * 4];
            unknown_data[i].uint8_data[1] = parsed_command[11 + i * 4];
            unknown_data[i].uint8_data[2] = parsed_command[12 + i * 4];
            unknown_data[i].uint8_data[3] = parsed_command[13 + i * 4];
            info1.unknown_data.push_back(unknown_data[i].float_data);
          }
          robomaster_s1_bridge_tx_info1_pub_.publish(info1);
          break;
        case 2:
          info2.seq = parsed_command[5]; // Command Counter
          float_uint8 orientation[4];
          for(int i=0; i < 4; i++){
            orientation[i].uint8_data[0] = parsed_command[6 + i * 4];
            orientation[i].uint8_data[1] = parsed_command[7 + i * 4];
            orientation[i].uint8_data[2] = parsed_command[8 + i * 4];
            orientation[i].uint8_data[3] = parsed_command[9 + i * 4];
          }
          info2.orientation.w = orientation[0].float_data;
          info2.orientation.x = orientation[1].float_data;
          info2.orientation.y = orientation[2].float_data;
          info2.orientation.z = orientation[3].float_data;

          int32_t int_data1;
          uint32_data = parsed_command[25];
          uint32_data = (uint32_data << 8) | parsed_command[24];
          uint32_data = (uint32_data << 8) | parsed_command[23];
          uint32_data = (uint32_data << 8) | parsed_command[22];
          int_data1 = (int32_t)uint32_data;
          info2.unknown_data.push_back(int_data1);

          int32_t int_data2;
          uint32_data = parsed_command[29];
          uint32_data = (uint32_data << 8) | parsed_command[28];
          uint32_data = (uint32_data << 8) | parsed_command[27];
          uint32_data = (uint32_data << 8) | parsed_command[26];
          int_data2 = (int32_t)uint32_data;
          info2.unknown_data.push_back(int_data2);

          info2.battery_soc = parsed_command[30];

          robomaster_s1_bridge_tx_info2_pub_.publish(info2);
          break;
        case 3:
          info3.seq = parsed_command[5]; // Command Counter

          // Unit is RPM
          for(int i=0; i < 4; i++){
            uint16_t data16 = parsed_command[7 + i * 2];
            data16 = (data16 << 8) | parsed_command[6 + i * 2];
            info3.wheel_angular_velocity.push_back((int16_t)data16);
          }
          for(int i=0; i < 4; i++){
            uint16_t data16 = parsed_command[15 + i * 2];
            data16 = (data16 << 8) | parsed_command[14 + i * 2];
            data16 = data16 << 1;
            int16_t int_data16 = (int16_t)data16;
            info3.wheel_angle.push_back(int_data16 / 32767.0 * 180.0);
          }
          for(int i=0; i < 4; i++){
            uint32_t data32 = parsed_command[25 + i * 4];
            data32 = (data32 << 8) | parsed_command[24 + i * 4];
            data32 = (data32 << 8) | parsed_command[23 + i * 4];
            data32 = (data32 << 8) | parsed_command[22 + i * 4];
            info3.m_bus_update_count.push_back(data32);
          }

          float_uint8 mag_data;
          mag_data.uint8_data[0] = parsed_command[38];
          mag_data.uint8_data[1] = parsed_command[39];
          mag_data.uint8_data[2] = parsed_command[40];
          mag_data.uint8_data[3] = parsed_command[41];
          info3.imu_mag_xy.x = mag_data.float_data;
          mag_data.uint8_data[0] = parsed_command[42];
          mag_data.uint8_data[1] = parsed_command[43];
          mag_data.uint8_data[2] = parsed_command[44];
          mag_data.uint8_data[3] = parsed_command[45];
          info3.imu_mag_xy.y = mag_data.float_data;

          float_uint8 linear_acceleration_data;
          linear_acceleration_data.uint8_data[0] = parsed_command[50];
          linear_acceleration_data.uint8_data[1] = parsed_command[51];
          linear_acceleration_data.uint8_data[2] = parsed_command[52];
          linear_acceleration_data.uint8_data[3] = parsed_command[53];
          info3.imu_linear_acceleration.x = linear_acceleration_data.float_data;
          linear_acceleration_data.uint8_data[0] = parsed_command[54];
          linear_acceleration_data.uint8_data[1] = parsed_command[55];
          linear_acceleration_data.uint8_data[2] = parsed_command[56];
          linear_acceleration_data.uint8_data[3] = parsed_command[57];
          info3.imu_linear_acceleration.y = linear_acceleration_data.float_data;
          linear_acceleration_data.uint8_data[0] = parsed_command[58];
          linear_acceleration_data.uint8_data[1] = parsed_command[59];
          linear_acceleration_data.uint8_data[2] = parsed_command[60];
          linear_acceleration_data.uint8_data[3] = parsed_command[61];
          info3.imu_linear_acceleration.z = linear_acceleration_data.float_data;
          
          float_uint8 angular_velocity_data;
          angular_velocity_data.uint8_data[0] = parsed_command[62];
          angular_velocity_data.uint8_data[1] = parsed_command[63];
          angular_velocity_data.uint8_data[2] = parsed_command[64];
          angular_velocity_data.uint8_data[3] = parsed_command[65];
          info3.imu_angular_velocity.x = angular_velocity_data.float_data;
          angular_velocity_data.uint8_data[0] = parsed_command[66];
          angular_velocity_data.uint8_data[1] = parsed_command[67];
          angular_velocity_data.uint8_data[2] = parsed_command[68];
          angular_velocity_data.uint8_data[3] = parsed_command[69];
          info3.imu_angular_velocity.y = angular_velocity_data.float_data;
          angular_velocity_data.uint8_data[0] = parsed_command[70];
          angular_velocity_data.uint8_data[1] = parsed_command[71];
          angular_velocity_data.uint8_data[2] = parsed_command[72];
          angular_velocity_data.uint8_data[3] = parsed_command[73];
          info3.imu_angular_velocity.z = angular_velocity_data.float_data;
          
          float_uint8 imu_rpy_data;
          imu_rpy_data.uint8_data[0] = parsed_command[78];
          imu_rpy_data.uint8_data[1] = parsed_command[79];
          imu_rpy_data.uint8_data[2] = parsed_command[80];
          imu_rpy_data.uint8_data[3] = parsed_command[81];
          info3.imu_rpy.x = imu_rpy_data.float_data;
          imu_rpy_data.uint8_data[0] = parsed_command[82];
          imu_rpy_data.uint8_data[1] = parsed_command[83];
          imu_rpy_data.uint8_data[2] = parsed_command[84];
          imu_rpy_data.uint8_data[3] = parsed_command[85];
          info3.imu_rpy.y = imu_rpy_data.float_data;
          imu_rpy_data.uint8_data[0] = parsed_command[86];
          imu_rpy_data.uint8_data[1] = parsed_command[87];
          imu_rpy_data.uint8_data[2] = parsed_command[88];
          imu_rpy_data.uint8_data[3] = parsed_command[89];
          info3.imu_rpy.z = imu_rpy_data.float_data;

          robomaster_s1_bridge_tx_info3_pub_.publish(info3);
          break;
        case 4:
          info4.seq = parsed_command[5]; // Command Counter
          float_uint8 gimbal_base_yaw_float_uint8;
          float_uint8 gimbal_map_yaw_float_uint8;
          gimbal_base_yaw_float_uint8.uint8_data[0] = parsed_command[6];
          gimbal_base_yaw_float_uint8.uint8_data[1] = parsed_command[7];
          gimbal_base_yaw_float_uint8.uint8_data[2] = parsed_command[8];
          gimbal_base_yaw_float_uint8.uint8_data[3] = parsed_command[9];
          info4.gimbal_base_yaw = gimbal_base_yaw_float_uint8.float_data;
          gimbal_map_yaw_float_uint8.uint8_data[0] = parsed_command[10];
          gimbal_map_yaw_float_uint8.uint8_data[1] = parsed_command[11];
          gimbal_map_yaw_float_uint8.uint8_data[2] = parsed_command[12];
          gimbal_map_yaw_float_uint8.uint8_data[3] = parsed_command[13];
          info4.gimbal_map_yaw = gimbal_map_yaw_float_uint8.float_data;
          robomaster_s1_bridge_tx_info4_pub_.publish(info4);

          break;
        case 5:
          info5.seq = parsed_command[5]; // Command Counter
          float_uint8 gimbal_base_pitch_float_uint8;
          float_uint8 gimbal_map_pitch_float_uint8;
          gimbal_base_pitch_float_uint8.uint8_data[0] = parsed_command[6];
          gimbal_base_pitch_float_uint8.uint8_data[1] = parsed_command[7];
          gimbal_base_pitch_float_uint8.uint8_data[2] = parsed_command[8];
          gimbal_base_pitch_float_uint8.uint8_data[3] = parsed_command[9];
          info5.gimbal_base_pitch = gimbal_base_pitch_float_uint8.float_data;
          gimbal_map_pitch_float_uint8.uint8_data[0] = parsed_command[10];
          gimbal_map_pitch_float_uint8.uint8_data[1] = parsed_command[11];
          gimbal_map_pitch_float_uint8.uint8_data[2] = parsed_command[12];
          gimbal_map_pitch_float_uint8.uint8_data[3] = parsed_command[13];
          info5.gimbal_map_pitch = gimbal_map_pitch_float_uint8.float_data;
          robomaster_s1_bridge_tx_info5_pub_.publish(info5);

          break;
        default:
          break;
        }
      }
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
