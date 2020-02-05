
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
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8MultiArray.h>
#include <ros_robomaster_s1_bridge/RoboMasterS1Info1.h>
#include <ros_robomaster_s1_bridge/RoboMasterS1Info2.h>
#include <ros_robomaster_s1_bridge/RoboMasterS1Info3.h>
#include <ros_robomaster_s1_bridge/RoboMasterS1Info4.h>
#include <ros_robomaster_s1_bridge/RoboMasterS1Info5.h>
#include "robomaster_s1_crc.h"

#define BUFFER_MAX 2048
#define BUFFER_SIZE 2048

class RoboMasterS1Bridge
{
private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // Publisher
  ros::Publisher robomaster_s1_bridge_tx_debug_pub_;
  ros::Publisher robomaster_s1_bridge_tx_info1_pub_;
  ros::Publisher robomaster_s1_bridge_tx_info2_pub_;
  ros::Publisher robomaster_s1_bridge_tx_info3_pub_;
  ros::Publisher robomaster_s1_bridge_tx_info4_pub_;
  ros::Publisher robomaster_s1_bridge_tx_info5_pub_;
  // Subscriber
  ros::Subscriber robomaster_s1_bridge_rx_sub_;
  ros::Subscriber robomaster_s1_bridge_rx_joy_sub_;
  ros::Subscriber robomaster_s1_bridge_rx_led_sub_;
  ros::Subscriber robomaster_s1_bridge_rx_lose_sub_;

  ros::Timer timer;

  int debug_print_;

  // RoboMaster S1 Data
  uint8_t command_buffer_[BUFFER_SIZE];
  int command_buffer_rp_;
  int command_buffer_wp_;
  int parseUsbData(uint8_t* in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t* out_data_size);

  // callback functions
  void twistCommandCallback(const geometry_msgs::Twist::ConstPtr &twist_command);
  void joyCommandCallback(const sensor_msgs::Joy::ConstPtr &joy_command);
  void loseCommandCallback(const std_msgs::Bool::ConstPtr &lose_command);
  void ledCommandCallback(const std_msgs::ColorRGBA::ConstPtr &led_command);
  void timer_callback(const ros::TimerEvent &);

public:
  // Constructor
  RoboMasterS1Bridge();

  // Destructor
  ~RoboMasterS1Bridge();
};
