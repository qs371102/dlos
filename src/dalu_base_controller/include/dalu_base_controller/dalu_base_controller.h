/*
 * dalu_base_controller.h
 *
 *  Created on: Nov 25, 2019
 *      Author: qiao
 */

#ifndef DALU_BASE_CONTROLLER_DALU_BASE_CONTROLLER_H_
#define DALU_BASE_CONTROLLER_DALU_BASE_CONTROLLER_H_

#include "dalu_robot/defaults.h"
#include "dalu_robot/control_command_generator.h"

#include <bitset>
#include <string>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>

#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>

using namespace dalu;

class BaseController
{
public:
  enum MessageType
  {
    UNKNOWN, SENSORS_MSG = 14, ULTRASOUNDS_MSG = 18, BASE_MSG = 54
  };

  enum ULTRASOUND_MODE
  {
    ULTRASOUND_AVOID_ABSTALE_MODE = 0, ULTRASOUND_CHARGE_NAV_MODE
  };

  enum SprayMode
  {
    STOP, SPARY_RIGHT, SPARY_LEFT, SPARY_REAR, SPARY_SWING
  };

  BaseController();

  void loop();

private:
  serial::Serial serial_;
  std::string port_;
  std::string base_control_topic_;
  ros::NodeHandle node_;
  ros::NodeHandle private_node_handle_;

  ros::Subscriber cmd_vel_nav_sub_;
  ros::Subscriber handle_head_lights_sub_;
  ros::Subscriber charge_nav_sub_;
  ros::Subscriber switch_spray_level_sub_;
  ros::Subscriber switch_spray_mode_sub_;
  ros::Subscriber switch_spray_angle_sub_;
  ros::Subscriber nebulizer_sub_;
  ros::Subscriber o3_sub_;

  ros::Publisher left_front_wheel_pub_;
  ros::Publisher right_front_wheel_pub_;
  ros::Publisher left_wheel_pub_;
  ros::Publisher right_wheel_pub_;
  ros::Publisher left_rear_wheel_pub_;
  ros::Publisher right_rear_wheel_pub_;
  ros::Publisher left_front_hall_pub_;
  ros::Publisher right_front_hall_pub_;
  ros::Publisher left_hall_pub_;
  ros::Publisher right_hall_pub_;
  ros::Publisher left_rear_hall_pub_;
  ros::Publisher right_rear_hall_pub_;

  ros::Publisher right_front_wheel_servo_angle_pub_;
  ros::Publisher left_front_wheel_servo_angle_pub_;
  ros::Publisher left_rear_wheel_servo_angle_pub_;
  ros::Publisher right_rear_wheel_servo_angle_pub_;
  ros::Publisher left_wheel_servo_angle_pub_;
  ros::Publisher right_wheel_servo_angle_pub_;

  ros::Publisher sensors_info_pub_;
  ros::Publisher collision_pub_;
  ros::Publisher battery_state_pub_;
  ros::Publisher temp_pub_;
  ros::Publisher humid_pub_;
  ros::Publisher pm_pub_;
  ros::Publisher fire_event_pub_;
  ros::Publisher ultrasound_pubs_[12];
  ros::Publisher ultrasound_raw_data_pub_;
  ros::Publisher head_pitch_pub_;
  ros::Publisher rain_pub_;
  ros::Publisher disinfectant_capacity_pub_;
  ros::Publisher joy_status_pub_;
  ros::Publisher charge_status_pub_;

  double ultrasound_fov_;
  double ultrasound_max_range_;
  double ultrasound_min_range_;

  std::string input_;
  int data_packet_start_;
  //0xaa, 0x55, 0x18
  const std::string packet_start_ = "\xaa\x55";

  bool use_hall_;
  int pkg_length_;
  int sensors_control_command_length_;

  std::bitset<8> hardware_status_;
  std::bitset<8> last_hardware_status_;
  std::bitset<8> sensors_status_;

  uint8_t charge_and_lights_status_;
  bool is_charging_nav_; //是否正在充电引导
  bool headlights_status_; //头灯状态
  bool trigger_;

  short temp_, humi_;
  unsigned short pm25_;
  uint8_t fire_;
  bool rain_;
  unsigned char current_ultra_mode_;

  int dest_spray_level_;
  int current_spray_level_;
  int steps_;
  int spray_level_scale_;
  int8_t dest_spray_angle_;
  int8_t current_spray_angle_;
  double interval_;
  short dest_angle_;
  short current_angle_;
  SprayMode dest_spray_mode_;
  SprayMode current_spray_mode_;
  bool has_init_;
  bool toggle_nebulizer_;
  bool toggle_o3_;
  bool pump_status_;
  ros::Time last_open_pump_time_;
  double pump_interval_;
  bool empty_;

  void wirteSerial(unsigned char *cmds, int cmd_length);
  void readSerial();
  void parseBaseMsg(unsigned char *msg, size_t length);
  void parseSensorsMsg(unsigned char *msg, size_t length);
  void parseUltrasoundsMsg(unsigned char *msg, size_t length);

  bool isValidData(unsigned short current_left_wheel_counter, unsigned short current_right_wheel_counter);

  //cmd vel callback
  void cmdVelNavCallback(const geometry_msgs::Twist &twist_aux);

  void robotStatusCallback(std_msgs::Int32 msg);

  void handleHeadLightCallback(std_msgs::Int32 msg);

  void handleChargeNavCallback(std_msgs::Bool msg);

  void switchSprayLevelCallback(std_msgs::Int32 msg);

  void switchSprayModeCallback(std_msgs::Int32 msg);

  void switchSprayAngleCallback(std_msgs::Int32 msg);

  void toggleNebulizerCallback(std_msgs::Bool msg);

  void toggleO3Callback(std_msgs::Bool msg);

  void reset();

  void printHexString(std::string read);

  void sendSerialBoardCommand(short angle);
};
#endif /* DALU_BASE_CONTROLLER_DALU_BASE_CONTROLLER_H_ */
