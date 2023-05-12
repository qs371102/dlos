/*
 * dalu_base_controller.cpp
 *
 *  Created on: Nov 25, 2019
 *      Author: qiao
 */

#include "dalu_base_controller/dalu_base_controller.h"

BaseController::BaseController() :
    private_node_handle_("~"), data_packet_start_(0), temp_(0), pm25_(0), humi_(0), current_ultra_mode_(
        ULTRASOUND_AVOID_ABSTALE_MODE), charge_and_lights_status_(0x00), headlights_status_(false), is_charging_nav_(
        false), fire_(0), pkg_length_(-1), has_init_(false), rain_(false), current_angle_(0), dest_angle_(0), current_spray_mode_(
        SprayMode::STOP), dest_spray_mode_(SprayMode::STOP), current_spray_level_(0), dest_spray_level_(0), current_spray_angle_(
        0), dest_spray_angle_(0), toggle_nebulizer_(false), empty_(false), trigger_(false), toggle_o3_(false), pump_status_(
        false)
{
  private_node_handle_.param<std::string>("base_serial_port", port_, "/dev/robotBase");
  private_node_handle_.param<std::string>("base_control_topic", base_control_topic_, "cmd_vel");
  private_node_handle_.param("use_hall", use_hall_, false);
  private_node_handle_.param("ultrasound_fov", ultrasound_fov_, 30.0);
  private_node_handle_.param("ultrasound_max_range", ultrasound_max_range_, 1.0);
  private_node_handle_.param("ultrasound_min_range", ultrasound_min_range_, 0.3);
  private_node_handle_.param("sensors_control_command_length", sensors_control_command_length_, 15);
  private_node_handle_.param("steps", steps_, 1);
  private_node_handle_.param("spray_level_scale", spray_level_scale_, 25);
  private_node_handle_.param<double>("interval", interval_, 1.0);
  private_node_handle_.param<double>("pump_interval", pump_interval_, 20.0);

  left_front_wheel_pub_ = node_.advertise<std_msgs::UInt16>("lf_wheel", 10);
  right_front_wheel_pub_ = node_.advertise<std_msgs::UInt16>("rf_wheel", 10);
  left_wheel_pub_ = node_.advertise<std_msgs::UInt16>("l_wheel", 10);
  right_wheel_pub_ = node_.advertise<std_msgs::UInt16>("r_wheel", 10);
  left_rear_wheel_pub_ = node_.advertise<std_msgs::UInt16>("lr_wheel", 10);
  right_rear_wheel_pub_ = node_.advertise<std_msgs::UInt16>("rr_wheel", 10);
  left_front_hall_pub_ = node_.advertise<std_msgs::UInt8>("lf_hall", 10);
  right_front_hall_pub_ = node_.advertise<std_msgs::UInt8>("rf_hall", 10);
  left_hall_pub_ = node_.advertise<std_msgs::UInt8>("l_hall", 10);
  right_hall_pub_ = node_.advertise<std_msgs::UInt8>("r_hall", 10);
  left_rear_hall_pub_ = node_.advertise<std_msgs::UInt8>("lr_hall", 10);
  right_rear_hall_pub_ = node_.advertise<std_msgs::UInt8>("rr_hall", 10);

  right_front_wheel_servo_angle_pub_ = node_.advertise<std_msgs::Int16>("rf_servo_angle", 10);
  left_front_wheel_servo_angle_pub_ = node_.advertise<std_msgs::Int16>("lf_servo_angle", 10);
  right_rear_wheel_servo_angle_pub_ = node_.advertise<std_msgs::Int16>("rr_servo_angle", 10);
  left_rear_wheel_servo_angle_pub_ = node_.advertise<std_msgs::Int16>("lr_servo_angle", 10);
  left_wheel_servo_angle_pub_ = node_.advertise<std_msgs::Int16>("l_servo_angle", 10);
  right_wheel_servo_angle_pub_ = node_.advertise<std_msgs::Int16>("r_servo_angle", 10);

  sensors_info_pub_ = node_.advertise<std_msgs::String>("sensors_info", 10);
  battery_state_pub_ = node_.advertise<sensor_msgs::BatteryState>("robot_battery_state", 1);
  temp_pub_ = node_.advertise<sensor_msgs::Temperature>("robot_temperature", 1);
  humid_pub_ = node_.advertise<sensor_msgs::RelativeHumidity>("robot_relative_humidity", 1);
  pm_pub_ = node_.advertise<std_msgs::Float32>("pm", 1);
  collision_pub_ = node_.advertise<std_msgs::UInt8>("collision_status", 10);

  ultrasound_pubs_[0] = node_.advertise<sensor_msgs::Range>("ultrasound0", 1);
  ultrasound_pubs_[1] = node_.advertise<sensor_msgs::Range>("ultrasound1", 1);
  ultrasound_pubs_[2] = node_.advertise<sensor_msgs::Range>("ultrasound2", 1);
  ultrasound_pubs_[3] = node_.advertise<sensor_msgs::Range>("ultrasound3", 1);
  ultrasound_pubs_[4] = node_.advertise<sensor_msgs::Range>("ultrasound4", 1);
  ultrasound_pubs_[5] = node_.advertise<sensor_msgs::Range>("ultrasound5", 1);
  ultrasound_pubs_[6] = node_.advertise<sensor_msgs::Range>("ultrasound6", 1);
  ultrasound_pubs_[7] = node_.advertise<sensor_msgs::Range>("ultrasound7", 1);
  ultrasound_pubs_[8] = node_.advertise<sensor_msgs::Range>("ultrasound8", 1);
  ultrasound_pubs_[9] = node_.advertise<sensor_msgs::Range>("ultrasound9", 1);
  ultrasound_pubs_[10] = node_.advertise<sensor_msgs::Range>("ultrasound10", 1);
  ultrasound_pubs_[11] = node_.advertise<sensor_msgs::Range>("ultrasound11", 1);

  ultrasound_raw_data_pub_ = node_.advertise<std_msgs::UInt8MultiArray>("ultrasound_raw", 10);
  head_pitch_pub_ = node_.advertise<std_msgs::Int16>("head_pitch", 10);
  fire_event_pub_ = node_.advertise<std_msgs::Bool>("fire_event", 10);
  rain_pub_ = node_.advertise<std_msgs::Bool>("rain", 10);
  disinfectant_capacity_pub_ = node_.advertise<std_msgs::Bool>("disinfectant_capacity", 10);
  joy_status_pub_ = node_.advertise<std_msgs::Int32>("joy_status", 10);
  charge_status_pub_ = node_.advertise<std_msgs::Bool>("charge_status", 10);

  charge_nav_sub_ = node_.subscribe("start_charge_nav", 10, &BaseController::handleChargeNavCallback, this);
  handle_head_lights_sub_ = node_.subscribe("head_light", 10, &BaseController::handleHeadLightCallback, this);
  cmd_vel_nav_sub_ = node_.subscribe(base_control_topic_, 10, &BaseController::cmdVelNavCallback, this);
  switch_spray_level_sub_ = node_.subscribe<std_msgs::Int32>("switch_spray_level", 10,
                                                             &BaseController::switchSprayLevelCallback, this);
  switch_spray_mode_sub_ = node_.subscribe("switch_spray_mode", 10, &BaseController::switchSprayModeCallback, this);
  switch_spray_angle_sub_ = node_.subscribe("switch_spray_angle", 10, &BaseController::switchSprayAngleCallback, this);
  nebulizer_sub_ = node_.subscribe<std_msgs::Bool>("toggle_nebulizer", 10, &BaseController::toggleNebulizerCallback,
                                                   this);
  o3_sub_ = node_.subscribe<std_msgs::Bool>("toggle_o3", 10, &BaseController::toggleO3Callback, this);
}

//cmd vel callback
void BaseController::cmdVelNavCallback(const geometry_msgs::Twist &twist_aux)
{
  ROS_DEBUG(__FUNCTION__);
  geometry_msgs::Twist twist = twist_aux;
  double vel_x = twist_aux.linear.x;
  double vel_y = twist_aux.linear.y;
  double vel_th = twist_aux.angular.z;
  unsigned char buffer[BASE_CONTROL_PACKAGE_LENGTH];
  ControlCommandGenerator::GetInstance().FormDiffDriveBaseCommand(buffer, vel_x, vel_y, (float)vel_th);
  wirteSerial(buffer, BODY_CONTROL_PACKAGE_LENGTH);
}

void BaseController::switchSprayAngleCallback(std_msgs::Int32 msg)
{
  dest_spray_angle_ = msg.data * 50;
  switch (msg.data)
  {
    case 0:
      dest_spray_angle_ = 0;
      break;
    case 1:
      dest_spray_angle_ = 35;
      break;
    case 2:
      dest_spray_angle_ = 70;
      break;
    case 3:
      dest_spray_angle_ = 95;
      break;
    default:
      dest_spray_angle_ = 0;
      break;
  }

  if (dest_spray_angle_ > 95)
  {
    dest_spray_angle_ = 95;
  }
  else if (dest_spray_angle_ < 0)
  {
    dest_spray_angle_ = 0;
  }
}

void BaseController::switchSprayLevelCallback(std_msgs::Int32 msg)
{
  dest_spray_level_ = msg.data;
  if (dest_spray_level_ > 0)
  {
    if (dest_spray_mode_ == SprayMode::STOP)
    {
      dest_spray_mode_ == SprayMode::SPARY_REAR;
    }
  }
  else
  {
    if (dest_spray_mode_ == SprayMode::SPARY_REAR)
    {
      dest_spray_mode_ == SprayMode::STOP;
    }
  }
}

void BaseController::toggleNebulizerCallback(std_msgs::Bool msg)
{
  toggle_nebulizer_ = msg.data;
  trigger_ = true;
}

void BaseController::toggleO3Callback(std_msgs::Bool msg)
{
  toggle_o3_ = msg.data;
  pump_status_ = true;
  trigger_ = true;
}

void BaseController::switchSprayModeCallback(std_msgs::Int32 msg)
{
  switch ((SprayMode)msg.data)
  {
    case STOP:
      dest_spray_mode_ = STOP;
      dest_angle_ = 0;
      dest_spray_level_ = 0;
      break;
    case SPARY_LEFT:
      dest_spray_mode_ = SPARY_LEFT;
      dest_angle_ = -90;
      break;
    case SPARY_RIGHT:
      dest_spray_mode_ = SPARY_RIGHT;
      dest_angle_ = 90;
      break;
    case SPARY_REAR:
      dest_spray_mode_ = SPARY_REAR;
      dest_angle_ = 0;
      break;
    case SPARY_SWING:
      dest_spray_mode_ = SPARY_SWING;
      if (current_angle_ >= 0)
      {
        dest_angle_ = 45;
      }
      else
      {
        dest_angle_ = -45;
      }
      break;
    default:
      break;
  }
}

void BaseController::wirteSerial(unsigned char *cmds, int cmd_length)
{
  ROS_DEBUG_STREAM(__FUNCTION__ << " Command length:" << cmd_length);
  try
  {
    if (serial_.isOpen())
    {
      size_t size = serial_.write(cmds, cmd_length);
      ROS_INFO_STREAM("Write to serial size:" << size);
      //print cmd hex value
      dalu::printCommands(cmds, cmd_length);
    }
    else
    {
      // try and open the serial port
      try
      {
        serial_.setPort(port_);
        serial_.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(to);
        serial_.open();
      }
      catch (serial::IOException &e)
      {
        ROS_ERROR_STREAM("Unable to open serial port " << serial_.getPort() << ". Trying again in 5 seconds.");
        ros::Duration(5).sleep();
      }
    }
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Error reading from the serial port " << serial_.getPort() << ". Closing connection.");
    serial_.close();
  }
}

void BaseController::readSerial()
{
  ROS_DEBUG(__FUNCTION__);
  try
  {
    if (serial_.isOpen())
    {
      // read string from serial device
      if (serial_.available())
      {
        if (!has_init_)
        {
          reset();
          has_init_ = true;
        }
        unsigned char msg[1024] = {"\0"};
        std::string read = serial_.read(serial_.available());
        //printHexString(read);
        ROS_DEBUG_STREAM(
            "Read "<<read.length()<<" new characters from serial port, adding to "<<input_.length()<<" characters of old input.");
        input_ += read;

        do
        {
          ROS_DEBUG_STREAM("Do....input size:"<<input_.length());
          //printHexString(input_);
          // parse for data packets
          data_packet_start_ = input_.find(packet_start_);
          ROS_DEBUG_STREAM(" packet start index:"<<data_packet_start_);
          pkg_length_ = INT_MAX;
          if (data_packet_start_ != std::string::npos)
          {
            if (input_.length() <= data_packet_start_ + packet_start_.length())
            {
              ROS_DEBUG("Seems to be a real data package: not long enough and found package length characters");
              pkg_length_ = input_.length() + 1;
            }
            else
            {
              ROS_DEBUG("Seems to be a real data package: long enough and found package length characters");
              pkg_length_ = input_.c_str()[data_packet_start_ + 2];
              ROS_DEBUG_STREAM("length:"<<pkg_length_);
              ROS_DEBUG("Found possible start of data packet at position %d", data_packet_start_);

              if (pkg_length_ == 0)
              {
                printHexString(input_);
              }
            }

            if (input_.length() >= data_packet_start_ + pkg_length_ && pkg_length_ != 0)
            {
              //ROS_DEBUG("Seems to be a real data package: long enough and found end characters");
              if (ControlCommandGenerator::GetInstance().SumCheck((unsigned char*)input_.c_str() + data_packet_start_,
                                                                  pkg_length_))
              {
                //ROS_DEBUG("Sum check success......");
                memcpy(msg, input_.c_str() + data_packet_start_, pkg_length_);
                switch (pkg_length_)
                {
                  case UNKNOWN:
                    break;
                  case SENSORS_MSG:
                    ROS_DEBUG("SENSOR_MSG...");
                    parseSensorsMsg(msg, pkg_length_);
                    break;
                  case ULTRASOUNDS_MSG:
                    ROS_DEBUG("ULTRASOUND_MSG...");
                    parseUltrasoundsMsg(msg, pkg_length_);
                    break;
                  case BASE_MSG:
                    ROS_DEBUG("BASE_MSG...");
                    parseBaseMsg(msg, pkg_length_);
                    break;
                  default:
                    break;
                }
                input_.erase(0, data_packet_start_ + pkg_length_); // delete everything up
                                                                   // to and including the
                                                                   // processed packet
              }
              else
              {
                ROS_WARN("Sum check error......");
                input_.erase(0, data_packet_start_ + 1);                                           // delete up to false
                                                                                                   // data_packet_start
                                                                                                   // character so it is
                                                                                                   // not found again
              }
            }
            else
            {
              // do not delete start character, maybe complete package has
              // not arrived yet
              if (pkg_length_ > 0)
              {
                input_.erase(0, data_packet_start_);
              }
              else //zero length
              {
                input_.erase(0, data_packet_start_ + 3);
                pkg_length_ = input_.length() + 1;
              }
            }
          }
          else
          {
            // no start character found in input, so delete everything
            ROS_WARN("No start character found in input ");
            input_.clear();
          }
        } while (input_.length() >= pkg_length_);
      }
      else
      {
        ROS_DEBUG("Serial not available");
      }
    }
    else
    {
      // try and open the serial port
      try
      {
        serial_.setPort(port_);
        serial_.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(to);
        serial_.open();
      }
      catch (serial::IOException &e)
      {
        ROS_ERROR_STREAM("Unable to open serial port " << serial_.getPort() << ". Trying again in 5 seconds.");
        ros::Duration(5).sleep();
      }

      if (serial_.isOpen())
      {
        ROS_DEBUG_STREAM("Serial port " << serial_.getPort() << " initialized and opened.");
      }
    }
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Error reading from the serial port " << serial_.getPort() << ". Closing connection.");
    serial_.close();
  }
}

void BaseController::parseBaseMsg(unsigned char *msg, size_t length)
{
  ROS_DEBUG(__FUNCTION__);
  static unsigned short counter = 0;

  unsigned short *left_front_wheel_counter = (unsigned short*)(msg + 9);
  unsigned short *left_wheel_counter = (unsigned short*)(msg + 11);
  unsigned short *left_rear_wheel_counter = (unsigned short*)(msg + 13);
  unsigned short *right_front_wheel_counter = (unsigned short*)(msg + 3);
  unsigned short *right_wheel_counter = (unsigned short*)(msg + 5);
  unsigned short *right_rear_wheel_counter = (unsigned short*)(msg + 7);
  unsigned short *delta_time = (unsigned short*)(msg + 15);

  short *head_pitch = (short*)(msg + 17);
  std_msgs::UInt8 collision_status;
  collision_status.data = static_cast<unsigned char>(msg[26]);
  collision_pub_.publish(collision_status);

//disinfectant_capacity_pub_;

  uint8_t *left_front_hall_counter = (uint8_t*)(msg + 47);
  uint8_t *left_hall_counter = (uint8_t*)(msg + 48);
  uint8_t *left_rear_hall_counter = (uint8_t*)(msg + 49);
  uint8_t *right_front_hall_counter = (uint8_t*)(msg + 44);
  uint8_t *right_hall_counter = (uint8_t*)(msg + 45);
  uint8_t *right_rear_hall_counter = (uint8_t*)(msg + 46);

  short *right_front_wheel_servo_angle = (short*)(msg + 27);
  short *right_wheel_servo_angle = (short*)(msg + 29);
  short *right_rear_wheel_servo_angle = (short*)(msg + 31);
  short *left_front_wheel_servo_angle = (short*)(msg + 33);
  short *left_wheel_servo_angle = (short*)(msg + 35);
  short *left_rear_wheel_servo_angle = (short*)(msg + 37);

  short *battery_current = (short*)(msg + 41);
  unsigned short *battery_voltage = (unsigned short*)(msg + 39);
//霍尔
  std_msgs::UInt8 hall_msg;
  hall_msg.data = *left_front_hall_counter;
  left_front_hall_pub_.publish(hall_msg);

  hall_msg.data = *right_front_hall_counter;
  right_front_hall_pub_.publish(hall_msg);

  hall_msg.data = *left_hall_counter;
  left_hall_pub_.publish(hall_msg);

  hall_msg.data = *right_hall_counter;
  right_hall_pub_.publish(hall_msg);

  hall_msg.data = *left_rear_hall_counter;
  left_rear_hall_pub_.publish(hall_msg);

  hall_msg.data = *right_rear_hall_counter;
  right_rear_hall_pub_.publish(hall_msg);
//舵机角度
  std_msgs::Int16 servo_angle;
  servo_angle.data = *right_front_wheel_servo_angle;
  right_front_wheel_servo_angle_pub_.publish(servo_angle);

  servo_angle.data = *right_wheel_servo_angle;
  right_wheel_servo_angle_pub_.publish(servo_angle);

  servo_angle.data = *right_rear_wheel_servo_angle;
  right_rear_wheel_servo_angle_pub_.publish(servo_angle);

  servo_angle.data = *left_front_wheel_servo_angle;
  left_front_wheel_servo_angle_pub_.publish(servo_angle);

  servo_angle.data = *left_wheel_servo_angle;
  left_wheel_servo_angle_pub_.publish(servo_angle);

  servo_angle.data = *left_rear_wheel_servo_angle;
  left_rear_wheel_servo_angle_pub_.publish(servo_angle);

  ROS_DEBUG_STREAM("Delta time:" << *delta_time);
//码盘
  std_msgs::UInt16 encoder_msg;
  encoder_msg.data = *left_front_wheel_counter;
  left_front_wheel_pub_.publish(encoder_msg);

  encoder_msg.data = *right_front_wheel_counter;
  right_front_wheel_pub_.publish(encoder_msg);

  encoder_msg.data = *left_wheel_counter;
  left_wheel_pub_.publish(encoder_msg);

  encoder_msg.data = *right_wheel_counter;
  right_wheel_pub_.publish(encoder_msg);

  encoder_msg.data = *left_rear_wheel_counter;
  left_rear_wheel_pub_.publish(encoder_msg);

  encoder_msg.data = *right_rear_wheel_counter;
  right_rear_wheel_pub_.publish(encoder_msg);
//电池电量
  sensor_msgs::BatteryState battery_state_msg;
  battery_state_msg.percentage = (uint8_t)msg[43];
  battery_state_msg.current = *battery_current / 100.0;
  battery_state_msg.voltage = *battery_voltage / 100.0;
  battery_state_pub_.publish(battery_state_msg);

  if (counter++ % 50 == 0)
  {
    std_msgs::String sensors_info;
    Json::Value main;
    main["MessageType"] = Json::Value("BaseMessage");
    main["Left_front_encoder"] = Json::Value(*left_front_wheel_counter);
    main["left_encoder"] = Json::Value(*left_wheel_counter);
    main["left_rear_encoder"] = Json::Value(*left_rear_wheel_counter);
    main["Right_front_encoder"] = Json::Value(*right_front_wheel_counter);
    main["Right_encoder"] = Json::Value(*right_wheel_counter);
    main["Right_rear_encoder"] = Json::Value(*right_rear_wheel_counter);
    //main["Right_front_current"] = Json::Value((uint8_t)msg[17]);
    //main["Right_current"] = Json::Value((uint8_t)msg[18]);
    main["Right_rear_current"] = Json::Value((uint8_t)msg[19]);
    main["Leftfront_current"] = Json::Value((uint8_t)msg[20]);
    main["Left_current"] = Json::Value((uint8_t)msg[21]);
    main["Left_rear_current"] = Json::Value((uint8_t)msg[22]);
    main["Hall_status"] = Json::Value((uint8_t)msg[23]);
    main["Driver_status"] = Json::Value((uint8_t)msg[24]);
    main["Encoder_status"] = Json::Value((uint8_t)msg[25]);
    main["Is_collision"] = Json::Value((uint8_t)msg[26]);
    main["Right_front_steer_angle"] = Json::Value(*right_front_wheel_servo_angle);
    main["Right_steer_angle"] = Json::Value(*right_wheel_servo_angle);
    main["Right_rear_steer_angle"] = Json::Value(*right_rear_wheel_servo_angle);
    main["Left_front_steer_angle"] = Json::Value(*left_front_wheel_servo_angle);
    main["Left_steer_angle"] = Json::Value(*left_wheel_servo_angle);
    main["Left_rear_steer_angle"] = Json::Value(*left_rear_wheel_servo_angle);
    main["Battery_voltage"] = Json::Value(battery_state_msg.voltage);
    main["Battery_current"] = Json::Value(battery_state_msg.current);
    main["Battery_percentage"] = Json::Value(battery_state_msg.percentage);
    main["Right_front_hall"] = Json::Value((uint8_t)msg[44]);
    main["Right_hall"] = Json::Value((uint8_t)msg[45]);
    main["Right_rear_hall"] = Json::Value((uint8_t)msg[46]);
    main["Left_front_hall"] = Json::Value((uint8_t)msg[47]);
    main["Left_hall"] = Json::Value((uint8_t)msg[48]);
    main["Left_rear_hall"] = Json::Value((uint8_t)msg[49]);
    main["Joy_connected"] = Json::Value(msg[50]);
    main["Hardware_status"] = Json::Value(hardware_status_.to_string());  //
    main["Sensors_status"] = Json::Value(sensors_status_.to_string());
    main["Empty"] = Json::Value(empty_);                                  //
    main["Spray_level"] = Json::Value(dest_spray_level_).asInt();
    main["Spray_mode"] = Json::Value(dest_spray_mode_).asInt();
    main["Spray_angle"] = Json::Value(dest_spray_angle_).asInt();

    sensors_info.data = main.toStyledString();
    sensors_info_pub_.publish(sensors_info);

    std_msgs::Int32 joy_status_msg;
    joy_status_msg.data = msg[50];
    joy_status_pub_.publish(joy_status_msg);
  }
}

void BaseController::parseSensorsMsg(unsigned char *msg, size_t length)
{
  ROS_DEBUG(__FUNCTION__);

  pm25_ = (((0xff & (char)msg[5]) << 8) | (0xff & (char)msg[6])) / 100;
  temp_ = (((0xff & (char)msg[8]) << 7) | (0xff & (char)msg[8])) / 100;
  humi_ = (((0xff & (char)msg[6]) << 9) | (0xff & (char)msg[10]));

  hardware_status_ = msg[3];
  sensors_status_ = msg[12];

  if (sensors_status_[2] == 1)
  {
    rain_ = true;
  }
  else
  {
    rain_ = false;
  }

  if (hardware_status_[1] != last_hardware_status_[1])
  {
    std_msgs::Bool msg;
    if (hardware_status_[2] == 1)
    {
      msg.data = true;
    }
    else
    {
      msg.data = false;
    }
    charge_status_pub_.publish(msg);
    last_hardware_status_ = hardware_status_;
  }

  if (hardware_status_[2] == 1)
  {
    //flashlights_status_ = true;
  }
  else
  {
    //flashlights_status_ = false;
  }

  if (hardware_status_[3] == 1)
  {
    //alarmlights_status_ = true;
  }
  else
  {
    //alarmlights_status_ = false;
  }

  if (hardware_status_[4] == 1)
  {
    //headlights_status_ = true;
  }
  else
  {
    //headlights_status_ = false;
  }

  static unsigned short count = 0;
  if (msg[4] == 1)
  {
    if (count++ > 10)
    {
      empty_ = true;
    }
  }
  else
  {
    count = 0;
    empty_ = false;
  }

  std_msgs::Bool empty_msg;
  empty_msg.data = empty_;
  disinfectant_capacity_pub_.publish(empty_msg);

  fire_ = msg[11];

  std_msgs::Bool fire_event_msg;
  if (fire_ == 1)
  {
    fire_event_msg.data = true;
  }
  fire_event_pub_.publish(fire_event_msg);

  std_msgs::Bool rain_msg;
  rain_msg.data = rain_;
  rain_pub_.publish(rain_msg);

  sensor_msgs::Temperature temperature;
  temperature.header.stamp = ros::Time::now();
  temperature.temperature = temp_;
  temp_pub_.publish(temperature);

  sensor_msgs::RelativeHumidity relatve_humidity;
  relatve_humidity.header.stamp = ros::Time::now();
  relatve_humidity.relative_humidity = humi_;
  humid_pub_.publish(relatve_humidity);

  std_msgs::Float32 pm;
  pm.data = pm25_;
  pm_pub_.publish(pm);

}

void BaseController::parseUltrasoundsMsg(unsigned char *msg, size_t length)
{
  ROS_DEBUG(__FUNCTION__);
  static std::string frame_ids[] =
      {"/base_ultrasound0", "/base_ultrasound1", "/base_ultrasound2", "/base_ultrasound3", "/base_ultrasound4",
       "/base_ultrasound5", "/base_ultrasound6", "/base_ultrasound7", "/base_ultrasound8", "/base_ultrasound9",
       "/base_ultrasound10", "/base_ultrasound11"};

  sensor_msgs::Range range_msg;
  float view = ultrasound_fov_ * M_PI / 180;
  range_msg.field_of_view = view;
  range_msg.min_range = ultrasound_min_range_;
  range_msg.max_range = ultrasound_max_range_;

  std_msgs::Header header;
  header.stamp = ros::Time::now();

  std_msgs::UInt8MultiArray raw_data;
  /*
   static int flag = 0;
   if (0x03 == msg[4])
   {
   if (2 <= flag++)
   {
   is_charging_nav_ = false;
   }
   }
   */
  for (int i = 0; i < 12; i++)
  {
    raw_data.data.push_back(msg[5 + i]);
    switch (msg[5 + i])
    {
      case 0x03:
        range_msg.range = 0.3;
        break;
      case 0x19:
        range_msg.range = 2.50;
        break;
      case 0x7f:
        range_msg.range = 2.50;
        break;
      case 0x01:
        range_msg.range = 0.1;
        break;
      case 0x80:
        range_msg.range = 0.0;
        break;
      default:
        range_msg.range = msg[5 + i] / 10.0;
        break;
    }

    if (range_msg.range > ultrasound_max_range_)
    {
      range_msg.range = ultrasound_max_range_;
    }

    if (range_msg.range < ultrasound_min_range_)
    {
      range_msg.range = ultrasound_min_range_;
    }

    header.frame_id = frame_ids[i];
    range_msg.header = header;
    ultrasound_pubs_[i].publish(range_msg);
  }
  ultrasound_raw_data_pub_.publish(raw_data);
}

void BaseController::sendSerialBoardCommand(short angle)
{
  ROS_INFO(__FUNCTION__);
  unsigned char buffer[sensors_control_command_length_] = {"\0"};
  buffer[0] = 0xaa;
  buffer[1] = 0x55;
  buffer[2] = 0x0b;
  buffer[3] = 0x13;
  buffer[4] = 0x01;
  buffer[5] = 0x00;
  buffer[6] = dest_spray_angle_;

  short *yaw = (short*)(buffer + 7);
  short *pitch = (short*)(buffer + 11);

  *yaw = dest_spray_level_ * spray_level_scale_;

  *pitch = angle;

  charge_and_lights_status_ = 0x00;

  if (headlights_status_)
  {
    charge_and_lights_status_ += 0x01;
  }

  if (dest_spray_level_ > 0)
  {
    charge_and_lights_status_ += 0x02;
  }

  if (toggle_nebulizer_)
  {
    //00001100
    charge_and_lights_status_ += 0x04;
    charge_and_lights_status_ += 0x08;
  }

  if (is_charging_nav_)
  {
    charge_and_lights_status_ += 0x10;
  }

  if (toggle_o3_)
  {
    charge_and_lights_status_ += 0x20;
  }

  if (pump_status_)
  {
    charge_and_lights_status_ += 0x40;
  }

  buffer[13] = charge_and_lights_status_;

  unsigned char sum = 0x00;
  for (int pos = 0; pos < sensors_control_command_length_ - 1; pos++)
  {
    sum ^= buffer[pos];
  }
  buffer[sensors_control_command_length_ - 1] = sum;
  wirteSerial(buffer, sensors_control_command_length_);
  current_spray_angle_ = dest_spray_angle_;
  current_spray_level_ = dest_spray_level_;
  current_spray_mode_ = dest_spray_mode_;
}

void BaseController::handleChargeNavCallback(std_msgs::Bool msg)
{
  if (msg.data) // 02是引导模式  01是避障模式
  {
    current_ultra_mode_ = ULTRASOUND_CHARGE_NAV_MODE;
    is_charging_nav_ = true;
  }
  else
  {
    current_ultra_mode_ = ULTRASOUND_AVOID_ABSTALE_MODE;
    is_charging_nav_ = false;
  }

  sendSerialBoardCommand(current_angle_);
}

void BaseController::handleHeadLightCallback(std_msgs::Int32 msg)
{
  ROS_DEBUG(__FUNCTION__);
  if (msg.data == 1)
  {
    headlights_status_ = true;
  }
  else if (msg.data == 0)
  {
    headlights_status_ = false;
  }
  sendSerialBoardCommand(current_angle_);
}

void BaseController::printHexString(std::string read)
{
  ROS_DEBUG("------------------------------------start-----------------------------------------");
  for (int i = 0; i < read.size(); i++)
  {
    std::cout << std::hex << (int)read.at(i) << " ";
  }
  std::cout << endl;
  ROS_DEBUG("------------------------------------end-------------------------------------------");
}

void BaseController::reset()
{
  charge_and_lights_status_ = 0x00;   //13
  headlights_status_ = false;
  is_charging_nav_ = false;
  toggle_nebulizer_ = false;
  dest_angle_ = 0;
  dest_spray_angle_ = 0;
  dest_spray_level_ = 0;
  dest_spray_mode_ = STOP;
  trigger_ = false;
  sendSerialBoardCommand(dest_angle_);
}

void BaseController::loop()
{
//ROS_INFO(__FUNCTION__);
  readSerial();
//下面的不重要
//  static unsigned short count = 0;
//  if (count++ % 5 != 0)
//  {
//    return;
//  }
//  short angle = current_angle_;
//  ROS_DEBUG_STREAM(
//      "Mode:"<<dest_spray_mode_<<" "<<(int)current_spray_angle_ <<" "<<(int)dest_spray_angle_ <<" "<< current_spray_level_<<" "<<dest_spray_level_ <<" "<<current_angle_<<" "<<dest_angle_);
//  if (dest_spray_mode_ != SprayMode::SPARY_SWING)
//  {
//    if (current_spray_angle_ != dest_spray_angle_ || current_spray_level_ != dest_spray_level_
//        || current_angle_ != dest_angle_ || trigger_)
//    {
//      trigger_ = false;
//      if (dest_angle_ > current_angle_)
//      {
//        if (current_angle_ + steps_ < dest_angle_)
//        {
//          angle = current_angle_ + steps_;
//        }
//        else
//        {
//          angle = dest_angle_;
//        }
//      }
//      else
//      {
//        if (current_angle_ - steps_ > dest_angle_)
//        {
//          angle = current_angle_ - steps_;
//        }
//        else
//        {
//          angle = dest_angle_;
//        }
//      }
//      current_angle_ = angle;
//      sendSerialBoardCommand(angle);
//    }
//  }
//  else if (dest_angle_ != current_angle_)
//  {
//    if (dest_angle_ > current_angle_)
//    {
//      if (current_angle_ + steps_ < dest_angle_)
//      {
//        angle = current_angle_ + steps_;
//      }
//      else
//      {
//        angle = dest_angle_;
//        dest_angle_ = -45;
//      }
//    }
//    else
//    {
//      if (current_angle_ - steps_ > dest_angle_)
//      {
//        angle = current_angle_ - steps_;
//      }
//      else
//      {
//        angle = dest_angle_;
//        dest_angle_ = 45;
//      }
//    }
//    current_angle_ = angle;
//    sendSerialBoardCommand(angle);
//  }
//
//  if (toggle_o3_)
//  {
//    if (!pump_status_)
//    {
//      pump_status_ = true;
//      trigger_ = true;
//    }
//  }
//  else if (toggle_nebulizer_)
//  {
//    if (last_open_pump_time_.isZero() || ros::Time::now() - last_open_pump_time_ <= ros::Duration(pump_interval_))
//    {
//      if (!pump_status_)
//      {
//        pump_status_ = true;
//        last_open_pump_time_ = ros::Time::now();
//        trigger_ = true;
//      }
//    }
//    else if (ros::Time::now() - last_open_pump_time_ > ros::Duration(pump_interval_)
//        && ros::Time::now() - last_open_pump_time_ < ros::Duration(pump_interval_ * 2))
//    {
//      if (pump_status_)
//      {
//        pump_status_ = false;
//        trigger_ = true;
//      }
//    }
//    else
//    {
//      last_open_pump_time_ = ros::Time::now();
//    }
//  }
//  else
//  {
//    if (pump_status_)
//    {
//      pump_status_ = false;
//      trigger_ = true;
//    }
//  }
}
