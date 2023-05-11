/*
 * robot_status.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: qiao
 */

#include "dalu_robot/defaults.h"
#include <iostream>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>
#include <sound_play/sound_play.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8MultiArray.h>
#include <stdlib.h>
#include <string>
#include <ros/package.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include <std_srvs/Trigger.h>

using namespace std;

const std::string kDefaultDataFilePath = "/home/robot/.ros/data.txt";

class RobotStatus
{
public:
  struct ComputerInfo
  {
    std::string ip_info_;
    std::string hardware_info_;
    bool has_init_;
    string enp2s0_;
    string wlp3s0_;

    ComputerInfo() :
        has_init_(false)
    {
      init();
    }

    void init()
    {
      if (!has_init_)
      {
        getIPInfo();
        getHardwareInfo();
      }
      has_init_ = true;
    }

    void getIPInfo()
    {
      if (!has_init_)
      {
        enp2s0_ = getIPAddressByInterface("enp2s0");
        wlp3s0_ = getIPAddressByInterface("wlp3s0");
      }
    }

    string getHardwareInfo()
    {
      if (!has_init_)
      {

      }
      return hardware_info_;
    }

    string getIPAddressByInterface(string interface)
    {
      char ip_address[16];
      int fd;
      struct ifreq ifr;

      /*AF_INET - to define network interface IPv4*/
      /*Creating soket for it.*/
      fd = socket(AF_INET, SOCK_DGRAM, 0);

      /*AF_INET - to define IPv4 Address type.*/
      ifr.ifr_addr.sa_family = AF_INET;

      /*eth0 - define the ifr_name - port name
       where network attached.*/
      memcpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);

      /*Accessing network interface information by
       passing address using ioctl.*/
      ioctl(fd, SIOCGIFADDR, &ifr);
      /*closing fd*/
      close(fd);
      /*Extract IP Address*/
      strcpy(ip_address, inet_ntoa(((struct sockaddr_in*)&ifr.ifr_addr)->sin_addr));

      //printf("System IP Address is: %s\n", ip_address);
      return string(ip_address);
    }
  };

  struct NodeStatus
  {
    bool aiui_status_;
    bool face_identification_status_;
    bool car_plate_detect_status_;
  };

  class RobotInfo
  {
  public:
    RobotInfo() :
        robot_name_("Andi"), robot_sn_(""), wheel_track_(0.57)
    {
    }

    std::string getSN()
    {
      return robot_sn_;
    }

    void setSN(std::string sn)
    {
      robot_sn_ = sn;
    }

    double getWheelTrack()
    {
      return wheel_track_;
    }

  private:
    string robot_name_;
    string robot_sn_;
    // 长宽高 轮距
    double wheel_track_;
  };

  struct SensorsInfo
  {
    SensorsInfo() :
        pm_(0.0), temperature_(0.0), humidity_(0.0), inf_camera_max_temperature_(0.0), inf_camera_min_temperature_(0.0), battery_level_(
            0.0), gas_a_(0.0), gas_b_(0.0), gas_c_(0.0), gas_d_(0.0), combustible_gas_value_(0.0), co_(0.0)
    {
    }

    double pm_;
    double temperature_;
    double humidity_;
    double inf_camera_max_temperature_;
    double inf_camera_min_temperature_;
    double battery_level_;
    double gas_a_;
    double gas_b_;
    double gas_c_;
    double gas_d_;
    double co_;
    double combustible_gas_value_;
    std::vector<double> sonar_;
  };

  struct RobotMotion
  {
    double latitude_;
    double longitude_;
    double lx_;
    double az_;
    dalu::Deriections deriection_;
  };

  RobotStatus() :
      private_nh_("~"), robot_info_(), sensors_info_(), status_(dalu::STATUS_IDLE), alarm_status_(false), broadcast_status_(
          false), total_odom_(0.0), total_run_time_(0.0), has_init_(false), data_file_path_(kDefaultDataFilePath), loop_mode_(
          false), need_charge_(false), is_mapping_(false), is_navigation_(false), is_charging_(false), is_joy_connect_(
          false), is_charging_nav_(false), broadcast_file_path_(""), last_broadcast_file_path_(""), played_seconds_(0)
  {
    private_nh_.param("low_power_auto_charge", low_power_auto_charge_, false);
    private_nh_.param("auto_charge_battery_percentage", auto_charge_battery_percentage_, 20.0);
    private_nh_.param("data_file_path", data_file_path_, kDefaultDataFilePath);

    getLastValidData();

    former_warnning_battery_percentage_ = auto_charge_battery_percentage_;
    //
    alarm_path_ = ros::package::getPath("dalu_robot") + "/data/alarm.mp3";

    status_pub_ = node_.advertise<std_msgs::Int32>("robot_status", 1);
    sensors_info_pub_ = node_.advertise<std_msgs::String>("sensors_info", 1);
    need_charge_pub_ = node_.advertise<std_msgs::Bool>("need_charge", 1);
    gps_converted_pub_ = node_.advertise<sensor_msgs::NavSatFix>("fix/gcj02", 10);

    temperature_sub_ = node_.subscribe("robot_temperature", 1, &RobotStatus::temperatureCallback, this);
    relativeHumidity_sub_ = node_.subscribe("robot_relative_humidity", 1, &RobotStatus::relativeHumidityCallback, this);
    gps_sub_ = node_.subscribe("fix", 10, &RobotStatus::gpsCallback, this);
    odom_sub_ = node_.subscribe("wheel_odom", 10, &RobotStatus::odomCallback, this);
    sonar_sub_ = node_.subscribe("sensor_reader", 100, &RobotStatus::sonarCallback, this);
    combustible_gas_sub_ = node_.subscribe("combustible_gas", 10, &RobotStatus::combustibleGasCallback, this);

    pm_sub_ = node_.subscribe("pm", 100, &RobotStatus::pmCallback, this);
    gasa_sub_ = node_.subscribe("mq6", 100, &RobotStatus::gasaCallback, this);
    gasb_sub_ = node_.subscribe("mq4", 100, &RobotStatus::gasbCallback, this);
    gasc_sub_ = node_.subscribe("mq135", 100, &RobotStatus::gascCallback, this);
    gasd_sub_ = node_.subscribe("mq3", 100, &RobotStatus::gasdCallback, this);
    co_sub_ = node_.subscribe("co", 10, &RobotStatus::coCallback, this);
    emergency_sub_ = node_.subscribe("emergency_button", 1, &RobotStatus::emergencyCallback, this);

    //TODO 1,和emergency_button要互斥。2，emergency_button的默认播放需要可以更改
    play_sound_file_sub_ = node_.subscribe("play_alarm", 1, &RobotStatus::playAlarmCallback, this);

    patrol_status_sub_ = node_.subscribe<std_msgs::Bool>("dalu_robot/patrol_status", 1,
                                                         &RobotStatus::patrolStatusCallback, this);
    battery_state_sub_ = node_.subscribe("robot_battery_state", 1, &RobotStatus::batteryStateCallback, this);
    charge_nav_status_sub_ = node_.subscribe("start_charge_nav", 1, &RobotStatus::chargeNavStatusCallback, this);
    nav_control_sub_ = node_.subscribe("dalu_robot/slam/nav", 1, &RobotStatus::navControlCallback, this);
    mapping_control_sub_ = node_.subscribe("dalu_robot/slam/mapping", 1, &RobotStatus::mappingControlCallback, this);
    map_save_sub_ = node_.subscribe("dalu_robot/slam/savemap", 1, &RobotStatus::mapSaveControlCallback, this);
    joy_status_sub_ = node_.subscribe<std_msgs::Int32>("joy_status", 10, &RobotStatus::joyStatusCallback, this);

    if (ros::service::waitForService("aiui_status", ros::Duration(5.0f)))
    {
      aiui_status_client_ = node_.serviceClient<std_srvs::Trigger>("aiui_status");
    }
  }

  void loop()
  {
    if (broadcast_status_ && !alarm_status_)
    {
      ROS_INFO("broadcast_status_ && !alarm_status_");
      if (loop_mode_)
      {
        ROS_INFO("loop mode");
        if (played_seconds_++ % sound_time_length_ == 0)
        {
          ROS_INFO("play");

          if (last_broadcast_file_path_ != "")
          {
            sc_.stopWave(last_broadcast_file_path_);
          }
          else if (played_seconds_ > 0)
          {
            sc_.stopWave(broadcast_file_path_);
          }
          last_broadcast_file_path_ = broadcast_file_path_;
          sc_.playWave(broadcast_file_path_);
        }
      }
      else
      {
        broadcast_status_ = false;
        if (last_broadcast_file_path_ != "")
        {
          sc_.stopWave(last_broadcast_file_path_);
        }
        else if (played_seconds_ > 0)
        {
          sc_.stopWave(broadcast_file_path_);
        }
        last_broadcast_file_path_ = broadcast_file_path_;
        sc_.playWave(broadcast_file_path_);
      }
    }
    else
    {
      played_seconds_ = 0;
    }

    updateStatus();

    if (robot_sn_.empty())
    {
      if (node_.hasParam("robot_sn"))
      {
        node_.getParam("robot_sn", robot_sn_);
      }
    }
    else
    {
      Json::Value info;
      info["MessageType"] = Json::Value("SensorMessage");
      info["sn"] = Json::Value(robot_sn_);
      info["pm"] = Json::Value(sensors_info_.pm_);
      info["mq6"] = Json::Value(sensors_info_.gas_a_);
      info["mq4"] = Json::Value(sensors_info_.gas_b_);
      info["mq135"] = Json::Value(sensors_info_.gas_c_);
      info["mq3"] = Json::Value(sensors_info_.gas_d_);
      info["co"] = Json::Value(sensors_info_.co_);
      info["combustible_gas"] = Json::Value(sensors_info_.combustible_gas_value_);
      info["temperature"] = Json::Value(sensors_info_.temperature_);
      info["humidity"] = Json::Value(sensors_info_.humidity_);
      info["battery_level"] = Json::Value(sensors_info_.battery_level_);
      info["aiui_status"] = Json::Value(node_status_.aiui_status_);

      info["inf_camera_max_temperature"] = Json::Value(sensors_info_.inf_camera_max_temperature_);
      info["inf_camera_min_temperature"] = Json::Value(sensors_info_.inf_camera_min_temperature_);
      info["running_status"] = Json::Value(status_);
      info["latitude"] = Json::Value(rm_.latitude_);
      info["longitude"] = Json::Value(rm_.longitude_);
      info["speed"] = Json::Value(rm_.lx_);
      info["angluar_speed"] = Json::Value(rm_.az_);
      info["wheel_track"] = Json::Value(robot_info_.getWheelTrack());
      info["alarm"] = Json::Value(alarm_status_);
      info["alarm_lamp"] = Json::Value("");
      info["broadcast"] = Json::Value(broadcast_status_);
      info["broadcast_path"] = Json::Value(broadcast_file_path_);
      info["move_deriection"] = Json::Value(rm_.deriection_);
      info["enp2s0"] = Json::Value(computer_info_.enp2s0_);
      info["wlp3s0"] = Json::Value(computer_info_.wlp3s0_);
      char buffer[20] = {"\0"};
      sprintf(buffer, "%ld", total_run_time_);
      info["total_run_time"] = Json::Value(string(buffer));
      info["total_odom"] = Json::Value(total_odom_);
      info["need_charge"] = Json::Value(need_charge_);
      info["is_charging"] = Json::Value(is_charging_);
      info["is_charging_nav"] = Json::Value(is_charging_nav_);
      info["is_joy_connect"] = Json::Value(is_joy_connect_);
      info["is_mapping"] = Json::Value(is_mapping_);
      info["is_navigation"] = Json::Value(is_navigation_);

      for (int i = 0; i < sensors_info_.sonar_.size(); i++)
      {
        std::ostringstream ss;
        ss << "sonar" << i;
        info[ss.str()] = Json::Value(sensors_info_.sonar_[i]);
      }

      std_msgs::String msg;
      msg.data = info.toStyledString();
      sensors_info_pub_.publish(msg);
    }

    static unsigned long count = 0;
    //每10s 保存一次里程 总运行时长数据
    if (count++ % 10 == 0)
    {
      total_run_time_ += 10;
      saveLastValidData();
      updateNodeStatus();
    }
  }

private:
  ros::NodeHandle private_nh_;
  ros::NodeHandle node_;
  ros::Publisher status_pub_;
  ros::Publisher sensors_info_pub_;
  ros::Publisher need_charge_pub_;
  ros::Publisher gps_converted_pub_;

  ros::Subscriber sensor_sub_;
  ros::Subscriber patrol_sub_;
  ros::Subscriber patrol_status_sub_;
  ros::Subscriber battery_state_sub_;
  ros::Subscriber temperature_sub_;
  ros::Subscriber relativeHumidity_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber charge_nav_status_sub_;
  ros::Subscriber sonar_sub_;
  ros::Subscriber pm_sub_;
  ros::Subscriber gasa_sub_;
  ros::Subscriber gasb_sub_;
  ros::Subscriber gasc_sub_;
  ros::Subscriber gasd_sub_;
  ros::Subscriber co_sub_;
  ros::Subscriber emergency_sub_;
  ros::Subscriber combustible_gas_sub_;
  ros::Subscriber play_sound_file_sub_;
  ros::Subscriber nav_control_sub_;
  ros::Subscriber mapping_control_sub_;
  ros::Subscriber map_save_sub_;
  ros::Subscriber joy_status_sub_;

  ros::ServiceServer need_charge_service;

  bool low_power_auto_charge_;
  double auto_charge_battery_percentage_;
  double former_warnning_battery_percentage_;
  RobotInfo robot_info_;
  SensorsInfo sensors_info_;
  ComputerInfo computer_info_;
  dalu::RobotStatus status_;

  sound_play::SoundClient sc_;
  bool loop_mode_;
  int sound_time_length_;

  RobotMotion rm_;
  std::string alarm_path_;
  bool alarm_status_;
  bool broadcast_status_;
  std::string broadcast_file_path_;
  std::string last_broadcast_file_path_;
  std::string data_file_path_;
  double total_odom_;
  long total_run_time_;
  ros::Time last_sampling_time_;
  bool has_init_;
  std::string lisenses_;
  NodeStatus node_status_;
  ros::ServiceClient aiui_status_client_;
  bool need_charge_;
  bool is_charging_;
  bool is_charging_nav_;
  bool is_joy_connect_;
  bool is_mapping_;
  bool is_navigation_;

  std::string robot_sn_;

  unsigned long played_seconds_;

  double transformLat(double x, double y)
  {
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * M_PI) + 40.0 * sin(y / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * M_PI) + 320 * sin(y * M_PI / 30.0)) * 2.0 / 3.0;
    return ret;
  }

  double transformLon(double x, double y)
  {
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * M_PI) + 40.0 * sin(x / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * M_PI) + 300.0 * sin(x / 30.0 * M_PI)) * 2.0 / 3.0;
    return ret;
  }

  void gps84ToGCJ02(double lat, double lon)
  {
    static double a = 6378245.0;
    static double ee = 0.00669342162296594323;

    double dLat = transformLat(lon - 105.0, lat - 35.0);
    double dLon = transformLon(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * M_PI;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * M_PI);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cosf(radLat) * M_PI);
    lat += dLat;
    lon += dLon;

    sensor_msgs::NavSatFix msg;
    msg.header.stamp = ros::Time::now();
    msg.altitude = lat;
    msg.longitude = lon;
    gps_converted_pub_.publish(msg);
  }

  void updateStatus()
  {
    if (is_mapping_)
    {
      status_ = dalu::STATUS_MAPPING;
    }
    else if (is_navigation_)
    {
      status_ = dalu::STATUS_PATROL;
    }
    else
    {
      status_ = dalu::STATUS_IDLE;
    }

    if (is_charging_nav_)
    {
      status_ = dalu::STATUS_NAV_TO_CHARGING_STATION;
    }
    else if (is_charging_)
    {
      status_ = dalu::STATUS_CHARGING;
    }

    if (is_joy_connect_)
    {
      status_ = dalu::STATUS_CONTROL;
    }

    std_msgs::Int32 stat;
    stat.data = status_;
    status_pub_.publish(stat);
  }


  void joyStatusCallback(std_msgs::Int32 msg)
  {
    if (msg.data == 1)
    {
      is_joy_connect_ = true;
    }
    else
    {
      is_joy_connect_ = false;
    }
  }

  void navControlCallback(std_msgs::String msg)
  {
    if (msg.data.length() == 0)
    {
      is_navigation_ = false;
    }
    else
    {
      is_navigation_ = true;
    }
  }

  void mappingControlCallback(std_msgs::Bool msg)
  {
    is_mapping_ = msg.data;
  }

  void mapSaveControlCallback(std_msgs::String msg)
  {
    is_mapping_ = false;
  }

  void pmCallback(std_msgs::Float32 data)
  {
    sensors_info_.pm_ = data.data;
  }

  void gasaCallback(std_msgs::Float32 data)
  {
    sensors_info_.gas_a_ = data.data;
  }

  void gasbCallback(std_msgs::Float32 data)
  {
    sensors_info_.gas_b_ = data.data;
  }

  void gascCallback(std_msgs::Float32 data)
  {
    sensors_info_.gas_c_ = data.data;
  }

  void gasdCallback(std_msgs::Float32 data)
  {
    sensors_info_.gas_d_ = data.data;
  }

  void coCallback(std_msgs::Float64 data)
  {
    sensors_info_.co_ = data.data;
  }
  void gpsCallback(sensor_msgs::NavSatFix msg)
  {
    rm_.latitude_ = msg.latitude;
    rm_.longitude_ = msg.longitude;
    gps84ToGCJ02(rm_.latitude_, rm_.longitude_);
  }

  void odomCallback(nav_msgs::Odometry msg)
  {
    if (!has_init_)
    {
      last_sampling_time_ = ros::Time::now();
      has_init_ = true;
    }
    else
    {
      ros::Time now = ros::Time::now();
      double interval = ros::Time::now().toSec() - last_sampling_time_.toSec();
      total_odom_ += (fabs(msg.twist.twist.linear.x) + fabs(msg.twist.twist.angular.z * robot_info_.getWheelTrack()))
          * interval;
      last_sampling_time_ = now;
    }
    rm_.lx_ = msg.twist.twist.linear.x;
    rm_.az_ = msg.twist.twist.angular.z;
    rm_.deriection_ = dalu::calculateMoveDeriection(rm_.lx_, rm_.az_);
  }

  void patrolStatusCallback(std_msgs::Bool msg)
  {
    is_navigation_ = !msg.data;
  }

  void emergencyCallback(std_msgs::Int32 msg)
  {
    static sound_play::Sound alarm_sound = sc_.waveSound(alarm_path_);
    if (msg.data == 1)
    {
      if (!alarm_status_)
      {
        alarm_status_ = true;
        alarm_sound.repeat();
      }
    }
    else
    {
      if (alarm_status_)
      {
        alarm_status_ = false;
        alarm_sound.stop();
      }
    }
  }

  void playAlarmCallback(std_msgs::String msg)
  {
    Json::Value json;
    Json::Reader reader;
    reader.parse(msg.data, json);
    std::string file_name = json["Filename"].asString();
    ROS_INFO_STREAM("Try to play:" << file_name << ",path:/home/robot/alarms/" + file_name);

    if (file_name == "")
    {
      sc_.stopAll();
      broadcast_status_ = false;
      loop_mode_ = false;
      played_seconds_ = 0;
    }
    else
    {
      sound_time_length_ = json["Num"].asInt();
      broadcast_status_ = true;
      broadcast_file_path_ = "/home/robot/alarms/" + file_name;
      played_seconds_ = 0;

      if (sound_time_length_ == 0)
      {
        loop_mode_ = false;
      }
      else
      {
        loop_mode_ = true;
      }
    }
  }

  void batteryStateCallback(sensor_msgs::BatteryState msg)
  {
    ROS_INFO(__FUNCTION__);
    sensors_info_.battery_level_ = msg.percentage;
    //过滤0值
    if (msg.percentage == 0.0)
    {
      return;
    }

    if (msg.current > -0.5)
    {
      is_charging_ = true;
    }
    else
    {
      is_charging_ = false;
    }

    //每降低1%响一次
    if (msg.percentage < auto_charge_battery_percentage_)
    {
      need_charge_ = true;
      node_.setParam("need_charge", need_charge_);
      ROS_INFO_STREAM("percent:"<<msg.percentage<<" former percent:"<<former_warnning_battery_percentage_);
      if (msg.percentage < former_warnning_battery_percentage_)
      {
        ROS_INFO_STREAM("Battery percentage:"<<msg.percentage);
        former_warnning_battery_percentage_ = msg.percentage;
        sc_.play(sound_play::SoundRequest::NEEDS_PLUGGING_BADLY);
      }
    }
    else
    {
      if (msg.percentage > auto_charge_battery_percentage_)
      {
        need_charge_ = false;
        node_.setParam("need_charge", need_charge_);
        former_warnning_battery_percentage_ = auto_charge_battery_percentage_;
        ROS_INFO_STREAM("Set param need charge false...battery percentage:"<<msg.percentage);
      }
    }

    if (low_power_auto_charge_)
    {
      std_msgs::Bool msg;
      msg.data = need_charge_;
      need_charge_pub_.publish(msg);
    }
  }

  void combustibleGasCallback(const std_msgs::Float32 msg)
  {
    sensors_info_.combustible_gas_value_ = msg.data;
  }

  void sonarCallback(const std_msgs::UInt8MultiArrayPtr msg)
  {
    int size = msg->data.size();
    unsigned char *data = new unsigned char[size + 1];

    for (size = 0; size < msg->data.size(); ++size)
    {
      data[size] = msg->data[size];
    }

    unsigned char type = msg->data[3];
    unsigned char *sub_type = NULL;

    if (0x02 == type)
    {
      std::vector<double> dist;
      std::vector<int> angle;
      sub_type = (data + 4);
      static int count = 0;

      if (count++ % 10 != 0)
      {
        return;
      }

      if (0x01 != *sub_type)
      {
        return;
      }

      /*
       0        1       2       3
       front
       11                       4

       10                       5
       back
       9        8       7       6
       */

      for (int s = 5; s != 17; s++)
      {
        sub_type = data + s;
        switch (*sub_type)
        {
          case 0x01:
            dist.push_back(0.0);
            angle.push_back(s);
            break;
          case 0x7f:
            dist.push_back(2.5);
            angle.push_back(s);
            break;
          case 0x80:
            dist.push_back(0.0);
            angle.push_back(s);
            break;
          default:
            dist.push_back((*sub_type) / 10.0);
            angle.push_back(s);
            break;
        }
      }

      sensors_info_.sonar_.resize(dist.size());
      sensors_info_.sonar_.clear();

      /*
       for (int i = 0; i < dist.size(); i++)
       {
       sensors_info_.sonar_.push_back(dist[i]);
       }
       */

      sensors_info_.sonar_ = dist;
    }

    delete[] data;
  }

  void temperatureCallback(sensor_msgs::Temperature msg)
  {
    sensors_info_.temperature_ = msg.temperature;
  }

  void relativeHumidityCallback(sensor_msgs::RelativeHumidity msg)
  {
    sensors_info_.humidity_ = msg.relative_humidity;
  }

  void chargeNavStatusCallback(std_msgs::Bool msg)
  {
    is_charging_nav_ = msg.data;
  }

  void getLastValidData()
  {
    std::ifstream data_file(data_file_path_.c_str());
    std::string data;
    if (data_file.is_open())
    {
      data_file >> total_run_time_;
      data_file >> total_odom_;
      ROS_INFO("Get total run time :%ld total odom:%f", total_run_time_, total_odom_);
      data_file.close();
    }
  }
  //保存运行时间  总里程
  void saveLastValidData()
  {
    std::ofstream data_file(data_file_path_.c_str());
    data_file << total_run_time_ << " " << total_odom_ << std::endl;
    data_file.close();
  }
  //aiui 状态
  void updateNodeStatus()
  {
    if (aiui_status_client_.isValid())
    {
      std_srvs::Trigger srv;
      if (aiui_status_client_.call(srv))
      {
        node_status_.aiui_status_ = srv.response.success;
      }
      else
      {
        ROS_ERROR("Failed to call service aiui_status");
      }
    }
  }
};

int main(int argc, char **args)
{
  ros::init(argc, args, "robot_status");
  RobotStatus robot_status;
  ros::Rate rate(1);

  while (ros::ok())
  {
    robot_status.loop();
    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}
