/*
 * deltas_odometry.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: qiao
 */

#include "dalu_robot/defaults.h"
#include <limits>
#include <string>
#include <bitset>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>

//#define USE_BASE_TIME

using namespace dalu;

class DeltaSToOdometry
{
public:
  enum OdomType
  {
    SIX_WHEEL_MODE = 0, BORDER_FOUR_WHEEL_MODE, MIDDLE_TWO_WHEELS_MODE
  };

  DeltaSToOdometry() :
      private_node_("~"), x_(0), y_(0), th_(0), vx_(0), vth_(0), left_front_counter_(0), left_counter_(0), left_rear_counter_(
          0), right_front_counter_(0), right_counter_(0), right_rear_counter_(0), left_front_encoder_(0), left_encoder_(
          0), left_rear_encoder_(0), right_front_encoder_(0), right_encoder_(0), right_rear_encoder_(0), right_front_servo_angle_(
          0), right_servo_angle_(0), right_rear_servo_angle_(0), left_front_servo_angle_(0), left_servo_angle_(0), left_rear_servo_angle_(
          0), former_left_front_counter_(0), former_left_counter_(0), former_left_rear_counter_(0), former_right_front_counter_(
          0), former_right_counter_(0), former_right_rear_counter_(0), meters_per_tick_(0), left_deltas_average_(0.0), right_deltas_average_(
          0.0), former_right_encoder_(0), former_left_encoder_(0), former_left_rear_encoder_(0), former_right_rear_encoder_(
          0), former_right_front_encoder_(0), former_left_front_encoder_(0), former_vx_(0), former_vth_(0)
  {
    private_node_.param<std::string>("odom_topic", topic_name_, "odom");
    private_node_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_node_.param<std::string>("base_frame", base_frame_, "base_link");
    private_node_.param<bool>("pub_tf", pub_tf_, true);
    private_node_.param<bool>("use_filter", use_filter_, false);

    int odom_type;
    private_node_.param<int>("odom_type", odom_type, MIDDLE_TWO_WHEELS_MODE);
    odom_type_ = (OdomType)odom_type;

    node_.param<double>("/dl_base_controller/wheel_diameter", wheel_diameter_, 0.146);
    node_.param<double>("/dl_deltas_odometry/wheel_track", wheel_track_, 0.57);
    node_.param<int>("/dl_base_controller/encoder_resolution", encoder_resolution_, 60);
    node_.param<bool>("/dl_base_controller/reverse_encoder", reverse_encoder_, false);
    node_.param<bool>("/dl_base_controller/use_hall", use_hall_, false);

    private_node_.param<int>("count_diff_limit", count_diff_limit_, encoder_resolution_ / 3);

    meters_per_tick_ = wheel_diameter_ * M_PI / encoder_resolution_;

    left_front_encoder_sub_ = node_.subscribe("flwheel", 10, &DeltaSToOdometry::leftFrontEncoderCallback, this);
    left_encoder_sub_ = node_.subscribe("lwheel", 10, &DeltaSToOdometry::leftEncoderCallback, this);
    left_rear_encoder_sub_ = node_.subscribe("rlwheel", 10, &DeltaSToOdometry::leftRearEncoderCallback, this);
    right_front_encoder_sub_ = node_.subscribe("frwheel", 10, &DeltaSToOdometry::rightFrontEncoderCallback, this);
    right_encoder_sub_ = node_.subscribe("rwheel", 10, &DeltaSToOdometry::rightEncoderCallback, this);
    right_rear_encoder_sub_ = node_.subscribe("rrwheel", 10, &DeltaSToOdometry::rightRearEncoderCallback, this);

    left_front_counter_sub_ = node_.subscribe("flhall", 10, &DeltaSToOdometry::leftFrontCounterCallback, this);
    left_counter_sub_ = node_.subscribe("lhall", 10, &DeltaSToOdometry::leftCounterCallback, this);
    left_rear_counter_sub_ = node_.subscribe("rlhall", 10, &DeltaSToOdometry::leftRearCounterCallback, this);
    right_front_counter_sub_ = node_.subscribe("frhall", 10, &DeltaSToOdometry::rightFrontCounterCallback, this);
    right_counter_sub_ = node_.subscribe("rhall", 10, &DeltaSToOdometry::rightCounterCallback, this);
    right_rear_counter_sub_ = node_.subscribe("rrhall", 10, &DeltaSToOdometry::rightRearCounterCallback, this);

    left_front_servo_angle_sub_ = node_.subscribe("lf_servo_angle", 10, &DeltaSToOdometry::leftFrontServoAngleCallback,
                                                  this);
    left_servo_angle_sub_ = node_.subscribe("l_servo_angle", 10, &DeltaSToOdometry::leftServoAngleCallback, this);
    left_rear_servo_angle_sub_ = node_.subscribe("lr_servo_angle", 10, &DeltaSToOdometry::leftRearServoAngleCallback,
                                                 this);
    right_front_servo_angle_sub_ = node_.subscribe("rf_servo_angle", 10,
                                                   &DeltaSToOdometry::rightFrontServoAngleCallback, this);
    right_servo_angle_sub_ = node_.subscribe("r_servo_angle", 10, &DeltaSToOdometry::rightServoAngleCallback, this);
    right_rear_servo_angle_sub_ = node_.subscribe("rr_servo_angle", 10, &DeltaSToOdometry::rightRearServoAngleCallback,
                                                  this);
    odom_pub_ = node_.advertise<nav_msgs::Odometry>(topic_name_, 10);

    if (pub_tf_)
    {
      odom_trans_.header.frame_id = odom_frame_;
      odom_trans_.child_frame_id = base_frame_;
    }
    else
    {
      ROS_INFO("Not pub tf");
    }
  }

  void loop()
  {
    double dt = 0.0;
    current_time_ = ros::Time::now();
    if (msg_status_ != 0xfff && msg_status_ != 0xb7f && msg_status_ != 0x03f)
    {
      ROS_INFO_STREAM("Encoder msg not ready..."<<msg_status_.to_string());
      return;
    }

    if (former_time_.isZero())
    {
      dt = 0.0;
      former_time_ = current_time_;
      if (use_hall_)
      {
        former_left_front_counter_ = left_front_counter_;
        former_left_counter_ = left_counter_;
        former_left_rear_counter_ = left_rear_counter_;
        former_right_front_counter_ = right_front_counter_;
        former_right_counter_ = right_counter_;
        former_right_rear_counter_ = right_rear_counter_;
      }
      else
      {
        former_left_front_encoder_ = left_front_encoder_;
        former_left_encoder_ = left_encoder_;
        former_left_rear_encoder_ = left_rear_encoder_;
        former_right_front_encoder_ = right_front_encoder_;
        former_right_encoder_ = right_encoder_;
        former_right_rear_encoder_ = right_rear_encoder_;
      }
      ROS_INFO_STREAM(
          "Init status:"<<msg_status_.to_string()<<" "<<(int)former_left_front_counter_<<" "<<(int)former_left_counter_<<" "<<(int)former_left_rear_counter_<<" "<<(int)former_right_front_counter_<<" "<<(int)former_right_counter_<<" "<<(int)former_right_rear_counter_<<" "<<(int)left_front_servo_angle_<<" "<<(int)left_rear_servo_angle_<<" "<<(int)right_front_servo_angle_<<" "<<(int)right_rear_servo_angle_);
      return;
    }
    else
    {
      dt = current_time_.toSec() - former_time_.toSec();
      former_time_ = current_time_;
    }
    left_deltas_average_ = 0.0;
    right_deltas_average_ = 0.0;
    vx_ = 0.0;
    vth_ = 0.0;

    ROS_DEBUG_STREAM(
        "Current status:"<<msg_status_.to_string()<<" "<<(int)former_left_front_counter_<<" "<<(int)former_left_counter_<<" "<<(int)former_left_rear_counter_<<" "<<(int)former_right_front_counter_<<" "<<(int)former_right_counter_<<" "<<(int)former_right_rear_counter_<<" "<<(int)left_front_servo_angle_<<" "<<(int)left_servo_angle_<<" "<<(int)left_rear_servo_angle_<<" "<<(int)right_front_servo_angle_<<" "<<(int)right_servo_angle_<<" "<<(int)right_rear_servo_angle_);

    calculateDeltas();
    ROS_DEBUG_STREAM("Left delta:"<<left_deltas_average_<<" right:"<<right_deltas_average_);

    if (left_deltas_average_ != 0.0 || right_deltas_average_ != 0.0)
    {
      double dxy_ave = (right_deltas_average_ + left_deltas_average_) / 2;
      //XXX(Q): 理论上:(right_deltas_average_ - left_deltas_average_)/2 / (wheel_track_/2)
      double delta_th = (right_deltas_average_ - left_deltas_average_) / wheel_track_;
      vx_ = dxy_ave / dt;
      vth_ = delta_th / dt;

      if (dxy_ave != 0)
      {
        double delta_x = cos(delta_th) * dxy_ave;
        double delta_y = -sin(delta_th) * dxy_ave;
        x_ += (cos(th_) * delta_x - sin(th_) * delta_y);
        y_ += (sin(th_) * delta_x + cos(th_) * delta_y);
      }

      if (delta_th != 0)
      {
        th_ += delta_th;
      }

      former_left_front_counter_ = left_front_counter_;
      former_left_counter_ = left_counter_;
      former_left_rear_counter_ = left_rear_counter_;
      former_right_front_counter_ = right_front_counter_;
      former_right_counter_ = right_counter_;
      former_right_rear_counter_ = right_rear_counter_;

      former_left_front_encoder_ = left_front_encoder_;
      former_left_encoder_ = left_encoder_;
      former_left_rear_encoder_ = left_rear_encoder_;
      former_right_front_encoder_ = right_front_encoder_;
      former_right_encoder_ = right_encoder_;
      former_right_rear_encoder_ = right_rear_encoder_;
    }
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th_);

    // filling the odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // velocity
    ROS_DEBUG_STREAM("vx:"<<vx_<<" former vx:"<<former_vx_);
    ROS_DEBUG_STREAM("vth:"<<vth_<<" former vth:"<<former_vth_);

    double vx = vx_, vth = vth_;
    if (vx_ * former_vx_ != 0.0)
    {
      vx = (vx_ + former_vx_) / 2;
    }

    if (vth_ * former_vth_ != 0.0)
    {
      vth = (vth_ + former_vth_) / 2;
    }

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vth;

    odom.pose.covariance[0] = .1;
    odom.pose.covariance[7] = .1;
    odom.pose.covariance[14] = .1;
    odom.pose.covariance[21] = 5e-2;
    odom.pose.covariance[28] = 5e-2;
    odom.pose.covariance[35] = 5e-2;

    odom.twist.covariance[0] = .1;
    odom.twist.covariance[7] = .1;
    odom.twist.covariance[14] = .1;
    odom.twist.covariance[21] = .1;
    odom.twist.covariance[28] = .1;
    odom.twist.covariance[35] = .1;

    // publishing the odometry and the new tf
    if (pub_tf_)
    {
      // update transform
      odom_trans_.header.stamp = current_time_;
      odom_trans_.transform.translation.x = x_;
      odom_trans_.transform.translation.y = y_;
      odom_trans_.transform.translation.z = 0.0;
      odom_trans_.transform.rotation = tf::createQuaternionMsgFromYaw(th_);
      broadcaster_.sendTransform(odom_trans_);
    }
    odom_pub_.publish(odom);

    former_vx_ = vx_;
    former_vth_ = vth_;

    ROS_DEBUG_STREAM(
        "Current status:"<<(int)former_left_front_counter_<<" "<<(int)former_left_counter_<<" "<<(int)former_left_rear_counter_<<" "<<(int)former_right_front_counter_<<" "<<(int)former_right_counter_<<" "<<(int)former_right_rear_counter_<<" "<<(int)left_front_servo_angle_<<" "<<(int)left_rear_servo_angle_<<" "<<(int)right_front_servo_angle_<<" "<<(int)right_rear_servo_angle_);
  }

private:
  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher odom_pub_;
  ros::Subscriber delta_s_sub_;
  std::string topic_name_;
  std::string odom_frame_;
  std::string base_frame_;
  tf::TransformBroadcaster broadcaster_;
  OdomType odom_type_;
  // position
  double x_;
  double y_;
  double th_;
  // velocity
  double vx_;
  double vth_;
  double former_vx_;
  double former_vth_;
  // base params
  double wheel_track_;
  double wheel_diameter_;
  int encoder_resolution_;
  double meters_per_tick_;
  int count_diff_limit_;

  bool pub_tf_;
  //XXX
  bool reverse_encoder_;
  bool use_hall_;
  ros::Time current_time_;
  ros::Time former_time_;
  geometry_msgs::TransformStamped odom_trans_;

  ros::Subscriber left_front_encoder_sub_;
  ros::Subscriber left_encoder_sub_;
  ros::Subscriber left_rear_encoder_sub_;
  ros::Subscriber right_front_encoder_sub_;
  ros::Subscriber right_encoder_sub_;
  ros::Subscriber right_rear_encoder_sub_;

  ros::Subscriber left_front_counter_sub_;
  ros::Subscriber left_counter_sub_;
  ros::Subscriber left_rear_counter_sub_;
  ros::Subscriber right_front_counter_sub_;
  ros::Subscriber right_counter_sub_;
  ros::Subscriber right_rear_counter_sub_;

  ros::Subscriber left_rear_servo_angle_sub_;
  ros::Subscriber left_servo_angle_sub_;
  ros::Subscriber left_front_servo_angle_sub_;
  ros::Subscriber right_front_servo_angle_sub_;
  ros::Subscriber right_servo_angle_sub_;
  ros::Subscriber right_rear_servo_angle_sub_;

  double left_deltas_average_;
  double right_deltas_average_;

  uint16_t left_front_encoder_;
  uint16_t left_encoder_;
  uint16_t left_rear_encoder_;
  uint16_t right_front_encoder_;
  uint16_t right_encoder_;
  uint16_t right_rear_encoder_;

  uint16_t former_left_front_encoder_;
  uint16_t former_left_encoder_;
  uint16_t former_left_rear_encoder_;
  uint16_t former_right_front_encoder_;
  uint16_t former_right_encoder_;
  uint16_t former_right_rear_encoder_;

  uint8_t left_front_counter_;
  uint8_t left_counter_;
  uint8_t left_rear_counter_;
  uint8_t right_front_counter_;
  uint8_t right_counter_;
  uint8_t right_rear_counter_;

  uint8_t former_left_front_counter_;
  uint8_t former_left_counter_;
  uint8_t former_left_rear_counter_;
  uint8_t former_right_front_counter_;
  uint8_t former_right_counter_;
  uint8_t former_right_rear_counter_;

  short left_front_servo_angle_;
  short left_servo_angle_;
  short left_rear_servo_angle_;
  short right_front_servo_angle_;
  short right_servo_angle_;
  short right_rear_servo_angle_;

  std::bitset<12> msg_status_;
  bool use_filter_;

  void leftFrontEncoderCallback(std_msgs::UInt16 msg)
  {
    if (!use_hall_)
    {
      msg_status_.set(0, 1);
    }
    left_front_encoder_ = msg.data;
  }

  void leftEncoderCallback(std_msgs::UInt16 msg)
  {
    if (!use_hall_)
    {
      msg_status_.set(1, 1);
    }
    left_encoder_ = msg.data;
  }

  void leftRearEncoderCallback(std_msgs::UInt16 msg)
  {
    if (!use_hall_)
    {
      msg_status_.set(2, 1);
    }
    left_rear_encoder_ = msg.data;
  }

  void rightFrontEncoderCallback(std_msgs::UInt16 msg)
  {
    if (!use_hall_)
    {
      msg_status_.set(3, 1);
    }
    right_front_encoder_ = msg.data;
  }

  void rightEncoderCallback(std_msgs::UInt16 msg)
  {
    if (!use_hall_)
    {
      msg_status_.set(4, 1);
    }
    right_encoder_ = msg.data;
  }

  void rightRearEncoderCallback(std_msgs::UInt16 msg)
  {
    if (!use_hall_)
    {
      msg_status_.set(5, 1);
    }
    right_rear_encoder_ = msg.data;
  }

  void leftFrontCounterCallback(std_msgs::UInt8 msg)
  {
    if (use_hall_)
    {
      msg_status_.set(0, 1);
    }
    left_front_counter_ = msg.data;
  }

  void leftCounterCallback(std_msgs::UInt8 msg)
  {
    if (use_hall_)
    {
      msg_status_.set(1, 1);
    }
    left_counter_ = msg.data;
  }

  void leftRearCounterCallback(std_msgs::UInt8 msg)
  {
    if (use_hall_)
    {
      msg_status_.set(2, 1);
    }
    left_rear_counter_ = msg.data;
  }

  void rightFrontCounterCallback(std_msgs::UInt8 msg)
  {
    if (use_hall_)
    {
      msg_status_.set(3, 1);
    }
    right_front_counter_ = msg.data;
  }

  void rightCounterCallback(std_msgs::UInt8 msg)
  {
    if (use_hall_)
    {
      msg_status_.set(4, 1);
    }
    right_counter_ = msg.data;
  }

  void rightRearCounterCallback(std_msgs::UInt8 msg)
  {
    if (use_hall_)
    {
      msg_status_.set(5, 1);
    }
    right_rear_counter_ = msg.data;
  }

  void leftFrontServoAngleCallback(std_msgs::Int16 msg)
  {
    msg_status_.set(6, 1);
    left_front_servo_angle_ = msg.data;
  }

  void leftServoAngleCallback(std_msgs::Int16 msg)
  {
    msg_status_.set(7, 1);
    left_servo_angle_ = msg.data;
  }

  void leftRearServoAngleCallback(std_msgs::Int16 msg)
  {
    msg_status_.set(8, 1);
    left_rear_servo_angle_ = msg.data;
  }

  void rightFrontServoAngleCallback(std_msgs::Int16 msg)
  {
    msg_status_.set(9, 1);
    right_front_servo_angle_ = msg.data;
  }

  void rightServoAngleCallback(std_msgs::Int16 msg)
  {
    msg_status_.set(10, 1);
    right_servo_angle_ = msg.data;
  }

  void rightRearServoAngleCallback(std_msgs::Int16 msg)
  {
    msg_status_[11] = 1;
    right_rear_servo_angle_ = msg.data;
  }

  void middleTwoWheelsMode()
  {
    static uint8_t max_try = 0;
    double left_deltas_, right_deltas_;
    if (use_hall_)
    {
      int8_t left_counter_diff = left_counter_ - former_left_counter_;
      int8_t right_counter_diff = right_counter_ - former_right_counter_;
      //限幅滤波过滤突发噪声
      if (abs(left_counter_diff > count_diff_limit_) || abs(right_counter_diff > count_diff_limit_))
      {
        if (max_try++ <= 5)
        {
          ROS_WARN_STREAM(
              __FUNCTION__<<" bad data:"<<(int)left_counter_<<" "<<(int)former_left_counter_<<" "<<(int)right_counter_<<" "<<(int)former_right_counter_<<" diff:"<<count_diff_limit_);
          left_deltas_average_ = 0.0;
          right_deltas_average_ = 0.0;
          return;
        }
      }
      left_deltas_ = meters_per_tick_ * left_counter_diff;
      right_deltas_ = meters_per_tick_ * right_counter_diff;
    }
    else
    {
      int16_t left_encoder_diff = left_encoder_ - former_left_encoder_;
      int16_t right_encoder_diff = right_encoder_ - former_right_encoder_;
      //限幅滤波过滤突发噪声
      if (abs(left_encoder_diff > count_diff_limit_) || abs(right_encoder_diff > count_diff_limit_))
      {
        if (max_try++ <= 5)
        {
          ROS_WARN_STREAM(
              __FUNCTION__<<" bad data:"<<(int)left_encoder_<<" "<<(int)former_left_encoder_<<" "<<(int)right_encoder_<<" "<<(int)former_right_encoder_<<" diff:"<<count_diff_limit_);
          left_deltas_average_ = 0.0;
          right_deltas_average_ = 0.0;
          return;
        }
      }
      left_deltas_ = meters_per_tick_ * left_encoder_diff;
      right_deltas_ = meters_per_tick_ * right_encoder_diff;
    }
    left_deltas_average_ = left_deltas_;
    right_deltas_average_ = right_deltas_;
    max_try = 0;
  }

  void borderFourWheelsMode()
  {
    static uint8_t max_try = 0;
    double left_front_deltas_, left_rear_deltas_, right_front_deltas_, right_rear_deltas_;
    if (use_hall_)
    {
      int8_t left_front_counter_diff = left_front_counter_ - former_left_front_counter_;
      int8_t left_rear_counter_diff = left_rear_counter_ - former_left_rear_counter_;
      int8_t right_front_counter_diff = right_front_counter_ - former_right_front_counter_;
      int8_t right_rear_counter_diff = right_rear_counter_ - former_right_rear_counter_;
      left_front_deltas_ = meters_per_tick_ * left_front_counter_diff;
      left_rear_deltas_ = meters_per_tick_ * left_rear_counter_diff;
      right_front_deltas_ = meters_per_tick_ * right_front_counter_diff;
      right_rear_deltas_ = meters_per_tick_ * right_rear_counter_diff;
      //异常噪声滤波
      if (abs(left_front_counter_diff > count_diff_limit_) || abs(left_rear_counter_diff > count_diff_limit_)
          || abs(left_front_counter_diff > count_diff_limit_) || abs(right_rear_counter_diff > count_diff_limit_))
      {
        if (max_try++ <= 5)
        {
          ROS_WARN_STREAM(
              __FUNCTION__<<" bad data:"<<left_front_counter_<<" "<<former_left_front_counter_<<" "<<left_rear_counter_<<" "<<former_left_rear_counter_<<" "<<right_front_counter_<<" "<<former_right_front_counter_<<right_rear_counter_<<" "<<former_right_rear_counter_);
          left_deltas_average_ = 0.0;
          right_deltas_average_ = 0.0;
          return;
        }
      }
    }
    else
    {
      int16_t left_front_encoder_diff = left_front_encoder_ - former_left_front_encoder_;
      int16_t left_rear_encoder_diff = left_rear_encoder_ - former_left_rear_encoder_;
      int16_t right_front_encoder_diff = right_front_encoder_ - former_right_front_encoder_;
      int16_t right_rear_encoder_diff = right_rear_encoder_ - former_right_rear_encoder_;
      left_front_deltas_ = meters_per_tick_ * left_front_encoder_diff;
      left_rear_deltas_ = meters_per_tick_ * left_rear_encoder_diff;
      right_front_deltas_ = meters_per_tick_ * right_front_encoder_diff;
      right_rear_deltas_ = meters_per_tick_ * right_rear_encoder_diff;

      //异常噪声滤波
      if (abs(left_front_encoder_diff > count_diff_limit_) || abs(left_rear_encoder_diff > count_diff_limit_)
          || abs(left_front_encoder_diff > count_diff_limit_) || abs(right_rear_encoder_diff > count_diff_limit_))
      {
        if (max_try++ <= 5)
        {
          ROS_WARN_STREAM(
              __FUNCTION__<<" bad data:"<<left_front_counter_<<" "<<former_left_front_counter_<<" "<<left_rear_counter_<<" "<<former_left_rear_counter_<<" "<<right_front_counter_<<" "<<former_right_front_counter_<<right_rear_counter_<<" "<<former_right_rear_counter_);
          left_deltas_average_ = 0.0;
          right_deltas_average_ = 0.0;
          return;
        }
      }
    }

    left_deltas_average_ = (left_front_deltas_ * cos(left_front_servo_angle_ * M_PI / 180.0)
        + left_rear_deltas_ * cos(left_rear_servo_angle_ * M_PI / 180.0)) / 2;
    right_deltas_average_ = (right_front_deltas_ * cos(right_front_servo_angle_ * M_PI / 180.0)
        + right_rear_deltas_ * cos(right_rear_servo_angle_ * M_PI / 180.0)) / 2;
    max_try = 0;
  }

  void sixWheelsMode()
  {
    double left_front_deltas_, left_deltas_, left_rear_deltas_, right_front_deltas_, right_deltas_, right_rear_deltas_;
    if (use_hall_)
    {
      int8_t left_front_counter_diff = left_front_counter_ - former_left_front_counter_;
      int8_t left_counter_diff = left_counter_ - former_left_counter_;
      int8_t left_rear_counter_diff = left_rear_counter_ - former_left_rear_counter_;
      int8_t right_front_counter_diff = right_front_counter_ - former_right_front_counter_;
      int8_t right_counter_diff = right_counter_ - former_right_counter_;
      int8_t right_rear_counter_diff = right_rear_counter_ - former_right_rear_counter_;
      left_front_deltas_ = meters_per_tick_ * left_front_counter_diff;
      left_deltas_ = meters_per_tick_ * left_counter_diff;
      left_rear_deltas_ = meters_per_tick_ * left_rear_counter_diff;
      right_front_deltas_ = meters_per_tick_ * right_front_counter_diff;
      right_deltas_ = meters_per_tick_ * right_counter_diff;
      right_rear_deltas_ = meters_per_tick_ * right_rear_counter_diff;
    }
    else
    {
      int16_t left_front_encoder_diff = left_front_encoder_ - former_left_front_encoder_;
      int16_t left_encoder_diff = left_encoder_ - former_left_encoder_;
      int16_t left_rear_encoder_diff = left_rear_encoder_ - former_left_rear_encoder_;
      int16_t right_front_encoder_diff = right_front_encoder_ - former_right_front_encoder_;
      int16_t right_encoder_diff = right_encoder_ - former_right_encoder_;
      int16_t right_rear_encoder_diff = right_rear_encoder_ - former_right_rear_encoder_;
      left_front_deltas_ = meters_per_tick_ * left_front_encoder_diff;
      left_deltas_ = meters_per_tick_ * left_encoder_diff;
      left_rear_deltas_ = meters_per_tick_ * left_rear_encoder_diff;
      right_front_deltas_ = meters_per_tick_ * right_front_encoder_diff;
      right_deltas_ = meters_per_tick_ * right_encoder_diff;
      right_rear_deltas_ = meters_per_tick_ * right_rear_encoder_diff;
    }

    //TODO: angle check
    double left_front_deltas_converted = left_front_deltas_ * cos(left_front_servo_angle_ * M_PI / 180.0);
    double left_rear_deltas_converted = left_rear_deltas_ * cos(left_rear_servo_angle_ * M_PI / 180.0);
    double right_front_deltas_converted = right_front_deltas_ * cos(right_front_servo_angle_ * M_PI / 180.0);
    double right_rear_deltas_converted = right_rear_deltas_ * cos(right_rear_servo_angle_ * M_PI / 180.0);
    ROS_DEBUG_STREAM("Left:"<<left_front_deltas_converted<< " "<<left_front_deltas_<<" "<<left_rear_deltas_converted);
    ROS_DEBUG_STREAM("Right:"<<left_front_deltas_converted<< " "<<left_front_deltas_<<" "<<left_rear_deltas_converted);
    if (!use_filter_)
    {
      left_deltas_average_ = ((left_front_deltas_converted + left_rear_deltas_converted) / 2 + left_deltas_) / 2;
      right_deltas_average_ = ((right_front_deltas_converted + right_rear_deltas_converted) / 2 + right_deltas_) / 2;
    }
    else
    {
      //XXX
    }
  }

  void calculateDeltas()
  {
    switch (odom_type_)
    {
      case SIX_WHEEL_MODE:
        if (msg_status_ == 0xfff || msg_status_ == 0xb7f)
        {
          sixWheelsMode();
        }
        else //0x03f
        {
          middleTwoWheelsMode();
        }
        break;
      case BORDER_FOUR_WHEEL_MODE:
        if (msg_status_ == 0xfff || msg_status_ == 0xb7f)
        {
          borderFourWheelsMode();
        }
        else //0x03f
        {
          middleTwoWheelsMode();
        }
        break;
      case MIDDLE_TWO_WHEELS_MODE:
        middleTwoWheelsMode();
        break;
      default:
        break;
    }

    if (reverse_encoder_)
    {
      right_deltas_average_ = -right_deltas_average_;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dl_deltas_odometry");
  DeltaSToOdometry odom;
  ros::Rate r(50);
  unsigned int counter = 0;
  while (ros::ok())
  {
    if (counter++ % 5 == 0)
    {
      odom.loop();
    }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
