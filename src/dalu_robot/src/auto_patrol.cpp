/*
 * auto_patrol.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: qiao
 */

#include "dalu_robot/defaults.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Bool.h>
#ifdef NEW_VERSION_PATROL
#include <std_msgs/Float32.h>
#endif
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

using namespace dalu;
const int kDefaultUsedTime = 10;
State state = PATROL;
int goal_index = 0;
bool has_init = false;
bool loop = false;
bool has_reached_the_endpoint = false;
double init_pose_x = 0.0;
double init_pose_y = 0.0;
double init_origintation_z = 0.0;
double init_origintation_w = 1.0;
bool auto_charge = false;
bool need_charge = false;

struct Pose
{
  double position[3];
  double orientation[4];
};

struct Action
{
  int action_type;
  int data[3] = {0};
  int time_to_use;
};

struct WayPoint
{
  std::string name;
  struct Pose pose;
#ifndef NEW_VERSION_PATROL
  int work_type;
#else
  int all_actions_used_time;
  std::vector<Action> actions;
#endif

  friend std::ostream& operator<<(std::ostream& os, WayPoint& wp)
  {
    os << "name:" << wp.name << " x:" << wp.pose.position[0] << " y:" << wp.pose.position[1] << " z:"
        << wp.pose.position[2] << " ox:" << wp.pose.orientation[0] << " oy:" << wp.pose.orientation[1] << " oz:"
        << wp.pose.orientation[2] << " ow:" << wp.pose.orientation[3];
    return os;
  }
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::vector<WayPoint> way_points;

move_base_msgs::MoveBaseGoal GetNextPatrolGoal()
{
  move_base_msgs::MoveBaseGoal goal;
  if (goal_index < way_points.size())
  {
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = way_points[goal_index].pose.position[0];
    goal.target_pose.pose.position.y = way_points[goal_index].pose.position[1];
    goal.target_pose.pose.position.z = way_points[goal_index].pose.position[2];
    goal.target_pose.pose.orientation.x = way_points[goal_index].pose.orientation[0];
    goal.target_pose.pose.orientation.y = way_points[goal_index].pose.orientation[1];
    goal.target_pose.pose.orientation.z = way_points[goal_index].pose.orientation[2];
    goal.target_pose.pose.orientation.w = way_points[goal_index].pose.orientation[3];
  }
  ROS_INFO("Sending goal x:%f y:%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
  return goal;
}

void NeedChargeCallback(std_msgs::Bool msg)
{
  if (msg.data)
  {
    if (state == PATROL)
    {
      //goal_index = way_points.size() - 1;
      need_charge = true;
    }
  }
}

bool FetchWayPoints(ros::NodeHandle& node)
{
#ifdef _DEBUG
  std::cout << __FUNCTION__ << std::endl;
#endif

  XmlRpc::XmlRpcValue v;
  if (node.hasParam("waypoints"))
  {
    node.param("waypoints", v, v);
    way_points.reserve(v.size());
    for (int i = 0; i < v.size(); i++)
    {
      WayPoint wp;
      wp.name = static_cast<std::string>(v[i]["name"]);
      wp.pose.position[0] = static_cast<double>(v[i]["pose"]["position"]["x"]);
      wp.pose.position[1] = static_cast<double>(v[i]["pose"]["position"]["y"]);
      wp.pose.position[2] = static_cast<double>(v[i]["pose"]["position"]["z"]);
      wp.pose.orientation[0] = static_cast<double>(v[i]["pose"]["orientation"]["x"]);
      wp.pose.orientation[1] = static_cast<double>(v[i]["pose"]["orientation"]["y"]);
      wp.pose.orientation[2] = static_cast<double>(v[i]["pose"]["orientation"]["z"]);
      wp.pose.orientation[3] = static_cast<double>(v[i]["pose"]["orientation"]["w"]);
#ifndef NEW_VERSION_PATROL
      wp.work_type = static_cast<int>(v[i]["work_type"]);
#else
      wp.all_actions_used_time = static_cast<int>(v[i]["all_actions_used_time"]);
      if (v[i].hasMember("actions"))
      {
        wp.actions.reserve(v[i]["actions"].size());
        for (int j = 0; j < v[i]["actions"].size(); j++)
        {
          Action at;
          at.action_type = static_cast<int>(v[i]["actions"][j]["action_type"]);
          at.time_to_use = static_cast<int>(v[i]["actions"][j]["time_to_use"]);
          at.data[0] = static_cast<int>(v[i]["actions"][j]["data"][0]);
          at.data[1] = static_cast<int>(v[i]["actions"][j]["data"][1]);
          at.data[2] = static_cast<int>(v[i]["actions"][j]["data"][2]);
          wp.actions.push_back(at);
        }
      }
#endif
      way_points.push_back(wp);
#ifdef _DEBUG
      ROS_INFO_STREAM("point" << i << ": " << wp);
#endif
    }
    ROS_INFO_STREAM("Way points counts:"<<v.size());

    if (v.size() > 0)
    {
      return true;
    }
  }
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dl_navigation");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  private_nh.param("initial_pose_x", init_pose_x, 0.0);
  private_nh.param("initial_pose_y", init_pose_y, 0.0);
  private_nh.param("initial_orientation_z", init_origintation_z, 0.0);
  private_nh.param("initial_orientation_w", init_origintation_w, 1.0);
  private_nh.param("loop", loop, false);
  private_nh.param("auto_charge", auto_charge, false);

  ROS_INFO("Init pose x:%f  y:%f  qz:%f  qw:%f", init_pose_x, init_pose_y, init_origintation_z, init_origintation_w);

  ros::Subscriber stop_sub = node.subscribe<std_msgs::Bool>("need_charge", 5, NeedChargeCallback);
#ifndef NEW_VERSION_PATROL
  ros::Publisher handle_head_neck_capture = node.advertise<std_msgs::Int32>("do_actions", 10);
#else
  ros::Publisher handle_body_pub = node.advertise<std_msgs::Float32>("handle_body", 1);
  ros::Publisher handle_neck_pub = node.advertise<std_msgs::Float32>("handle_neck", 1);
  ros::Publisher handle_head_pub = node.advertise<dalu_robot::head>("handle_head", 1);
  ros::Publisher handle_light_pub = node.advertise<std_msgs::Int32>("light", 1);
  ros::Publisher handle_capture_pub = node.advertise<std_msgs::Int32>("capture_image", 1);
#endif

  ros::Publisher patrol_status_pub = node.advertise<std_msgs::Bool>("dalu_robot/patrol_status", 1);
  ros::Publisher start_charge_nav_pub = node.advertise<std_msgs::Bool>("start_charge_nav", 1);
  ros::Publisher init_pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

  if (!FetchWayPoints(private_nh))
  {
    ROS_INFO("No way points recorded in file...");
    std_msgs::Bool status;
    status.data = true;
    patrol_status_pub.publish(status);
    state = IDLE;
    has_init = false;
    return 0;
  }

  ROS_INFO("Way points number:%zd", way_points.size());
  sleep(1);
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server");
  }

  has_reached_the_endpoint = false;
  goal_index = 0;
  geometry_msgs::PoseWithCovarianceStamped init_pose;
  init_pose.header.frame_id = "map";
  init_pose.header.stamp = ros::Time::now();
  init_pose.pose.pose.position.x = init_pose_x;
  init_pose.pose.pose.position.y = init_pose_y;
  init_pose.pose.pose.position.z = 0.0;
  init_pose.pose.pose.orientation.x = 0.0;
  init_pose.pose.pose.orientation.y = 0.0;
  init_pose.pose.pose.orientation.z = init_origintation_z;
  init_pose.pose.pose.orientation.w = init_origintation_w;
  ROS_INFO("start pose x:%f  y:%f  z:%f  w:%f", init_pose_x, init_pose_y, init_origintation_z, init_origintation_w);
  init_pose_pub.publish(init_pose);

  sleep(2);

  while (ros::ok())
  {
    if (state == PATROL)
    {
      //初始化脖子位置 头部角度
      if (!has_init)
      {
#ifndef _DEBUG
#ifndef NEW_VERSION_PATROL
        std_msgs::Int32 cmd;
        cmd.data = 0;
        handle_head_neck_capture.publish(cmd);
#endif
#endif
        std_msgs::Bool status;
        status.data = false;
        patrol_status_pub.publish(status);

        has_init = !has_init;
      }
      move_base_msgs::MoveBaseGoal goal = GetNextPatrolGoal();
      ac.sendGoal(goal);

      ac.waitForResult();
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("You have arrived to the goal position");
// TODO(Q):暂时为休眠固定时间间隔，之后应加入 反馈
        /* 第八位为1 则表示需要 操作脖子 身体
         * 1 拍摄下部
         * 2 拍摄中部
         * 3 拍摄上部
         * 4 拍摄 下中
         * 5 拍摄 下上
         * 6 拍摄 中上
         * 7 拍摄 下中上
         * 8 中部 三连拍
         * 9 最上
         */
#ifndef NEW_VERSION_PATROL
        if (way_points[goal_index].work_type == 1)
        {
          // TODO(Q):call back
          std_msgs::Int32 cmd;
          cmd.data = 1;
          handle_head_neck_capture.publish(cmd);
          sleep(kDefaultUsedTime);
        }
        else if (way_points[goal_index].work_type == 2)
        {
          std_msgs::Int32 cmd;
          cmd.data = 2;
          handle_head_neck_capture.publish(cmd);
          sleep(kDefaultUsedTime);
        }
        else if (way_points[goal_index].work_type == 3)
        {
          std_msgs::Int32 cmd;
          cmd.data = 3;
          handle_head_neck_capture.publish(cmd);
          sleep(29);
        }
        else if (way_points[goal_index].work_type == 4)
        {
          std_msgs::Int32 cmd;
          cmd.data = 4;
          handle_head_neck_capture.publish(cmd);
          sleep(kDefaultUsedTime);
        }
        else if (way_points[goal_index].work_type == 5)
        {
          std_msgs::Int32 cmd;
          cmd.data = 5;
          handle_head_neck_capture.publish(cmd);
          sleep(kDefaultUsedTime);
        }
        else if (way_points[goal_index].work_type == 6)
        {
          std_msgs::Int32 cmd;
          cmd.data = 2;
          handle_head_neck_capture.publish(cmd);
          sleep(10);
          cmd.data = 3;
          handle_head_neck_capture.publish(cmd);
          sleep(45);
        }
        else if (way_points[goal_index].work_type == 7)
        {
          /*
           std_msgs::Int32 cmd;
           cmd.data = 1;
           handle_head_neck_capture.publish(cmd);
           sleep(10);
           cmd.data = 2;
           handle_head_neck_capture.publish(cmd);
           sleep(10);
           cmd.data = 3;
           */
          std_msgs::Int32 cmd;
          cmd.data = 6;
          handle_head_neck_capture.publish(cmd);
          sleep(20);
        }
        else if (way_points[goal_index].work_type == 8)
        {
          std_msgs::Int32 cmd;
          cmd.data = 8;
          handle_head_neck_capture.publish(cmd);
          sleep(8);
        }
        else if (way_points[goal_index].work_type == 9)
        {
          std_msgs::Int32 cmd;
          cmd.data = 5;
          handle_head_neck_capture.publish(cmd);
          sleep(25);
        }
#else
        for (int i = 0; i < way_points[goal_index].actions.size(); i++)
        {
          switch (way_points[goal_index].actions[i].action_type)
          {
            //0:身子 1:脖子 2:头 3:拍照 4:灯光 5:警报
            case 0:
            {
              std_msgs::Float32 msg;
              msg.data = way_points[goal_index].actions[i].data[0];
              handle_body_pub.publish(msg);
              break;
            }
            case 1:
            {
              std_msgs::Float32 msg;
              msg.data = way_points[goal_index].actions[i].data[0];
              handle_neck_pub.publish(msg);
              break;
            }
            case 2:
            {
              dalu_robot::head msg;
              msg.yaw = way_points[goal_index].actions[i].data[0];
              msg.pitch = way_points[goal_index].actions[i].data[1];
              msg.roll = way_points[goal_index].actions[i].data[2];
              handle_head_pub.publish(msg);
              break;
            }
            case 3:
            {
              std_msgs::Int32 msg;
              msg.data = way_points[goal_index].actions[i].data[0];
              handle_capture_pub.publish(msg);
              break;
            }
            case 4:
            {
              std_msgs::Int32 msg;
              msg.data = way_points[goal_index].actions[i].data[0];
              handle_light_pub.publish(msg);
              break;
            }
            case 5:
            {
              char cmd[128] =
              { 0};
              if (way_points[goal_index].actions[i].data[0] == 1)
              {
                std::string path = ros::package::getPath("dalu_robot") + "/data/alarm.mp3";
                sprintf(cmd, "rosrun sound_play play.py %s", path.c_str());
              }
              else
              sprintf(cmd, "rosrun sound_play shutup.py");
              int result = std::system(cmd);
              if (result < 0)
              std::cout << "Control alarm failed" << std::endl;
              break;
            }
            default:
            break;
          }
          sleep(way_points[goal_index].actions[i].time_to_use);
        }
#endif
        goal_index++;
        if (goal_index == way_points.size())
        {
          has_reached_the_endpoint = true;
        }
      }
      else
      {
        ROS_INFO("Navigation failed for some reason...");
        goal_index++;
      }

      if (has_reached_the_endpoint)
      {
        if (loop)
        {
          has_reached_the_endpoint = false;
          goal_index = 0;
          if (need_charge)
          {
            //更新导航状态
            std_msgs::Bool status;
            status.data = true;
            patrol_status_pub.publish(status);
            //开始充电引导
            std_msgs::Bool msg;
            msg.data = true;
            start_charge_nav_pub.publish(msg);
            break;
          }
          else if (node.hasParam("need_charge"))
          {
            node.getParam("need_charge", need_charge);
            if (need_charge)
            {
              //更新导航状态
              std_msgs::Bool status;
              status.data = true;
              patrol_status_pub.publish(status);
              //开始充电引导
              std_msgs::Bool msg;
              msg.data = true;
              start_charge_nav_pub.publish(msg);
              break;
            }
          }
        }
        else
        {
          //更新导航状态
          std_msgs::Bool status;
          status.data = true;
          patrol_status_pub.publish(status);

          if (auto_charge)
          {
            std_msgs::Bool msg;
            msg.data = true;
            start_charge_nav_pub.publish(msg);
          }

          break;
        }
      }
    }
    else
    {
      sleep(1);
    }
    ros::spinOnce();
  }
  return 0;
}
