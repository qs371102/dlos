/*
 * record_points.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: robot
 */

#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>

enum RECORD_OPTION
{
  RECORD_POSE_ONLY = 0, RECORD_POSE_THEN_CAPTURE
};

const char file_name[] = "/home/robot/robot_patrol_points.txt";

bool collect_request = false;
bool continue_collection = true;
bool first_data_got = false;
std::string end_button_sym, collect_button_sym;
int end_button_num = 0, collect_button_num = 0;
geometry_msgs::PoseWithCovarianceStamped current_pose;

void JoyCallback(const sensor_msgs::Joy joy_msg)
{
  if (joy_msg.buttons[collect_button_num] == 1)
  {
    collect_request = true;
  }
  else
  {
    collect_request = false;
  }

  if (joy_msg.buttons[end_button_num] == 1)
  {
    continue_collection = false;
  }
}

void CollectControlSub(std_msgs::Int32 msg)
{
  switch (msg.data)
  {
    case 0:
      continue_collection = false;
      break;
    case 1:
      collect_request = true;
      break;
    case 2:
      collect_request = false;
      break;
    default:
      break;
  }
}

int main(int argc, char **argv)
{
  //Initialize variables
  int num_waypoints = 0;
  std::string path_local;
  bool has_init(false);
  ros::init(argc, argv, "dl_record");
  ros::NodeHandle node;
  ros::Time::init();
  ros::Time time_last;
  ros::Time time_current;
  ros::Duration duration_min(1);
  tf::TransformListener listener;

  // Get button numbers to collect waypoints and end collection
  ros::param::get("/dl_record_patrol_points/collect_button_num", collect_button_num);
  ros::param::get("/dl_record_patrol_points/end_button_num", end_button_num);

  ros::Subscriber collect_control_sub = node.subscribe<std_msgs::Int32>("record_pose", 10, CollectControlSub);
  ros::Subscriber joy_sub = node.subscribe("joy", 100, JoyCallback);

  ROS_INFO("Initiated collect_gps_waypoints node");

  // Initiate publisher to send end of node message
  ros::Publisher pubCollectionNodeEnded = node.advertise<std_msgs::Bool>("dalu_robot/collection_status", 100);

  //Read file path and create/open file
  ros::param::get("/dl_record_patrol_points/waypoints_file", path_local);
  std::string path_abs = ros::package::getPath("dalu_robot") + path_local;
  std::ofstream waypointsFile(path_abs.c_str());
  ROS_INFO("Saving coordinates to: %s", path_abs.c_str());

  // Give instructions:
  ros::param::get("/dl_record_patrol_points/collect_button_sym", collect_button_sym);
  ros::param::get("/dl_record_patrol_points/end_button_sym", end_button_sym);
  std::cout << std::endl;
  std::cout << "Press " << collect_button_sym.c_str() << " button to collect and store waypoint." << std::endl;
  std::cout << "Press " << end_button_sym.c_str() << " button to end waypoint collection." << std::endl;
  std::cout << std::endl;

  listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(5.0));
  ros::Rate r(100);

  if (waypointsFile.is_open())
  {
    ros::Rate r(100);

    while (continue_collection && ros::ok())
    {
      ros::spinOnce();
      time_current = ros::Time::now();

      if (collect_request == true)
      {
        if (time_current - time_last > duration_min)
        {
          if (!has_init)
          {
            has_init = true;
            waypointsFile << "waypoints:" << std::endl;
          }
          tf::StampedTransform transform;
          try
          {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            // write waypoint
            ROS_INFO("You have collected another waypoint!");
            ROS_INFO("Press %s button to collect and store another waypoint.", collect_button_sym.c_str());
            ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
            std::cout << std::endl;
            num_waypoints++;
            waypointsFile << "  - name: point" << num_waypoints << std::endl;
            waypointsFile << "    pose:" << std::endl;
            waypointsFile << "      position:" << std::endl;
            waypointsFile << std::fixed << std::setprecision(1) << "        x: " << transform.getOrigin().getX()
                << std::endl;
            waypointsFile << std::fixed << std::setprecision(1) << "        y: " << transform.getOrigin().getY()
                << std::endl;
            waypointsFile << std::fixed << std::setprecision(1) << "        z: " << transform.getOrigin().getZ()
                << std::endl;
            waypointsFile << "      orientation:" << std::endl;
            waypointsFile << std::fixed << std::setprecision(1) << "        x: " << transform.getRotation().getX()
                << std::endl;
            waypointsFile << std::fixed << std::setprecision(1) << "        y: " << transform.getRotation().getY()
                << std::endl;
            waypointsFile << std::fixed << std::setprecision(1) << "        z: " << transform.getRotation().getZ()
                << std::endl;
            waypointsFile << std::fixed << std::setprecision(1) << "        w: " << transform.getRotation().getW()
                << std::endl;
            waypointsFile << "    work_type: 0" << std::endl;
            waypointsFile << "    action_group:" << std::endl;
            waypointsFile << "      - action_content:" << std::endl;
            waypointsFile << "          light_open: false" << std::endl;
            waypointsFile << "          body_up: false" << std::endl;
            waypointsFile << "          delay_time: 0" << std::endl;
            waypointsFile << "          rotation: 0.0" << std::endl;
            waypointsFile << "          picture: 0" << std::endl;
            waypointsFile << "          head_rpy: " << std::endl;
            waypointsFile << "            pitch: 0.0" << std::endl;
            waypointsFile << "            roll: 0.0" << std::endl;
            waypointsFile << "            yaw: 0.0" << std::endl;
            waypointsFile << std::endl;
            time_last = time_current;
            // ros::Duration(duration_min).sleep();
            collect_request = false;
          }
          catch (tf::TransformException& ex)
          {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
          }

        }
        else
        {
          ROS_WARN("Waypoint not saved, you have not moved enough");
          collect_request = false;
        }
      }
      ros::spinOnce();
      r.sleep();
    }

    waypointsFile.close();
    ROS_INFO("End request registered.");
  }
  else
  {
    ROS_ERROR("Unable to open file.");
    ROS_INFO("Exiting..");
  }

  ROS_INFO("Closed waypoint file, you have collected %d waypoints.", num_waypoints);
  ROS_INFO("Ending node...");

  // Notify joy_launch_control that calibration is complete
  std_msgs::Bool node_ended;
  node_ended.data = true;
  pubCollectionNodeEnded.publish(node_ended);

  ros::shutdown();
  return 0;
}
