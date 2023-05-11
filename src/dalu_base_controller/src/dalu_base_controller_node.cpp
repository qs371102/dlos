/*
 * dalu_base_controller_node.cpp
 *
 *  Created on: Nov 25, 2019
 *      Author: qiao
 */

#include "dalu_base_controller/dalu_base_controller.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dalu_base_controller");
  BaseController base_controller;
  ros::Rate rate(50);
  while (ros::ok())
  {
    base_controller.loop();
    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}
