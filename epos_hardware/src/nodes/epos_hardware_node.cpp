#include <ros/ros.h>
#include <ros/spinner.h>
#include "epos_hardware/epos_hardware.h"
#include <controller_manager/controller_manager.h>
#include <vector>

int main(int argc, char** argv) {
  ros::init(argc, argv, "epos_velocity_hardware");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::vector<std::string> motor_names;
  for(int i = 0; i < argc-1; ++i) {
    motor_names.push_back(argv[i+1]);
  }
  epos_hardware::EposHardware robot(nh, pnh, motor_names);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Initializing Motors");
  if(!robot.init()) {
    ROS_FATAL("Failed to initialize motors");
    return 1;
  }
  ROS_INFO("Motors Initialized");

  ros::Rate controller_rate(50);
  ros::SteadyTime last = ros::SteadyTime::now();
  while (ros::ok()) {
    robot.read();
    ros::SteadyTime now = ros::SteadyTime::now();
    ros::WallDuration period = now-last;
    cm.update(ros::Time::now(), ros::Duration(period.sec, period.nsec));
    robot.write();
    last = now;
    robot.update_diagnostics();
    controller_rate.sleep();
  }

}
