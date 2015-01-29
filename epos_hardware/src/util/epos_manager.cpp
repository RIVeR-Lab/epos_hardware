#include "epos_hardware/epos_manager.h"
#include <boost/foreach.hpp>

namespace epos_hardware {

EposManager::EposManager(hardware_interface::ActuatorStateInterface& asi,
			 hardware_interface::VelocityActuatorInterface& avi,
			 hardware_interface::PositionActuatorInterface& api,
			 ros::NodeHandle& nh, ros::NodeHandle& pnh,
			 const std::vector<std::string>& motor_names)
  : asi_(&asi), avi_(&avi), api_(&api) {
  BOOST_FOREACH(const std::string& motor_name, motor_names) {
    ROS_INFO_STREAM("Loading EPOS: " << motor_name);
    ros::NodeHandle motor_config_nh(pnh, motor_name);
    boost::shared_ptr<Epos> motor(new Epos(motor_name, nh, motor_config_nh, &epos_factory, *asi_, *avi_, *api_));
    motors_.push_back(motor);
  }
}


bool EposManager::init() {
  bool success = true;
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    if(!motor->init()) {
      ROS_ERROR_STREAM("Could not configure motor: " << motor->name());
      success = false;
    }
  }
  return success;
}

void EposManager::update_diagnostics() {
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    motor->update_diagnostics();
  }
}

void EposManager::read() {
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    motor->read();
  }
}

void EposManager::write() {
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    motor->write();
  }
}


}
