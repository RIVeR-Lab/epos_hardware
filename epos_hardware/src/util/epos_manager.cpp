#include "epos_hardware/epos_manager.h"
#include <boost/foreach.hpp>

namespace epos_hardware {

EposManager::EposManager(hardware_interface::ActuatorStateInterface& asi,
			 hardware_interface::VelocityActuatorInterface& avi,
			 hardware_interface::PositionActuatorInterface& api,
			 ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : asi_(&asi), avi_(&avi), api_(&api), diagnostic_updater_(nh, pnh) {
  diagnostic_updater_.setHardwareID("EPOS");
}


void EposManager::load(XmlRpc::XmlRpcValue& motors_xml) {
  ROS_ASSERT(motors_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i = 0; i < motors_xml.size(); ++i) {
    XmlRpc::XmlRpcValue motor_xml = motors_xml[i];
    boost::shared_ptr<Epos> motor(new Epos(motor_xml, &epos_factory, *asi_, *avi_, *api_));
    motors_.push_back(motor);
    diagnostic_updater_.add(motor->name(), boost::bind(&Epos::buildStatus, motor, _1));
  }
}

bool EposManager::init() {
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    if(!motor->init()) {
      ROS_ERROR_STREAM("Could not configure motor: " << motor->name());
      return false;
    }
  }
  return true;
}

void EposManager::update_diagnostics() {
  diagnostic_updater_.update();
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
