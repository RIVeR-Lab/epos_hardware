#include "epos_hardware/epos_manager.h"
#include <boost/foreach.hpp>

namespace epos_hardware {

EposManager::EposManager(hardware_interface::ActuatorStateInterface& asi,
			 hardware_interface::VelocityActuatorInterface& avi,
			 hardware_interface::PositionActuatorInterface& api)
  : asi_(&asi), avi_(&avi), api_(&api) { }


void EposManager::load(XmlRpc::XmlRpcValue& motors_xml) {
  ROS_ASSERT(motors_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i = 0; i < motors_xml.size(); ++i) {
    XmlRpc::XmlRpcValue motor_xml = motors_xml[i];
    boost::shared_ptr<Epos> motor(new Epos(motor_xml, &epos_factory, *asi_, *avi_, *api_));
    motors_.push_back(motor);
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
