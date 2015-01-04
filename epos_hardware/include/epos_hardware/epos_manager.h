#ifndef EPOS_HARDWARE_EPOS_MANAGER_H_
#define EPOS_HARDWARE_EPOS_MANAGER_H_

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include "epos_hardware/utils.h"
#include "epos_hardware/epos.h"


namespace epos_hardware {

class EposManager {
public:
  EposManager(hardware_interface::ActuatorStateInterface& asi,
	      hardware_interface::VelocityActuatorInterface& avi,
	      hardware_interface::PositionActuatorInterface& api);
  void load(XmlRpc::XmlRpcValue& motors_xml);
  bool init();
  void read();
  void write();
  std::vector<boost::shared_ptr<Epos> > motors() { return motors_; };
private:
  std::vector<boost::shared_ptr<Epos> > motors_;
  EposFactory epos_factory;

  hardware_interface::ActuatorStateInterface* asi_;
  hardware_interface::VelocityActuatorInterface* avi_;
  hardware_interface::PositionActuatorInterface* api_;
};

}


#endif
