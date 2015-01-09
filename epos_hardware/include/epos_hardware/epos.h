#ifndef EPOS_HARDWARE_EPOS_H_
#define EPOS_HARDWARE_EPOS_H_

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "epos_hardware/utils.h"

namespace epos_hardware {

class Epos {
public:
  typedef enum {
    PROFILE_POSITION_MODE = 1,
    PROFILE_VELOCITY_MODE = 3
  } OperationMode;

  Epos(XmlRpc::XmlRpcValue& config_xml, EposFactory* epos_factory,
       hardware_interface::ActuatorStateInterface& asi,
       hardware_interface::VelocityActuatorInterface& avi,
       hardware_interface::PositionActuatorInterface& api);
  ~Epos();
  bool init();
  void read();
  void write();
  std::string name() { return name_; }
  void buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
private:
  XmlRpc::XmlRpcValue config_xml_;
  EposFactory* epos_factory_;
  std::string name_;
  uint64_t serial_number_;
  OperationMode operation_mode_;
  NodeHandlePtr node_handle_;
  bool has_init_;

  double position_;
  double velocity_;
  double effort_;
  double current_;

  double position_cmd_;
  double velocity_cmd_;
};

}


#endif
