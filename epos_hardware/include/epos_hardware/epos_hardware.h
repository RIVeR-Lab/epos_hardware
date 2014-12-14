#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
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
private:
  XmlRpc::XmlRpcValue config_xml_;
  EposFactory* epos_factory_;
  std::string name_;
  uint64_t serial_number_;
  OperationMode operation_mode_;
  NodeHandlePtr node_handle_;

  double position_;
  double velocity_;
  double effort_;
  double current_;

  double position_cmd_;
  double velocity_cmd_;
};

class EposHardware : public hardware_interface::RobotHW {
public:
  EposHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  bool init();
  void read();
  void write();
private:
  std::vector<std::string> motor_names;
  std::vector<boost::shared_ptr<Epos> > motors;
  EposFactory epos_factory;

  transmission_interface::RobotTransmissions robot_transmissions;
  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader;

  hardware_interface::ActuatorStateInterface asi;
  hardware_interface::VelocityActuatorInterface avi;
  hardware_interface::PositionActuatorInterface api;
};


}
