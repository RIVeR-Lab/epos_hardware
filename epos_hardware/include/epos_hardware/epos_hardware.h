#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "epos_hardware/utils.h"


namespace epos_hardware {

class Epos {
public:
  typedef enum {
    PROFILE_POSITION_MODE = 1,
    PROFILE_VELOCITY_MODE = 3
  } OperationMode;

  Epos(XmlRpc::XmlRpcValue& config_xml, EposFactory* epos_factory);
  ~Epos();
  bool init();
  void read();
  void write();
private:
  XmlRpc::XmlRpcValue config_xml_;
  EposFactory* epos_factory_;
  std::string name_;
  uint64_t serial_number_;
  OperationMode operation_mode_;
  NodeHandlePtr node_handle_;
};

class EposHardware : public hardware_interface::RobotHW {
public:
  EposHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void init();
  void read();
  void write();
private:
  std::vector<boost::shared_ptr<Epos> > motors;
  EposFactory epos_factory;
};


}
