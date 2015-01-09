#include "epos_hardware/epos.h"

namespace epos_hardware {

Epos::Epos(XmlRpc::XmlRpcValue& config_xml, EposFactory* epos_factory,
	   hardware_interface::ActuatorStateInterface& asi,
	   hardware_interface::VelocityActuatorInterface& avi,
	   hardware_interface::PositionActuatorInterface& api)
  : config_xml_(config_xml), epos_factory_(epos_factory),
    has_init_(false),
    position_(0), velocity_(0), effort_(0), current_(0),
    position_cmd_(0), velocity_cmd_(0) {
  ROS_ASSERT(config_xml_.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  ROS_ASSERT(config_xml_["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
  name_ = static_cast<std::string>(config_xml_["name"]);

  ROS_ASSERT(config_xml_["serial_number"].getType() == XmlRpc::XmlRpcValue::TypeString);
  ROS_ASSERT(SerialNumberFromHex(static_cast<std::string>(config_xml_["serial_number"]), &serial_number_));

  ROS_ASSERT(config_xml_["operation_mode"].getType() == XmlRpc::XmlRpcValue::TypeInt);
  operation_mode_ = (OperationMode)static_cast<int>(config_xml_["operation_mode"]);

  hardware_interface::ActuatorStateHandle state_handle(name_, &position_, &velocity_, &effort_);
  asi.registerHandle(state_handle);

  hardware_interface::ActuatorHandle position_handle(state_handle, &position_cmd_);
  api.registerHandle(position_handle);
  hardware_interface::ActuatorHandle velocity_handle(state_handle, &velocity_cmd_);
  avi.registerHandle(velocity_handle);
}
Epos::~Epos() {
  unsigned int error_code;
  if(node_handle_)
    VCS_SetDisableState(node_handle_->device_handle->ptr, node_handle_->node_id, &error_code);
}

bool Epos::init() {
  ROS_INFO_STREAM("Initializing: 0x" << std::hex << serial_number_);
  unsigned int error_code;
  node_handle_ = epos_factory_->CreateNodeHandle("EPOS2", "MAXON SERIAL V2", "USB", serial_number_, &error_code);
  if(!node_handle_)
    return false;
  ROS_INFO_STREAM("Found Motor");

  if(!VCS_SetProtocolStackSettings(node_handle_->device_handle->ptr, 1000000, 500, &error_code)){
    return false;
  }

  if(!VCS_SetDisableState(node_handle_->device_handle->ptr, node_handle_->node_id, &error_code))
    return false;


  if(!VCS_SetOperationMode(node_handle_->device_handle->ptr, node_handle_->node_id, operation_mode_, &error_code))
    return false;

  ROS_INFO("Configuring Motor");
  {
    XmlRpc::XmlRpcValue& motor_xml = config_xml_["motor"];
    ROS_ASSERT(motor_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    ROS_ASSERT(motor_xml["type"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if(!VCS_SetMotorType(node_handle_->device_handle->ptr, node_handle_->node_id, static_cast<int>(motor_xml["type"]), &error_code))
      return false;

    if(motor_xml.hasMember("dc_motor")) {
      XmlRpc::XmlRpcValue& dc_motor_xml = motor_xml["dc_motor"];
      ROS_ASSERT(dc_motor_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(dc_motor_xml["nominal_current"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(dc_motor_xml["max_output_current"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(dc_motor_xml["thermal_time_constant"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      if(!VCS_SetDcMotorParameter(node_handle_->device_handle->ptr, node_handle_->node_id,
				  1000 * static_cast<double>(dc_motor_xml["nominal_current"]), // A -> mA
				  1000 * static_cast<double>(dc_motor_xml["max_output_current"]), // A -> mA
				  10 * static_cast<double>(dc_motor_xml["thermal_time_constant"]), // s -> 100ms
				  &error_code))
	return false;
    }

    if(motor_xml.hasMember("ec_motor")) {
      XmlRpc::XmlRpcValue& ec_motor_xml = motor_xml["ec_motor"];
      ROS_ASSERT(ec_motor_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(ec_motor_xml["nominal_current"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(ec_motor_xml["max_output_current"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(ec_motor_xml["thermal_time_constant"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(ec_motor_xml["number_of_pole_pairs"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetEcMotorParameter(node_handle_->device_handle->ptr, node_handle_->node_id,
				  1000 * static_cast<double>(ec_motor_xml["nominal_current"]), // A -> mA
				  1000 * static_cast<double>(ec_motor_xml["max_output_current"]), // A -> mA
				  10 * static_cast<double>(ec_motor_xml["thermal_time_constant"]), // s -> 100ms
				  static_cast<int>(ec_motor_xml["number_of_pole_pairs"]),
				  &error_code))
	return false;
    }
  }

  ROS_INFO("Configuring Sensor");
  {
    XmlRpc::XmlRpcValue& sensor_xml = config_xml_["sensor"];
    ROS_ASSERT(sensor_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    ROS_ASSERT(sensor_xml["type"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if(!VCS_SetSensorType(node_handle_->device_handle->ptr, node_handle_->node_id, static_cast<int>(sensor_xml["type"]), &error_code))
      return false;

    if(sensor_xml.hasMember("incremental_encoder")) {
      XmlRpc::XmlRpcValue& inc_encoder_xml = sensor_xml["incremental_encoder"];
      ROS_ASSERT(inc_encoder_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(inc_encoder_xml["resolution"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(inc_encoder_xml["inverted_polarity"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
      if(!VCS_SetIncEncoderParameter(node_handle_->device_handle->ptr, node_handle_->node_id,
				     static_cast<int>(inc_encoder_xml["resolution"]),
				     static_cast<bool>(inc_encoder_xml["inverted_polarity"]),
				     &error_code))
	return false;
    }

    if(sensor_xml.hasMember("hall_sensor")) {
      XmlRpc::XmlRpcValue& hall_sensor_xml = sensor_xml["hall_sensor"];
      ROS_ASSERT(hall_sensor_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(hall_sensor_xml["inverted_polarity"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
      if(!VCS_SetHallSensorParameter(node_handle_->device_handle->ptr, node_handle_->node_id,
				     static_cast<bool>(hall_sensor_xml["inverted_polarity"]),
				     &error_code))
	return false;
    }

    if(sensor_xml.hasMember("ssi_absolute_encoder")) {
      XmlRpc::XmlRpcValue& ssi_absolute_encoder_xml = sensor_xml["ssi_absolute_encoder"];
      ROS_ASSERT(ssi_absolute_encoder_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(ssi_absolute_encoder_xml["data_rate"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(ssi_absolute_encoder_xml["number_of_multiturn_bits"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(ssi_absolute_encoder_xml["number_of_singleturn_bits"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(ssi_absolute_encoder_xml["inverted_polarity"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
      if(!VCS_SetSsiAbsEncoderParameter(node_handle_->device_handle->ptr, node_handle_->node_id,
					static_cast<int>(ssi_absolute_encoder_xml["data_rate"]),
					static_cast<int>(ssi_absolute_encoder_xml["number_of_multiturn_bits"]),
					static_cast<int>(ssi_absolute_encoder_xml["number_of_singleturn_bits"]),
					static_cast<bool>(ssi_absolute_encoder_xml["inverted_polarity"]),
					&error_code))
	return false;
    }

  }

  if(config_xml_.hasMember("safety")) {
    ROS_INFO("Configuring Safety");
    XmlRpc::XmlRpcValue& safety_xml = config_xml_["safety"];
    ROS_ASSERT(safety_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if(safety_xml.hasMember("max_following_error")) {
      ROS_ASSERT(safety_xml["max_following_error"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetMaxFollowingError(node_handle_->device_handle->ptr, node_handle_->node_id,
				  static_cast<int>(safety_xml["max_following_error"]),
				  &error_code))
	return false;
    }

    if(safety_xml.hasMember("max_profile_velocity")) {
      ROS_ASSERT(safety_xml["max_profile_velocity"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetMaxProfileVelocity(node_handle_->device_handle->ptr, node_handle_->node_id,
				  static_cast<int>(safety_xml["max_profile_velocity"]),
				  &error_code))
	return false;
    }

    if(safety_xml.hasMember("max_acceleration")) {
      ROS_ASSERT(safety_xml["max_acceleration"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetMaxAcceleration(node_handle_->device_handle->ptr, node_handle_->node_id,
				  static_cast<int>(safety_xml["max_acceleration"]),
				  &error_code))
	return false;
    }
  }

  if(config_xml_.hasMember("position_regulator")) {
    ROS_INFO("Position Regulator");
    XmlRpc::XmlRpcValue& position_xml = config_xml_["position_regulator"];
    ROS_ASSERT(position_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if(position_xml.hasMember("gain")) {
      XmlRpc::XmlRpcValue& gain_xml = position_xml["gain"];
      ROS_ASSERT(gain_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(gain_xml["p"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(gain_xml["i"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(gain_xml["d"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetPositionRegulatorGain(node_handle_->device_handle->ptr, node_handle_->node_id,
				       static_cast<int>(gain_xml["p"]),
				       static_cast<int>(gain_xml["i"]),
				       static_cast<int>(gain_xml["d"]),
				       &error_code))
	return false;
    }

    if(position_xml.hasMember("feed_forward")) {
      XmlRpc::XmlRpcValue& feed_forward_xml = position_xml["feed_forward"];
      ROS_ASSERT(feed_forward_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(feed_forward_xml["velocity"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(feed_forward_xml["acceleration"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetPositionRegulatorFeedForward(node_handle_->device_handle->ptr, node_handle_->node_id,
					      static_cast<int>(feed_forward_xml["velocity"]),
					      static_cast<int>(feed_forward_xml["acceleration"]),
					      &error_code))
	return false;
    }

  }

  if(config_xml_.hasMember("velocity_regulator")) {
    ROS_INFO("Velocity Regulator");
    XmlRpc::XmlRpcValue& velocity_xml = config_xml_["velocity_regulator"];
    ROS_ASSERT(velocity_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if(velocity_xml.hasMember("gain")) {
      XmlRpc::XmlRpcValue& gain_xml = velocity_xml["gain"];
      ROS_ASSERT(gain_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(gain_xml["p"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(gain_xml["i"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetVelocityRegulatorGain(node_handle_->device_handle->ptr, node_handle_->node_id,
				       static_cast<int>(gain_xml["p"]),
				       static_cast<int>(gain_xml["i"]),
				       &error_code))
	return false;
    }

    if(velocity_xml.hasMember("feed_forward")) {
      XmlRpc::XmlRpcValue& feed_forward_xml = velocity_xml["feed_forward"];
      ROS_ASSERT(feed_forward_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(feed_forward_xml["velocity"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(feed_forward_xml["acceleration"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetVelocityRegulatorFeedForward(node_handle_->device_handle->ptr, node_handle_->node_id,
					      static_cast<int>(feed_forward_xml["velocity"]),
					      static_cast<int>(feed_forward_xml["acceleration"]),
					      &error_code))
	return false;
    }

  }

  if(config_xml_.hasMember("current_regulator")) {
    ROS_INFO("Current Regulator");
    XmlRpc::XmlRpcValue& current_xml = config_xml_["current_regulator"];
    ROS_ASSERT(current_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if(current_xml.hasMember("gain")) {
      XmlRpc::XmlRpcValue& gain_xml = current_xml["gain"];
      ROS_ASSERT(gain_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(gain_xml["p"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(gain_xml["i"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if(!VCS_SetCurrentRegulatorGain(node_handle_->device_handle->ptr, node_handle_->node_id,
				      static_cast<int>(gain_xml["p"]),
				      static_cast<int>(gain_xml["i"]),
				      &error_code))
	return false;
    }
  }

  if(config_xml_.hasMember("position_profile")) {
    ROS_INFO("Position Profile");
    XmlRpc::XmlRpcValue& position_xml = config_xml_["position_profile"];
    ROS_ASSERT(position_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(position_xml["velocity"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(position_xml["acceleration"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(position_xml["deceleration"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if(!VCS_SetPositionProfile(node_handle_->device_handle->ptr, node_handle_->node_id,
			       static_cast<int>(position_xml["velocity"]),
			       static_cast<int>(position_xml["acceleration"]),
			       static_cast<int>(position_xml["deceleration"]),
			       &error_code))
      return false;
    if(position_xml.hasMember("window")) {
      XmlRpc::XmlRpcValue& window_xml = position_xml["window"];
      ROS_ASSERT(window_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(window_xml["window"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(window_xml["time"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      if(!VCS_EnablePositionWindow(node_handle_->device_handle->ptr, node_handle_->node_id,
				   static_cast<int>(window_xml["window"]),
				   1000 * static_cast<double>(window_xml["time"]), // s -> ms
				   &error_code))
	return false;
    }
  }



  if(config_xml_.hasMember("velocity_profile")) {
    ROS_INFO("Velocity Profile");
    XmlRpc::XmlRpcValue& velocity_xml = config_xml_["velocity_profile"];
    ROS_ASSERT(velocity_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(velocity_xml["acceleration"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(velocity_xml["deceleration"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if(!VCS_SetVelocityProfile(node_handle_->device_handle->ptr, node_handle_->node_id,
				    static_cast<int>(velocity_xml["acceleration"]),
				    static_cast<int>(velocity_xml["deceleration"]),
				    &error_code))
      return false;
    if(velocity_xml.hasMember("window")) {
      XmlRpc::XmlRpcValue& window_xml = velocity_xml["window"];
      ROS_ASSERT(window_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(window_xml["window"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(window_xml["time"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      if(!VCS_EnableVelocityWindow(node_handle_->device_handle->ptr, node_handle_->node_id,
				   static_cast<int>(window_xml["window"]),
				   1000 * static_cast<double>(window_xml["time"]), // s -> ms
				   &error_code))
	return false;
    }
  }



  ROS_INFO("Querying Faults");
  unsigned char num_errors;
  if(!VCS_GetNbOfDeviceError(node_handle_->device_handle->ptr, node_handle_->node_id, &num_errors, &error_code))
    return false;
  for(int i = 1; i<= num_errors; ++i) {
    unsigned int error_number;
    if(!VCS_GetDeviceErrorCode(node_handle_->device_handle->ptr, node_handle_->node_id, i, &error_number, &error_code))
      return false;
    ROS_WARN_STREAM("EPOS Device Error: 0x" << std::hex << error_number);
  }

  if(config_xml_.hasMember("clear_faults")) {
    ROS_ASSERT(config_xml_["clear_faults"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
    if(static_cast<bool>(config_xml_["clear_faults"])){
      if(!VCS_ClearFault(node_handle_->device_handle->ptr, node_handle_->node_id, &error_code))
	return false;
      else
	ROS_INFO_STREAM("Cleared faults");
    }
  }

  ROS_INFO_STREAM("Enabling Motor");
  if(!VCS_SetEnableState(node_handle_->device_handle->ptr, node_handle_->node_id, &error_code))
    return false;

  has_init_ = true;
  return true;
}
void Epos::read() {
  if(!has_init_)
    return;

  unsigned int error_code;
  int position_raw;
  int velocity_raw;
  short current_raw;
  VCS_GetPositionIs(node_handle_->device_handle->ptr, node_handle_->node_id, &position_raw, &error_code);
  VCS_GetVelocityIs(node_handle_->device_handle->ptr, node_handle_->node_id, &velocity_raw, &error_code);
  VCS_GetCurrentIs(node_handle_->device_handle->ptr, node_handle_->node_id, &current_raw, &error_code);
  position_ = position_raw;
  velocity_ = velocity_raw;
  effort_ = 0;
  current_ = current_raw  * 1000.0; // mA -> A
}

void Epos::write() {
  if(!has_init_)
    return;

  unsigned int error_code;
  if(operation_mode_ == PROFILE_VELOCITY_MODE) {
    VCS_MoveWithVelocity(node_handle_->device_handle->ptr, node_handle_->node_id, (int)velocity_cmd_, &error_code);
  }
  else if(operation_mode_ == PROFILE_POSITION_MODE) {
    VCS_MoveToPosition(node_handle_->device_handle->ptr, node_handle_->node_id, (int)position_cmd_, true, true, &error_code);
  }
}

void Epos::buildStatus(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.add("Serial Number", serial_number_);

  unsigned int error_code;
  if(has_init_) {
    unsigned short state;
    std::string state_str;
    if(VCS_GetState(node_handle_->device_handle->ptr, node_handle_->node_id, &state, &error_code)) {
      if(state == ST_DISABLED) {
	stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Disabled");
	state_str = "Disabled";
      }
      else if(state == ST_ENABLED) {
	stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Enabled");
	state_str = "Enabled";
      }
      else if(state == ST_QUICKSTOP) {
	stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Quickstop");
	state_str = "Quickstop";
      }
      else if(state == ST_FAULT) {
	stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Fault");
	state_str = "Fault";
      }
      else {
	stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown State");
	state_str = "Unknown";
      }
      stat.add("State", state_str);
    }
    else {
      std::string error_str;
      if(GetErrorInfo(error_code, &error_str)) {
	std::stringstream error_msg;
	error_msg << "Could not read state: " << error_str;
	stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
      }
      else {
	stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not read state");
      }
      return;
    }

    unsigned char num_errors;
    if(VCS_GetNbOfDeviceError(node_handle_->device_handle->ptr, node_handle_->node_id, &num_errors, &error_code)) {
      for(int i = 1; i<= num_errors; ++i) {
	unsigned int error_number;
	if(VCS_GetDeviceErrorCode(node_handle_->device_handle->ptr, node_handle_->node_id, i, &error_number, &error_code)) {
	  std::stringstream error_msg;
	  error_msg << "EPOS Device Error: 0x" << std::hex << error_number;
	  stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
	}
	else {
	  std::string error_str;
	  if(GetErrorInfo(error_code, &error_str)) {
	    std::stringstream error_msg;
	    error_msg << "Could not read device error: " << error_str;
	    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
	  }
	  else {
	    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not read device error");
	  }
	}
      }
    }
    else {
      std::string error_str;
      if(GetErrorInfo(error_code, &error_str)) {
	std::stringstream error_msg;
	error_msg << "Could not read device errors: " << error_str;
	stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
      }
      else {
	stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not read device errors");
      }
    }


  }
  else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "EPOS not initialized");
  }
}


}
