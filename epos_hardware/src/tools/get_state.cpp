#include <ros/ros.h>
#include "epos_hardware/utils.h"
#include "epos_library/Definitions.h"
#include <boost/foreach.hpp>

int main(int argc, char** argv){
  uint64_t serial_number;
  if(argc == 2){
    if(!SerialNumberFromHex(argv[1], &serial_number)) {
      std::cerr << "Expected a serial number" << std::endl;
      return 1;
    }
  }
  else {
    std::cerr << "Expected exactly one argument that is a serial number" << std::endl;
    return 1;
  }

  std::string error_string;
  unsigned int error_code = 0;

  std::cout << "Searching for USB EPOS2: 0x" << std::hex << serial_number << std::endl;

  std::string port_name;

  EposFactory epos_factory;

  NodeHandlePtr handle;
  if(handle = epos_factory.CreateNodeHandle("EPOS2", "MAXON SERIAL V2", "USB", serial_number, &error_code)) {
    int position;
    if(VCS_GetPositionIs(handle->device_handle->ptr, handle->node_id, &position, &error_code)){
      std::cout << "Position: " << std::dec << position << std::endl;
    }
    else {
      if(GetErrorInfo(error_code, &error_string)){
	std::cerr << "Could not get position: " << error_string << std::endl;
      } else {
	std::cerr << "Could not get position" << std::endl;
      }
    }

    int velocity;
    if(VCS_GetVelocityIs(handle->device_handle->ptr, handle->node_id, &velocity, &error_code)){
      std::cout << "Velocity: " << std::dec << velocity << std::endl;
    }
    else {
      if(GetErrorInfo(error_code, &error_string)){
	std::cerr << "Could not get velocity: " << error_string << std::endl;
      } else {
	std::cerr << "Could not get velocity" << std::endl;
      }
    }

    short current;
    if(VCS_GetCurrentIs(handle->device_handle->ptr, handle->node_id, &current, &error_code)){
      std::cout << "Current: " << std::dec << current << std::endl;
    }
    else {
      if(GetErrorInfo(error_code, &error_string)){
	std::cerr << "Could not get current: " << error_string << std::endl;
      } else {
	std::cerr << "Could not get current" << std::endl;
      }
    }

  }
  else {
    if(GetErrorInfo(error_code, &error_string)){
      std::cerr << "Could not open device: " << error_string << std::endl;
    } else {
      std::cerr << "Could not open device" << std::endl;
    }
    return 1;
  }
}
