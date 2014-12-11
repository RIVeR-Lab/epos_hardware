#include <ros/ros.h>
#include "epos_hardware/utils.h"
#include "epos_library/Definitions.h"
#include <boost/foreach.hpp>

int main(int argc, char** argv){
  uint64_t serial_number;
  if(argc == 2){
    try {
      std::stringstream ss;
      ss << std::hex << argv[1];
      ss >> serial_number;
    } catch(boost::bad_lexical_cast &e) {
      std::cerr << e.what() << std::endl;
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
  unsigned short node_id;

  std::vector<EnumeratedDevice> devices;
  if(EnumerateDevices("EPOS2", "MAXON SERIAL V2", "USB", &devices, &error_code)) {
    BOOST_FOREACH(const EnumeratedDevice& device, devices) {
      if(device.serial_number == serial_number){
	port_name = device.port_name;
	node_id = device.node_id;
	break;
      }
    }
  }
  else {
    if(GetErrorInfo(error_code, &error_string)){
      std::cerr << "Could not enumerate devices: " << error_string << std::endl;
    } else {
      std::cerr << "Could not enumerate devices" << std::endl;
    }
    return 1;
  }

  std::cout << "Found device on " << port_name << " : " << std::dec << node_id << std::endl;

  void* handle;
  if(handle = OpenDevice("EPOS2", "MAXON SERIAL V2", "USB", port_name, &error_code)) {
    int position;
    if(VCS_GetPositionIs(handle, node_id, &position, &error_code)){
      std::cout << "Position: " << position << std::endl;
    }
    else {
      if(GetErrorInfo(error_code, &error_string)){
	std::cerr << "Could not get position: " << error_string << std::endl;
      } else {
	std::cerr << "Could not get position" << std::endl;
      }
    }

    int velocity;
    if(VCS_GetVelocityIs(handle, node_id, &velocity, &error_code)){
      std::cout << "Velocity: " << velocity << std::endl;
    }
    else {
      if(GetErrorInfo(error_code, &error_string)){
	std::cerr << "Could not get velocity: " << error_string << std::endl;
      } else {
	std::cerr << "Could not get velocity" << std::endl;
      }
    }

    short current;
    if(VCS_GetCurrentIs(handle, node_id, &current, &error_code)){
      std::cout << "Current: " << current << std::endl;
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
  VCS_CloseDevice(handle, &error_code);
}
