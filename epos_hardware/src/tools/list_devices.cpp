#include <ros/ros.h>
#include "epos_hardware/utils.h"
#include "epos_library/Definitions.h"
#include <boost/foreach.hpp>

int main(int argc, char** argv){
  bool skip_rs232 = true;
  if(argc == 2){
    if(!strcmp(argv[1], "--rs232"))
      skip_rs232 = false;
    else{
      std::cerr << "unknown option: " << argv[1] << std::endl;
      return 1;
    }
  }
  else if(argc > 1){
    std::cerr << "unknown options" << std::endl;
    return 1;
  }

  std::string error_string;
  unsigned int error_code = 0;

  EposFactory epos_factory;

  std::cout << "Listing Devices:" << std::endl;

  std::vector<std::string> device_names;
  if(GetDeviceNameList(&device_names, &error_code)) {
    BOOST_FOREACH(const std::string& device_name, device_names) {
      std::cout << device_name <<std::endl;

      std::vector<std::string> protocol_stack_names;
      if(GetProtocolStackNameList(device_name, &protocol_stack_names, &error_code)) {
	BOOST_FOREACH(const std::string& protocol_stack_name, protocol_stack_names) {
	  std::cout << "\t" << protocol_stack_name <<std::endl;

	  std::vector<std::string> interface_names;
	  if(GetInterfaceNameList(device_name, protocol_stack_name, &interface_names, &error_code)) {
	    BOOST_FOREACH(const std::string& interface_name, interface_names) {
	      std::cout << "\t\t" << interface_name <<std::endl;
	      if(skip_rs232 && interface_name == "RS232"){
		std::cout << "\t\t\tSkipping RS232" <<std::endl;
		continue;
	      }

	      std::vector<std::string> port_names;
	      if(GetPortNameList(device_name, protocol_stack_name, interface_name, &port_names, &error_code)) {
		BOOST_FOREACH(const std::string& port_name, port_names) {
		  std::cout << "\t\t\t" << port_name <<std::endl;

		  std::vector<unsigned int> baudrates;
		  if(GetBaudrateList(device_name, protocol_stack_name, interface_name, port_name, &baudrates, &error_code)) {
		    std::cout << "\t\t\t\tBaudrates:" << std::endl;
		    BOOST_FOREACH(unsigned int baudrate, baudrates) {
		      std::cout << "\t\t\t\t\t" << std::dec << baudrate <<std::endl;
		    }

		    std::vector<EnumeratedNode> devices;
		    if(epos_factory.EnumerateNodes(device_name, protocol_stack_name, interface_name, port_name, &devices, &error_code)) {
		      std::cout << "\t\t\t\tDevices:" << std::endl;
		      BOOST_FOREACH(const EnumeratedNode& node, devices) {
			std::cout << "\t\t\t\t\tNode Id: " << std::dec << node.node_id << std::endl;
			std::cout << "\t\t\t\t\t\tSerial Number: 0x" << std::hex << node.serial_number << std::endl;
			std::cout << "\t\t\t\t\t\tHardware Version: 0x" << std::hex << node.hardware_version << std::endl;
			std::cout << "\t\t\t\t\t\tSoftware Version: 0x" << std::hex << node.software_version << std::endl;
			std::cout << "\t\t\t\t\t\tApplication Number: 0x" << std::hex << node.application_number << std::endl;
			std::cout << "\t\t\t\t\t\tApplication Version: 0x" << std::hex << node.application_version << std::endl;
		      }
		    }
		    else {
		      if(GetErrorInfo(error_code, &error_string)){
			std::cerr << "Could not enumerate devices: " << error_string << std::endl;
		      } else {
			std::cerr << "Could not enumerate devices" << std::endl;
		      }
		    }

		  }
		  else {
		    if(GetErrorInfo(error_code, &error_string)){
		      std::cerr << "Could not get baudrates: " << error_string << std::endl;
		    } else {
		      std::cerr << "Could not get baudrates" << std::endl;
		    }
		  }

		}
	      }
	      else {
		if(GetErrorInfo(error_code, &error_string)){
		  std::cerr << "Could not get port names: " << error_string << std::endl;
		} else {
		  std::cerr << "Could not get port names" << std::endl;
		}
	      }

	    }
	  }
	  else {
	    if(GetErrorInfo(error_code, &error_string)){
	      std::cerr << "Could not get interface names: " << error_string << std::endl;
	    } else {
	      std::cerr << "Could not get interface names" << std::endl;
	    }
	  }

	}
      }
      else {
	if(GetErrorInfo(error_code, &error_string)){
	  std::cerr << "Could not get protocol stack names: " << error_string << std::endl;
	} else {
	  std::cerr << "Could not get protocol stack names" << std::endl;
	}
      }

    }
  }
  else {
    if(GetErrorInfo(error_code, &error_string)){
      std::cerr << "Could not get device names: " << error_string << std::endl;
    } else {
      std::cerr << "Could not get device names" << std::endl;
    }
  }
}
