#include "epos_hardware/utils.h"
#include <boost/foreach.hpp>

#define MAX_STRING_SIZE 1000

int GetErrorInfo(unsigned int error_code, std::string* error_string) {
  char buffer[MAX_STRING_SIZE];
  int result;
  if(result = VCS_GetErrorInfo(error_code, buffer, MAX_STRING_SIZE)) {
    *error_string = buffer;
  }
  return result;
}


int GetDeviceNameList(std::vector<std::string>* device_names, unsigned int* error_code) {
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetDeviceNameSelection(true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if(!result)
    return result;
  device_names->push_back(buffer);

  while(!end_of_selection) {
    result = VCS_GetDeviceNameSelection(false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
      return result;
    device_names->push_back(buffer);
  }

  return 1;
}

int GetProtocolStackNameList(const std::string device_name, std::vector<std::string>* protocol_stack_names, unsigned int* error_code) {
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetProtocolStackNameSelection((char*)device_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if(!result)
    return result;
  protocol_stack_names->push_back(buffer);

  while(!end_of_selection) {
    result = VCS_GetProtocolStackNameSelection((char*)device_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
      return result;
    protocol_stack_names->push_back(buffer);
  }

  return 1;
}


int GetInterfaceNameList(const std::string device_name, const std::string protocol_stack_name, std::vector<std::string>* interface_names, unsigned int* error_code) {
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetInterfaceNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if(!result)
    return result;
  interface_names->push_back(buffer);

  while(!end_of_selection) {
    result = VCS_GetInterfaceNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
      return result;
    interface_names->push_back(buffer);
  }

  return 1;
}

int GetPortNameList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name, std::vector<std::string>* port_names, unsigned int* error_code) {
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetPortNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if(!result)
    return result;
  port_names->push_back(buffer);

  while(!end_of_selection) {
    result = VCS_GetPortNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
      return result;
    port_names->push_back(buffer);
  }

  return 1;
}

int GetBaudrateList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		    const std::string port_name, std::vector<unsigned int>* baudrates, unsigned int* error_code) {
  unsigned int baudrate;
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetBaudrateSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), (char*)port_name.c_str(), true, &baudrate, &end_of_selection, error_code);
  if(!result)
    return result;
  baudrates->push_back(baudrate);

  while(!end_of_selection) {
    result = VCS_GetBaudrateSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), (char*)port_name.c_str(), false, &baudrate, &end_of_selection, error_code);
    if(!result)
      return result;
    baudrates->push_back(baudrate);
  }

  return 1;
}

void* OpenDevice(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		 const std::string port_name, unsigned int* error_code) {
  return VCS_OpenDevice((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), (char*)port_name.c_str(), error_code);
}

int EnumerateDevices(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		     const std::string port_name, std::vector<EnumeratedDevice>* devices, unsigned int* error_code) {
  void* handle;
  if(!(handle = OpenDevice(device_name, protocol_stack_name, interface_name, port_name, error_code))){
    return 0;
  }
  for(unsigned short i = 1; i < 127; ++i) {
    EnumeratedDevice device;
    device.port_name = port_name;
    device.node_id = i;
    if(!VCS_GetVersion(handle, i, &device.hardware_version, &device.software_version, &device.application_number, &device.application_version, error_code)){
      VCS_CloseDevice(handle, error_code);
      return 1;
    }
    unsigned int bytes_read;
    if(!VCS_GetObject(handle, i, 0x2004, 0x00, &device.serial_number, 8, &bytes_read, error_code)){
      device.serial_number = 0;
    }
    devices->push_back(device);
  }
  VCS_CloseDevice(handle, error_code);
  return 1;
}


int EnumerateDevices(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		     std::vector<EnumeratedDevice>* devices, unsigned int* error_code) {
  std::vector<std::string> port_names;
  if(GetPortNameList(device_name, protocol_stack_name, interface_name, &port_names, error_code)) {
    BOOST_FOREACH(const std::string& port_name, port_names) {
      if(!EnumerateDevices(device_name, protocol_stack_name, interface_name, port_name, devices, error_code)){
	return 0;
      }
    }
    return 1;
  }
  else
    return 0;
}
