#ifndef EPOS_HARDWARE_UTILS_H_
#define EPOS_HARDWARE_UTILS_H_

#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include "epos_library/Definitions.h"
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

bool SerialNumberFromHex(const std::string& str, uint64_t* serial_number);

int GetErrorInfo(unsigned int error_code, std::string* error_string);

int GetDeviceNameList(std::vector<std::string>* device_names, unsigned int* error_code);

int GetProtocolStackNameList(const std::string device_name, std::vector<std::string>* protocol_stack_names, unsigned int* error_code);

int GetInterfaceNameList(const std::string device_name, const std::string protocol_stack_name,
			 std::vector<std::string>* interface_names, unsigned int* error_code);

int GetPortNameList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		    std::vector<std::string>* port_names, unsigned int* error_code);

int GetBaudrateList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		    const std::string port_name, std::vector<unsigned int>* port_names, unsigned int* error_code);


// An object that wraps a handle and closes it when destroyed
class DeviceHandle {
public:
  DeviceHandle(void* ptr) : ptr(ptr) {}
  ~DeviceHandle() {
    unsigned int error_code;
    VCS_CloseDevice(const_cast<void*>(ptr), &error_code);
  }
  void* const ptr;
};
typedef boost::shared_ptr<DeviceHandle> DeviceHandlePtr;

class NodeHandle {
public:
  NodeHandle(DeviceHandlePtr device_handle, unsigned short node_id)
    : device_handle(device_handle), node_id(node_id) {}
  const DeviceHandlePtr device_handle;
  const unsigned short node_id;
};
typedef boost::shared_ptr<NodeHandle> NodeHandlePtr;

typedef struct {
  std::string device_name;
  std::string protocol_stack_name;
  std::string interface_name;
  std::string port_name;
  unsigned short node_id;
  uint64_t serial_number;
  unsigned short hardware_version;
  unsigned short software_version;
  unsigned short application_number;
  unsigned short application_version;
} EnumeratedNode;

class EposFactory {
public:

  EposFactory();
  DeviceHandlePtr CreateDeviceHandle(const std::string device_name,
				     const std::string protocol_stack_name,
				     const std::string interface_name,
				     const std::string port_name,
				     unsigned int* error_code);

  NodeHandlePtr CreateNodeHandle(const std::string device_name,
				 const std::string protocol_stack_name,
				 const std::string interface_name,
				 uint64_t serial_number,
				 unsigned int* error_code);

  NodeHandlePtr CreateNodeHandle(const EnumeratedNode& node,
				 unsigned int* error_code);


  int EnumerateNodes(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		     const std::string port_name, std::vector<EnumeratedNode>* devices, unsigned int* error_code);

  int EnumerateNodes(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		     std::vector<EnumeratedNode>* devices, unsigned int* error_code);


private:
  std::map<std::string, boost::weak_ptr<DeviceHandle> > existing_handles;
};


#endif
