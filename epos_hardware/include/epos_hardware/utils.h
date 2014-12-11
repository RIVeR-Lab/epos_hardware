#include <string>
#include <vector>
#include <stdint.h>
#include "epos_library/Definitions.h"

int GetErrorInfo(unsigned int error_code, std::string* error_string);

int GetDeviceNameList(std::vector<std::string>* device_names, unsigned int* error_code);

int GetProtocolStackNameList(const std::string device_name, std::vector<std::string>* protocol_stack_names, unsigned int* error_code);

int GetInterfaceNameList(const std::string device_name, const std::string protocol_stack_name,
			 std::vector<std::string>* interface_names, unsigned int* error_code);

int GetPortNameList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		    std::vector<std::string>* port_names, unsigned int* error_code);

int GetBaudrateList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		    const std::string port_name, std::vector<unsigned int>* port_names, unsigned int* error_code);

void* OpenDevice(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		 const std::string port_name, unsigned int* error_code);


typedef struct {
  std::string port_name;
  unsigned short node_id;
  uint64_t serial_number;
  unsigned short hardware_version;
  unsigned short software_version;
  unsigned short application_number;
  unsigned short application_version;
} EnumeratedDevice;

int EnumerateDevices(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		     const std::string port_name, std::vector<EnumeratedDevice>* devices, unsigned int* error_code);

int EnumerateDevices(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
		     std::vector<EnumeratedDevice>* devices, unsigned int* error_code);
