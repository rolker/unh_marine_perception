#include "depthai/device/Device.hpp"

#include <iostream>

int main(int argc, char **argv)
{
  auto available_devices = dai::Device::getAllAvailableDevices();
  for(auto device_info: available_devices)
  {
    std::cout << device_info.toString() << std::endl;
  }
  return 0;
}
