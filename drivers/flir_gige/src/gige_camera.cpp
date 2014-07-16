#include "flir_gige/gige_camera.h"

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <algorithm>

#define BUFFER_COUNT (16)

namespace flir_gige {

using namespace std;

GigeCamera::GigeCamera(const std::string &ip_address)
    : ip_address_{ip_address} {
  // Find all devices on the network
  PvResult result = system_.Find();
  if (!result.IsOK()) {
    ostringstream error_msg;
    error_msg << "PvSystem::Find Error: " << result.GetCodeString().GetAscii();
    throw runtime_error(error_msg.str());
  }
}

void GigeCamera::FindDevice() {
  uint32_t interface_count = system_.GetInterfaceCount();
  cout << "Interface count: " << interface_count << endl;

  // Go through all interfaces, but we only care about network interface
  // For other interfaces such as usb, refer to sample code DeviceFinder.cpp
  vector<const PvDeviceInfoGEV *> device_info_gev_vec;
  for (uint32_t i = 0; i < interface_count; ++i) {
    cout << "Interface: " << i << endl;
    // Get pointer to the interface
    const PvInterface *interface = system_.GetInterface(i);
    // Is it a PvNetworkAdapter?
    const auto *nic = dynamic_cast<const PvNetworkAdapter *>(interface);
    if (nic) {
      cout << "  Interface: " << interface->GetDisplayID().GetAscii() << endl;
      // Go through all the devices attached to the network interface
      for (uint32_t j = 0; j < interface->GetDeviceCount(); ++j) {
        const PvDeviceInfo *device_info = interface->GetDeviceInfo(j);
        cout << "  Device: " << j << endl;
        // Is it a GigE Vision device?
        const auto *device_info_gev =
            dynamic_cast<const PvDeviceInfoGEV *>(device_info);
        if (device_info_gev) {
          cout << "    Display ID: " << device_info->GetDisplayID().GetAscii()
               << endl;
          device_info_gev_vec.push_back(device_info_gev);
        }
      }
    }
  }

  // Check GigE devices found on network adaptor
  if (device_info_gev_vec.empty()) {
    throw runtime_error("GigeCamera: No device found");
  }
  // Try finding the device with the correct ip address
  string ip(ip_address_);
  const auto it =
      find_if(device_info_gev_vec.cbegin(), device_info_gev_vec.cend(),
              [ip](const PvDeviceInfoGEV *dinfo) {
        return ip == dinfo->GetIPAddress().GetAscii();
      });

  if (it == device_info_gev_vec.end()) {
    ostringstream error_msg;
    error_msg << "GigeCamera: Device not found. Available IP Address:";
    for (const auto &dinfo: device_info_gev_vec) {
      error_msg << " " << dinfo->GetIPAddress().GetAscii();
    }
    throw runtime_error(error_msg.str());
  } else {
    cout << "Found device: " << (*it)->GetIPAddress().GetAscii() << endl;
    // Is the IP address valid?
    if ((*it)->IsConfigurationValid()) {
      // Try connect and disconnect to verify
      device_info_ = *it;
      PvResult result;
      cout << "Connecting to " << device_info_->GetDisplayID().GetAscii()
           << endl;
      // Creates and connects the device controller
      PvDevice *device = PvDevice::CreateAndConnect(device_info_, &result);
      if (result.IsOK()) {
        cout << "Successfully connected to "
             << device_info_->GetDisplayID().GetAscii() << endl;
        cout << "Disconnecting the device "
             << device_info_->GetDisplayID().GetAscii() << endl;
        PvDevice::Free(device);
      } else {
        cout << "Unable to connect to "
             << device_info_->GetDisplayID().GetAscii() << endl;
      }
    }
  }
}

}  // namespace flir_gige
