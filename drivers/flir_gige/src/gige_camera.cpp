#include "flir_gige/gige_camera.h"

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define BUFFER_COUNT (16)

namespace flir_gige {

using namespace std;

GigeCamera::GigeCamera(const std::string &ip_address)
    : ip_address_{ip_address}
//      device_{nullptr, PvDevice::Free},
//      stream_{nullptr, PvStream::Free}
{
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
    // Did not find device with given ip address
    ostringstream error_msg;
    error_msg << "GigeCamera: Device not found. Available IP Address:";
    for (const auto &dinfo : device_info_gev_vec) {
      error_msg << " " << dinfo->GetIPAddress().GetAscii();
    }
    throw runtime_error(error_msg.str());
  } else {
    // Found device with given ip address
    cout << "Found device: " << (*it)->GetIPAddress().GetAscii() << endl;
    // Is the IP address valid?
    if ((*it)->IsConfigurationValid()) {
      // Try connect and disconnect to verify
      device_info_ = *it;
      PvResult result;
      cout << "-- Connecting to " << device_info_->GetDisplayID().GetAscii()
           << endl;
      // Creates and connects the device controller
      PvDevice *device = PvDevice::CreateAndConnect(device_info_, &result);
      if (result.IsOK()) {
        cout << "-- Successfully connected to "
             << device_info_->GetDisplayID().GetAscii() << endl;
        cout << "-- Disconnecting the device "
             << device_info_->GetDisplayID().GetAscii() << endl;
        PvDevice::Free(device);
      } else {
        // Maybe throw an exception here?
        cout << "Unable to connect to "
             << device_info_->GetDisplayID().GetAscii() << endl;
      }
    }
  }
}

void GigeCamera::ConnectDevice() {
  PvResult result;
  cout << "Connecting to " << device_info_->GetDisplayID().GetAscii() << endl;
  // Use a unique_ptr to manage device resource
  device_ = PvDevicePtr(PvDevice::CreateAndConnect(device_info_, &result));
  if (result.IsOK()) {
    cout << "Successfully connected to "
         << device_info_->GetDisplayID().GetAscii() << endl;
  } else {
    ostringstream error_msg;
    error_msg << "Unable to connect to "
              << device_info_->GetDisplayID().GetAscii();
    throw runtime_error(error_msg.str());
  }
}

void GigeCamera::OpenStream() {
  cout << "Opening stream to device" << endl;
  PvResult result;
  stream_ = PvStreamPtr(PvStream::CreateAndOpen(device_info_->GetConnectionID(),
                                                &result));
  if (stream_) {
    cout << "Successfully opend stream "
         << device_info_->GetDisplayID().GetAscii() << endl;
  } else {
    // Maybe a function for throw exception?
    ostringstream error_msg;
    error_msg << "Unable to stream form "
              << device_info_->GetDisplayID().GetAscii();
    throw runtime_error(error_msg.str());
  }
}

void GigeCamera::ConfigureStream()  {
  // If this is a GigE Vision devie, configure GigE Vision specific parameters
  auto *device_gev = dynamic_cast<PvDeviceGEV*>(device_.get());
  if (device_gev) {
    cout << "Configureing gev stream" << endl;
    auto *stream_gev = static_cast<PvStreamGEV*>(stream_.get());
    // Negotiate packet size
    device_gev->NegotiatePacketSize();
    // Configure device streaming destination
    device_gev->SetStreamDestination(stream_gev->GetLocalIPAddress(),
                                     stream_gev->GetLocalPort());
  } else {
     cout << "This is not a gev device" << endl;
  }
}

void GigeCamera::CreatePipeline() {
  cout << "Creating pipeline" << endl;
  // Create the PvPipeline object
  pipeline_ = PvPipelinePtr(new PvPipeline(stream_.get()));
  // Reading payload size from device
  uint32_t payload_size =device_->GetPayloadSize();
  // Set the Buffer count and the Buffer size
  pipeline_->SetBufferCount(4);
  pipeline_->SetBufferSize(payload_size);
}

void GigeCamera::AcquireImages() {
  // Get device parameters need to control streaming
  PvGenParameterArray *device_params = device_->GetParameters();
  // Set device in RGB8
  device_params->SetEnumValue("PixelFormat", PvPixelMono8);

  // Map the GenICam AcquisitionStart and AcuisitionStop commands
  auto *start_cmd = dynamic_cast<PvGenCommand*>(device_params->Get("AcquisitionStart"));
  auto *stop_cmd = dynamic_cast<PvGenCommand*>(device_params->Get("AcquisitionStop"));

  // Note: the pipeline must be initialized before we start acquisition
  cout << "Starting pipeline" << endl;
  pipeline_->Start();

  // Get stream parameters
  PvGenParameterArray *stream_params = stream_->GetParameters();

  // Map a few GenICam stats counters
  auto *frame_rate = dynamic_cast<PvGenFloat*>(stream_params->Get("AcquisitionRate"));
  auto *band_width = dynamic_cast<PvGenFloat*>(stream_params->Get("Bandwidth"));

  // Enable streaming and send the AcquisitionStart command
  cout << "Enabling streaming and sending AcquisitionStart command" << endl;
  device_->StreamEnable();
  start_cmd->Execute();
  cout << "Start acquisition" << endl;

  double frame_rate_val = 0;
  double band_width_val = 0;
  cv::namedWindow("flir", cv::WINDOW_AUTOSIZE);
  int64_t height = 0, width = 0;
  device_params->GetIntegerValue("Width", width);
  device_params->GetIntegerValue("Height", height);
  cout << "Width: " << width << " height: " << height << endl;
  cv::Mat image_raw(cv::Size(width, height), CV_8UC1);
  // Some test to dispaly image
  for (int i = 0; i < 500; ++i) {
    PvBuffer *buffer;
    PvResult operation_result;

    // Retrieve next buffer
    PvResult result = pipeline_->RetrieveNextBuffer(&buffer, 1000, &operation_result);
    if (result.IsOK()) {
      if (operation_result.IsOK()) {
        PvPayloadType payload_type;
        // We now have a valid buffer. This is where you would typically
        // process the buffer
        frame_rate->GetValue(frame_rate_val);
        band_width->GetValue(band_width_val);

        // If the buffer contains an image, display the width and height
        uint32_t width = 0, height = 0;
        payload_type = buffer->GetPayloadType();

        if (payload_type == PvPayloadTypeImage) {
          // Get image specific buffer interface
          PvImage *image = buffer->GetImage();
          // Read width, height
          width = image->GetWidth();
          height = image->GetHeight();
//          cout << " W: " << width << " H: " << height
//               << " Px: " << image->GetPaddingX()
//               << " Py: " << image->GetPaddingY();
          // Attach
          image->Attach(image_raw.data, image->GetWidth(), image->GetHeight(),
                        PvPixelMono8, image->GetPaddingX(), image->GetPaddingY());
          // Display image
          cv::imshow("flir", image_raw);
          if (cv::waitKey(50) >= 0) break;
        } else {
          cout << "Buffer does not contain image" << endl;
        }
        cout << "  " << frame_rate_val << " FPS " << (band_width_val / 100000.0)
             << " Mb/s" << endl;
      } else {
        cout << "Non Ok operation result" << endl;
      }
      // Release the buffer back to the pipeline
      pipeline_->ReleaseBuffer(buffer);
    } else {
      cout << "Retrieve buffer failure" << endl;
    }
  }
  // Tell the device to stop sending images
  cout << "Stop acquisition" << endl;
  stop_cmd->Execute();

  // Disable streaming on the device
  cout << "Disable streaming on the controller" << endl;
  device_->StreamDisable();

  // Stop the pipeline
  cout << "Stop pipeline" << endl;
  pipeline_->Stop();
}

}  // namespace flir_gige
