#include "flir_gige/gige_camera.h"

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

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
  FindDevice();
}

void GigeCamera::Connect() {
  ConnectDevice();
  OpenStream();
  ConfigureStream();
  CreatePipeline();
}

void GigeCamera::Disconnect() {
  // Release all the recourses we had
  pipeline_.reset();
  stream_.reset();
  device_.reset();
}

void GigeCamera::Configure(bool color) {
  // Do nothing now, just print something
  color_ = color;
}

void GigeCamera::Start() {
  StartAcquisition();
  // Set acquire to true
  acquire_ = true;
  // Creat a new thread for acquisition
  image_thread_.reset(new std::thread(&GigeCamera::AcquireImages, this));
}

void GigeCamera::Stop() {
  // Set acquire to false to stop the thread
  acquire_ = false;
  // Wait for the thread to finish
  image_thread_->join();

  StopAcquisition();
}

void GigeCamera::FindDevice() {
  auto interface_count = system_.GetInterfaceCount();
  cout << label_ << "Interface count: " << interface_count << endl;

  // Go through all interfaces, but we only care about network interface
  // For other interfaces such as usb, refer to sample code DeviceFinder.cpp
  vector<const PvDeviceInfoGEV *> device_info_gev_vec;
  for (uint32_t i = 0; i < interface_count; ++i) {
    // Get pointer to the interface
    const PvInterface *interface = system_.GetInterface(i);
    // Is it a PvNetworkAdapter?
    const auto *nic = dynamic_cast<const PvNetworkAdapter *>(interface);
    if (nic) {
      cout << label_ << i << " - " << interface->GetDisplayID().GetAscii()
           << endl;
      // Go through all the devices attached to the network interface
      for (uint32_t j = 0; j < interface->GetDeviceCount(); ++j) {
        const PvDeviceInfo *device_info = interface->GetDeviceInfo(j);
        // Is it a GigE Vision device?
        const auto *device_info_gev =
            dynamic_cast<const PvDeviceInfoGEV *>(device_info);
        if (device_info_gev) {
          cout << label_ << "  " << j << " - "
               << device_info->GetDisplayID().GetAscii() << endl;
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
    cout << label_ << "Found device: " << (*it)->GetIPAddress().GetAscii()
         << endl;
    // Is the IP address valid?
    if ((*it)->IsConfigurationValid()) {
      // Try connect and disconnect to verify
      dinfo_ = *it;
      PvResult result;
      cout << label_ << "--?-- " << dinfo_->GetDisplayID().GetAscii() << endl;
      // Creates and connects the device controller
      PvDevice *device = PvDevice::CreateAndConnect(dinfo_, &result);
      if (result.IsOK()) {
        cout << label_ << "-->-- " << dinfo_->GetDisplayID().GetAscii() << endl;
        cout << label_ << "--x-- " << dinfo_->GetDisplayID().GetAscii() << endl;
        PvDevice::Free(device);
      } else {
        // Maybe throw an exception here?
        ostringstream error_msg;
        error_msg << "GigeCamera: Unable to connect to "
                  << dinfo_->GetDisplayID().GetAscii();
        throw runtime_error(error_msg.str());
      }
    }
  }
}

void GigeCamera::ConnectDevice() {
  cout << label_ << "Connecting to " << dinfo_->GetDisplayID().GetAscii();
  PvResult result;
  // Use a unique_ptr to manage device resource
  device_.reset(PvDevice::CreateAndConnect(dinfo_, &result));

  if (result.IsOK()) {
    cout << " ... Done." << endl;
  } else {
    ostringstream error_msg;
    error_msg << "GigeCamera: Unable to connect to "
              << dinfo_->GetDisplayID().GetAscii();
    throw runtime_error(error_msg.str());
  }
}

void GigeCamera::OpenStream() {
  cout << label_ << "Opening stream to " << dinfo_->GetDisplayID().GetAscii();
  PvResult result;
  // Use a unique_ptr to manage stream resource
  stream_.reset(PvStream::CreateAndOpen(dinfo_->GetConnectionID(), &result));

  if (stream_) {
    cout << " ... Done. " << endl;
  } else {
    // Maybe a function for throw exception?
    ostringstream error_msg;
    error_msg << "GigeCamera: Unable to stream form "
              << dinfo_->GetDisplayID().GetAscii();
    throw runtime_error(error_msg.str());
  }
}

void GigeCamera::ConfigureStream() {
  // If this is a GigE Vision devie, configure GigE Vision specific parameters
  auto *device_gev = dynamic_cast<PvDeviceGEV *>(device_.get());
  if (device_gev) {
    cout << label_ << "Configureing gev stream" << endl;
    auto *stream_gev = static_cast<PvStreamGEV *>(stream_.get());
    // Negotiate packet size
    device_gev->NegotiatePacketSize();
    // Configure device streaming destination
    device_gev->SetStreamDestination(stream_gev->GetLocalIPAddress(),
                                     stream_gev->GetLocalPort());
  } else {
    ostringstream error_msg;
    error_msg << "GigeCamera: This is not a GigE Vision device "
              << dinfo_->GetDisplayID().GetAscii();
    throw runtime_error(error_msg.str());
  }
}

void GigeCamera::CreatePipeline() {
  cout << label_ << "Creating pipeline" << endl;
  // Create the PvPipeline object
  pipeline_.reset(new PvPipeline(stream_.get()));
  // Reading payload size from device
  auto payload_size = device_->GetPayloadSize();
  // Set the Buffer count and the Buffer size
  // BufferCount should be at least 4
  pipeline_->SetBufferCount(4);
  pipeline_->SetBufferSize(payload_size);
}

void GigeCamera::StartAcquisition() {
  // Set device in Mono8
  // device_params->SetEnumValue("PixelFormat", PvPixelMono8);
  // Note: the pipeline must be initialized before we start acquisition
  cout << label_ << "Starting pipeline ... ";
  pipeline_->Start();
  // Enable streaming
  cout << "Enabling streaming ... ";
  device_->StreamEnable();
  // Start acquisition
  cout << "Start acquisition ... ";
  // Get device parameters need to control streaming
  PvGenParameterArray *device_params = device_->GetParameters();
  device_params->ExecuteCommand("AcquisitionStart");
  cout << "Done" << endl;
}

void GigeCamera::StopAcquisition() {
  // Get device parameters need to control streaming
  PvGenParameterArray *device_params = device_->GetParameters();
  // Stop image acquisition
  cout << label_ << "Stop acquisition ... ";
  device_params->ExecuteCommand("AcquisitionStop");
  // Get controller out of streaming
  cout << "Disabling streaming ... ";
  device_->StreamDisable();
  // Stop pipeline
  cout << "Stoping pipeline ... ";
  pipeline_->Stop();
  cout << "Done" << endl;
}

void GigeCamera::AcquireImages() {
  // Get device parameters need to control streaming
  PvGenParameterArray *device_params = device_->GetParameters();
  // Get stream parameters
  PvGenParameterArray *stream_params = stream_->GetParameters();

  // Map a few GenICam stats counters
  double frame_rate_val = 0;
  double band_width_val = 0;
  auto *frame_rate =
      dynamic_cast<PvGenFloat *>(stream_params->Get("AcquisitionRate"));
  auto *band_width =
      dynamic_cast<PvGenFloat *>(stream_params->Get("Bandwidth"));

  // Create a cv::Mat to pass back to ros
  //  cv::namedWindow("flir", cv::WINDOW_AUTOSIZE);
  int64_t height = 0, width = 0;
  device_params->GetIntegerValue("Width", width);
  device_params->GetIntegerValue("Height", height);
  cout << label_ << "Width: " << width << " Height: " << height << endl;
  cv::Mat image_raw(cv::Size(width, height), CV_8UC1);

  // Start loop for acquisition
  while (acquire_) {
    PvBuffer *buffer;
    PvResult op_result;

    // Retrieve next buffer
    PvResult result = pipeline_->RetrieveNextBuffer(&buffer, 1000, &op_result);

    if (result.IsOK()) {
      if (op_result.IsOK()) {
        PvPayloadType payload_type;
        // We now have a valid buffer. This is where you would typically
        // process the buffer
        frame_rate->GetValue(frame_rate_val);
        band_width->GetValue(band_width_val);

        // If the buffer contains an image, display the width and height
        payload_type = buffer->GetPayloadType();

        if (payload_type == PvPayloadTypeImage) {
          // Get image specific buffer interface
          PvImage *image = buffer->GetImage();
          // Attach PvBuffer to an external memory
          image->Attach(image_raw.data, image->GetWidth(), image->GetHeight(),
                        PvPixelMono8, image->GetPaddingX(),
                        image->GetPaddingY());
          if (color_) {
            cv::Mat image_color;
            cv::applyColorMap(image_raw, image_color, cv::COLORMAP_JET);
            use_image(image_color);
          } else {
            // Send image to be published by ros
            use_image(image_raw);
          }
          // Display image
          // cv::imshow("flir", image_raw);
          // if (cv::waitKey(5) >= 0) break;
        } else {
          cout << label_ << "Buffer does not contain image" << endl;
        }
      } else {
        cout << label_ << "Non Ok operation result" << endl;
      }
      // Release the buffer back to the pipeline
      pipeline_->ReleaseBuffer(buffer);
    } else {
      cout << label_ << "Retrieve buffer failure" << endl;
    }
  }
}

}  // namespace flir_gige
