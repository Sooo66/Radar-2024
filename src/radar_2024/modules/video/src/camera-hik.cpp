#include <MvCameraControl.h>
#include <glog/logging.h>

#include <opencv2/imgproc.hpp>

#include "srm/video/camera.h"

namespace srm::video {

/**
 * @brief 海康相机接口类
 * @warning 禁止直接构造此类，请使用 @code srm::video::CreateCamera("HikCamera") @endcode 获取该类的公共接口指针
 */
class HikCamera final : public Camera {
  inline static auto registry = RegistrySub<Camera, HikCamera>("HikCamera");  ///< 相机注册信息
 public:
  HikCamera() = default;
  ~HikCamera() final;

  bool OpenCamera(std::string REF_IN serial_number, std::string REF_IN config_file) final;
  bool CloseCamera() final;
  bool StartStream() final;
  bool StopStream() final;
  bool GetFrame(Frame REF_OUT frame) final;
  bool IsConnected() final;
  bool ExportConfigurationFile(std::string REF_IN config_file) final;
  bool ImportConfigurationFile(std::string REF_IN config_file) final;
  bool SetExposureTime(uint32_t exposure_time) final;
  bool SetGainValue(float gain) final;
  bool SetHardwareTriggerMode(bool hardware_trigger) final;
  bool SetFrameRate(double frame_rate) final;

 private:
  static void __stdcall ImageCallbackEx(unsigned char *image_data, MV_FRAME_OUT_INFO_EX *frame_info, void *obj);
  static void *DaemonThreadFunction(void *obj);
  bool SetExposureTimeHikImplementation(float exposure_time);

  void *device_{};  ///< 设备句柄
};

HikCamera::~HikCamera() {
  for (auto &&callback : callback_list_) {
    UnregisterFrameCallback(callback.first);
  }
  if (stream_running_) {
    StopStream();
  }
  if (device_ != nullptr) {
    CloseCamera();
  }
}

bool HikCamera::OpenCamera(std::string REF_IN serial_number, std::string REF_IN config_file) {
  if (device_ != nullptr) {
    return false;
  }
  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  auto status_code = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to enumerate devices with error code "
               << "0x" << std::hex << status_code << ".";
    return false;
  }
  if (device_list.nDeviceNum > 0) {
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      MV_CC_DEVICE_INFO *device_info = device_list.pDeviceInfo[i];
      std::string sn;
      switch (device_info->nTLayerType) {
        case MV_GIGE_DEVICE: {
          sn = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stGigEInfo.chSerialNumber));
          DLOG(INFO) << "GigE device with serial number " << sn << " found.";
          break;
        }
        case MV_USB_DEVICE: {
          sn = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
          DLOG(INFO) << "USB device with serial number " << sn << " found.";
          break;
        }
        default: {
          LOG(WARNING) << "Device with unsupported device transport layer protocol type " << sn << "found.";
        }
      }
      if (sn == serial_number) {
        LOG(INFO) << "Found device with serial number " << serial_number << ".";
        status_code = MV_CC_CreateHandle(&device_, device_info);
        if (MV_OK != status_code) {
          LOG(ERROR) << "Failed to create device handle with error "
                     << "0x" << std::hex << status_code << ".";
          return false;
        }
        break;
      }
    }
  } else {
    LOG(ERROR) << "No device found.";
    return false;
  }
  if (device_ == nullptr) {
    LOG(ERROR) << "Device with serial number " << serial_number << " not found.";
    return false;
  }
  status_code = MV_CC_OpenDevice(device_, MV_ACCESS_Exclusive, 1);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to open device with error "
               << "0x" << std::hex << status_code << ".";
    status_code = MV_CC_DestroyHandle(device_);
    if (MV_OK != status_code) {
      LOG(ERROR) << "Failed to destroy device handle with error "
                 << "0x" << std::hex << status_code << ".";
    }
    return false;
  }
  if (!config_file.empty() && !ImportConfigurationFile(config_file)) {
    status_code = MV_CC_CloseDevice(device_);
    if (MV_OK != status_code) {
      LOG(ERROR) << "Failed to close device with error "
                 << "0x" << std::hex << status_code << ".";
    }
    status_code = MV_CC_DestroyHandle(device_);
    if (MV_OK != status_code) {
      LOG(ERROR) << "Failed to destroy device handle with error "
                 << "0x" << std::hex << status_code << ".";
    }
    return false;
  }

  // TODO 此处填写覆盖配置文件的设置（请长期保留此条目）
  status_code = MV_CC_SetEnumValueByString(device_, "BalanceWhiteAuto", "Continuous") &
                MV_CC_SetEnumValueByString(device_, "PixelFormat", "BayerRG8");
  if (MV_OK != status_code) {
    status_code = MV_CC_CloseDevice(device_);
    LOG(ERROR) << "Failed to open device with error "
               << "0x" << std::hex << status_code << ".";
    status_code = MV_CC_DestroyHandle(device_);
    if (MV_OK != status_code) {
      LOG(ERROR) << "Failed to destroy device handle with error "
                 << "0x" << std::hex << status_code << ".";
    }
    return false;
  }

  status_code = MV_CC_RegisterImageCallBackEx(device_, ImageCallbackEx, this);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to register image callback with error "
               << "0x" << std::hex << status_code << ".";
    status_code = MV_CC_CloseDevice(device_);
    if (MV_OK != status_code) {
      LOG(ERROR) << "Failed to close device with error "
                 << "0x" << std::hex << status_code << ".";
    }
    status_code = MV_CC_DestroyHandle(device_);
    if (MV_OK != status_code) {
      LOG(ERROR) << "Failed to destroy device handle with error "
                 << "0x" << std::hex << status_code << ".";
    }
    return false;
  } else {
    DLOG(INFO) << "Registered " << serial_number_ << "'s capture callback.";
  }
  serial_number_ = serial_number;
  if (!daemon_thread_id_) {
    stop_flag_ = false;
    daemon_thread_id_ = std::make_unique<std::thread>(DaemonThreadFunction, this);
    DLOG(INFO) << serial_number_ << "'s daemon thread " << daemon_thread_id_->get_id() << " started.";
  }
  LOG(INFO) << "Opened camera " << serial_number << ".";
  return true;
}

bool HikCamera::CloseCamera() {
  if (stream_running_) {
    return false;
  }
  if (device_ == nullptr) {
    return false;
  }
  stop_flag_ = true;
  daemon_thread_id_->join();
  stop_flag_ = false;
  DLOG(INFO) << serial_number_ << "'s daemon thread " << daemon_thread_id_->get_id() << " stopped.";
  auto status_code = MV_CC_CloseDevice(device_);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to close camera with error "
               << "0x" << std::hex << status_code << ".";
  }
  status_code = MV_CC_DestroyHandle(device_);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to destroy handle with error "
               << "0x" << std::hex << status_code << ".";
  }
  LOG(INFO) << "Closed camera " << serial_number_ << ".";
  serial_number_ = "";
  device_ = nullptr;
  daemon_thread_id_.reset();
  return true;
}

bool HikCamera::StartStream() {
  if (device_ == nullptr) {
    return false;
  }
  ExportConfigurationFile("../cache/" + serial_number_ + ".txt");
  auto status_code = MV_CC_StartGrabbing(device_);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to start stream with error "
               << "0x" << std::hex << status_code << ".";
    return false;
  }
  stream_running_ = true;
  LOG(INFO) << serial_number_ << "'s stream started.";
  return true;
}

bool HikCamera::StopStream() {
  if (device_ == nullptr) {
    return false;
  }
  if (!stream_running_) {
    return false;
  }
  stream_running_ = false;
  auto status_code = MV_CC_StopGrabbing(device_);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to stop stream with error "
               << "0x" << std::hex << status_code << ".";
    return false;
  }
  LOG(INFO) << serial_number_ << "'s stream stopped.";
  return true;
}

bool HikCamera::GetFrame(Frame REF_OUT frame) {
  if (device_ == nullptr) {
    return false;
  }
  return buffer_.Pop(frame);
}

bool HikCamera::IsConnected() {
  if (device_ == nullptr) {
    return false;
  }
  return MV_CC_IsDeviceConnected(device_);
}

bool HikCamera::ImportConfigurationFile(std::string REF_IN config_file) {
  if (device_ == nullptr) {
    return false;
  }
  auto status_code = MV_CC_FeatureLoad(device_, config_file.c_str());
  if (status_code != MV_OK) {
    LOG(INFO) << "Failed to import " << serial_number_ << "'s configuration to " << config_file << " with error 0x"
              << std::hex << status_code << ".";
    return false;
  }
  LOG(INFO) << "Imported " << serial_number_ << "'s configuration to " << config_file << ".";
  return true;
}

bool HikCamera::ExportConfigurationFile(std::string REF_IN config_file) {
  if (device_ == nullptr) {
    return false;
  }
  auto status_code = MV_CC_FeatureSave(device_, config_file.c_str());
  if (status_code != MV_OK) {
    LOG(INFO) << "Failed to save " << serial_number_ << "'s configuration to " << config_file << " with error 0x"
              << std::hex << status_code << ".";
    return false;
  }
  LOG(INFO) << "Saved " << serial_number_ << "'s configuration to " << config_file << ".";
  return true;
}

bool HikCamera::SetExposureTime(uint32_t exposure_time) {
  if (device_ == nullptr) {
    return false;
  }
  return SetExposureTimeHikImplementation((float)exposure_time);
}

bool HikCamera::SetGainValue(float gain) {
  if (device_ == nullptr) {
    return false;
  }
  if (MV_CC_SetFloatValue(device_, "Gain", gain) != MV_OK) {
    LOG(ERROR) << "Failed to set " << serial_number_ << "'s gain to " << std::to_string(gain) << ".";
    return false;
  }
  DLOG(INFO) << "Set " << serial_number_ << "'s gain to " << std::to_string(gain) << ".";
  return true;
}

bool HikCamera::SetHardwareTriggerMode(bool hardware_trigger) {
  if (device_ == nullptr) {
    return false;
  }
  if (MV_CC_SetEnumValueByString(device_, "TriggerMode", hardware_trigger ? "On" : "Off") != MV_OK) {
    LOG(ERROR) << "Failed to set " << serial_number_ << "'s trigger mode to " << (hardware_trigger ? "on" : "off")
               << ".";
    return false;
  }
  if (hardware_trigger) {
    if (MV_CC_SetBoolValue(device_, "AcquisitionFrameRateEnable", false) != MV_OK) {
      LOG(ERROR) << "Failed to disable " << serial_number_ << "'s acquisition frame rate control.";
      return false;
    }
    // TODO 检查硬触发接线（请长期保留此条目）
    if (MV_CC_SetEnumValueByString(device_, "TriggerSource", "Line2") != MV_OK) {
      LOG(ERROR) << "Failed to set " << serial_number_ << "'s trigger source to line 2.";
      return false;
    }
    if (MV_CC_SetEnumValueByString(device_, "TriggerActivation", "RisingEdge") != MV_OK) {
      LOG(ERROR) << "Failed to set " << serial_number_ << "'s trigger activation to rising edge.";
      return false;
    }
  }
  DLOG(INFO) << "Set " << serial_number_ << "'s trigger mode to " << (hardware_trigger ? "hardware" : "software")
             << ".";
  return true;
}

bool HikCamera::SetFrameRate(double frame_rate) {
  if (MV_CC_SetFloatValue(device_, "AcquisitionFrameRate", static_cast<float>(frame_rate)) != MV_OK) {
    LOG(ERROR) << "Failed to set " << serial_number_ << "'s acquisition frame rate to " << frame_rate << ".";
    return false;
  }
  if (MV_CC_SetBoolValue(device_, "AcquisitionFrameRateEnable", true) != MV_OK) {
    LOG(ERROR) << "Failed to enable " << serial_number_ << "'s acquisition frame rate control.";
    return false;
  }
  return true;
}

void HikCamera::ImageCallbackEx(unsigned char *image_data, MV_FRAME_OUT_INFO_EX *frame_info, void *obj) {
  auto *self = static_cast<HikCamera *>(obj);
  Frame frame;
  switch (frame_info->enPixelType) {
    case PixelType_Gvsp_BayerRG8: {
      cv::Mat image(frame_info->nHeight, frame_info->nWidth, CV_8UC1, image_data);
      frame.image = image.clone();
      cv::cvtColor(frame.image, frame.image, cv::COLOR_BayerRG2RGB);
      break;
    }
    case PixelType_Gvsp_BGR8_Packed: {
      cv::Mat image(frame_info->nHeight, frame_info->nWidth, CV_8UC3, image_data);
      frame.image = image.clone();
      break;
    }
    default:
      LOG(WARNING) << "Unknown pixel type 0x" << std::hex << frame_info->enPixelType << " detected.";
  }
  frame.time_stamp = (uint64_t)frame_info->nDevTimeStampHigh;
  frame.time_stamp <<= 32;
  frame.time_stamp += frame_info->nDevTimeStampLow;
  frame.time_stamp *= self->time_stamp_ns_;
  for (auto p : self->callback_list_) {
    (*p.first)(p.second, frame);
  }
  if (frame.valid) {
    self->buffer_.Push(std::move(frame));
  }
}

void *HikCamera::DaemonThreadFunction(void *obj) {
  auto *self = (HikCamera *)obj;
  while (!self->stop_flag_) {
    sleep(1);
    if (!MV_CC_IsDeviceConnected(self->device_)) {
      LOG(ERROR) << self->serial_number_ << " is disconnected unexpectedly.";
      LOG(INFO) << "Preparing for reconnection...";
      MV_CC_StopGrabbing(self->device_);
      MV_CC_CloseDevice(self->device_);
      MV_CC_DestroyHandle(self->device_);
      self->device_ = nullptr;
      while (!self->OpenCamera(self->serial_number_, "../cache/" + self->serial_number_ + ".txt")) {
        sleep(1);
      }
      LOG(INFO) << self->serial_number_ << " is successfully reconnected.";
      if (self->stream_running_) {
        self->StartStream();
      }
    }
  }
  return nullptr;
}

bool HikCamera::SetExposureTimeHikImplementation(float exposure_time) {
  if (MV_CC_SetFloatValue(device_, "ExposureTime", exposure_time) != MV_OK) {
    LOG(ERROR) << "Failed to set " << serial_number_ << "'s exposure time to " << std::to_string(exposure_time) << ".";
    return false;
  }
  DLOG(INFO) << "Set " << serial_number_ << "'s exposure time to " << std::to_string(exposure_time) << ".";
  return true;
}

}  // namespace srm::video
