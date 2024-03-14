#include <glog/logging.h>

#include "srm/common/config.hpp"
#include "srm/video/camera.h"
#include "srm/video/reader.h"

namespace srm::video {
/**
 * @brief 相机视频源接口类
 * @warning 禁止直接构造此类，请使用 @code srm::video::CreateReader("camera") @endcode 获取该类的公共接口指针
 */
class CameraReader final : public Reader {
  inline static auto registry = RegistrySub<Reader, CameraReader>("camera");  ///< 视频源注册信息
 public:
  CameraReader() = default;
  ~CameraReader() final = default;

  bool Initialize(std::string REF_IN prefix) final;
  bool IsEmpty(int id = 0) final;
  bool GetFrame(Frame REF_OUT frame, int id = 0) final;
  void RegisterFrameCallback(FrameCallback callback, void *obj, int id = 0) final;
  void UnregisterFrameCallback(FrameCallback callback, int id = 0) final;
  const cv::Mat &IntrinsicMat(int id = 0) const final;
  const cv::Mat &DistortionMat(int id = 0) const final;

 private:
  struct CameraInfo {
    std::unique_ptr<Camera> camera{};  ///< 相机指针
    cv::Mat intrinsic_mat;             ///< 相机针孔模型矩阵
    cv::Mat distortion_mat;            ///< 相机畸变矩阵
  };

  std::vector<std::unique_ptr<CameraInfo>> cams_info_{};  ///< 相机公共接口指针
};

bool CameraReader::Initialize(std::string REF_IN prefix) {
  auto create_camera = [prefix](const std::string &cam_name,
                                std::unique_ptr<CameraReader::CameraInfo> REF_OUT cam_info) {
    auto &[camera, intrinsic_mat, distortion_mat] = *cam_info;
    std::string cameras_prefix = "video.cameras." + cam_name;

    auto cameratype = cfg.Get<std::string>({cameras_prefix, "type"});
    camera.reset(video::CreateCamera(cameratype));
    if (!camera) {
      LOG(ERROR) << "Failed to create camera object of type " << cameratype << ".";
      return false;
    }

    if (!camera->OpenCamera(cfg.Get<std::string>({cameras_prefix, "sn"}), "")) {
      camera.reset();
      return false;
    }

    if (!camera->SetExposureTime(static_cast<uint32_t>(cfg.Get<int>({cameras_prefix, "exposure_time"})))) {
      LOG(WARNING) << "Failed to set exposure time of camera.";
    }

    if (!camera->SetGainValue(cfg.Get<double>({cameras_prefix, "gain_value"}))) {
      LOG(WARNING) << "Failed to set gain value of camera.";
    }

    if (!camera->SetTimeStampNS(cfg.Get<int>({cameras_prefix, "time_stamp_ns"}))) {
      LOG(WARNING) << "Invalid value for time stamp unit. Please check your configuration.";
    }

    intrinsic_mat = cfg.Get<cv::Mat>({cameras_prefix, "intrinsic_mat"});
    distortion_mat = cfg.Get<cv::Mat>({cameras_prefix, "distortion_mat"});

    if (!camera->SetFrameRate(cfg.Get<double>({prefix, "frame_rate"}))) {
      LOG(WARNING) << "Failed to set frame rate of camera. Fallback to default value.";
    }

    if (!camera->SetHardwareTriggerMode(cfg.Get<int>({prefix, "hardware_trigger"}))) {
      LOG(WARNING) << "Failed to set hardware trigger mode of camera. Fallback to software trigger mode.";
    }

    if (!camera->StartStream()) {
      camera.reset();
      return false;
    }
    return true;
  };

  source_count_ = cfg.Get<int>({prefix, "cam_num"});
  cams_info_.resize(source_count_);

  for (int i = 0; i < source_count_; i++) {
    cams_info_[i] = std::make_unique<CameraInfo>();
    auto cam_name = cfg.Get<std::string>({prefix, "camera" + std::to_string(i)});
    if (!create_camera(cam_name, cams_info_[i])) {
      LOG(ERROR) << "Create camera " + cam_name + " failed.";
      return false;
    }
  }

  return true;
}

bool CameraReader::IsEmpty(int id) { return cams_info_[id]->camera->IsEmpty(); }

bool CameraReader::GetFrame(Frame REF_OUT frame, int id) {
  return cams_info_[id] && cams_info_[id]->camera && cams_info_[id]->camera->GetFrame(frame);
}

void CameraReader::RegisterFrameCallback(FrameCallback callback, void *obj, int id) {
  if (cams_info_[id] && cams_info_[id]->camera) {
    cams_info_[id]->camera->RegisterFrameCallback(callback, obj);
  }
  DLOG(INFO) << "Registered video source callback FUNC " << callback << " OBJ " << obj << ".";
}

void CameraReader::UnregisterFrameCallback(FrameCallback callback, int id) {
  if (cams_info_[id] && cams_info_[id]->camera) {
    cams_info_[id]->camera->UnregisterFrameCallback(callback);
  }
  DLOG(INFO) << "Unregistered video source callback FUNC " << callback << ".";
}

const cv::Mat &CameraReader::IntrinsicMat(int id) const { return cams_info_[id]->intrinsic_mat; }

const cv::Mat &CameraReader::DistortionMat(int id) const { return cams_info_[id]->distortion_mat; }
}  // namespace srm::video
