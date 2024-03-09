#include <memory>
#include <opencv2/opencv.hpp>
#include "srm/core.hpp"

namespace srm::core {

bool BaseCore::InitializeReader() {
  auto type = cfg.Get<std::string>({"type"});
  auto reader_type = cfg.Get<std::string>({"video.reader"});
  reader_.reset(video::CreateReader(reader_type));
  if (!reader_) {
    LOG(ERROR) << "Failed to create " << reader_type << " reader.";
    return false;
  }
  if (!reader_->Initialize("video." + type + "." + reader_type)) {
    LOG(ERROR) << "Failed to initialize reader.";
    return false;
  }
  frame_list_.resize(reader_->SourceCount());
  for (int i = 0; i < reader_->SourceCount(); i++) {
    while (!reader_->GetFrame(frame_list_[i], i)) {
      LOG(WARNING) << "Waiting for the first frame frome video" + std::to_string(i) + ".";
      sleep(1);
    }
  }
  LOG(INFO) << "Reader is initialized successfully.";
  return true;
}

bool BaseCore::InitializeViewer() {
  viewer_ = std::make_unique<viewer::VideoViewer>();
  if (!viewer_->Initialize(cfg.Get<std::string>({"viewer.web.shm_name"}))) {
    LOG(ERROR) << "Failed to initialize viewer.";
    return false;
  }
  LOG(INFO) << "Viewer is initialized successfully.";
  return true;
}

bool BaseCore::Initialize() {
  fps_controller_ = std::make_unique<FpsController>();
  fps_controller_->Initialize(srm::cfg.Get<double>({"fps_limit"}));
  bool ret = true;
  ret &= InitializeReader();
  ret &= InitializeViewer();
  if (!ret) {
    reader_.reset();
    return false;
  }
  for (auto REF_IN[callback, id] : frame_callback_list_) {
    reader_->RegisterFrameCallback(callback, this, id);  // 注册回调函数
  }
  
  LOG(INFO) << "Initializing Radar";
  return true;
}

bool BaseCore::UpdateFrameList() {
  static bool show_warning = true;
  auto ret = true;
  for (int i = 0; i < reader_->SourceCount(); i++) {
    ret &= !reader_->IsEmpty(i);
  }
  if (ret) {
    for (int i = 0; i < reader_->SourceCount(); i++) {
      ret &= reader_->GetFrame(frame_list_[i], i);
    }
  }
  if (!ret && show_warning) {
    LOG(WARNING) << "Failed to get frame data from video source. "
                 << "Wait for reconnecting the camera or press Ctrl-C to exit.";
  }

  auto show_image = frame_list_[0].image.clone();
  viewer_->SendFrame(show_image);

  show_warning = ret;
  return ret;
}

int BaseCore::Run() {
  
  while (!exit_signal) {
    fps_controller_->Tick();
    LOG_EVERY_N(INFO, 100) << fps_controller_->GetFPS();
    if (!UpdateFrameList()) {
      continue;
    }
  }
  return 0;
}

BaseCore::~BaseCore() {
  for (auto REF_IN[callback, id] : frame_callback_list_) {
    reader_->UnregisterFrameCallback(callback, id);
    delete callback;
  }
}

}  // namespace srm::core