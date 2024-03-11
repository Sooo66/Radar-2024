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

bool BaseCore::InitializeDetector() {
  detector_ = std::make_unique<nn::Detector>();
  if (!detector_->Initialize()) {
    LOG(INFO) << "Failed to initialized detector";
    return false;
  }
  LOG(INFO) << "Detector is initialized successfully";
  return true;
}

bool BaseCore::Initialize() {
  fps_controller_ = std::make_unique<FpsController>();
  fps_controller_->Initialize(srm::cfg.Get<double>({"fps_limit"}));
  bool ret = true;
  ret &= InitializeReader();
  ret &= InitializeViewer();
  ret &= InitializeDetector();
  if (!ret) {
    reader_.reset();
    viewer_.reset();
    detector_.reset();
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

  // auto show_image = frame_list_[0].image.clone();
  std::vector<cv::Mat> show_image_list(frame_list_.size());
  for (auto &t: fram_list_) {
    show_image_list.push_back(frame_list_.image);
  }
  std::vector<Armor> armor_list;
  detector_->Run(show_image_list, armor_list);
  auto show_image = show_image_list[0];
  
  for (auto &armor: armor_list) {
  auto &roi_list = armor.roi_list;
  auto &pts_list = armor.pts_list;

  // 绘制装甲板的 ROI 和角点
  for (size_t i = 0; i < roi_list.size(); i++) {
    auto &roi = roi_list[i];
    auto &pts = pts_list[i];

    // 绘制 ROI
    cv::rectangle(show_image, roi.top_left, cv::Point2f(roi.top_left.x + roi.width, roi.top_left.y + roi.height), cv::Scalar(0, 255, 0), 2);

    // 在 ROI 旁边添加文本 "car"
    cv::putText(show_image, "car", cv::Point2f(roi.top_left.x, roi.top_left.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    // 绘制装甲板的角点
    for (auto &pt: pts) {
      cv::circle(show_image, pt, 3, cv::Scalar(0, 0, 255), -1);
    }

    // 在角点旁边添加文本 "armor"
    cv::putText(show_image, "armor", pts[0] + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
  }
}

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