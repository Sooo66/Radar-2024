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

  std::vector<cv::Mat> show_image_list;
  for (auto &t: frame_list_) {
    show_image_list.push_back(t.image);
  }
  std::vector<srm::nn::Armor> armor_list;
  detector_->Run(show_image_list, armor_list);
  auto show_image = show_image_list[0].clone();

  for (auto &armor: armor_list) {
    auto draw_color = cv::Scalar(255, 255, 255);
    std::string id = "";
    if (armor.color == srm::nn::Color::kBlue) {
      draw_color = cv::Scalar(0, 255, 0);
      id += "B";
    }
    else {
      draw_color = cv::Scalar(0, 0, 255);
      id += "R";
    }
    id += std::to_string(armor.id);
    // roi
    auto &roi = armor.roi;
    cv::rectangle(show_image, roi.top_left, cv::Point2f(roi.top_left.x + roi.width, roi.top_left.y + roi.height), draw_color, 4);
    cv::putText(show_image, id, cv::Point2f(roi.top_left.x, roi.top_left.y - 5), cv::FONT_HERSHEY_SIMPLEX, 1, draw_color, 2);
    // armor
    cv::rectangle(show_image, roi.top_left + armor.pts[0], roi.top_left + armor.pts[1], draw_color, 4);
  }

  // viewer_->SendFrame(show_image);
  cv::imshow("image", show_image);
  cv::waitKey(10);

  show_warning = ret;
  return ret;
}

int BaseCore::Run() {
  
  while (!exit_signal) {
    fps_controller_->Tick();
    LOG_EVERY_N(INFO, 100) << fps_controller_->GetFPS();
    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::resizeWindow("image", 1920, 1080);
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