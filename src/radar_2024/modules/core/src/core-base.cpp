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

bool BaseCore::InitializeLidar() {
  lidar_ = std::make_unique<lidar::Lidar>();

  lidar_subscriber_ = node_handler_->subscribe<sensor_msgs::PointCloud2>("lidar_node", 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);
    lidar_->Update(point_cloud);
  });
  
  cv::Size img_size = frame_list_[0].image.size();

  LOG(INFO) << "source: " << reader_->SourceCount();

  std::vector<cv::Mat> intrinsic_mat, homogeneous_mat;
  for (size_t i = 0; i < reader_->SourceCount(); i ++) {
    std::string prefix = "lidar.";
    prefix += std::to_string(i);
    std::cout << "prefix: " << prefix << "\n";
    cv::Mat K = cfg.Get<cv::Mat>({prefix, "intrinsic_mat"});
    std::cout << K << "\n";
    intrinsic_mat.push_back(K);
    cv::Mat T = cfg.Get<cv::Mat>({prefix, "homogeneous_mat"});
    homogeneous_mat.push_back(T);
  }

  if (!lidar_->Initialize(img_size, intrinsic_mat, homogeneous_mat)) {
    LOG(ERROR) << "Failed to initialize lidar.";
    return false;
  }
  lidar_thread_ = std::thread([](srm::lidar::Lidar *self) {
    while (!exit_signal) {
      ros::spinOnce();
    }
  }, this->lidar_.get());
  LOG(INFO) << "Lidar and Lidar Thread are initialized successfully.";
  return true;
}

bool BaseCore::InitializeLocate() {
  locater_ = std::make_unique<srm::locate::Locate>();
  std::vector<cv::Mat> intrinsic_mat, distortion_mat;
  for (int i = 0; i < reader_->SourceCount(); i ++) {
    intrinsic_mat.push_back(reader_->IntrinsicMat(i));
    distortion_mat.push_back(reader_->DistortionMat(i));
  }
  if (!locater_->Initialize(intrinsic_mat, distortion_mat)) {
    LOG(INFO) << "Failed to initialize locater.";
    return false;
  }
  return true;
}

bool BaseCore::Initialize(ros::NodeHandle* nh) {
  fps_controller_ = std::make_unique<FpsController>();
  fps_controller_->Initialize(srm::cfg.Get<double>({"fps_limit"}));

  node_handler_ = std::make_shared<ros::NodeHandle>();
  node_handler_.reset(nh);

  bool ret = true;
  ret &= InitializeReader();
  ret &= InitializeViewer();
  ret &= InitializeDetector();
  ret &= InitializeLidar();
  ret &= InitializeLocate();
  if (!ret) {
    reader_.reset();
    viewer_.reset();
    detector_.reset();
    lidar_.reset();
    locater_.reset();
    return false;
  }
  for (auto REF_IN[callback, id] : frame_callback_list_) {
    reader_->RegisterFrameCallback(callback, this, id);  // 注册回调函数
  }
  
  LOG(INFO) << "Initializing Radar Successfully.";
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

  // for (auto &armor: armor_list) {
  //   float depth = lidar_->GetDepth(armor.source, armor.pts[0], armor.pts[1]);
  //   if (depth != -1) {
  //     armor.depth = depth;
  //   }
  //   else {
  //     armor_list.erase(std::remove(armor_list.begin(), armor_list.end(), armor), armor_list.end());
  //   }
  // }

  for (auto it = armor_list.begin(); it != armor_list.end(); ) {
    auto &armor = *it;
    float depth = lidar_->GetDepth(armor.source, armor.pts[0], armor.pts[1]);
    if (depth != -1) {
      armor.depth = depth;
      ++ it;
    }
    else {
      it = armor_list.erase(it);
    }
  }

  std::vector<srm::nn::LocateInfo> locate_list;

  locater_->Run(armor_list, locate_list);

  // 展示在小地图上
  
  // 发送串口

  auto show_image = show_image_list[0].clone();

  for (auto &locate: locate_list) {
    auto draw_color = cv::Scalar(255, 255, 255);
    std::string id = "";
    if (locate.color == srm::nn::Color::kBlue) {
      draw_color = cv::Scalar(0, 255, 0);
      id += "B";
    }
    else {
      draw_color = cv::Scalar(0, 0, 255);
      id += "R";
    }
    id += std::to_string(locate.id);

    auto roi = locate.roi;
    cv::rectangle(show_image, roi.top_left, cv::Point2f(roi.top_left.x + roi.width, roi.top_left.y + roi.height), draw_color, 4);
    cv::rectangle(show_image, roi.top_left + locate.pts[0], roi.top_left + locate.pts[1], draw_color, 4);
    std::string depth = std::to_string(locate.depth);
    cv::putText(show_image, id, cv::Point2f(roi.top_left.x, roi.top_left.y - 5), cv::FONT_HERSHEY_SIMPLEX, 1, draw_color, 2);
    cv::putText(show_image, depth, cv::Point2f(roi.top_left.x, roi.top_left.y + roi.height + 30), cv::FONT_HERSHEY_SIMPLEX, 1, draw_color, 2);
    std::string loc = "(";
    loc += std::to_string(locate.x);
    loc += ", ";
    loc += std::to_string(locate.y);
    loc += ", ";
    loc += std::to_string(locate.z);
    loc += ")";
    cv::putText(show_image, loc, cv::Point2f(roi.top_left.x, roi.top_left.y + roi.height + 60), cv::FONT_HERSHEY_SIMPLEX, 1, draw_color, 2);
  }

  // for (auto &armor: armor_list) {
  //   auto draw_color = cv::Scalar(255, 255, 255);
  //   std::string id = "";
  //   if (armor.color == srm::nn::Color::kBlue) {
  //     draw_color = cv::Scalar(0, 255, 0);
  //     id += "B";
  //   }
  //   else {
  //     draw_color = cv::Scalar(0, 0, 255);
  //     id += "R";
  //   }
  //   id += std::to_string(armor.id);
  //   std::string depth = "";
  //   float armor_depth = lidar_->GetDepth(armor.pts[0], armor.pts[1]);
  //   if (armor_depth > 0) {
  //     depth = std::to_string(armor_depth);
  //   }
  //   else {
  //     depth = "N/A";
  //   }
  //   LOG(INFO) << depth;
  //   // roi
  //   auto &roi = armor.roi;
  //   cv::rectangle(show_image, roi.top_left, cv::Point2f(roi.top_left.x + roi.width, roi.top_left.y + roi.height), draw_color, 4);
  //   cv::putText(show_image, id, cv::Point2f(roi.top_left.x, roi.top_left.y - 5), cv::FONT_HERSHEY_SIMPLEX, 1, draw_color, 2);
  //   cv::putText(show_image, depth, cv::Point2f(roi.top_left.x, roi.top_left.y + roi.height + 20), cv::FONT_HERSHEY_SIMPLEX, 1, draw_color, 2);
  //   // armor
  //   cv::rectangle(show_image, roi.top_left + armor.pts[0], roi.top_left + armor.pts[1], draw_color, 4);
  // }

  // cv::Mat depth_img = lidar_->Show();



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
    cv::resizeWindow("image", 1440, 1080);

    auto waitkey = []() {
      auto key = cv::waitKey(1);
      if (key == ' ') {
        // 暂停
        key = cv::waitKey(0);
        if (key == ' ') {
          return cv::waitKey(10);
        }
      }
    };

    waitkey();

    // cv::namedWindow("lidar", cv::WINDOW_NORMAL);
    // cv::resizeWindow("lidar", 1920, 1080);
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

  if (lidar_thread_.joinable()) {
    lidar_thread_.join();
  }
}

}  // namespace srm::core