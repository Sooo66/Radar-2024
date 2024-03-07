#include <memory>

#include "srm/core.hpp"

namespace srm::core {

bool BaseCore::Initialize() {
  car_.reset(srm::nn::CreateYolo("car"));
  armor_.reset(srm::nn::CreateYolo("armor"));
  car_->Initialize(srm::cfg.Get<std::string>({"nn", "yolo", "car", "tensorrt"}), 1);
  armor_->Initialize(srm::cfg.Get<std::string>({"nn", "yolo", "armor", "tensorrt"}), 1);

  fps_controller_ = std::make_unique<FpsController>();
  fps_controller_->Initialize(srm::cfg.Get<double>({"fps_limit"}));

  LOG(INFO) << "Initializing Radar";
  return true;
}

int BaseCore::Run() {
  cv::Mat image;
  cv::VideoCapture cap(srm::cfg.Get<std::string>({"test", "file"}));
  
  if (!cap.isOpened()) {
    LOG(ERROR) << "Failed to open video file.";
    return 1;
  }

  cv::namedWindow("image", 0);
  cv::resizeWindow("image", 1280, 720);

  int total_car = 0, total_armor = 0;
  // int a = 0;
  while (!exit_signal) {
    cap >> image;
    if (image.empty()) {
      LOG(ERROR) << "Failed to read frame.";
      return 1;
    }
    
    auto cars = car_->Infer(image);
    for (auto &&car : cars) {
      cv::rectangle(image, cv::Point(car.x1, car.y1), cv::Point(car.x2, car.y2), cv::Scalar(0, 255, 0), 2);
    }

    total_car += cars.size();
    auto roi = car_->PostProcess(cars, image);
    for (int i = 0; i < roi.size(); i++) {
      auto kx = cars[i].x1, ky = cars[i].y1;
      auto armors = armor_->Infer(roi[i]);
      total_armor += armors.size();
      for (auto &&armor : armors) {
        armor.x1 += kx;
        armor.y1 += ky;
        armor.x2 += kx;
        armor.y2 += ky;
        cv::rectangle(image, cv::Point(armor.x1, armor.y1), cv::Point(armor.x2, armor.y2), cv::Scalar(0, 255, 0), 2);
      }
    }

    cv::imshow("image", image);

    fps_controller_->Tick();

    LOG(INFO) << "FPS: " << " Car: " << total_car << " Armor: " << total_armor;

    if (cv::waitKey(1) == 27) {
      break;
    }
  }
}

BaseCore::~BaseCore() {
}

}  // namespace srm::core