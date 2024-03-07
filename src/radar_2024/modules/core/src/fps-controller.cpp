#include "srm/core/fps-controller.h"

#include <iostream>
#include <thread>

namespace srm::core {

void FpsController::Initialize(double target_fps) {
  frame_interval_ = 1.0 / target_fps;
  last_time_ = std::chrono::high_resolution_clock::now();
}

void FpsController::Tick() {
  auto current_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time = current_time - last_time_;
  if (elapsed_time.count() < frame_interval_) {
    std::this_thread::sleep_for(std::chrono::duration<double>(frame_interval_ - elapsed_time.count()));
  }
  current_time = std::chrono::high_resolution_clock::now();
  actual_fps_ = 1e9 / (current_time - last_time_).count();
  last_time_ = current_time;
}

}  // namespace srm::core