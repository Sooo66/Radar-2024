#include "srm/lidar.hpp"

#include <memory>
namespace srm::lidar {

bool Lidar::Initialize(std::string REF_IN prefix, cv::Mat intrinsic_mat, cv::Mat distortion_mat) {
  // intrinsic_mat_ = std::move(intrinsic_mat);
  // distortion_mat_ = std::move(distortion_mat);
  // intrinsic_mat_eigen = cv::cv2eigen(intrinsic_mat_, intrinsic_mat_eigen);
  // distortion_mat_eigen = cv::cv2eigen(distortion_mat_, distortion_mat_eigen);
  // std::cout << intrinsic_mat_eigen << "\n";
  // std::cout << distortion_mat_eigen << "\n";
  // height_ = cfg.Get<int>({prefix, "height"});
  // width_ = cfg.Get<int>({prefix, "width"});
  // depth_mat_.resize(height_, std::vector<float>(width_));
  return true;
}

}  // namespace srm::lidar