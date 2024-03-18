#include "srm/lidar.hpp"

#include <memory>
namespace srm::lidar {

bool Locate::Initialize(std::vector<cv::Mat> REF_IN intrinsic_mat, std::vector<cv::Mat> REF_IN distortion_mat) {
  int source_count = intrinsic_mat.size();
  homogeneous_mat_.resize(source_count);
  homogeneous_mat_t_.resize(source_count);
  intrinsic_mat_.resize(source_count);
  distortion_mat_.resize(source_count);

  for (int i = 0; i < source_count; i ++) {
    intrinsic_mat_ = intrinsic_mat[i];
    distortion_mat_ = distortion_mat[i];
    std::prefix = "locate.camera";
    prefix += std::to_string(i);
    cv::Mat T = cfg.Get<cv::Mat>({prefix});
    cv::cv2eigen(T, homogeneous_mat_[i]);
    std::cout << homogeneous_mat_[i] << "\n";
    homogeneous_mat_t_[i] = homogeneous_mat_[i].inverse();
    Eigen::Matrix<float, 4, 1> I;
    I << 0.f, 0.f, 0.f, 1.f;
    camera_position_[i] = (homogeneous_mat_t_[i] * I).topRows(3);
    std::cout << camera_position_[i] << "\n";
  }

  return true;
}

void Locate::Run(std::vector<srm::nn::Armor> REF_IN armor_list, std::vector<LocateInfo> REF_OUT locate_list) {
  for (auto &armor: armor_list) {
    if (armor.depth != -1) {
      Eigen::Matrix<float, 4, 1> xyz1;
      int source = armor.source;
      auto depth = armor.depth;
      cv::Point2f center = armor.pts[0] + armor.pts[1];
      auto height = armor.pts[0].x - armor.pts[1].x;
      auto width = armor.pts[0].y - armor.pts[1].y;
      cv::undistortPoints(center, center, intrinsic_mat_[source], distortion_mat_[source]);
      xyz1 << center.x * depth , center.y * depth, depth, 1.;

      Eigen::Matrix<float, 4, 1> location;
      location = homogeneous_mat_t_[source] * xyz1;
      locate_list.push_back({
        armor.id,
        armor.color,
        location[0],
        location[1],
        location[2]
      });
    }
  }
}

}  // namespace srm::lidar