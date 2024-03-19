#ifndef SRM_LOCATE_LOCATE_H_
#define SRM_LOCATE_LOCATE_H_

#include <Eigen/Dense>
#include <srm/nn/info.hpp>
#include "srm/common.hpp"

namespace srm::locate {

/// 机器人主控公共接口类
class Locate {
 public:
  Locate() = default;
  ~Locate() = default;

  bool Initialize(std::vector<cv::Mat> REF_IN intrinsic_mat, std::vector<cv::Mat> REF_IN distortion_mat);

  void Run(std::vector<srm::nn::Armor> REF_IN armor_list, std::vector<srm::nn::LocateInfo> REF_OUT locate_list);

 protected:
  std::vector<Eigen::Matrix4f> homogeneous_mat_; ///< 齐次变换矩阵(camera2world)
  std::vector<Eigen::Matrix4f> homogeneous_mat_t_; ///< 逆矩阵

  std::vector<cv::Mat> intrinsic_mat_;
  std::vector<cv::Mat> distortion_mat_;

  std::vector<Eigen::Matrix<float, 3, 1>> camera_position_;
};

}  // namespace srm::locate

#endif  // SRM_LOCATE_LOCATE_H_