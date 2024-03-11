#ifndef SRM_LIDAR_LIDAR_LIVOX_H_
#define SRM_LIDAR_LIDAR_LIVOX_H_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <queue>
#include <vector>

#include "srm/common/buffer.hpp"
#include "srm/common/config.hpp"
#include "srm/common/tags.hpp"

namespace srm::lidar {

/// 机器人主控公共接口类
class Lidar {
 static constexpr int kBufferSize = 100;  ///< 点云队列大小
 static constexpr int kPointCloudNum = 10000;  ///< 最大点云数量
 public:
  Lidar() = default;
  ~Lidar() = default;

  bool Initialize(std::string REF_IN prefix, cv::Mat intrinsic_mat, cv::Mat distortion_mat);

  int Update(pcl::PointCloud<pcl::PointXYZ> REF_IN);

  /**
   * @brief 取点云矩形框
   * @param x 左上角 
   * @param y 右下角
   * @return 框内深度值
   */
  std::vector<std::vector<float>> Rect(std::pair<int, int> x, std::pair<int, int> y);

 protected:
  Buffer<Eigen::Matrix<int, 2, kPointCloudNum>, kBufferSize> buffer_;  ///< 点云队列

  cv::Mat intrinsic_mat_;                ///< 相机矩阵
  cv::Mat distortion_mat_;               ///< 畸变矩阵

  Eigen::Matrix<float, 3, 3> intrinsic_mat_eigen; ///< 相机内参矩阵
  Eigen::Matrix<float, 1, 14> distortion_mat_eigen; ///< 相机畸变系数矩阵
  Eigen::Matrix<float, 4, 4> homogeneous_mat_; ///< 雷达到相机的外参变换矩阵

  int height_{};
  int width_{};

  std::vector<std::vector<float>> depth_mat_; ///< 深度图
};

}  // namespace srm::lidar

#endif  // SRM_LIDAR_LIDAR_LIVOX_H_