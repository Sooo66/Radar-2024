#ifndef SRM_LIDAR_LIDAR_DEPTH_IMG_H_
#define SRM_LIDAR_LIDAR_DEPTH_IMG_H_

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <queue>
#include <vector>

#include "srm/common.hpp"

namespace srm::lidar {

/// 机器人主控公共接口类
class Lidar {
 static constexpr size_t kBufferSize = 100;  ///< 点云队列大小
 static constexpr size_t kPointCloudNum = 10000;  ///< 最大点云数量
 public:
  Lidar() = default;
  ~Lidar() = default;

  bool Initialize(cv::Size REF_IN img_size, std::vector<cv::Mat REF_IN> intrinsic_mat, std::vector<cv::Mat> REF_IN homogeneous_mat);

  void Update(pcl::PointCloud<pcl::PointXYZ> REF_IN point_cloud);

  /**
   * @brief 取点云矩形框
   * @param x 左上角 
   * @param y 右下角
   * @param depth 框内深度均值
   * @return 是否成功得到深度
   */
  float GetDepth(cv::Point2f REF_IN x, cv::Point2f REF_IN y);

  cv::Mat Show();

 protected:
  int source_count_;

  std::vector<Buffer<Eigen::Matrix<int, 2, kPointCloudNum>, kBufferSize>> buffer_;  ///< 点云队列

  std::vector<cv::Mat> intrinsic_mat_;                ///< 相机内参矩阵
  std::vector<cv::Mat> homogeneous_mat_;              ///< 雷达到相机的外参变换矩阵

  std::vector<Eigen::Matrix<float, 3, 3>> intrinsic_mat_eigen_; ///< eigen格式相机内参矩阵
  std::vector<Eigen::Matrix<float, 4, 4>> homogeneous_mat_eigen_; ///< eigen格式雷达到相机的外参变换矩阵

  int height_{};
  int width_{};

  std::vector<std::vector<std::vector<float>>> depth_mat_; ///< 深度图
};

}  // namespace srm::lidar

#endif  // SRM_LIDAR_LIDAR_DEPTH_IMG_H_