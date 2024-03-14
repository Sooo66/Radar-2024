#include "srm/lidar.hpp"

#include <memory>
constexpr float Epsilon = 1e-6;
namespace srm::lidar {

bool Lidar::Initialize(cv::Size REF_IN img_size, cv::Mat REF_IN intrinsic_mat, cv::Mat REF_IN homogeneous_mat) {
  intrinsic_mat_ = std::move(intrinsic_mat);
  homogeneous_mat_ = std::move(homogeneous_mat);

  cv::cv2eigen(intrinsic_mat_, intrinsic_mat_eigen_);
  cv::cv2eigen(homogeneous_mat_, homogeneous_mat_eigen_);

  std::cout << intrinsic_mat_eigen_ << "\n";
  std::cout << homogeneous_mat_eigen_ << "\n";

  height_ = img_size.height, width_ = img_size.width;
  depth_mat_.resize(height_, std::vector<float>(width_));

  return true;
}

void Lidar::Update(pcl::PointCloud<pcl::PointXYZ> REF_IN point_cloud) {
  Eigen::Matrix4Xf pcl_matrix = point_cloud.getMatrixXfMap();
  auto pts_num = pcl_matrix.cols();
  // Lidar coord -> camera coord
  Eigen::Matrix3Xf camera_points = (homogeneous_mat_eigen_ * pcl_matrix).topRows(3);

  Eigen::Matrix<float, 3, kPointCloudNum> pts_coord;  // point data with size == kPoinCloudtNum
  Eigen::Matrix<int, 2, kPointCloudNum> img_coord;    // u, v
  Eigen::Matrix<float, 1, kPointCloudNum> pts_depth;  // depth of (u, v)
  
  pts_coord.leftCols(pts_num) << camera_points;
  pts_depth.leftCols(pts_num) << camera_points.row(2);

  // 计算pts_coord在像素坐标系下的坐标(u, v), 储存在img_coord中
  for (size_t i = 0; i < pts_num; i ++) {
    if (std::fabs(pts_depth(0, i)) < Epsilon) {
      break;
    }
    else {
      Eigen::Matrix<float, 3, 1> pixel_coord = intrinsic_mat_eigen_ * pts_coord.col(i);
      int x = static_cast<int>(pixel_coord(0) / pixel_coord(2));
      int y = static_cast<int>(pixel_coord(1) / pixel_coord(2));
      img_coord.col(i) << x, y;
    }
  }

  if (buffer_.Full()) {
    Eigen::Matrix<int, 2, kPointCloudNum> out_points;
    buffer_.Pop(out_points);
    for (size_t i = 0; i < kPointCloudNum; i ++) {
      int x = out_points(1, i), y = out_points(0, i);
      depth_mat_[x][y] = 0;
    }
  }
  else {
    for (size_t i = 0; i < kPointCloudNum; i ++) {
      if (img_coord(0, i) < 0 || img_coord(0, i) >= width_ || img_coord(1, i) < 0 || img_coord(1, i) >= height_) {
        continue;
      }
      if (pts_depth(0, i) > 0) {
        int x = img_coord(1, i), y = img_coord(0, i);
        if (depth_mat_[x][y] < Epsilon || (depth_mat_[x][y] > Epsilon && depth_mat_[x][y] > pts_depth(0, i))) {
          depth_mat_[x][y] = pts_depth(0, i);
        }
      }
      else break;
    }
  }
  buffer_.Push(std::move(img_coord));
}

float Lidar::GetDepth(cv::Point2f REF_IN x, cv::Point2f REF_IN y) {
  float sum = 0, cnt = 0;
  for (auto i = x.x; i <= y.x; i ++) {
    for (auto j = x.y; j <= y.y; j ++) {
      sum += depth_mat_[i][j];
      cnt += 1;
    }
  }
  return sum / cnt;
}

cv::Mat Lidar::Show() {
  cv::Mat depth_img(height_, width_, CV_8UC3);
  for (size_t i = 0; i < height_; i ++) {
    for (size_t j = 0; j < width_; j ++) {
      // depth_img.at<float>(i, j) = depth_mat_[i][j] * 255;
      int depth = static_cast<int>(depth_mat_[i][j] * 255);
      depth_img.at<cv::Vec3b>(i, j) = cv::Vec3b(depth, depth, depth);
    }
  }
  return depth_img;
}

}  // namespace srm::lidar