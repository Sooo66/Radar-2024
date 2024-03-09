#ifndef SRM_NN_INFO_HPP_
#define SRM_NN_INFO_HPP_

#include <opencv2/core/mat.hpp>

#include "srm/coord/coord-base.h"

namespace srm::nn {

/// 颜色
enum class Color {
  kBlue = 1,
  kRed = 2,
  // kGrey = 3,
  // kPurple = 4,
};

struct ROI {
  cv::Point2f top_left;     ///< roi左上角在原图中的坐标
  int width;              ///< roi的宽
  int height;             ///< roi的长
  cv::Mat roi_mat;        ///< roi本身
};

// struct Armor {
//   ROI roi;                       ///< 车辆的roi 
//   float x1;                      ///< 预测框左上角顶点横坐标
//   float y1;                      ///< 预测框左上角顶点纵坐标
//   float x2;                      ///< 预测框右下角顶点横坐标
//   float y2;                      ///< 预测框右下角顶点纵坐标
//   float prob;                    ///< 置信度
//   int cls;                       ///< 类编号
// }

/// 传入战斗信息
struct BattleInfo {
  std::vector<cv::Mat> image_list;  ///< 图片列表
  uint64_t time_stamp;              ///< 时间戳
  // coord::RMat rm_self;              ///< 位姿矩阵
  Color color;                      ///< 自身颜色
};

/// 传出打弹信息
struct LocateInfo {
  int id;
  Color color;
  float x;
  float y;
  float z;
};

/// 装甲板信息
struct Armor {
  std::vector<ROI> roi_list;                         ///< 该装甲板在各个图片列表下的roi
  std::vector<std::array<cv::Point2f, 4>> pts_list;  ///< 该装甲板在各个图片列表下的角点信息
  int id;                                            ///< 装甲板编号
  Color color;                                       ///< 装甲板颜色
};

}  // namespace srm::autoaim

#endif  // SRM_NN_INFO_HPP_