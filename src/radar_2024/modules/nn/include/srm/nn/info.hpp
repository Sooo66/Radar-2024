#ifndef SRM_NN_INFO_HPP_
#define SRM_NN_INFO_HPP_

#include <opencv2/core/mat.hpp>

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
};

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
  ROI roi;                                           ///< 该装甲板roi  
  std::array<cv::Point2f, 2> pts;                    ///< 装甲板角点信息
  float prob;
  int id;                                            ///< 装甲板编号
  Color color;                                       ///< 装甲板颜色
  int source;                                        ///< 该装甲板是由哪个相机源预测得到的
  float depth;
};

}  // namespace srm::autoaim

#endif  // SRM_NN_INFO_HPP_