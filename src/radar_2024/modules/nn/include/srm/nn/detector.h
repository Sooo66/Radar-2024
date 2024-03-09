#ifndef SRM_NN_DETECTOR_H_
#define SRM_NN_DETECTOR_H_
#include <cstring>
#include <opencv2/opencv.hpp>

#include "srm/common/factory.hpp"
#include "srm/common/tags.hpp"
#include "cuda-help.h"
#include "info.hpp"

namespace srm::nn {

/// yolov8神经网络类
class Detector {
 public:
  Detector() = default;
  virtual ~Detector() = default;

  /**
   * @brief 初始化网络
   * @param model_file 模型文件路径
   * @param num_classes 识别物体的种类数量
   * @param num_points 识别物体的关键点数量
   * @return 是否初始化成功
   */
  virtual bool Initialize();

  /**
   * @brief 运行神经网络
   * @param image 需要检测的图片
   * @return 检测到的物体
   */
  virtual bool Run(std::vector<cv::Mat> REF_IN image_list, std::vector<Armor> REF_OUT armor_list);

 protected:
  std::unique_ptr<CarNetwork> car_nn_;
  std::unique_ptr<ArmorNetwork> armor_nn_;
};

}  // namespace srm::nn

#endif  // SRM_NN_DETECTOR_H_