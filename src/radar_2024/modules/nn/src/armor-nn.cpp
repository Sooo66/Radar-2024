#include "srm/nn/yolo.h"
#include "srm/nn/info.hpp"
namespace srm::nn {

// struct Armor {
//   ROI roi;  ///< 车辆的roi 
//   float x1;                      ///< 预测框左上角顶点横坐标
//   float y1;                      ///< 预测框左上角顶点纵坐标
//   float x2;                      ///< 预测框右下角顶点横坐标
//   float y2;                      ///< 预测框右下角顶点纵坐标
//   float prob;                    ///< 置信度
//   int cls;                       ///< 类编号
// }

class ArmorNetwork : public Yolo {
 public:
  ArmorNetwork() = default;
  ~ArmorNetwork() override;

 private:
  inline static auto registry_ = RegistrySub<Yolo, ArmorNetwork>("armor"); ///< 推理平台注册信息
};

ArmorNetwork::~ArmorNetwork() {
  delete execution_context_;
  delete engine_;
  delete runtime_;
  cudaFreeHost(input_data_host_);
  cudaFreeHost(output_data_host_);
}

// Armor ArmorNetwork::Infer(ROI REF_IN roi) {
//   auto &image = roi.roi_mat;

// }

} // namespace srm::n