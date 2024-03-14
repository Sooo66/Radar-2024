#include "srm/nn/info.hpp"
#include "srm/nn/yolo.h"
namespace srm::nn {

class CarNetwork : public Yolo {
public:
  CarNetwork() = default;
  ~CarNetwork() override;

  /// @brief 根据推理结果切割车辆，作为后续装甲板识别的输入
  std::vector<ROI> GetROI(std::vector<Objects> REF_IN objs, cv::Mat REF_IN image);

private:
  inline static auto registry_ = RegistrySub<Yolo, CarNetwork>("car"); ///< 推理平台注册信息
};

CarNetwork::~CarNetwork() {
  delete execution_context_;
  delete engine_;
  delete runtime_;
  cudaFreeHost(input_data_host_);
  cudaFreeHost(output_data_host_);
}

std::vector<ROI> CarNetwork::GetROI(std::vector<Objects> REF_IN objs, cv::Mat REF_IN image) {
  std::vector<ROI> result;
  for (auto &&obj : objs) {
    // cv::Mat roi = image(cv::Rect(obj.x1, obj.y1, obj.x2 - obj.x1, obj.y2 - obj.y1));
    // result.push_back(roi);
    cv::Point top_left = {obj.x1, obj.y1};
    result.push_back({top_left,
                      obj.x2 - obj.x1,
                      obj.y2 - obj.y1,
                     });
  }
  return result;
}

} // namespace srm::n