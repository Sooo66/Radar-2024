#include "srm/nn/yolo.h"
namespace srm::nn {
class Car : public Yolo {
 public:
  Car() = default;
  ~Car() override;
  
  /// @brief 根据推理结果切割车辆，作为后续装甲板识别的输入
  std::vector<cv::Mat> PostProcess(std::vector<Objects> objs, cv::Mat image) override;  

 private:
  inline static auto registry_ = RegistrySub<Yolo, Car>("car");  ///< 推理平台注册信息
};

Car::~Car() {
  delete execution_context_;
  delete engine_;
  delete runtime_;
  cudaFreeHost(input_data_host_);
  cudaFreeHost(output_data_host_);
}

std::vector<cv::Mat> Car::PostProcess(std::vector<Objects> objs, cv::Mat image) {
  std::vector<cv::Mat> result;
  for (auto &&obj : objs) { 
    cv::Mat roi = image(cv::Rect(obj.x1, obj.y1, obj.x2 - obj.x1, obj.y2 - obj.y1));
    result.push_back(roi);
  }

  return result;
}
    
} // namespace srm::n 