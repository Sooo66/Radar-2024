#include "srm/nn/yolo.h"
namespace srm::nn {
class Armor : public Yolo {
 public:
  Armor() = default;
  ~Armor() override;
  
 private:
  inline static auto registry_ = RegistrySub<Yolo, Armor>("armor");  ///< 推理平台注册信息
};

Armor::~Armor() {
  delete execution_context_;
  delete engine_;
  delete runtime_;
  cudaFreeHost(input_data_host_);
  cudaFreeHost(output_data_host_);
}
    
} // namespace srm::n 