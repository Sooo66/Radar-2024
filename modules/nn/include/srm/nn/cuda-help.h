#ifndef _SRM_NN_TENSORRT_CUDA_HELP_H_
#define _SRM_NN_TENSORRT_CUDA_HELP_H_

#include <NvInferPlugin.h>
#include <cuda_runtime.h>
#include <dirent.h>
#include <glog/logging.h>

#include <fstream>
#include <iterator>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "srm/common/tags.hpp"

namespace srm::nn {
template <typename T>
std::shared_ptr<T> make_shared(T *ptr) {
  return std::shared_ptr<T>(ptr, [](T *p) { delete p; });
}

#define checkRuntime(op) __check_cuda_runtime((op), #op, __FILE__, __LINE__)
inline bool __check_cuda_runtime(cudaError_t code, char PTR_IN op, char PTR_IN file, int line) {
  if (code != cudaSuccess) {
    auto err_name = cudaGetErrorName(code);
    auto err_message = cudaGetErrorString(code);
    LOG(ERROR) << "runtime eror " << file << ": " << line << " " << op << " failed.\n code = " << err_name
               << ", message = " << err_message;
    return false;
  }
  return true;
}

class TRTLogger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, nvinfer1::AsciiChar PTR_IN msg) noexcept override {
    if (severity <= Severity::kERROR)
      DLOG(ERROR) << msg;
    else if (severity <= Severity::kWARNING)
      DLOG(WARNING) << msg;
    else if (severity <= Severity::kINFO)
      DLOG(INFO) << msg;
  }
};

}  // namespace srm::nn

#endif  // _SRM_NN_TENSORRT_CUDA_HELP_H_
