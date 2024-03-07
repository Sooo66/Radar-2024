#ifndef SRM_CORE_CORE_BASE_H_
#define SRM_CORE_CORE_BASE_H_

#include "srm/common.hpp"
#include "fps-controller.h"
#include "srm/nn.hpp"

/// 抓取并处理来自操作系统的控制信号
void SignalHandler(int);

enable_factory(srm::core, BaseCore, CreateCore);

namespace srm::core {

/// 机器人主控公共接口类
class BaseCore {
  friend void ::SignalHandler(int);
  inline static std::atomic_bool exit_signal = false;  ///< 主循环退出信号

 public:
  BaseCore() = default;
  virtual ~BaseCore();

  /**
   * @brief 初始化机器人
   * @return 是否初始化成功
   */
  virtual bool Initialize();

  /**
   * @brief 执行主控制循环
   * @return 错误码，可作为 main() 的返回值
   */
  virtual int Run();

 protected:
  std::unique_ptr<srm::core::FpsController> fps_controller_;  ///< 帧率控制器
  std::unique_ptr<srm::nn::Yolo> car_;  ///< 车辆检测神经网络
  std::unique_ptr<srm::nn::Yolo> armor_;  ///< 装甲板检测神经网络
};

}  // namespace srm::core

#endif  // SRM_CORE_CORE_BASE_H_