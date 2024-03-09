#ifndef SRM_CORE_CORE_BASE_H_
#define SRM_CORE_CORE_BASE_H_

#include "srm/common.hpp"
#include "fps-controller.h"
#include "srm/video.hpp"
#include "srm/viewer.hpp"

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
  std::vector<video::Frame> frame_list_;  ///< 帧数据
  
    /**
   * @brief 取图回调函数
   * @details 设第二个键值为k, 则表示这是第k个视频源的回调函数
   */
  std::vector<std::pair<srm::video::FrameCallback, int>> frame_callback_list_;

  
  std::unique_ptr<srm::core::FpsController> fps_controller_;  ///< 帧率控制器
  std::unique_ptr<srm::video::Reader> reader_;  ///< 视频读入接口
  std::unique_ptr<srm::nn::Detector> detector_; ///< 装甲板检测接口
  std::unique_ptr<viewer::VideoViewer> viewer_;        ///< 图像显示接口


 
 protected:
  virtual bool InitializeReader();
  virtual bool InitializeDetector();
  virtual bool InitializeViewer();

  virtual bool UpdateFrameList(); 
  
};

}  // namespace srm::core

#endif  // SRM_CORE_CORE_BASE_H_