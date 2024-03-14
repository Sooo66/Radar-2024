#ifndef SRM_VIEWER_VIEWER_VIDEO_H_
#define SRM_VIEWER_VIEWER_VIDEO_H_

#include <atomic>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "srm/common.hpp"

namespace srm::viewer {

/// 参数可视化类
class VideoViewer {
 public:
  ~VideoViewer();

  /**
   * @brief 初始化video可视化类
   * @return 是否初始化成功
   */
  bool Initialize(std::string FWD_IN shm_path);

  /**
   * @brief 发送图像到网页
   * @param frame 要发送的图像
   * @return 是否发送成功
   */
  bool SendFrame(cv::Mat REF_IN img);

 private:
  const char* key_{nullptr};         ///< 共享内存的key
  int shm_fd_{};                     ///< 共享内存的文件描述符
  unsigned char* shm_ptr_{nullptr};  ///< 指向共享内存起始地址的指针
  size_t shm_size_{};                ///< 共享内存大小
  std::thread thread_;               ///< 编码和发送数据的线程
  std::atomic_bool stop_flag_;       ///< 线程停止信号
  // Buffer<cv::UMat, 2> buffer_;       ///< 待编码的图像
  Buffer<cv::Mat, 4> buffer_;  ///< 待编码的图像
};

}  // namespace srm::viewer

#endif  // SRM_VIEWER_VIEWER_VIDEO_H_