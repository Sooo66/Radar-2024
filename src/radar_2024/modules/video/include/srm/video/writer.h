#ifndef SRM_VIDEO_WRITER_H_
#define SRM_VIDEO_WRITER_H_

#include <atomic>
#include <opencv2/videoio.hpp>
#include <thread>

#include "srm/common/buffer.hpp"

namespace srm::video {
/// @todo 双目的视频源支持
/// 视频录像接口
class Writer final {
  static constexpr size_t kBufferSize = 512;  ///< 缓冲区大小
  static constexpr size_t kFrameRate = 30;    ///< 录制帧数
 public:
  Writer() = default;
  ~Writer();

  /**
   * @brief 打开或创建新视频文件
   * @param [in] video_file 文件名
   * @param frame_size 图像长宽大小
   * @param fps 录像帧率
   * @return
   */
  bool Open(std::string REF_IN video_file, cv::Size frame_size);

  /**
   * @brief 写入视频
   * @param [in] frame 图像数据
   * @param time 当前时间
   * @return 是否实际写入数据
   */
  bool Write(cv::Mat FWD_IN frame, double time);

 private:
  static void WritingThreadFunction(void *obj);

  std::unique_ptr<cv::VideoWriter> writer_;  ///< 视频写入接口
  Buffer<cv::Mat, kBufferSize> buffer_;      ///< 缓冲区
  std::thread thread_;                       ///< 写入线程
  std::atomic_bool stop_flag_{};             ///< 写入线程停止信号
  double time_{};                            ///< 当前时间
};
}  // namespace srm::video

#endif  // SRM_VIDEO_WRITER_H_
