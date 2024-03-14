#ifndef SRM_VIDEO_FRAME_HPP_
#define SRM_VIDEO_FRAME_HPP_

#include <opencv2/core/mat.hpp>
#if __cplusplus <= 201703L
#include <functional>
#endif

namespace srm::video {

/// 帧信息结构体
struct Frame {
  bool valid = true;                ///< 此帧数据是否完整
  cv::Mat image;                    ///< 获取的图像
  std::shared_ptr<void> sync_data;  ///< 同步接受的信息
  uint64_t time_stamp;              ///< 时间戳，单位 ns
};

/// 帧回调函数类型
using FrameCallback = std::function<void(void *, Frame &)> *;

}  // namespace srm::video

#endif  // SRM_VIDEO_FRAME_H_
