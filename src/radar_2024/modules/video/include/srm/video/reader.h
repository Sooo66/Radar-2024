#ifndef SRM_VIDEO_READER_H_
#define SRM_VIDEO_READER_H_

#include <atomic>
#include <opencv2/videoio.hpp>

#include "srm/common/config.hpp"
#include "srm/video/camera.h"
#include "srm/video/frame.hpp"

enable_factory(srm::video, Reader, CreateReader);

namespace srm::video {
/// 视频源公共接口类
class Reader {
 public:
  Reader() = default;
  virtual ~Reader() = default;

  /// 视频源数量
  attr_reader_ref(source_count_, SourceCount);

  /**
   * @brief 初始化视频源
   * @param [in] prefix 前置路径
   * @return 是否初始化成功
   */
  virtual bool Initialize(std::string REF_IN prefix) = 0;

  /**
   * @brief 判断视频源状态
   * @param id 视频源编号
   * @return 视频源是否有图
   */
  virtual bool IsEmpty(int id = 0) = 0;

  /**
   * @brief 获取帧数据
   * @param [out] frame 帧数据输出
   * @param id 视频源编号
   * @return 是否获取成功
   */
  virtual bool GetFrame(Frame REF_OUT frame, int id = 0) = 0;

  /**
   * @brief 注册帧数据回调函数
   * @param callback 回调函数
   * @param obj 回调函数持有者指针
   * @param id 视频源编号
   */
  virtual void RegisterFrameCallback(FrameCallback callback, void *obj, int id = 0) = 0;

  /**
   * @brief 反注册帧数据回调函数
   * @param callback 回调函数
   * @param id 视频源编号
   */
  virtual void UnregisterFrameCallback(FrameCallback callback, int id = 0) = 0;

  /**
   * @brief 读取相机针孔模型矩阵
   * @param id 视频源编号
   * @return const cv::Mat& 相机针孔模型矩阵
   */
  virtual const cv::Mat &IntrinsicMat(int id = 0) const = 0;

  /**
   * @brief 读取相机畸变矩阵
   * @param id 视频源编号
   * @return const cv::Mat& 相机畸变矩阵
   */
  virtual const cv::Mat &DistortionMat(int id = 0) const = 0;

 protected:
  int source_count_;  ///< 视频源数量
};
}  // namespace srm::video

#endif  // SRM_VIDEO_READER_H_
