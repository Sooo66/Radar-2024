#ifndef SRM_VIDEO_CAMERA_H_
#define SRM_VIDEO_CAMERA_H_

#include <atomic>
#include <thread>

#include "srm/common/buffer.hpp"
#include "srm/common/factory.hpp"
#include "srm/video/frame.hpp"

enable_factory(srm::video, Camera, CreateCamera);

namespace srm::video {
/// 相机公共接口类
class Camera {
  static constexpr size_t kBufferSize = 4;  ///< 缓冲区大小
 public:
  Camera() = default;
  virtual ~Camera() = default;

  /**
   * @brief 打开指定序列号的相机
   * @param [in] serial_number 序列号
   * @param [in] config_file 相机配置文件路径
   * @return 是否成功打开相机
   */
  virtual bool OpenCamera(std::string REF_IN serial_number, std::string REF_IN config_file) = 0;

  /**
   * @brief 关闭相机
   * @return 相机是否成功关闭
   */
  virtual bool CloseCamera() = 0;

  /**
   * @brief 检查缓冲区状态
   * @return 缓冲区是否为空
   */
  virtual bool IsEmpty();

  /**
   * @brief 从缓冲区中取出一帧
   * @param [out] frame 帧结构体
   * @return 是否成功取出（缓冲区中是否有数据）
   */
  virtual bool GetFrame(Frame REF_OUT frame) = 0;

  /**
   * @brief 开启视频流
   * @return 是否成功开启视频流
   */
  virtual bool StartStream() = 0;

  /**
   * @brief 关闭视频流
   * @return 是否成功关闭视频流
   */
  virtual bool StopStream() = 0;

  /**
   * @brief 导入相机配置文件
   * @param [in] config_file 配置文件路径
   * @return 是否成功导入配置
   */
  virtual bool ImportConfigurationFile(std::string REF_IN config_file) = 0;

  /**
   * @brief 导出相机配置文件
   * @param [in] config_file 配置文件路径
   * @return 是否成功导出配置
   */
  virtual bool ExportConfigurationFile(std::string REF_IN config_file) = 0;

  /**
   * @brief 检查相机连接状态
   * @return 相机是否连接
   */
  virtual bool IsConnected() = 0;

  /**
   * @brief 设置曝光时间
   * @param exposure_time 曝光时间，单位 us
   * @return 是否设置成功
   */
  virtual bool SetExposureTime(uint32_t exposure_time) = 0;

  /**
   * @brief 设置增益
   * @param gain 增益，单位 dB
   * @return 是否设置成功
   */
  virtual bool SetGainValue(float gain) = 0;

  /**
   * @brief 设置硬件触发相机
   * @param hardware_trigger 是否启用硬件触发
   * @return 是否设置成功
   */
  virtual bool SetHardwareTriggerMode(bool hardware_trigger) = 0;

  /**
   * @brief 设置软触发帧率
   * @note 该设置仅在软触发模式生效
   * @param frame_rate 帧率设置
   * @return 是否设置成功
   */
  virtual bool SetFrameRate(double frame_rate) = 0;

  /**
   * @brief 设置相机的时间戳单位
   * @param time_stamp_ns 表示多少纳秒
   */
  bool SetTimeStampNS(int time_stamp_ns);

  /**
   * @brief 注册相机回调，取图完毕时可立即修改 frame 的内容
   * @param callback 回调函数指针
   * @param obj 回调函数持有者指针
   */
  void RegisterFrameCallback(FrameCallback callback, void *obj);

  /**
   * @brief 反注册相机回调
   * @param callback 回调函数指针
   */
  void UnregisterFrameCallback(FrameCallback callback);

 protected:
  /// 注册的回调函数列表
  std::vector<std::pair<FrameCallback, void *>> callback_list_;
  std::string serial_number_;                        ///< 相机序列号
  int time_stamp_ns_{};                              ///< 时间戳单位
  bool stream_running_{};                            ///< 视频流运行标记
  std::atomic_bool stop_flag_{};                     ///< 停止守护线程信号
  std::unique_ptr<std::thread> daemon_thread_id_{};  ///< 守护线程
  Buffer<Frame, kBufferSize> buffer_;                ///< 帧数据缓冲区
};
}  // namespace srm::video

#endif  // SRM_VIDEO_CAMERA_H_
