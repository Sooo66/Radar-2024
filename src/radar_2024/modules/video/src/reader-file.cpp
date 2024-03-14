#include <glog/logging.h>

#include <opencv2/videoio.hpp>
#include <thread>

#include "srm/common/buffer.hpp"
#include "srm/common/config.hpp"
#include "srm/video/reader.h"

namespace srm::video {

/**
 * @brief 文件读取视频源接口类
 * @warning 禁止直接构造此类，请使用 @code srm::video::CreateReader("file") @endcode 获取该类的公共接口指针
 */
class FileReader final : public Reader {
  inline static auto registry = RegistrySub<Reader, FileReader>("file");  ///< 视频源注册信息
  static constexpr size_t kBufferSize = 2;                                ///< 缓冲区大小
 public:
  FileReader() = default;
  ~FileReader() override;

  bool Initialize(std::string REF_IN prefix) final;
  bool IsEmpty(int id = 0) final;
  bool GetFrame(Frame REF_OUT frame, int id = 0) final;
  void RegisterFrameCallback(FrameCallback callback, void *obj, int id = 0) final;
  void UnregisterFrameCallback(FrameCallback callback, int id = 0) final;
  const cv::Mat &IntrinsicMat(int id = 0) const final;
  const cv::Mat &DistortionMat(int id = 0) const final;

 private:
  struct VideoInfo {
    cv::VideoCapture video;                                       ///< 视频读取接口
    cv::Mat intrinsic_mat;                                        ///< 相机针孔模型矩阵
    cv::Mat distortion_mat;                                       ///< 相机畸变矩阵
    Buffer<Frame, 4> buffer;                                      ///< 缓冲区
    std::vector<std::pair<FrameCallback, void *>> callback_list;  ///< 注册回调函数列表
  };

  double frame_rate_{};                                  ///< 帧率
  uint64_t time_stamp_{};                                ///< 时间戳
  std::shared_ptr<std::thread> thread_{};                ///< 读图线程
  std::atomic_bool stop_flag_{};                         ///< 线程停止信号
  std::vector<std::unique_ptr<VideoInfo>> videos_info_;  ///< 视频接口
};

FileReader::~FileReader() {
  stop_flag_ = true;
  if (thread_->joinable()) {
    thread_->join();
  }
}

bool FileReader::Initialize(std::string REF_IN prefix) {
  source_count_ = cfg.Get<int>({prefix, "cam_num"});
  videos_info_.resize(source_count_);

  for (int i = 0; i < source_count_; i++) {
    videos_info_[i] = std::make_unique<VideoInfo>();
    auto &[video, intrinsic_mat, distortion_mat, _, __] = *videos_info_[i];
    auto cam_name = cfg.Get<std::string>({prefix, "camera" + std::to_string(i)});
    std::string cameras_prefix = "video.cameras." + cam_name;
    intrinsic_mat = cfg.Get<cv::Mat>({cameras_prefix, "intrinsic_mat"});
    distortion_mat = cfg.Get<cv::Mat>({cameras_prefix, "distortion_mat"});
    if (intrinsic_mat.empty() || distortion_mat.empty()) {
      LOG(ERROR) << "Invalid intrinsic matrix or distortion matrix. Please check your configuration.";
      return false;
    }
    auto video_file = cfg.Get<std::string>({prefix, "video" + std::to_string(i)});
    video.open(video_file);
    if (!video.isOpened()) {
      LOG(ERROR) << "Failed to open video file " << video_file << ".";
      intrinsic_mat.release();
      distortion_mat.release();
      return false;
    }
  }

  frame_rate_ = videos_info_[0]->video.get(cv::CAP_PROP_FPS);
  for (auto &video_info : videos_info_) {
    if (video_info->video.get(cv::CAP_PROP_FPS) != frame_rate_) {
      LOG(ERROR) << "Different frame rate of videos";
      return false;
    }
  }

  thread_ = std::make_unique<std::thread>(
      [](FileReader *self) {
        while (!self->stop_flag_) {
          double fps = cfg.Get<double>({"fps_limit"}) + 10;
          if (self->videos_info_[0]->buffer.Full()) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<uint64_t>(1e9 / fps)));
            continue;
          }

          auto time_start = std::chrono::system_clock::now();
          for (int i = 0; i < self->SourceCount(); i++) {
            Frame frame;
            auto &video_info = self->videos_info_[i];
            if (!video_info->video.read(frame.image)) {
              continue;
            }
            self->time_stamp_ += uint64_t(1e9 / self->frame_rate_);
            frame.time_stamp = self->time_stamp_;
            for (auto p : video_info->callback_list) {
              (*p.first)(p.second, frame);
            }
            if (frame.valid) {
              video_info->buffer.Push(std::move(frame));
            }
          }
          std::chrono::duration<double, std::nano> delta_time = std::chrono::system_clock::now() - time_start;
          if (delta_time.count() < 1e9 / fps) {
            std::chrono::duration<double, std::nano> delay_ns(1e9 / fps - delta_time.count());
            std::this_thread::sleep_for(
                std::chrono::nanoseconds(std::chrono::duration_cast<std::chrono::nanoseconds>(delay_ns)));
          }
        }
      },
      this);
  LOG(INFO) << "Initialized file video source.";
  return true;
}

bool FileReader::IsEmpty(int id) { return videos_info_[id]->buffer.Empty(); }

bool FileReader::GetFrame(Frame REF_OUT frame, int id) { return videos_info_[id]->buffer.Pop(frame); }

void FileReader::RegisterFrameCallback(FrameCallback callback, void *obj, int id) {
  videos_info_[id]->callback_list.emplace_back(callback, obj);
  DLOG(INFO) << "Registered video source callback FUNC " << callback << " OBJ " << obj << ".";
}

void FileReader::UnregisterFrameCallback(FrameCallback callback, int id) {
  auto filter = [callback](auto p) { return p.first == callback; };
  auto &callback_list = videos_info_[id]->callback_list;
  callback_list.erase(std::remove_if(callback_list.begin(), callback_list.end(), filter), callback_list.end());
  DLOG(INFO) << "Unregistered video source callback FUNC " << callback << ".";
}

const cv::Mat &FileReader::IntrinsicMat(int id) const { return videos_info_[id]->intrinsic_mat; }

const cv::Mat &FileReader::DistortionMat(int id) const { return videos_info_[id]->distortion_mat; }
}  // namespace srm::video
