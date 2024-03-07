#include "srm/video/writer.h"

#include <glog/logging.h>

namespace srm::video {

Writer::~Writer() {
  if (writer_) {
    stop_flag_ = true;
    LOG(INFO) << "Waiting for writing video...";
    if (thread_.joinable()) {
      thread_.join();
    }
    LOG(INFO) << "Writing video finished.";
  }
}

bool Writer::Open(std::string REF_IN video_file, cv::Size frame_size) {
  writer_ = std::make_unique<cv::VideoWriter>();
  if (!writer_->open(video_file, cv::CAP_FFMPEG, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), kFrameRate, frame_size)) {
    LOG(INFO) << "Failed to open video file " << video_file << ".";
    writer_.reset();
    return false;
  }
  thread_ = std::thread(WritingThreadFunction, this);
  LOG(INFO) << "Opened video file " << video_file << ".";
  return true;
}

bool Writer::Write(cv::Mat FWD_IN frame, double time) {
  if (writer_ && time >= time_ + 1.0 / kFrameRate - 1e-6) {
    buffer_.Push(std::move(frame));
    time_ = time;
    return true;
  } else {
    return false;
  }
}

void Writer::WritingThreadFunction(void *obj) {
  auto *self = static_cast<Writer *>(obj);
  cv::Mat frame;
  while (!self->stop_flag_) {
    while (self->buffer_.Pop(frame)) {
      self->writer_->write(frame);
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(0xff));
  }
}
}  // namespace srm::video
