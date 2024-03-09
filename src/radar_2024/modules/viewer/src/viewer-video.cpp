#include "srm/viewer/viewer-video.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>

#include "srm/common/config.hpp"
#include "srm/common/tags.hpp"

namespace srm::viewer {

VideoViewer::~VideoViewer() {
  stop_flag_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  system("pkill -f gunicorn");
  if (shm_ptr_ != nullptr) {
    munmap(shm_ptr_, shm_size_);
  }
  if (shm_fd_ != -1) {
    close(shm_fd_);
  }
  if (key_ != nullptr) {
    shm_unlink(key_);
  }
}

bool VideoViewer::Initialize(std::string FWD_IN shm_name) {
  if (std::filesystem::exists(shm_name)) {
    LOG(WARNING) << "The shm file already exists. Please check whether this module is reused.";
    std::filesystem::remove(shm_name);
  }
  key_ = shm_name.c_str();
  shm_unlink(key_);
  shm_fd_ = shm_open(key_, O_CREAT | O_RDWR, 0666);
  if (shm_fd_ == -1) {
    LOG(ERROR) << "Failed to open shared memory.";
    return false;
  }

  shm_size_ = cfg.Get<int>({"viewer.web.shm_size"});
  if (ftruncate(shm_fd_, shm_size_) == -1) {
    LOG(ERROR) << "Failed to truncate shared memory.";
    return false;
  }

  shm_ptr_ = (unsigned char *)mmap(nullptr, shm_size_, PROT_WRITE, MAP_SHARED, shm_fd_, 0);
  if (shm_ptr_ == MAP_FAILED) {
    LOG(ERROR) << "Failed to map shared memory.";
    return false;
  }

  system("pkill -f gunicorn");
  if (system("cd ..; nohup gunicorn -b 127.0.0.1:9003 modules.viewer.video:app -w 2 --timeout 0 > log/web.log "
             "2>&1 &") != 0) {
    LOG(ERROR) << "Failed to start web server. Please check your python configuration.";
    return false;
  }

  stop_flag_ = false;
  thread_ = std::thread(
      [](VideoViewer *self) {
        std::vector<uchar> buf;
        cv::Mat img;
        while (!self->stop_flag_) {
          if (self->buffer_.Empty()) {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(5ms);
            continue;
          }
          using namespace std::chrono_literals;
          std::this_thread::sleep_for(30ms);
          self->buffer_.Pop(img);
          cv::imencode(".jpg", img, buf);
          int buf_size = buf.size();
          memcpy(self->shm_ptr_, &buf_size, sizeof(int));
          memcpy(self->shm_ptr_ + sizeof(int), buf.data(), sizeof(uchar) * buf.size());
        }
      },
      this);

  return true;
}

bool VideoViewer::SendFrame(cv::Mat REF_IN img) {
  auto img_clone = img.clone();
  buffer_.Push(std::move(img_clone));
  return true;
}

}  // namespace srm::viewer