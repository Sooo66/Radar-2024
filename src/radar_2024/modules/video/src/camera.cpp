#include "srm/video/camera.h"

#include <glog/logging.h>

namespace srm::video {

void Camera::RegisterFrameCallback(FrameCallback callback, void *obj) {
  callback_list_.emplace_back(callback, obj);
  DLOG(INFO) << "Registered camera callback FUNC " << callback << " OBJ " << obj << ".";
}

void Camera::UnregisterFrameCallback(FrameCallback callback) {
  auto filter = [callback](auto p) { return p.first == callback; };
  callback_list_.erase(std::remove_if(callback_list_.begin(), callback_list_.end(), filter), callback_list_.end());
  DLOG(INFO) << "Unregistered camera callback FUNC " << callback << ".";
}

bool Camera::IsEmpty() { return buffer_.Empty(); }

bool Camera::SetTimeStampNS(int time_stamp_ns) {
  if (time_stamp_ns == 0) {
    return false;
  }
  time_stamp_ns_ = time_stamp_ns;
  return true;
}

}  // namespace srm::video
