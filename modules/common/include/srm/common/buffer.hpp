#ifndef SRM_COMMON_BUFFER_H_
#define SRM_COMMON_BUFFER_H_

#include <mutex>

#include "srm/common/tags.hpp"

namespace srm {
/**
 * @brief 带锁循环队列，自动丢弃旧数据
 * @tparam T 数据类型
 * @tparam N 循环队列大小
 */
template <typename T, size_t N>
class Buffer final {
 private:
  std::array<T, N> data_;  ///< 数据存储
  size_t head_{};          ///< 头指针
  size_t tail_{};          ///< 尾指针
  bool full_{};            ///< 是否队满
  std::mutex lock_;        ///< 操作锁

 public:
  Buffer() = default;
  ~Buffer() = default;

  /**
   * @brief 放入数据，队列已满时将覆盖旧数据
   * @param [in] obj 待移动数据
   */
  void Push(T FWD_IN obj) {
    std::lock_guard<std::mutex> lock{lock_};
    data_[tail_] = std::forward<T>(obj);
    ++tail_ %= N;
    if (full_) {
      ++head_ %= N;
    }
    full_ = head_ == tail_;
  }

  /**
   * @brief 取出数据
   * @param [out] obj 数据目标位置
   * @return 队列是否为空
   */
  bool Pop(T REF_OUT obj) {
    std::lock_guard<std::mutex> lock{lock_};
    if (head_ == tail_ && !full_) {
      return false;
    }
    obj = std::move(data_[head_]);
    ++head_ %= N;
    full_ = false;
    return true;
  }

  /**
   * @brief 判断是否为空
   * @return 队列是否为空
   */
  bool Empty() { return head_ == tail_ && !full_; }

  attr_reader_val(full_, Full);
};
}  // namespace srm
#endif  // SRM_COMMON_BUFFER_H_
