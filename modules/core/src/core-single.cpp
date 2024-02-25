#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "srm/core.hpp"

namespace srm::core {

/**
 * @brief 单目机器人主控类
 * @warning 禁止直接构造此类，请使用 @code srm::core::CreateCore("single") @endcode 获取该类的公共接口指针
 */
class SingleCore final : public BaseCore {
  inline static auto registry = RegistrySub<BaseCore, SingleCore>("single");  ///< 主控注册信息
 public:
  SingleCore() = default;
  ~SingleCore() override = default;
};

}  // namespace srm::core