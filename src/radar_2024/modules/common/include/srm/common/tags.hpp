#ifndef attr_reader_val
/**
 * @brief 将类私有成员变量设置为外界只读
 * @param _var 目标成员变量
 * @param _func 读取器函数名称
 * @note 此版本生成值传递函数，使用 attr_reader_ref 生成引用传递函数
 * @details 在类定义的 public 部分添加指令，生成读取器函数
 * @code{.cpp}
 * class Foo() {
 *  public:
 *   attr_reader_val(private_var_, PrivateVar);
 *  private:
 *   int private_var_{};
 * }
 * int main() {
 *   Foo bar;
 *   int private_var = bar.PrivateVar();  // 调用函数获取值
 * }
 * @endcode
 */
#define attr_reader_val(_var, _func) \
  auto _func() const { return _var; }
#endif

#ifndef attr_writer_val
/**
 * @brief 将类私有成员变量设置为外界可写
 * @param _var 目标成员变量
 * @param _func 设定器函数名称
 * @note 此版本生成值传递函数，使用 attr_writer_ref 生成引用传递函数
 * @details 在类定义的 public 部分添加指令，生成设定器函数
 * @code{.cpp}
 * class Foo() {
 *  public:
 *   attr_writer_val(private_var_, SetPrivateVar);
 *  private:
 *   int private_var_{};
 * }
 * int main() {
 *   Foo bar;
 *   bar.SetPrivateVar(42);  // 调用函数设定值
 * }
 * @endcode
 */
#define attr_writer_val(_var, _func) \
  template <typename T>              \
  void _func(const T& value) {       \
    _var = value;                    \
  }
#endif

#ifndef attr_reader_ref
/**
 * @brief 将类私有成员变量设置为外界只读
 * @param _var 目标成员变量
 * @param _func 读取器函数名称
 * @note 此版本生成引用传递函数，使用 attr_reader_val 生成值传递函数
 * @details 在类定义的 public 部分添加指令，生成读取器函数
 * @code{.cpp}
 * class Foo() {
 *  public:
 *   attr_reader_ref(private_var_, PrivateVar);
 *  private:
 *   std::string private_var_{};
 * }
 * int main() {
 *   Foo bar;
 *   std::string private_var = bar.PrivateVar();  // 调用函数获取值
 * }
 * @endcode
 */
#define attr_reader_ref(_var, _func) \
  auto&& _func() const { return _var; }
#endif

#ifndef REF_IN
#define REF_IN const&
#endif

#ifndef PTR_IN
#define PTR_IN const*
#endif

#ifndef FWD_IN
#define FWD_IN &&
#endif

#ifndef REF_OUT
#define REF_OUT &
#endif

#ifndef PTR_OUT
#define PTR_OUT *
#endif