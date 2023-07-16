#pragma once

#include <functional>
#include <memory>

namespace vtc::delegate {

namespace details {

template<typename ReturnT, typename... Args>
class func_wrapper
{
public:
  virtual ~func_wrapper() = default;
  virtual ReturnT func(Args... args) = 0;
};

template<typename T, typename ReturnT, typename... Args>
class member_func_wrapper : public func_wrapper<ReturnT, Args...>
{
public:
  member_func_wrapper(T *obj, ReturnT (T::*func)(Args...)) : obj_(obj), func_(func) {}

  ReturnT func(Args... args) override
  {
    return std::invoke(func_, obj_, std::forward<Args>(args)...);
  }

private:
  T *obj_;
  ReturnT (T::*func_)(Args...);
};

template<typename T, typename ReturnT, typename... Args>
class const_member_func_wrapper : public func_wrapper<ReturnT, Args...>
{
public:
  const_member_func_wrapper(const T *obj, ReturnT (T::*func)(Args...) const) : obj_(obj), func_(func) {}

  ReturnT func(Args... args) override
  {
    return std::invoke(func_, obj_, std::forward<Args>(args)...);
  }

private:
  const T *obj_;
  ReturnT (T::*func_)(Args...) const;
};

template<typename ReturnT, typename... Args>
class nonmember_func_wrapper : public func_wrapper<ReturnT, Args...>
{
public:
  explicit nonmember_func_wrapper(ReturnT (*func)(Args...)) : _func(func) {}

  ReturnT func(Args... args) override
  {
    return _func(std::forward<Args>(args)...);
  }

private:
  ReturnT (*_func)(Args...);
};

template<typename ReturnT, typename... Args>
class func_obj_wrapper : public func_wrapper<ReturnT, Args...>
{
public:
  explicit func_obj_wrapper(std::function<ReturnT(Args...)> func) : func_(func) {}

  ReturnT func(Args... args) override
  {
    return std::invoke(func_, std::forward<Args>(args)...);
  }

private:
  std::function<ReturnT(Args...)> func_;
};

}// namespace details

template<typename>
class delegate;

template<typename ReturnT, typename... Args>
class delegate<ReturnT(Args...)>
{
public:
  template<typename T>
  [[maybe_unused]] delegate(T *obj, ReturnT (T::*func)(Args...)) : wrapper_(
      std::make_unique<details::member_func_wrapper<T, ReturnT, Args...>>(obj, func))
  {}

  template<typename T>
  [[maybe_unused]] delegate(const T *obj, ReturnT (T::*func)(Args...) const) : wrapper_(
      std::make_unique<details::const_member_func_wrapper<T, ReturnT, Args...>>(obj, func))
  {}

  [[maybe_unused]] explicit delegate(ReturnT(func)(Args...)) : wrapper_(
      std::make_unique<details::nonmember_func_wrapper<ReturnT, Args...>>(func))
  {}

  explicit delegate(std::function<ReturnT(Args...)> func) : wrapper_(
      std::make_unique<details::func_obj_wrapper<ReturnT, Args...>>(func))
  {}

  /*
    Does not support automatic type deduction for lambda delegates. Must specify
    the template parameters for the delegate object if using a lambda argument:

    delegate<bool(int,int)> d([](int a, int b){ return a == b; });

    This is no way to do compile-time check if the argument is lambda, c.f:
    §5.1.2 [expr.prim.lambda] p3

    The type of the lambda-expression (which is also the type of the closure object)
    is a unique, unnamed nonunion class type [...]
  */
  explicit delegate(auto func) : delegate(static_cast<std::function<ReturnT(Args...)>>(func))
  {}

  ReturnT operator()(Args... args)
  {
    return wrapper_->func(std::forward<Args>(args)...);
  }

private:
  std::unique_ptr<details::func_wrapper<ReturnT, Args...>> wrapper_;
};

template<typename T, typename ReturnT, typename... Args>
delegate(T *, ReturnT (T::*)(Args...)) -> delegate<ReturnT(Args...)>;

template<typename T, typename ReturnT, typename... Args>
delegate(const T *, ReturnT (T::*)(Args...) const) -> delegate<ReturnT(Args...)>;

template<typename ReturnT, typename... Args>
delegate(ReturnT (*)(Args...)) -> delegate<ReturnT(Args...)>;

template<typename ReturnT, typename... Args>
delegate(std::function<ReturnT(Args...)>) -> delegate<ReturnT(Args...)>;

}// namespace vtc::delegate
