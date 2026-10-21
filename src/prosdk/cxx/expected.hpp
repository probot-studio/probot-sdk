// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk::cxx::expected: a C++20 implementation of C++23 std::expected ([expected]).
// Vendored code is rewritten from std::expected/unexpected/unexpect/bad_expected_access to
// these names by tools/upstream/wpilib.py. This is always its own type, never an alias of
// std::expected, so headers and the precompiled archive agree under any -std.

#pragma once

#include <exception>
#include <functional>
#include <initializer_list>
#include <memory>
#include <type_traits>
#include <utility>

namespace prosdk::cxx {

template <class E>
class unexpected;

template <class T, class E>
class expected;

namespace detail {
template <class T>
inline constexpr bool is_unexpected = false;
template <class E>
inline constexpr bool is_unexpected<unexpected<E>> = true;

template <class T>
inline constexpr bool is_expected = false;
template <class T, class E>
inline constexpr bool is_expected<expected<T, E>> = true;

template <class E>
concept can_be_unexpected = std::is_object_v<E> && !std::is_array_v<E> &&
                            !is_unexpected<E> && !std::is_const_v<E> &&
                            !std::is_volatile_v<E>;

// T must be constructible from expected<U, G> in any form (used to reject converting ctors).
template <class T, class U, class G>
inline constexpr bool converts_from_any_cvref =
    std::is_constructible_v<T, expected<U, G>&> ||
    std::is_constructible_v<T, expected<U, G>> ||
    std::is_constructible_v<T, const expected<U, G>&> ||
    std::is_constructible_v<T, const expected<U, G>> ||
    std::is_convertible_v<expected<U, G>&, T> ||
    std::is_convertible_v<expected<U, G>, T> ||
    std::is_convertible_v<const expected<U, G>&, T> ||
    std::is_convertible_v<const expected<U, G>, T>;

template <class E, class U, class G>
inline constexpr bool unexpected_converts_from =
    std::is_constructible_v<unexpected<E>, expected<U, G>&> ||
    std::is_constructible_v<unexpected<E>, expected<U, G>> ||
    std::is_constructible_v<unexpected<E>, const expected<U, G>&> ||
    std::is_constructible_v<unexpected<E>, const expected<U, G>>;

struct in_place_from_invoke_t {
  explicit in_place_from_invoke_t() = default;
};
struct unexpect_from_invoke_t {
  explicit unexpect_from_invoke_t() = default;
};
}  // namespace detail

/** Exception thrown by expected::value() when the object holds an error. */
template <class E>
class bad_expected_access;

template <>
class bad_expected_access<void> : public std::exception {
 protected:
  bad_expected_access() noexcept = default;
  bad_expected_access(const bad_expected_access&) = default;
  bad_expected_access(bad_expected_access&&) = default;
  bad_expected_access& operator=(const bad_expected_access&) = default;
  bad_expected_access& operator=(bad_expected_access&&) = default;
  ~bad_expected_access() override = default;

 public:
  const char* what() const noexcept override { return "bad access to expected without value"; }
};

template <class E>
class bad_expected_access : public bad_expected_access<void> {
 public:
  explicit bad_expected_access(E e) : m_unex(std::move(e)) {}
  E& error() & noexcept { return m_unex; }
  const E& error() const& noexcept { return m_unex; }
  E&& error() && noexcept { return std::move(m_unex); }
  const E&& error() const&& noexcept { return std::move(m_unex); }

 private:
  E m_unex;
};

/** Tag type for in-place construction of an error. */
struct unexpect_t {
  explicit unexpect_t() = default;
};
inline constexpr unexpect_t unexpect{};

/** Wrapper that marks a value as the error alternative of an expected. */
template <class E>
class unexpected {
  static_assert(detail::can_be_unexpected<E>);

 public:
  constexpr unexpected(const unexpected&) = default;
  constexpr unexpected(unexpected&&) = default;

  template <class Err = E>
    requires(!std::is_same_v<std::remove_cvref_t<Err>, unexpected> &&
             !std::is_same_v<std::remove_cvref_t<Err>, std::in_place_t> &&
             std::is_constructible_v<E, Err>)
  constexpr explicit unexpected(Err&& e) noexcept(std::is_nothrow_constructible_v<E, Err>)
      : m_unex(std::forward<Err>(e)) {}

  template <class... Args>
    requires std::is_constructible_v<E, Args...>
  constexpr explicit unexpected(std::in_place_t, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<E, Args...>)
      : m_unex(std::forward<Args>(args)...) {}

  template <class U, class... Args>
    requires std::is_constructible_v<E, std::initializer_list<U>&, Args...>
  constexpr explicit unexpected(std::in_place_t, std::initializer_list<U> il, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<E, std::initializer_list<U>&, Args...>)
      : m_unex(il, std::forward<Args>(args)...) {}

  constexpr unexpected& operator=(const unexpected&) = default;
  constexpr unexpected& operator=(unexpected&&) = default;

  constexpr const E& error() const& noexcept { return m_unex; }
  constexpr E& error() & noexcept { return m_unex; }
  constexpr const E&& error() const&& noexcept { return std::move(m_unex); }
  constexpr E&& error() && noexcept { return std::move(m_unex); }

  constexpr void swap(unexpected& other) noexcept(std::is_nothrow_swappable_v<E>)
    requires std::is_swappable_v<E>
  {
    using std::swap;
    swap(m_unex, other.m_unex);
  }

  template <class Err>
  friend constexpr bool operator==(const unexpected& x, const unexpected<Err>& y) {
    return x.m_unex == y.error();
  }

  friend constexpr void swap(unexpected& x, unexpected& y) noexcept(noexcept(x.swap(y)))
    requires std::is_swappable_v<E>
  {
    x.swap(y);
  }

 private:
  E m_unex;
};

template <class E>
unexpected(E) -> unexpected<E>;

namespace detail {
// Strong exception guarantee helper for assignment between alternatives ([expected.object.assign]).
template <class T, class U, class... Args>
constexpr void reinit_expected(T& newval, U& oldval, Args&&... args) {
  if constexpr (std::is_nothrow_constructible_v<T, Args...>) {
    std::destroy_at(std::addressof(oldval));
    std::construct_at(std::addressof(newval), std::forward<Args>(args)...);
  } else if constexpr (std::is_nothrow_move_constructible_v<T>) {
    T tmp(std::forward<Args>(args)...);
    std::destroy_at(std::addressof(oldval));
    std::construct_at(std::addressof(newval), std::move(tmp));
  } else {
    U tmp(std::move(oldval));
    std::destroy_at(std::addressof(oldval));
    try {
      std::construct_at(std::addressof(newval), std::forward<Args>(args)...);
    } catch (...) {
      std::construct_at(std::addressof(oldval), std::move(tmp));
      throw;
    }
  }
}
}  // namespace detail

/** expected<T, E>: holds either a value of type T or an error of type E. */
template <class T, class E>
class expected {
  static_assert(!std::is_reference_v<T> && !std::is_function_v<T> &&
                !std::is_same_v<std::remove_cv_t<T>, std::in_place_t> &&
                !std::is_same_v<std::remove_cv_t<T>, unexpect_t> &&
                !detail::is_unexpected<std::remove_cv_t<T>>);
  static_assert(detail::can_be_unexpected<E>);

  template <class U, class G, class UF, class GF>
  static constexpr bool cons_from_expected =
      std::is_constructible_v<T, UF> && std::is_constructible_v<E, GF> &&
      (std::is_same_v<std::remove_cv_t<T>, bool> ||
       !detail::converts_from_any_cvref<T, U, G>) &&
      !detail::unexpected_converts_from<E, U, G>;

  template <class, class>
  friend class expected;

 public:
  using value_type = T;
  using error_type = E;
  using unexpected_type = unexpected<E>;

  template <class U>
  using rebind = expected<U, error_type>;

  // [expected.object.cons]
  constexpr expected() noexcept(std::is_nothrow_default_constructible_v<T>)
    requires std::is_default_constructible_v<T>
      : m_val(), m_has_val(true) {}

  constexpr expected(const expected&) = default;
  constexpr expected(const expected& x) noexcept(
      std::is_nothrow_copy_constructible_v<T> && std::is_nothrow_copy_constructible_v<E>)
    requires std::is_copy_constructible_v<T> && std::is_copy_constructible_v<E> &&
             (!std::is_trivially_copy_constructible_v<T> ||
              !std::is_trivially_copy_constructible_v<E>)
      : m_has_val(x.m_has_val) {
    if (m_has_val) {
      std::construct_at(std::addressof(m_val), x.m_val);
    } else {
      std::construct_at(std::addressof(m_unex), x.m_unex);
    }
  }

  constexpr expected(expected&&) = default;
  constexpr expected(expected&& x) noexcept(
      std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_constructible_v<E>)
    requires std::is_move_constructible_v<T> && std::is_move_constructible_v<E> &&
             (!std::is_trivially_move_constructible_v<T> ||
              !std::is_trivially_move_constructible_v<E>)
      : m_has_val(x.m_has_val) {
    if (m_has_val) {
      std::construct_at(std::addressof(m_val), std::move(x).m_val);
    } else {
      std::construct_at(std::addressof(m_unex), std::move(x).m_unex);
    }
  }

  template <class U, class G>
    requires cons_from_expected<U, G, const U&, const G&>
  constexpr explicit(!std::is_convertible_v<const U&, T> || !std::is_convertible_v<const G&, E>)
      expected(const expected<U, G>& x) noexcept(
          std::is_nothrow_constructible_v<T, const U&> &&
          std::is_nothrow_constructible_v<E, const G&>)
      : m_has_val(x.m_has_val) {
    if (m_has_val) {
      std::construct_at(std::addressof(m_val), x.m_val);
    } else {
      std::construct_at(std::addressof(m_unex), x.m_unex);
    }
  }

  template <class U, class G>
    requires cons_from_expected<U, G, U, G>
  constexpr explicit(!std::is_convertible_v<U, T> || !std::is_convertible_v<G, E>)
      expected(expected<U, G>&& x) noexcept(std::is_nothrow_constructible_v<T, U> &&
                                            std::is_nothrow_constructible_v<E, G>)
      : m_has_val(x.m_has_val) {
    if (m_has_val) {
      std::construct_at(std::addressof(m_val), std::move(x).m_val);
    } else {
      std::construct_at(std::addressof(m_unex), std::move(x).m_unex);
    }
  }

  template <class U = std::remove_cv_t<T>>
    requires(!std::is_same_v<std::remove_cvref_t<U>, expected> &&
             !std::is_same_v<std::remove_cvref_t<U>, std::in_place_t> &&
             !std::is_same_v<std::remove_cvref_t<U>, unexpect_t> &&
             !detail::is_unexpected<std::remove_cvref_t<U>> &&
             std::is_constructible_v<T, U> &&
             (!std::is_same_v<std::remove_cv_t<T>, bool> ||
              !detail::is_expected<std::remove_cvref_t<U>>))
  constexpr explicit(!std::is_convertible_v<U, T>)
      expected(U&& v) noexcept(std::is_nothrow_constructible_v<T, U>)
      : m_val(std::forward<U>(v)), m_has_val(true) {}

  template <class G>
    requires std::is_constructible_v<E, const G&>
  constexpr explicit(!std::is_convertible_v<const G&, E>)
      expected(const unexpected<G>& e) noexcept(std::is_nothrow_constructible_v<E, const G&>)
      : m_unex(e.error()), m_has_val(false) {}

  template <class G>
    requires std::is_constructible_v<E, G>
  constexpr explicit(!std::is_convertible_v<G, E>)
      expected(unexpected<G>&& e) noexcept(std::is_nothrow_constructible_v<E, G>)
      : m_unex(std::move(e).error()), m_has_val(false) {}

  template <class... Args>
    requires std::is_constructible_v<T, Args...>
  constexpr explicit expected(std::in_place_t, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<T, Args...>)
      : m_val(std::forward<Args>(args)...), m_has_val(true) {}

  template <class U, class... Args>
    requires std::is_constructible_v<T, std::initializer_list<U>&, Args...>
  constexpr explicit expected(std::in_place_t, std::initializer_list<U> il, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>)
      : m_val(il, std::forward<Args>(args)...), m_has_val(true) {}

  template <class... Args>
    requires std::is_constructible_v<E, Args...>
  constexpr explicit expected(unexpect_t, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<E, Args...>)
      : m_unex(std::forward<Args>(args)...), m_has_val(false) {}

  template <class U, class... Args>
    requires std::is_constructible_v<E, std::initializer_list<U>&, Args...>
  constexpr explicit expected(unexpect_t, std::initializer_list<U> il, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<E, std::initializer_list<U>&, Args...>)
      : m_unex(il, std::forward<Args>(args)...), m_has_val(false) {}

  // [expected.object.dtor]
  constexpr ~expected() = default;
  constexpr ~expected()
    requires(!std::is_trivially_destructible_v<T> || !std::is_trivially_destructible_v<E>)
  {
    if (m_has_val) {
      std::destroy_at(std::addressof(m_val));
    } else {
      std::destroy_at(std::addressof(m_unex));
    }
  }

  // [expected.object.assign]
  expected& operator=(const expected&) = delete;

  constexpr expected& operator=(const expected& x) noexcept(
      std::is_nothrow_copy_constructible_v<T> && std::is_nothrow_copy_constructible_v<E> &&
      std::is_nothrow_copy_assignable_v<T> && std::is_nothrow_copy_assignable_v<E>)
    requires std::is_copy_assignable_v<T> && std::is_copy_constructible_v<T> &&
             std::is_copy_assignable_v<E> && std::is_copy_constructible_v<E> &&
             (std::is_nothrow_move_constructible_v<T> ||
              std::is_nothrow_move_constructible_v<E>)
  {
    if (x.m_has_val) {
      this->assign_val(x.m_val);
    } else {
      this->assign_unex(x.m_unex);
    }
    return *this;
  }

  constexpr expected& operator=(expected&& x) noexcept(
      std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_constructible_v<E> &&
      std::is_nothrow_move_assignable_v<T> && std::is_nothrow_move_assignable_v<E>)
    requires std::is_move_assignable_v<T> && std::is_move_constructible_v<T> &&
             std::is_move_assignable_v<E> && std::is_move_constructible_v<E> &&
             (std::is_nothrow_move_constructible_v<T> ||
              std::is_nothrow_move_constructible_v<E>)
  {
    if (x.m_has_val) {
      this->assign_val(std::move(x.m_val));
    } else {
      this->assign_unex(std::move(x.m_unex));
    }
    return *this;
  }

  template <class U = std::remove_cv_t<T>>
    requires(!std::is_same_v<expected, std::remove_cvref_t<U>> &&
             !detail::is_unexpected<std::remove_cvref_t<U>> &&
             std::is_constructible_v<T, U> && std::is_assignable_v<T&, U> &&
             (std::is_nothrow_constructible_v<T, U> ||
              std::is_nothrow_move_constructible_v<T> ||
              std::is_nothrow_move_constructible_v<E>))
  constexpr expected& operator=(U&& v) {
    assign_val(std::forward<U>(v));
    return *this;
  }

  template <class G>
    requires std::is_constructible_v<E, const G&> && std::is_assignable_v<E&, const G&> &&
             (std::is_nothrow_constructible_v<E, const G&> ||
              std::is_nothrow_move_constructible_v<T> ||
              std::is_nothrow_move_constructible_v<E>)
  constexpr expected& operator=(const unexpected<G>& e) {
    assign_unex(e.error());
    return *this;
  }

  template <class G>
    requires std::is_constructible_v<E, G> && std::is_assignable_v<E&, G> &&
             (std::is_nothrow_constructible_v<E, G> ||
              std::is_nothrow_move_constructible_v<T> ||
              std::is_nothrow_move_constructible_v<E>)
  constexpr expected& operator=(unexpected<G>&& e) {
    assign_unex(std::move(e).error());
    return *this;
  }

  template <class... Args>
    requires std::is_nothrow_constructible_v<T, Args...>
  constexpr T& emplace(Args&&... args) noexcept {
    if (m_has_val) {
      std::destroy_at(std::addressof(m_val));
    } else {
      std::destroy_at(std::addressof(m_unex));
      m_has_val = true;
    }
    return *std::construct_at(std::addressof(m_val), std::forward<Args>(args)...);
  }

  template <class U, class... Args>
    requires std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>
  constexpr T& emplace(std::initializer_list<U> il, Args&&... args) noexcept {
    if (m_has_val) {
      std::destroy_at(std::addressof(m_val));
    } else {
      std::destroy_at(std::addressof(m_unex));
      m_has_val = true;
    }
    return *std::construct_at(std::addressof(m_val), il, std::forward<Args>(args)...);
  }

  // [expected.object.swap]
  constexpr void swap(expected& x) noexcept(
      std::is_nothrow_move_constructible_v<T> && std::is_nothrow_swappable_v<T> &&
      std::is_nothrow_move_constructible_v<E> && std::is_nothrow_swappable_v<E>)
    requires std::is_swappable_v<T> && std::is_swappable_v<E> &&
             std::is_move_constructible_v<T> && std::is_move_constructible_v<E> &&
             (std::is_nothrow_move_constructible_v<T> ||
              std::is_nothrow_move_constructible_v<E>)
  {
    if (m_has_val) {
      if (x.m_has_val) {
        using std::swap;
        swap(m_val, x.m_val);
      } else {
        this->swap_val_unex(x);
      }
    } else {
      if (x.m_has_val) {
        x.swap_val_unex(*this);
      } else {
        using std::swap;
        swap(m_unex, x.m_unex);
      }
    }
  }

  friend constexpr void swap(expected& x, expected& y) noexcept(noexcept(x.swap(y)))
    requires requires { x.swap(y); }
  {
    x.swap(y);
  }

  // [expected.object.obs]
  constexpr const T* operator->() const noexcept { return std::addressof(m_val); }
  constexpr T* operator->() noexcept { return std::addressof(m_val); }
  constexpr const T& operator*() const& noexcept { return m_val; }
  constexpr T& operator*() & noexcept { return m_val; }
  constexpr const T&& operator*() const&& noexcept { return std::move(m_val); }
  constexpr T&& operator*() && noexcept { return std::move(m_val); }

  constexpr explicit operator bool() const noexcept { return m_has_val; }
  constexpr bool has_value() const noexcept { return m_has_val; }

  constexpr const T& value() const& {
    if (m_has_val) {
      return m_val;
    }
    throw bad_expected_access<E>(m_unex);
  }
  constexpr T& value() & {
    if (m_has_val) {
      return m_val;
    }
    const auto& unex = m_unex;
    throw bad_expected_access<E>(unex);
  }
  constexpr const T&& value() const&& {
    if (m_has_val) {
      return std::move(m_val);
    }
    throw bad_expected_access<E>(std::move(m_unex));
  }
  constexpr T&& value() && {
    if (m_has_val) {
      return std::move(m_val);
    }
    throw bad_expected_access<E>(std::move(m_unex));
  }

  constexpr const E& error() const& noexcept { return m_unex; }
  constexpr E& error() & noexcept { return m_unex; }
  constexpr const E&& error() const&& noexcept { return std::move(m_unex); }
  constexpr E&& error() && noexcept { return std::move(m_unex); }

  template <class U = std::remove_cv_t<T>>
  constexpr T value_or(U&& v) const& {
    static_assert(std::is_copy_constructible_v<T> && std::is_convertible_v<U, T>);
    if (m_has_val) {
      return m_val;
    }
    return static_cast<T>(std::forward<U>(v));
  }

  template <class U = std::remove_cv_t<T>>
  constexpr T value_or(U&& v) && {
    static_assert(std::is_move_constructible_v<T> && std::is_convertible_v<U, T>);
    if (m_has_val) {
      return std::move(m_val);
    }
    return static_cast<T>(std::forward<U>(v));
  }

  template <class G = E>
  constexpr E error_or(G&& e) const& {
    static_assert(std::is_copy_constructible_v<E> && std::is_convertible_v<G, E>);
    if (m_has_val) {
      return std::forward<G>(e);
    }
    return m_unex;
  }

  template <class G = E>
  constexpr E error_or(G&& e) && {
    static_assert(std::is_move_constructible_v<E> && std::is_convertible_v<G, E>);
    if (m_has_val) {
      return std::forward<G>(e);
    }
    return std::move(m_unex);
  }

  // [expected.object.monadic]
  template <class Fn>
    requires std::is_constructible_v<E, E&>
  constexpr auto and_then(Fn&& f) & {
    return and_then_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<E, const E&>
  constexpr auto and_then(Fn&& f) const& {
    return and_then_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<E, E&&>
  constexpr auto and_then(Fn&& f) && {
    return and_then_impl(std::move(*this), std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<E, const E&&>
  constexpr auto and_then(Fn&& f) const&& {
    return and_then_impl(std::move(*this), std::forward<Fn>(f));
  }

  template <class Fn>
    requires std::is_constructible_v<T, T&>
  constexpr auto or_else(Fn&& f) & {
    return or_else_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<T, const T&>
  constexpr auto or_else(Fn&& f) const& {
    return or_else_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<T, T&&>
  constexpr auto or_else(Fn&& f) && {
    return or_else_impl(std::move(*this), std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<T, const T&&>
  constexpr auto or_else(Fn&& f) const&& {
    return or_else_impl(std::move(*this), std::forward<Fn>(f));
  }

  template <class Fn>
    requires std::is_constructible_v<E, E&>
  constexpr auto transform(Fn&& f) & {
    return transform_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<E, const E&>
  constexpr auto transform(Fn&& f) const& {
    return transform_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<E, E&&>
  constexpr auto transform(Fn&& f) && {
    return transform_impl(std::move(*this), std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<E, const E&&>
  constexpr auto transform(Fn&& f) const&& {
    return transform_impl(std::move(*this), std::forward<Fn>(f));
  }

  template <class Fn>
    requires std::is_constructible_v<T, T&>
  constexpr auto transform_error(Fn&& f) & {
    return transform_error_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<T, const T&>
  constexpr auto transform_error(Fn&& f) const& {
    return transform_error_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<T, T&&>
  constexpr auto transform_error(Fn&& f) && {
    return transform_error_impl(std::move(*this), std::forward<Fn>(f));
  }
  template <class Fn>
    requires std::is_constructible_v<T, const T&&>
  constexpr auto transform_error(Fn&& f) const&& {
    return transform_error_impl(std::move(*this), std::forward<Fn>(f));
  }

  // [expected.object.eq]
  template <class T2, class E2>
    requires(!std::is_void_v<T2>)
  friend constexpr bool operator==(const expected& x, const expected<T2, E2>& y) {
    if (x.has_value() != y.has_value()) {
      return false;
    }
    return x.has_value() ? bool(*x == *y) : bool(x.error() == y.error());
  }

  template <class T2>
    requires(!detail::is_expected<T2>)
  friend constexpr bool operator==(const expected& x, const T2& v) {
    return x.has_value() && bool(*x == v);
  }

  template <class E2>
  friend constexpr bool operator==(const expected& x, const unexpected<E2>& e) {
    return !x.has_value() && bool(x.error() == e.error());
  }

 private:
  template <class Self, class Fn>
  static constexpr auto and_then_impl(Self&& self, Fn&& f) {
    using U = std::remove_cvref_t<std::invoke_result_t<Fn, decltype(std::forward<Self>(self).m_val)>>;
    static_assert(detail::is_expected<U>, "the result of f must be a specialization of expected");
    static_assert(std::is_same_v<typename U::error_type, E>,
                  "the result of f must have the same error_type");
    if (self.has_value()) {
      return std::invoke(std::forward<Fn>(f), std::forward<Self>(self).m_val);
    }
    return U(unexpect, std::forward<Self>(self).m_unex);
  }

  template <class Self, class Fn>
  static constexpr auto or_else_impl(Self&& self, Fn&& f) {
    using G = std::remove_cvref_t<std::invoke_result_t<Fn, decltype(std::forward<Self>(self).m_unex)>>;
    static_assert(detail::is_expected<G>, "the result of f must be a specialization of expected");
    static_assert(std::is_same_v<typename G::value_type, T>,
                  "the result of f must have the same value_type");
    if (self.has_value()) {
      return G(std::in_place, std::forward<Self>(self).m_val);
    }
    return std::invoke(std::forward<Fn>(f), std::forward<Self>(self).m_unex);
  }

  template <class Self, class Fn>
  static constexpr auto transform_impl(Self&& self, Fn&& f) {
    using U = std::remove_cv_t<std::invoke_result_t<Fn, decltype(std::forward<Self>(self).m_val)>>;
    using Res = expected<U, E>;
    if (self.has_value()) {
      if constexpr (std::is_void_v<U>) {
        std::invoke(std::forward<Fn>(f), std::forward<Self>(self).m_val);
        return Res();
      } else {
        return Res(detail::in_place_from_invoke_t{}, std::forward<Fn>(f),
                   std::forward<Self>(self).m_val);
      }
    }
    return Res(unexpect, std::forward<Self>(self).m_unex);
  }

  template <class Self, class Fn>
  static constexpr auto transform_error_impl(Self&& self, Fn&& f) {
    using G = std::remove_cv_t<std::invoke_result_t<Fn, decltype(std::forward<Self>(self).m_unex)>>;
    using Res = expected<T, G>;
    if (self.has_value()) {
      return Res(std::in_place, std::forward<Self>(self).m_val);
    }
    return Res(detail::unexpect_from_invoke_t{}, std::forward<Fn>(f),
               std::forward<Self>(self).m_unex);
  }

  template <class Fn, class V>
  constexpr expected(detail::in_place_from_invoke_t, Fn&& f, V&& v)
      : m_val(std::invoke(std::forward<Fn>(f), std::forward<V>(v))), m_has_val(true) {}

  template <class Fn, class V>
  constexpr expected(detail::unexpect_from_invoke_t, Fn&& f, V&& v)
      : m_unex(std::invoke(std::forward<Fn>(f), std::forward<V>(v))), m_has_val(false) {}

  template <class V>
  constexpr void assign_val(V&& v) {
    if (m_has_val) {
      m_val = std::forward<V>(v);
    } else {
      detail::reinit_expected(m_val, m_unex, std::forward<V>(v));
      m_has_val = true;
    }
  }

  template <class V>
  constexpr void assign_unex(V&& v) {
    if (m_has_val) {
      detail::reinit_expected(m_unex, m_val, std::forward<V>(v));
      m_has_val = false;
    } else {
      m_unex = std::forward<V>(v);
    }
  }

  // *this holds a value, rhs holds an error
  constexpr void swap_val_unex(expected& rhs) {
    if constexpr (std::is_nothrow_move_constructible_v<E>) {
      E tmp(std::move(rhs.m_unex));
      std::destroy_at(std::addressof(rhs.m_unex));
      try {
        std::construct_at(std::addressof(rhs.m_val), std::move(m_val));
        std::destroy_at(std::addressof(m_val));
        std::construct_at(std::addressof(m_unex), std::move(tmp));
      } catch (...) {
        std::construct_at(std::addressof(rhs.m_unex), std::move(tmp));
        throw;
      }
    } else {
      T tmp(std::move(m_val));
      std::destroy_at(std::addressof(m_val));
      try {
        std::construct_at(std::addressof(m_unex), std::move(rhs.m_unex));
        std::destroy_at(std::addressof(rhs.m_unex));
        std::construct_at(std::addressof(rhs.m_val), std::move(tmp));
      } catch (...) {
        std::construct_at(std::addressof(m_val), std::move(tmp));
        throw;
      }
    }
    m_has_val = false;
    rhs.m_has_val = true;
  }

  union {
    T m_val;
    E m_unex;
  };
  bool m_has_val;
};

/** expected<void, E>: holds either nothing (success) or an error of type E. */
template <class T, class E>
  requires std::is_void_v<T>
class expected<T, E> {
  static_assert(detail::can_be_unexpected<E>);

  template <class, class>
  friend class expected;

  template <class U, class G, class GF>
  static constexpr bool cons_from_expected =
      std::is_void_v<U> && std::is_constructible_v<E, GF> &&
      !detail::unexpected_converts_from<E, U, G>;

 public:
  using value_type = T;
  using error_type = E;
  using unexpected_type = unexpected<E>;

  template <class U>
  using rebind = expected<U, error_type>;

  constexpr expected() noexcept : m_void(), m_has_val(true) {}

  constexpr expected(const expected&) = default;
  constexpr expected(const expected& x) noexcept(std::is_nothrow_copy_constructible_v<E>)
    requires std::is_copy_constructible_v<E> && (!std::is_trivially_copy_constructible_v<E>)
      : m_void(), m_has_val(x.m_has_val) {
    if (!m_has_val) {
      std::construct_at(std::addressof(m_unex), x.m_unex);
    }
  }

  constexpr expected(expected&&) = default;
  constexpr expected(expected&& x) noexcept(std::is_nothrow_move_constructible_v<E>)
    requires std::is_move_constructible_v<E> && (!std::is_trivially_move_constructible_v<E>)
      : m_void(), m_has_val(x.m_has_val) {
    if (!m_has_val) {
      std::construct_at(std::addressof(m_unex), std::move(x).m_unex);
    }
  }

  template <class U, class G>
    requires cons_from_expected<U, G, const G&>
  constexpr explicit(!std::is_convertible_v<const G&, E>)
      expected(const expected<U, G>& x) noexcept(std::is_nothrow_constructible_v<E, const G&>)
      : m_void(), m_has_val(x.m_has_val) {
    if (!m_has_val) {
      std::construct_at(std::addressof(m_unex), x.m_unex);
    }
  }

  template <class U, class G>
    requires cons_from_expected<U, G, G>
  constexpr explicit(!std::is_convertible_v<G, E>)
      expected(expected<U, G>&& x) noexcept(std::is_nothrow_constructible_v<E, G>)
      : m_void(), m_has_val(x.m_has_val) {
    if (!m_has_val) {
      std::construct_at(std::addressof(m_unex), std::move(x).m_unex);
    }
  }

  template <class G>
    requires std::is_constructible_v<E, const G&>
  constexpr explicit(!std::is_convertible_v<const G&, E>)
      expected(const unexpected<G>& e) noexcept(std::is_nothrow_constructible_v<E, const G&>)
      : m_unex(e.error()), m_has_val(false) {}

  template <class G>
    requires std::is_constructible_v<E, G>
  constexpr explicit(!std::is_convertible_v<G, E>)
      expected(unexpected<G>&& e) noexcept(std::is_nothrow_constructible_v<E, G>)
      : m_unex(std::move(e).error()), m_has_val(false) {}

  constexpr explicit expected(std::in_place_t) noexcept : expected() {}

  template <class... Args>
    requires std::is_constructible_v<E, Args...>
  constexpr explicit expected(unexpect_t, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<E, Args...>)
      : m_unex(std::forward<Args>(args)...), m_has_val(false) {}

  template <class U, class... Args>
    requires std::is_constructible_v<E, std::initializer_list<U>&, Args...>
  constexpr explicit expected(unexpect_t, std::initializer_list<U> il, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<E, std::initializer_list<U>&, Args...>)
      : m_unex(il, std::forward<Args>(args)...), m_has_val(false) {}

  constexpr ~expected() = default;
  constexpr ~expected()
    requires(!std::is_trivially_destructible_v<E>)
  {
    if (!m_has_val) {
      std::destroy_at(std::addressof(m_unex));
    }
  }

  expected& operator=(const expected&) = delete;

  constexpr expected& operator=(const expected& x) noexcept(
      std::is_nothrow_copy_constructible_v<E> && std::is_nothrow_copy_assignable_v<E>)
    requires std::is_copy_constructible_v<E> && std::is_copy_assignable_v<E>
  {
    if (x.m_has_val) {
      emplace();
    } else {
      assign_unex(x.m_unex);
    }
    return *this;
  }

  constexpr expected& operator=(expected&& x) noexcept(
      std::is_nothrow_move_constructible_v<E> && std::is_nothrow_move_assignable_v<E>)
    requires std::is_move_constructible_v<E> && std::is_move_assignable_v<E>
  {
    if (x.m_has_val) {
      emplace();
    } else {
      assign_unex(std::move(x.m_unex));
    }
    return *this;
  }

  template <class G>
    requires std::is_constructible_v<E, const G&> && std::is_assignable_v<E&, const G&>
  constexpr expected& operator=(const unexpected<G>& e) {
    assign_unex(e.error());
    return *this;
  }

  template <class G>
    requires std::is_constructible_v<E, G> && std::is_assignable_v<E&, G>
  constexpr expected& operator=(unexpected<G>&& e) {
    assign_unex(std::move(e.error()));
    return *this;
  }

  constexpr void emplace() noexcept {
    if (!m_has_val) {
      std::destroy_at(std::addressof(m_unex));
      m_has_val = true;
    }
  }

  constexpr void swap(expected& x) noexcept(std::is_nothrow_swappable_v<E> &&
                                            std::is_nothrow_move_constructible_v<E>)
    requires std::is_swappable_v<E> && std::is_move_constructible_v<E>
  {
    if (m_has_val) {
      if (!x.m_has_val) {
        std::construct_at(std::addressof(m_unex), std::move(x.m_unex));
        std::destroy_at(std::addressof(x.m_unex));
        m_has_val = false;
        x.m_has_val = true;
      }
    } else {
      if (x.m_has_val) {
        x.swap(*this);
      } else {
        using std::swap;
        swap(m_unex, x.m_unex);
      }
    }
  }

  friend constexpr void swap(expected& x, expected& y) noexcept(noexcept(x.swap(y)))
    requires requires { x.swap(y); }
  {
    x.swap(y);
  }

  constexpr explicit operator bool() const noexcept { return m_has_val; }
  constexpr bool has_value() const noexcept { return m_has_val; }
  constexpr void operator*() const noexcept {}

  constexpr void value() const& {
    if (m_has_val) {
      return;
    }
    throw bad_expected_access<E>(m_unex);
  }
  constexpr void value() && {
    if (m_has_val) {
      return;
    }
    throw bad_expected_access<E>(std::move(m_unex));
  }

  constexpr const E& error() const& noexcept { return m_unex; }
  constexpr E& error() & noexcept { return m_unex; }
  constexpr const E&& error() const&& noexcept { return std::move(m_unex); }
  constexpr E&& error() && noexcept { return std::move(m_unex); }

  template <class G = E>
  constexpr E error_or(G&& e) const& {
    if (m_has_val) {
      return std::forward<G>(e);
    }
    return m_unex;
  }
  template <class G = E>
  constexpr E error_or(G&& e) && {
    if (m_has_val) {
      return std::forward<G>(e);
    }
    return std::move(m_unex);
  }

  template <class Fn>
  constexpr auto and_then(Fn&& f) & {
    return and_then_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto and_then(Fn&& f) const& {
    return and_then_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto and_then(Fn&& f) && {
    return and_then_impl(std::move(*this), std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto and_then(Fn&& f) const&& {
    return and_then_impl(std::move(*this), std::forward<Fn>(f));
  }

  template <class Fn>
  constexpr auto or_else(Fn&& f) & {
    return or_else_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto or_else(Fn&& f) const& {
    return or_else_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto or_else(Fn&& f) && {
    return or_else_impl(std::move(*this), std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto or_else(Fn&& f) const&& {
    return or_else_impl(std::move(*this), std::forward<Fn>(f));
  }

  template <class Fn>
  constexpr auto transform(Fn&& f) & {
    return transform_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto transform(Fn&& f) const& {
    return transform_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto transform(Fn&& f) && {
    return transform_impl(std::move(*this), std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto transform(Fn&& f) const&& {
    return transform_impl(std::move(*this), std::forward<Fn>(f));
  }

  template <class Fn>
  constexpr auto transform_error(Fn&& f) & {
    return transform_error_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto transform_error(Fn&& f) const& {
    return transform_error_impl(*this, std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto transform_error(Fn&& f) && {
    return transform_error_impl(std::move(*this), std::forward<Fn>(f));
  }
  template <class Fn>
  constexpr auto transform_error(Fn&& f) const&& {
    return transform_error_impl(std::move(*this), std::forward<Fn>(f));
  }

  template <class T2, class E2>
    requires std::is_void_v<T2>
  friend constexpr bool operator==(const expected& x, const expected<T2, E2>& y) {
    if (x.has_value() != y.has_value()) {
      return false;
    }
    return x.has_value() || bool(x.error() == y.error());
  }

  template <class E2>
  friend constexpr bool operator==(const expected& x, const unexpected<E2>& e) {
    return !x.has_value() && bool(x.error() == e.error());
  }

 private:
  template <class Self, class Fn>
  static constexpr auto and_then_impl(Self&& self, Fn&& f) {
    using U = std::remove_cvref_t<std::invoke_result_t<Fn>>;
    static_assert(detail::is_expected<U>, "the result of f must be a specialization of expected");
    static_assert(std::is_same_v<typename U::error_type, E>,
                  "the result of f must have the same error_type");
    if (self.has_value()) {
      return std::invoke(std::forward<Fn>(f));
    }
    return U(unexpect, std::forward<Self>(self).m_unex);
  }

  template <class Self, class Fn>
  static constexpr auto or_else_impl(Self&& self, Fn&& f) {
    using G = std::remove_cvref_t<std::invoke_result_t<Fn, decltype(std::forward<Self>(self).m_unex)>>;
    static_assert(detail::is_expected<G>, "the result of f must be a specialization of expected");
    static_assert(std::is_same_v<typename G::value_type, T>,
                  "the result of f must have the same value_type");
    if (self.has_value()) {
      return G();
    }
    return std::invoke(std::forward<Fn>(f), std::forward<Self>(self).m_unex);
  }

  template <class Self, class Fn>
  static constexpr auto transform_impl(Self&& self, Fn&& f) {
    using U = std::remove_cv_t<std::invoke_result_t<Fn>>;
    using Res = expected<U, E>;
    if (self.has_value()) {
      if constexpr (std::is_void_v<U>) {
        std::invoke(std::forward<Fn>(f));
        return Res();
      } else {
        return Res(std::in_place, std::invoke(std::forward<Fn>(f)));
      }
    }
    return Res(unexpect, std::forward<Self>(self).m_unex);
  }

  template <class Self, class Fn>
  static constexpr auto transform_error_impl(Self&& self, Fn&& f) {
    using G = std::remove_cv_t<std::invoke_result_t<Fn, decltype(std::forward<Self>(self).m_unex)>>;
    using Res = expected<T, G>;
    if (self.has_value()) {
      return Res();
    }
    return Res(unexpect, std::invoke(std::forward<Fn>(f), std::forward<Self>(self).m_unex));
  }

  template <class V>
  constexpr void assign_unex(V&& v) {
    if (m_has_val) {
      std::construct_at(std::addressof(m_unex), std::forward<V>(v));
      m_has_val = false;
    } else {
      m_unex = std::forward<V>(v);
    }
  }

  struct Void {};
  union {
    Void m_void;
    E m_unex;
  };
  bool m_has_val;
};

}  // namespace prosdk::cxx
