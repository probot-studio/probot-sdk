#ifndef WPIUTIL_WPI_REVERSEITERATION_H
#define WPIUTIL_WPI_REVERSEITERATION_H

#include "prosdk/util/PointerLikeTypeTraits.hpp"

namespace prosdk::util {

template <class T = void *> constexpr bool shouldReverseIterate() {
#if LLVM_ENABLE_REVERSE_ITERATION
  return detail::IsPointerLike<T>::value;
#else
  return false;
#endif
}

} // namespace prosdk::util
#endif
