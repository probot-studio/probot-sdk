// Copyright (c) Sleipnir contributors

#include "prosdk/thirdparty/sleipnir/sleipnir/util/pool.hpp"

slp::PoolResource& slp::global_pool_resource() {
#if PROSDK_TARGET_HW
  // prosdk: small chunks on the ESP32 (expressions do not use the pool there,
  // see autodiff/expression.hpp; this only serves direct users of the pool)
  thread_local PoolResource pool{64};
#else
  thread_local PoolResource pool{16384};
#endif
  return pool;
}
