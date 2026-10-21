// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Tests for the C++23 shims in src/prosdk/cxx (design/PORT.md section 6).

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/cxx/bit.hpp"
#include "prosdk/cxx/expected.hpp"
#include "prosdk/cxx/functional.hpp"
#include "prosdk/cxx/print.hpp"
#include "prosdk/cxx/utility.hpp"

using prosdk::cxx::expected;
using prosdk::cxx::unexpect;
using prosdk::cxx::unexpected;

namespace {
expected<int, std::string> Parse(bool ok) {
  if (ok) {
    return 21;
  }
  return unexpected(std::string{"bad"});
}

enum class Color : std::uint8_t { kRed = 3 };
}  // namespace

TEST_CASE("expected value and error", "[cxx]") {
  auto v = Parse(true);
  REQUIRE(v.has_value());
  CHECK(*v == 21);
  CHECK(v.value_or(0) == 21);
  auto e = Parse(false);
  REQUIRE_FALSE(e);
  CHECK(e.error() == "bad");
  CHECK(e.value_or(5) == 5);
  CHECK_THROWS_AS(e.value(), prosdk::cxx::bad_expected_access<std::string>);
  CHECK(e == unexpected(std::string{"bad"}));
  CHECK(v == 21);
}

TEST_CASE("expected monadic operations", "[cxx]") {
  auto doubled = Parse(true).transform([](int x) { return x * 2; });
  CHECK(*doubled == 42);
  auto chained = Parse(true).and_then([](int x) -> expected<std::string, std::string> {
    return std::to_string(x);
  });
  CHECK(*chained == "21");
  auto recovered = Parse(false).or_else([](const std::string&) -> expected<int, std::string> {
    return 7;
  });
  CHECK(*recovered == 7);
  auto len = Parse(false).transform_error([](const std::string& s) { return s.size(); });
  CHECK(len.error() == 3u);
}

TEST_CASE("expected assignment and swap", "[cxx]") {
  expected<std::unique_ptr<int>, int> a{std::make_unique<int>(1)};
  expected<std::unique_ptr<int>, int> b{unexpect, 9};
  a.swap(b);
  CHECK(a.error() == 9);
  CHECK(**b == 1);
  a = std::make_unique<int>(4);
  CHECK(**a == 4);
  b = unexpected(2);
  CHECK(b.error() == 2);

  expected<void, int> ok;
  CHECK(ok.has_value());
  expected<void, int> err{unexpect, 5};
  CHECK(err.error() == 5);
  ok = err;
  CHECK(ok.error() == 5);
  ok.emplace();
  CHECK(ok.has_value());
}

TEST_CASE("expected is constexpr", "[cxx]") {
  constexpr expected<int, int> c{5};
  static_assert(*c == 5);
  constexpr expected<int, int> d{unexpect, 1};
  static_assert(d.error() == 1);
  SUCCEED();
}

TEST_CASE("byteswap, to_underlying, invoke_r", "[cxx]") {
  static_assert(prosdk::cxx::byteswap<std::uint16_t>(0x1122) == 0x2211);
  static_assert(prosdk::cxx::byteswap<std::uint32_t>(0x11223344u) == 0x44332211u);
  static_assert(prosdk::cxx::byteswap<std::uint64_t>(0x0102030405060708ull) ==
                0x0807060504030201ull);
  static_assert(prosdk::cxx::byteswap<std::int8_t>(-3) == -3);
  static_assert(prosdk::cxx::to_underlying(Color::kRed) == 3);
  CHECK(prosdk::cxx::invoke_r<long>([](int x) { return x + 1; }, 1) == 2L);
}

TEST_CASE("print writes formatted text", "[cxx]") {
  std::FILE* f = std::tmpfile();
  REQUIRE(f != nullptr);
  prosdk::cxx::print(f, "{}-{}", 1, "a");
  prosdk::cxx::println(f, "{}", 2.5);
  std::rewind(f);
  char buf[32] = {};
  auto n = std::fread(buf, 1, sizeof(buf) - 1, f);
  std::fclose(f);
  CHECK(std::string(buf, n) == "1-a2.5\n");
}
