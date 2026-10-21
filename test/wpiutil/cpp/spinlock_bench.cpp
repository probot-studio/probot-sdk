// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/util/spinlock.hpp"

#include <chrono>
#include <mutex>
#include <thread>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_range_equals.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>

#include "prosdk/util/mutex.hpp"
#include "prosdk/util/print.hpp"

static std::mutex std_mutex;
static std::recursive_mutex std_recursive_mutex;
static prosdk::util::mutex wpi_mutex;
static prosdk::util::recursive_mutex wpi_recursive_mutex;
static prosdk::util::spinlock spinlock;
static prosdk::util::recursive_spinlock1 recursive_spinlock1;
static prosdk::util::recursive_spinlock2 recursive_spinlock2;
static prosdk::util::recursive_spinlock recursive_spinlock;

TEST_CASE("SpinlockTest Benchmark", "[wpiutil]") {
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  // warmup
  std::thread thr([]() {
    [[maybe_unused]]
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 10000000; i++) {
      std::scoped_lock lock(std_mutex);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    (void)start;
    (void)stop;
  });
  thr.join();

  std::thread thrb([]() {
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
      std::scoped_lock lock(std_mutex);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    prosdk::util::print("std::mutex sizeof: {} time: {} value: {}\n",
                     sizeof(std_mutex),
                     duration_cast<microseconds>(stop - start).count(), value);
  });
  thrb.join();

  std::thread thrb2([]() {
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
      std::scoped_lock lock(std_recursive_mutex);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    prosdk::util::print("std::recursive_mutex sizeof: {} time: {} value: {}\n",
                     sizeof(std_recursive_mutex),
                     duration_cast<microseconds>(stop - start).count(), value);
  });
  thrb2.join();

  std::thread thr2([]() {
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
      std::scoped_lock lock(wpi_mutex);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    prosdk::util::print("prosdk::util::mutex sizeof: {} time: {} value: {}\n",
                     sizeof(wpi_mutex),
                     duration_cast<microseconds>(stop - start).count(), value);
  });
  thr2.join();

  std::thread thr2b([]() {
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
      std::scoped_lock lock(wpi_recursive_mutex);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    prosdk::util::print(
        "prosdk::util::recursive_mutex sizeof: {} time: {} value: {}\n",
        sizeof(wpi_recursive_mutex),
        duration_cast<microseconds>(stop - start).count(), value);
  });
  thr2b.join();

  std::thread thr3([]() {
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
      std::scoped_lock lock(spinlock);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    prosdk::util::print("spinlock sizeof: {} time: {} value: {}\n",
                     sizeof(spinlock),
                     duration_cast<microseconds>(stop - start).count(), value);
  });
  thr3.join();

  std::thread thr4([]() {
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
      std::scoped_lock lock(recursive_spinlock1);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    prosdk::util::print("recursive_spinlock1 sizeof: {} time: {} value: {}\n",
                     sizeof(recursive_spinlock1),
                     duration_cast<microseconds>(stop - start).count(), value);
  });
  thr4.join();

  std::thread thr4b([]() {
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
      std::scoped_lock lock(recursive_spinlock2);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    prosdk::util::print("recursive_spinlock2 sizeof: {} time: {} value: {}\n",
                     sizeof(recursive_spinlock2),
                     duration_cast<microseconds>(stop - start).count(), value);
  });
  thr4b.join();

  std::thread thr4c([]() {
    int value = 0;

    auto start = high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
      std::scoped_lock lock(recursive_spinlock);
      ++value;
    }
    auto stop = high_resolution_clock::now();
    prosdk::util::print("recursive_spinlock sizeof: {} time: {} value: {}\n",
                     sizeof(recursive_spinlock),
                     duration_cast<microseconds>(stop - start).count(), value);
  });
  thr4c.join();
}
