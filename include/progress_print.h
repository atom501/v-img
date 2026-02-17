#pragma once

#include <fmt/chrono.h>

#include <atomic>
#include <chrono>
#include <condition_variable>

// struct from "Stopping long-sleep threads" stack overflow
struct timer_killer {
  // std::unique_lock<std::mutex> should release lock when going out of scope automatically
  // returns false if killed:
  template <class R, class P> bool wait_for(std::chrono::duration<R, P> const& time) const {
    std::unique_lock<std::mutex> lock(m);
    return !cv.wait_for(lock, time, [&] { return terminate; });
  }
  void kill() {
    std::unique_lock<std::mutex> lock(m);
    terminate = true;
    cv.notify_all();
  }
  // explicitly delete/default special member functions
  timer_killer() = default;
  timer_killer(timer_killer&&) = delete;
  timer_killer(timer_killer const&) = delete;
  timer_killer& operator=(timer_killer&&) = delete;
  timer_killer& operator=(timer_killer const&) = delete;

private:
  mutable std::condition_variable cv;
  mutable std::mutex m;
  bool terminate = false;
};

inline void print_time_taken(const std::chrono::steady_clock::time_point& start_time,
                             const std::chrono::steady_clock::time_point& end_time,
                             const std::string& activity) {
  auto duration_micro
      = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration_micro);
  duration_micro %= std::chrono::milliseconds(1);

  auto duration_sec = std::chrono::duration_cast<std::chrono::seconds>(duration_ms);
  duration_ms %= std::chrono::seconds(1);

  auto duration_min = std::chrono::duration_cast<std::chrono::minutes>(duration_sec);
  duration_sec %= std::chrono::minutes(1);

  if (duration_min.count() > 0) {
    fmt::println("Time take by {}: {} {} {} {}", activity, duration_min, duration_sec, duration_ms,
                 duration_micro);
  } else if (duration_sec.count() > 0) {
    fmt::println("Time take by {}: {} {} {}", activity, duration_sec, duration_ms, duration_micro);
  } else if (duration_ms.count() > 0) {
    fmt::println("Time take by {}: {} {}", activity, duration_ms, duration_micro);
  } else {
    fmt::println("Time take by {}: {}", activity, duration_micro);
  }
}