#pragma once

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