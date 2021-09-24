#pragma once

#include <chrono>

namespace demo {

using clock_t = std::chrono::high_resolution_clock;
using time_point_t = std::chrono::time_point<clock_t>;

class StopWatch {
public:
  StopWatch() : m_start(now()) {}

  void start() { m_start = now(); }

  time_point_t now() { return std::chrono::high_resolution_clock::now(); }

  double fps(uint64_t frames) { return (1000. * frames) / elapsed().count(); }

  std::chrono::milliseconds elapsed() {
    auto duration = now() - m_start;
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  }

private:
  time_point_t m_start;
};
} // namespace demo