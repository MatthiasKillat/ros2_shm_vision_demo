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

protected:
  time_point_t m_start;
};

class FpsEstimator : public StopWatch {
public:
  FpsEstimator() {}

  void new_frame(uint32_t frames = 1) {
    m_frames += frames;
    auto t = now();
    auto duration = t - m_start;
    if (duration >= std::chrono::milliseconds(1000)) {
      m_start = t;
      m_fps = DECAY * m_fps + (1 - DECAY) * m_frames;
      m_frames = 0;
    }
  }

  double fps() { return m_fps; }

private:
  static constexpr double DECAY = 0.5;
  uint32_t m_frames{0};
  double m_fps{0};
};
} // namespace demo