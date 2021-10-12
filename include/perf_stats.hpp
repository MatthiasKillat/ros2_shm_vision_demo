#pragma once

#include <iomanip>
#include <iostream>

#include "fps_estimator.hpp"

namespace demo {

class PerfStats {
private:
  double NANO_TO_MS = 1. / 1000000;

public:
  PerfStats(uint64_t skipFirst = 1)
      : m_skipFirst(skipFirst), m_skipCount(skipFirst) {}
  void new_frame(uint64_t frameNum, uint64_t timestamp) {

    if (m_skipCount > 0) {
      --m_skipCount;
      return;
    }

    // note: clocks are not synchronized so this is a rough approximation
    m_latency = NANO_TO_MS * (m_fpsEstimator.timestamp() - timestamp);

    if (m_count == 0) {
      m_fpsEstimator.start();
    } else {
      if (frameNum != m_frameNum + 1) {
        if (frameNum > m_frameNum) {
          m_lost += frameNum - m_frameNum - 1;
        } else {
          reset();
        }
      }
    }

    m_frameNum = frameNum;
    m_fpsEstimator.new_frame();

    m_latencyAvg = m_count * m_latencyAvg + m_latency;
    ++m_count;
    m_latencyAvg /= m_count;
  }

  void reset() {
    m_count = 0;
    m_lost = 0;
    m_frameNum = 0;
    m_latency = 0;
    m_latencyAvg = 0;
    m_skipCount = m_skipFirst;
  }

  uint64_t count() { return m_count; }

  uint64_t timestamp() { return m_fpsEstimator.timestamp(); }

  void print(const std::string &prefix = std::string()) {
    if (m_skipCount > 0) {
      return;
    }
    std::cout << std::fixed << std::setprecision(2);

    auto fps = m_fpsEstimator.fps();
    auto avgFps = m_fpsEstimator.avgFps();
    double loss = (100. * m_lost) / (m_count + m_lost);

    std::cout << prefix.c_str() << "frame " << std::setw(5) << m_frameNum
              << " lost " << std::setw(5) << m_lost << " (" << std::setw(5)
              << loss << "%) fps " << std::setw(5) << fps << " ("
              << std::setw(5) << avgFps << ") latency " << std::setw(5)
              << m_latency << "ms (" << std::setw(5) << m_latencyAvg << "ms)"
              << std::endl; //       \r" << std::flush;
  }

  FpsEstimator m_fpsEstimator;
  uint64_t m_count{0};
  uint64_t m_lost{0};
  uint64_t m_frameNum{0};
  double m_latency{0};
  double m_latencyAvg{0};
  uint64_t m_skipFirst{1};
  uint64_t m_skipCount{1};
  // bool m_skip{true};
};

} // namespace demo