#pragma once

#include <iomanip>
#include <iostream>

#include "stop_watch.hpp"

namespace demo {

class PerfStats {
public:
  PerfStats() : m_lastTimestamp(m_fpsEstimator.timestamp()) {}
  void new_frame(uint64_t frameNum, uint64_t timestamp) {

    // note: clocks are not synchronized so this is a rough approximation
    auto latency = 0.000001 * (m_fpsEstimator.timestamp() - timestamp);
    m_lastTimestamp = timestamp;
    m_latency = 0.5 * m_latency + 0.5 * latency;
    if (m_count == 0) {
      m_fpsEstimator.start();
    } else {
      if (frameNum != m_frameNum + 1) {
        if (frameNum > m_frameNum) {
          m_lost += frameNum - m_frameNum - 1;
        } else {
          m_count = 0;
          m_lost = 0;
          m_fpsEstimator.start();
        }
      }
    }
    m_frameNum = frameNum;
    m_fpsEstimator.new_frame();

    m_latencyAvg = m_count * m_latencyAvg + latency;
    ++m_count;
    m_latencyAvg /= m_count;
  }

  uint64_t count() { return m_count; }

  uint64_t timestamp() { return m_fpsEstimator.timestamp(); }

  void print() {
    std::cout << std::fixed << std::setprecision(2);

    auto fps = m_fpsEstimator.fps();
    auto avgFps = m_fpsEstimator.avgFps();

    double loss = (100. * m_lost) / (m_count + m_lost);
    std::cout << "input frame " << m_frameNum << " lost " << m_lost << " ("
              << loss << "%) fps " << fps << " (" << avgFps << ") latency "
              << m_latency << "ms (" << m_latencyAvg << "ms)"
              << std::endl; //       \r" << std::flush;
  }

  FpsEstimator m_fpsEstimator;
  uint64_t m_count{0};
  uint64_t m_lost{0};
  uint64_t m_frameNum{0};
  uint64_t m_latency{0};
  uint64_t m_latencyAvg{0};
  double m_lastTimestamp{0};
};

} // namespace demo