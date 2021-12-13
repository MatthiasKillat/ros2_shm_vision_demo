// Copyright 2021 Matthias Killiat
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef SHM_VISION_DEMO__PERF_STATS_HPP_
#define SHM_VISION_DEMO__PERF_STATS_HPP_
#include <iomanip>
#include <iostream>
#include <string>

#include "shm_vision_demo/fps_estimator.hpp"

namespace demo
{

class PerfStats
{
private:
  double NANO_TO_MS = 1. / 1000000;

public:
  explicit PerfStats(uint64_t skipFirst = 1)
  : m_skipFirst(skipFirst), m_skipCount(skipFirst) {}
  void new_frame(uint64_t frameNum, uint64_t send_time, uint64_t receive_time)
  {
    if (m_skipCount > 0) {
      --m_skipCount;
      return;
    }

    m_latency = NANO_TO_MS * (receive_time - send_time);

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

    // m_latencyAvg = 0.9 * m_latencyAvg + 0.1 * m_latency;
    m_latencyAvg = m_count * m_latencyAvg + m_latency;
    ++m_count;
    m_latencyAvg /= m_count;
  }

  void reset()
  {
    m_count = 0;
    m_lost = 0;
    m_frameNum = 0;
    m_latency = 0;
    m_latencyAvg = 0;
    m_skipCount = m_skipFirst;
  }

  uint64_t count() {return m_count;}

  uint64_t timestamp() {return m_fpsEstimator.timestamp();}

  void print(const std::string & prefix = std::string())
  {
    if (m_skipCount > 0) {
      return;
    }
    std::cout << std::fixed << std::setprecision(2);

    auto fps = m_fpsEstimator.fps();
    auto avgFps = m_fpsEstimator.avgFps();
    double loss = (100. * m_lost) / (m_count + m_lost);

    std::cout << prefix.c_str() << "frame " << std::setw(5) << m_frameNum <<
      " lost " << std::setw(5) << m_lost << " (" << std::setw(5) <<
      loss << "%) fps " << std::setw(5) << fps << " (" <<
      std::setw(5) << avgFps << ") latency " << std::setw(5) <<
      m_latency << "ms (" << std::setw(5) << m_latencyAvg << "ms)" <<
      std::endl;  //       \r" << std::flush;
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

}  // namespace demo
#endif  // SHM_VISION_DEMO__PERF_STATS_HPP_
