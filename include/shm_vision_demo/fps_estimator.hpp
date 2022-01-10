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
#ifndef SHM_VISION_DEMO__FPS_ESTIMATOR_HPP_
#define SHM_VISION_DEMO__FPS_ESTIMATOR_HPP_
#include <chrono>

namespace demo
{

using clock_t = std::chrono::high_resolution_clock;
using time_point_t = std::chrono::time_point<clock_t>;

class StopWatch
{
public:
  StopWatch()
  : m_start(now()) {}

  void start() {m_start = now();}

  time_point_t now() {return std::chrono::high_resolution_clock::now();}

  double fps(uint64_t frames) {return (1000. * frames) / elapsed().count();}

  std::chrono::milliseconds elapsed()
  {
    auto duration = now() - m_start;
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  }

  uint64_t timestamp()
  {
    auto d = now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
  }

protected:
  time_point_t m_start;
};

class FpsEstimator : public StopWatch
{
public:
  FpsEstimator() {m_prev = m_start;}

  void new_frame(uint32_t frames = 1)
  {
    m_frames += frames;
    auto t = now();
    auto duration = t - m_prev;
    if (duration >= std::chrono::milliseconds(1000)) {
      m_prev = t;
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration)
        .count();
      m_fps = DECAY * m_fps + (1 - DECAY) * 1000.0 * m_frames / ms;
      m_frames = 0;
      // average the windowed values, otherwise we would need to consider a
      // starting point where we already have received one frame
      m_avgFps = m_count * m_avgFps + m_fps;
      m_avgFps /= ++m_count;
    }
  }

  double fps() {return m_fps;}

  double avgFps() {return m_avgFps;}

private:
  static constexpr double DECAY = 0.5;
  uint32_t m_frames{0};
  double m_fps{0};  // windowed decayed average
  double m_avgFps{0};
  time_point_t m_prev;
  uint64_t m_count;
};
}  // namespace demo
#endif  // SHM_VISION_DEMO__FPS_ESTIMATOR_HPP_
