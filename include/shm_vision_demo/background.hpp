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
#ifndef SHM_VISION_DEMO__BACKGROUND_HPP_
#define SHM_VISION_DEMO__BACKGROUND_HPP_

#include <opencv2/opencv.hpp>

namespace demo
{

class BackgroundEstimator
{
public:
  void reset() {m_count = 0;}

  void process_frame(const cv::Mat & gray)
  {
    if (m_count == 0) {
      m_avg = gray.clone();
      background(gray, m_bg);
    } else {
      m_avg = NEW_FRAME_WEIGHT * gray + (1 - NEW_FRAME_WEIGHT) * m_avg;
      if (BG_WEIGHT >= EPS) {
        cv::Mat bg;
        background(gray, bg);
        m_bg = BG_WEIGHT * bg + (1 - BG_WEIGHT) * m_bg;
      } else {
        // ignore the past values
        background(gray, m_bg);
      }
    }
    ++m_count;
  }

  const cv::Mat & avg() {return m_avg;}
  const cv::Mat & background_avg() {return m_bg;}

  void background(const cv::Mat & gray, cv::Mat & bg)
  {
    bg = cv::Mat(gray.rows, gray.cols, gray.type());
    for (int i = 0; i < gray.rows; ++i) {
      for (int j = 0; j < gray.cols; ++j) {
        int32_t val = gray.at<uchar>(i, j);
        int32_t avg = m_avg.at<uchar>(i, j);
        bg.at<uchar>(i, j) = std::abs(val - avg);
      }
    }
    // leads to discontinuities across frames
    // cv::normalize(bg, bg, 0, 255, cv::NORM_MINMAX);
  }

  void background_mask(const cv::Mat & gray, uint32_t threshold, cv::Mat & bg)
  {
    bg = cv::Mat(gray.rows, gray.cols, gray.type());
    for (int i = 0; i < gray.rows; ++i) {
      for (int j = 0; j < gray.cols; ++j) {
        int32_t val = gray.at<uchar>(i, j);
        int32_t avg = m_avg.at<uchar>(i, j);
        bg.at<uchar>(i, j) = (std::abs(val - avg) > threshold) ? 0 : 255;
      }
    }
  }

private:
  static constexpr double NEW_FRAME_WEIGHT = 1. / 30;
  static constexpr double EPS = 0.0001;
  static constexpr double BG_WEIGHT = 0.;
  cv::Mat m_avg;
  cv::Mat m_bg;
  uint64_t m_count{0};
};

}  // namespace demo
#endif  // SHM_VISION_DEMO__BACKGROUND_HPP_
