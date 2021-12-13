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
#ifndef SHM_VISION_DEMO__FILTER_HPP_
#define SHM_VISION_DEMO__FILTER_HPP_
#include <opencv2/opencv.hpp>

namespace demo
{

class Filter
{
public:
  void blur(const cv::Mat & in, uint32_t k, cv::Mat & out)
  {
    k = 2 * k + 1;
    cv::GaussianBlur(in, out, cv::Size(k, k), 0);
  }

  void to_gray(const cv::Mat & in, cv::Mat & out)
  {
    cv::cvtColor(in, out, cv::COLOR_BGR2GRAY);
  }

  void project_to_blue(cv::Mat & bgr)
  {
    for (int i = 0; i < bgr.rows; ++i) {
      for (int j = 0; j < bgr.cols; ++j) {
        auto & val = bgr.at<cv::Vec3b>(i, j);
        val[1] = 0;
        val[2] = 0;
      }
    }
  }

  void project_to_blue(const cv::Mat & bgr, cv::Mat & out)
  {
    out = cv::Mat(bgr.rows, bgr.cols, bgr.type());
    for (int i = 0; i < bgr.rows; ++i) {
      for (int j = 0; j < bgr.cols; ++j) {
        auto val = bgr.at<cv::Vec3b>(i, j);
        val[1] = 0;
        val[2] = 0;
        out.at<cv::Vec3b>(i, j) = val;
      }
    }
  }

  void extract_channel(const cv::Mat & bgr, uint8_t channel, cv::Mat & out)
  {
    out = cv::Mat(bgr.rows, bgr.cols, CV_8UC1);
    for (int i = 0; i < bgr.rows; ++i) {
      for (int j = 0; j < bgr.cols; ++j) {
        auto val = bgr.at<cv::Vec3b>(i, j);
        out.at<uchar>(i, j) = val[channel];
      }
    }
  }

  void downscale(const cv::Mat & in, int scaleFactor, cv::Mat & out)
  {
    int rows = in.rows / scaleFactor;
    int cols = in.cols / scaleFactor;
    cv::resize(in, out, cv::Size(cols, rows), cv::INTER_LINEAR);
  }

  void upscale(const cv::Mat & in, int scaleFactor, cv::Mat & out)
  {
    int rows = in.rows * scaleFactor;
    int cols = in.cols * scaleFactor;
    cv::resize(in, out, cv::Size(cols, rows), cv::INTER_LINEAR);
  }

  void scale(const cv::Mat & in, double scaleFactor, cv::Mat & out)
  {
    int rows = static_cast<int>(in.rows * scaleFactor);
    int cols = static_cast<int>(in.cols * scaleFactor);
    cv::resize(in, out, cv::Size(cols, rows), cv::INTER_LINEAR);
  }

  void blend(
    const cv::Mat & bgr1, const cv::Mat & bgr2, double blendFactor,
    cv::Mat & out)
  {
    double b = blendFactor;
    out = cv::Mat(bgr1.rows, bgr1.cols, bgr1.type());
    for (int i = 0; i < bgr1.rows; ++i) {
      for (int j = 0; j < bgr1.cols; ++j) {
        auto & val1 = bgr1.at<cv::Vec3b>(i, j);
        auto & val2 = bgr2.at<cv::Vec3b>(i, j);
        out.at<cv::Vec3b>(i, j) = b * val1 + (1 - b) * val2;
      }
    }
  }

  void blend(
    const cv::Mat & bgr1, const cv::Mat & bgr2,
    const cv::Mat & blendFactor, cv::Mat & out)
  {
    out = cv::Mat(bgr1.rows, bgr1.cols, bgr1.type());
    for (int i = 0; i < bgr1.rows; ++i) {
      for (int j = 0; j < bgr1.cols; ++j) {
        auto & val1 = bgr1.at<cv::Vec3b>(i, j);
        auto & val2 = bgr2.at<cv::Vec3b>(i, j);
        double b = blendFactor.at<uchar>(i, j) / 255.;
        out.at<cv::Vec3b>(i, j) = b * val1 + (1 - b) * val2;
      }
    }
  }

private:
  cv::Mat m_gray;
};
}  // namespace demo
#endif  // SHM_VISION_DEMO__FILTER_HPP_
