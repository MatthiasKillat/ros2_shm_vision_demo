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
#ifndef SHM_VISION_DEMO__SALIENCY_HPP_
#define SHM_VISION_DEMO__SALIENCY_HPP_
#include <opencv2/opencv.hpp>

#include <array>


namespace demo
{

class SaliencyFilter
{
public:
  std::array<uint32_t, 256> histogram(const cv::Mat & gray)
  {
    std::array<uint32_t, 256> h{};
    for (int i = 0; i < gray.rows; ++i) {
      for (int j = 0; j < gray.cols; ++j) {
        auto & val = gray.at<uchar>(i, j);
        ++h[val];
      }
    }

    return h;
  }

  std::array<uchar, 256> saliency_map(const cv::Mat & gray)
  {
    auto h = histogram(gray);
    uint64_t max = 0;
    std::array<uint64_t, 256> dissimilarity{};
    for (int i = 0; i < 256; ++i) {
      for (int j = 0; j < 256; ++j) {
        dissimilarity[i] += std::abs(i - j) * h[j];
      }
      if (max < dissimilarity[i]) {
        max = dissimilarity[i];
      }
    }

    max /= 255;
    std::array<uchar, 256> saliencyMap{};
    for (int i = 0; i < 256; ++i) {
      saliencyMap[i] = (uchar)(dissimilarity[i] / max);
    }

    return saliencyMap;
  }

  void saliency(const cv::Mat & gray, cv::Mat & out)
  {
    auto sal = saliency_map(gray);
    out = cv::Mat(gray.rows, gray.cols, gray.type());
    for (int i = 0; i < gray.rows; ++i) {
      for (int j = 0; j < gray.cols; ++j) {
        auto & val = gray.at<uchar>(i, j);
        out.at<uchar>(i, j) = sal[val];
      }
    }
  }

private:
};
}  // namespace demo
#endif  // SHM_VISION_DEMO__SALIENCY_HPP_
