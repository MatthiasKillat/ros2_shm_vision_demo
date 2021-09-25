#pragma once

#include <array>

#include <opencv2/opencv.hpp>

namespace demo {

class SaliencyFilter {
public:
  std::array<uint32_t, 256> histogram(const cv::Mat &gray) {

    std::array<uint32_t, 256> h{};
    for (int i = 0; i < gray.rows; ++i) {
      for (int j = 0; j < gray.cols; ++j) {
        auto &val = gray.at<uchar>(i, j);
        ++h[val];
      }
    }

    return h;
  }

  std::array<uchar, 256> saliency_map(const cv::Mat &gray) {

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

  void saliency(const cv::Mat &gray, cv::Mat &out) {
    auto sal = saliency_map(gray);
    out = cv::Mat(gray.rows, gray.cols, gray.type());
    for (int i = 0; i < gray.rows; ++i) {
      for (int j = 0; j < gray.cols; ++j) {
        auto &val = gray.at<uchar>(i, j);
        out.at<uchar>(i, j) = sal[val];
      }
    }
  }

private:
};

} // namespace demo