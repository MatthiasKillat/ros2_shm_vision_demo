#pragma once

#include <opencv2/opencv.hpp>

namespace demo {

class Filter {
public:
  void blur(const cv::Mat &in, uint32_t k, cv::Mat &out) {
    k = 2 * k + 1;
    cv::GaussianBlur(in, out, cv::Size(k, k), 0);
  }

  void to_gray(const cv::Mat &in, cv::Mat &out) {
    cv::cvtColor(in, out, cv::COLOR_BGR2GRAY);
  }

private:
};
} // namespace demo