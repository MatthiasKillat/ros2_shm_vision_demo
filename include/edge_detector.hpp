#pragma once

#include <array>

#include <opencv2/opencv.hpp>

#include "filter.hpp"

namespace demo {

class EdgeDetector {
public:
  void sobel(const cv::Mat &in, cv::Mat &out) {
    cv::Sobel(in, out, CV_8U, 1, 1, 5);
  }

  void laplace(const cv::Mat &in, cv::Mat &out) {
    cv::Laplacian(in, out, in.type(), 3, 1, 0, cv::BORDER_DEFAULT);
  }

  void canny(const cv::Mat &in, cv::Mat &out, int t1 = 100, int t2 = 150) {
    cv::Canny(in, out, t1, t2, 3);
  }

private:
};

} // namespace demo