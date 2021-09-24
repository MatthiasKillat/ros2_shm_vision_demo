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

  void edges(const cv::Mat &in, cv::Mat &out) {
    to_gray(in, m_gray);
    blur(m_gray, 2, m_gray);
    cv::Sobel(m_gray, out, CV_8U, 1, 1, 5);
  }

  void project_to_blue(cv::Mat &bgr) {
    for (int i = 0; i < bgr.rows; ++i) {
      for (int j = 0; j < bgr.cols; ++j) {
        auto &val = bgr.at<cv::Vec3b>(i, j);
        val[1] = 0;
        val[2] = 0;
      }
    }
  }

  void project_to_blue(const cv::Mat &bgr, cv::Mat &out) {
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

  void scale(const cv::Mat &in, double scaleFactor, cv::Mat &out) {
    int rows = int(in.rows * scaleFactor);
    int cols = int(in.cols * scaleFactor);
    cv::resize(in, out, cv::Size(cols, rows), cv::INTER_LINEAR);
  }

private:
  cv::Mat m_gray;
};
} // namespace demo