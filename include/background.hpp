#pragma once

#include <opencv2/opencv.hpp>

namespace demo {

class BackgroundEstimator {
public:
  void reset() { m_count = 0; }

  void process_frame(const cv::Mat &gray) {
    if (m_count == 0) {
      m_avg = gray.clone();
      background(gray, m_bg);
    } else {
      m_avg = NEW_FRAME_WEIGHT * gray + (1 - NEW_FRAME_WEIGHT) * m_avg;
      cv::Mat bg;
      background(gray, bg);
      m_bg = BG_WEIGHT * bg + (1 - BG_WEIGHT) * m_bg;
    }
    ++m_count;
  }

  const cv::Mat &avg() { return m_avg; }
  const cv::Mat &background_avg() { return m_bg; }

  void background(const cv::Mat &gray, cv::Mat &bg) {
    bg = cv::Mat(gray.rows, gray.cols, gray.type());
    for (int i = 0; i < gray.rows; ++i) {
      for (int j = 0; j < gray.cols; ++j) {
        int32_t val = gray.at<uchar>(i, j);
        int32_t avg = m_avg.at<uchar>(i, j);
        bg.at<uchar>(i, j) = std::abs(val - avg);
      }
    }
    // cv::normalize(bg, bg, 0, 255, cv::NORM_MINMAX);
  }

  void background_mask(const cv::Mat &gray, uint32_t threshold, cv::Mat &bg) {
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
  static constexpr double BG_WEIGHT = 1. / 3;
  cv::Mat m_avg;
  cv::Mat m_bg;
  uint64_t m_count{0};
};

} // namespace demo