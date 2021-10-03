#pragma once

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_shm_vision_demo/msg/image.hpp"

namespace demo {

using ImageMsg = ros2_shm_vision_demo::msg::Image;

void fill_loaned_message(rclcpp::LoanedMessage<ImageMsg> &loanedMsg,
                         const cv::Mat &frame, uint64_t timestamp,
                         uint64_t count) {

  ImageMsg &msg = loanedMsg.get();
  auto size = frame.elemSize() * frame.total();
  if (size > ImageMsg::MAX_SIZE) {
    std::stringstream s;
    s << "MAX_SIZE exceeded - message requires " << size << "bytes\n";
    throw std::runtime_error(s.str());
  }

  msg.rows = frame.rows;
  msg.cols = frame.cols;
  msg.size = size;
  msg.channels = frame.channels();
  msg.type = frame.type();
  msg.offset = 0; // TODO alignment?
  msg.count = count;
  msg.timestamp = timestamp;

  // TODO: avoid if possible
  std::memcpy(msg.data.data(), frame.data, size);
}

void from_loaned_message(const rclcpp::LoanedMessage<ImageMsg> &loanedMsg,
                         cv::Mat &frame) {
  ImageMsg &msg = loanedMsg.get();
  auto buffer = (uint8_t *)msg.data.data(); // + msg.offset;
  frame = cv::Mat(msg.rows, msg.cols, msg.type, buffer);
}

void from_message(const ImageMsg::SharedPtr &msg, cv::Mat &frame) {
  auto buffer = (uint8_t *)msg->data.data();
  frame = cv::Mat(msg->rows, msg->cols, msg->type, buffer);
}

} // namespace demo