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
#ifndef SHM_VISION_DEMO__IMAGE_MESSAGE_HPP_
#define SHM_VISION_DEMO__IMAGE_MESSAGE_HPP_
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "shm_vision_demo/msg_types.hpp"

namespace demo
{

void fill_loaned_message(
  rclcpp::LoanedMessage<ImageMsg> & loanedMsg,
  const cv::Mat & frame, uint64_t timestamp,
  uint64_t count)
{
  ImageMsg & msg = loanedMsg.get();
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
  msg.offset = 0;  // TODO(matthiaskillat) alignment?
  msg.count = count;
  msg.timestamp = timestamp;

  // TODO(matthiaskillat): avoid if possible
  std::memcpy(msg.data.data(), frame.data, size);
}

void from_loaned_message(
  const rclcpp::LoanedMessage<ImageMsg> & loanedMsg,
  cv::Mat & frame)
{
  ImageMsg & msg = loanedMsg.get();
  auto buffer = reinterpret_cast<uint8_t *>(msg.data.data());  // + msg.offset;
  frame = cv::Mat(msg.rows, msg.cols, msg.type, buffer);
}

void from_message(const ImageMsg::SharedPtr & msg, cv::Mat & frame)
{
  void * buffer = msg->data.data();
  frame = cv::Mat(msg->rows, msg->cols, msg->type, buffer);
}

}  // namespace demo
#endif  // SHM_VISION_DEMO__IMAGE_MESSAGE_HPP_
