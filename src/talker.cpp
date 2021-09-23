// Copyright 2021 Apex.AI, Inc.
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

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "ros2_shm_vision_demo/msg/image.hpp"
#include "ros2_shm_vision_demo/msg/shm_topic.hpp"

using namespace std::chrono_literals;

namespace demo {

const std::string video_path("./video/sintel.webm");

class Talker : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit Talker(const rclcpp::NodeOptions &options)
      : Node("shm_vision_demo_talker", options),
        m_thread(&Talker::loop, this, 30, std::ref(m_keep_running)) {
    rclcpp::QoS qos(rclcpp::KeepLast(3));
    m_publisher = this->create_publisher<ImageMsg>("input_video", qos);
  }

  void stop() { m_keep_running = false; }

private:
  uint64_t m_count = 1;
  rclcpp::Publisher<ImageMsg>::SharedPtr m_publisher;

  void fill_loaned_message(rclcpp::LoanedMessage<ImageMsg> &loanedMsg,
                           const cv::Mat &frame) {

    ImageMsg &msg = loanedMsg.get();
    auto size = frame.elemSize() * frame.total();
    if (size > ImageMsg::MAX_SIZE) {
      throw std::runtime_error("MAX_SIZE exceeded\n");
    }

    msg.rows = frame.rows;
    msg.cols = frame.cols;
    msg.size = size;
    msg.channels = frame.channels();
    msg.type = frame.type();
    msg.offset = 0; // TODO

    // TODO: avoid if possible
    std::memcpy(msg.data.data(), frame.data, size);
  }

  void from_loaned_message(const rclcpp::LoanedMessage<ImageMsg> &loanedMsg,
                           cv::Mat &frame) {
    ImageMsg &msg = loanedMsg.get();
    auto buffer = (uint8_t *)msg.data.data(); // + msg.offset;
    frame = cv::Mat(msg.rows, msg.cols, msg.type, buffer);
  }

  using clock_t = std::chrono::high_resolution_clock;
  using time_point_t = std::chrono::time_point<clock_t>;

  std::atomic<bool> m_keep_running{true};
  std::thread m_thread;

  void loop(int fps, std::atomic<bool> &keep_running) {

    cv::VideoCapture cap;

    std::chrono::milliseconds time_per_frame(1000 / fps);
    time_point_t next_time;
    if (!cap.open(video_path.c_str())) {
      return;
    }

    cv::Mat frame;
    while (keep_running) {
      next_time = std::chrono::high_resolution_clock::now() + time_per_frame;
      cap >> frame;

      if (frame.empty()) {
        break;
      }

      std::cout << "frame " << ++m_count << " : rows " << frame.rows << " cols "
                << frame.cols << " channels " << frame.channels() << " cvtype "
                << frame.type() << " elemSize " << frame.elemSize() << " bytes "
                << frame.total() * frame.elemSize() << std::endl;

      // cv::imshow("input", frame);

      auto loanedMsg = m_publisher->borrow_loaned_message();
      fill_loaned_message(loanedMsg, frame);

      cv::Mat restoredFrame;
      from_loaned_message(loanedMsg, restoredFrame);
      cv::imshow("restored", restoredFrame);

      m_publisher->publish(std::move(loanedMsg));

      if (cv::waitKey(10) == 27) {
        break;
      }
      std::this_thread::sleep_until(next_time);
    }
  }
};

} // namespace demo

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::Talker>(options));
  rclcpp::shutdown();

  // demo::loop(30);

  return 0;
}