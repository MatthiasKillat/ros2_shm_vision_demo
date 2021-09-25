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
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "ros2_shm_vision_demo/msg/image.hpp"
#include "stop_watch.hpp"

using namespace std::chrono_literals;

std::string INPUT_PATH;
uint32_t INPUT_FPS = 60;

namespace demo {

class Talker : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit Talker(const rclcpp::NodeOptions &options)
      : Node("shm_vision_demo_talker", options),
        m_thread(&Talker::loop, this, std::ref(m_keep_running)) {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    m_publisher = this->create_publisher<ImageMsg>("input_stream", qos);
  }

  void stop() { m_keep_running = false; }

private:
  FpsEstimator m_fpsEstimator;
  uint64_t m_count{0};
  rclcpp::Publisher<ImageMsg>::SharedPtr m_publisher;

  std::atomic<bool> m_keep_running{true};
  std::thread m_thread;
  uint32_t m_fps{INPUT_FPS};

private:
  void fill_loaned_message(rclcpp::LoanedMessage<ImageMsg> &loanedMsg,
                           const cv::Mat &frame) {

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
    msg.count = m_count;

    // TODO: avoid if possible
    std::memcpy(msg.data.data(), frame.data, size);
  }

  void from_loaned_message(const rclcpp::LoanedMessage<ImageMsg> &loanedMsg,
                           cv::Mat &frame) {
    ImageMsg &msg = loanedMsg.get();
    auto buffer = (uint8_t *)msg.data.data(); // + msg.offset;
    frame = cv::Mat(msg.rows, msg.cols, msg.type, buffer);
  }

  void loop(std::atomic<bool> &keep_running) {

    cv::VideoCapture cap;

    std::chrono::milliseconds time_per_frame(1000 / m_fps);
    time_point_t next_time;

    if (INPUT_PATH.empty()) {
      if (!cap.open(0)) {
        return;
      }
    } else {
      if (!cap.open(INPUT_PATH.c_str())) {
        return;
      }
    }

    cv::Mat frame;
    m_fpsEstimator.start();

    std::cout << std::fixed << std::setprecision(2);

    while (keep_running) {
      next_time = m_fpsEstimator.now() + time_per_frame;
      cap >> frame;

      if (frame.empty()) {
        break;
      }
      // std::cout << m_publisher->get_subscription_count() << std::endl;

      ++m_count;
      m_fpsEstimator.new_frame();

      auto fps = m_fpsEstimator.fps();
      std::cout << "frame " << m_count << " fps " << fps << " : rows "
                << frame.rows << " cols " << frame.cols << " channels "
                << frame.channels() << " cvtype " << frame.type()
                << " elemSize " << frame.elemSize() << " total bytes "
                << frame.total() * frame.elemSize() << "\r" << std::flush;

      // cv::imshow("Talker", frame);

      auto loanedMsg = m_publisher->borrow_loaned_message();
      fill_loaned_message(loanedMsg, frame);

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

  if (argc > 1) {
    INPUT_PATH = std::string(argv[1]);
    std::cout << "VIDEO: " << INPUT_PATH << std::endl;
  }
  if (argc > 2) {
    INPUT_FPS = std::atoi(argv[2]);
  }
  std::cout << "INPUT FPS: " << INPUT_FPS << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::Talker>(options));
  rclcpp::shutdown();

  return 0;
}