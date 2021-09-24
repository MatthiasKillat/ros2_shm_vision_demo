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

#include <cstring>
#include <iomanip>
#include <memory>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "ros2_shm_vision_demo/msg/image.hpp"
#include "stop_watch.hpp"

namespace demo {
class Listener : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit Listener(const rclcpp::NodeOptions &options)
      : Node("shm_demo_vision_listener", options) {

    // subscription callback to process arriving data
    auto callback = [this](const ImageMsg::SharedPtr msg) -> void {
      process_message(msg);
    };

    rclcpp::QoS qos(rclcpp::KeepLast(3));
    m_subscription =
        create_subscription<ImageMsg>("input_video", qos, callback);
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr m_subscription;
  StopWatch m_stopWatch;
  uint64_t m_count{0};
  uint64_t m_prev{0};
  uint64_t m_lost{0};

  void from_message(const ImageMsg::SharedPtr &msg, cv::Mat &frame) {
    auto buffer = (uint8_t *)msg->data.data();
    frame = cv::Mat(msg->rows, msg->cols, msg->type, buffer);
  }

  void process_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);

    auto frameNum = msg->count;

    if (m_count == 0) {
      m_stopWatch.start(); // start with first frame
    } else {
      if (frameNum != m_prev + 1) {
        ++m_lost;
      }
    }

    m_prev = frameNum;

    std::cout << std::fixed << std::setprecision(2);

    auto fps = m_stopWatch.fps(++m_count);
    std::cout << "frame " << frameNum << " lost " << m_lost << " fps " << fps
              << " : rows " << frame.rows << " cols " << frame.cols
              << " channels " << frame.channels() << " cvtype " << frame.type()
              << " elemSize " << frame.elemSize() << " total bytes "
              << frame.total() * frame.elemSize() << "\r" << std::flush;

    cv::imshow("Listener", frame);
    cv::waitKey(1);
  }
};

} // namespace demo

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::Listener>(options));
  rclcpp::shutdown();

  return 0;
}