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

    rclcpp::QoS qos(rclcpp::KeepLast(1));

    auto inputCallback = [this](const ImageMsg::SharedPtr msg) -> void {
      process_input_message(msg);
    };

    m_lastTimestamp = m_fpsEstimator.timestamp();
    m_inputSubscription =
        create_subscription<ImageMsg>("input_stream", qos, inputCallback);

    auto filterCallback = [this](const ImageMsg::SharedPtr msg) -> void {
      process_filtered_message(msg);
    };
    m_filterSubscription =
        create_subscription<ImageMsg>("filtered_stream", qos, filterCallback);

    auto edgesCallback = [this](const ImageMsg::SharedPtr msg) -> void {
      process_edges_message(msg);
    };
    m_edgesSubscription =
        create_subscription<ImageMsg>("edges_stream", qos, edgesCallback);

    auto flowCallback = [this](const ImageMsg::SharedPtr msg) -> void {
      process_optical_flow_message(msg);
    };
    m_flowSubscription =
        create_subscription<ImageMsg>("optical_flow_stream", qos, flowCallback);
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr m_inputSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_filterSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_edgesSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_flowSubscription;
  FpsEstimator m_fpsEstimator;
  uint64_t m_count{0};
  uint64_t m_frameNum{0};
  uint64_t m_lost{0};
  uint64_t m_latency{0};
  uint64_t m_lastTimestamp{0};

  void from_message(const ImageMsg::SharedPtr &msg, cv::Mat &frame) {
    auto buffer = (uint8_t *)msg->data.data();
    frame = cv::Mat(msg->rows, msg->cols, msg->type, buffer);
  }

  void process_input_message(const ImageMsg::SharedPtr &msg) {
    auto latency = msg->timestamp - m_lastTimestamp;
    m_lastTimestamp = msg->timestamp;
    m_latency = 0.000001 * (0.5 * m_latency + 0.5 * latency);

    auto frameNum = msg->count;
    if (m_count == 0) {
      m_fpsEstimator.start();
    } else {
      if (frameNum != m_frameNum + 1) {
        if (frameNum > m_frameNum) {
          m_lost += frameNum - m_frameNum - 1;
        } else {
          m_count = 0;
          m_lost = 0;
          m_fpsEstimator.start();
        }
      }
    }
    m_frameNum = frameNum;
    ++m_count;
    m_fpsEstimator.new_frame();

    cv::Mat frame;
    from_message(msg, frame);
    display(frame);
  }

  void process_filtered_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    cv::imshow("Listener: filter", frame);
    cv::waitKey(1);
  }

  void process_edges_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    cv::imshow("Listener: edges", frame);
    cv::waitKey(1);
  }

  void process_optical_flow_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    cv::imshow("Listener: optical flow", frame);
    cv::waitKey(1);
  }

  void display(const cv::Mat &frame) {

    std::cout << std::fixed << std::setprecision(2);

    auto fps = m_fpsEstimator.fps();
    double loss = (100. * m_lost) / (m_count + m_lost);
    std::cout << "input frame " << m_frameNum << " lost " << m_lost << " ("
              << loss << "%) fps " << fps << " latency " << m_latency << "ms\r"
              << std::flush;

    cv::imshow("Listener - input ", frame);
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