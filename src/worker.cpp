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
#include <thread>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "filter.hpp"
#include "ros2_shm_vision_demo/msg/image.hpp"
#include "stop_watch.hpp"

namespace demo {
class Worker : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit Worker(const rclcpp::NodeOptions &options)
      : Node("shm_demo_vision_worker", options) {

    // only work with the latest message and drop the others
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    m_publisher = this->create_publisher<ImageMsg>("filtered_video", qos);

    auto callback = [this](const ImageMsg::SharedPtr msg) -> void {
      process_message(msg);
    };

    m_subscription =
        create_subscription<ImageMsg>("input_video", qos, callback);
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr m_subscription;
  FpsEstimator m_fpsEstimator;
  uint64_t m_count{0};
  uint64_t m_lost{0};
  uint64_t m_frameNum{0};

  Filter m_filter;
  rclcpp::Publisher<ImageMsg>::SharedPtr m_publisher;
  cv::Mat m_filtered;

  void from_message(const ImageMsg::SharedPtr &msg, cv::Mat &frame) {
    auto buffer = (uint8_t *)msg->data.data();
    frame = cv::Mat(msg->rows, msg->cols, msg->type, buffer);
  }

  void process_message(const ImageMsg::SharedPtr &msg) {
    auto frameNum = msg->count;
    if (m_count == 0) {
      m_fpsEstimator.start();
    } else {
      if (frameNum != m_frameNum + 1) {
        m_lost += (frameNum - m_frameNum - 1);
      }
    }
    m_frameNum = frameNum;
    ++m_count;
    m_fpsEstimator.new_frame();

    cv::Mat frame;
    from_message(msg, frame);

    display(frame);

    cv::Mat scaled;
    cv::Mat blue;
    cv::Mat gray;
    cv::Mat edges;

    m_filter.scale(frame, 0.5, scaled);
    m_filter.to_gray(scaled, gray);
    m_filter.edges(scaled, edges);
    m_filter.project_to_blue(scaled, blue);

    cv::cvtColor(gray, gray, cv::COLOR_GRAY2BGR);
    cv::cvtColor(edges, edges, cv::COLOR_GRAY2BGR);

    cv::hconcat(scaled, blue, blue);
    cv::hconcat(gray, edges, edges);
    cv::vconcat(blue, edges, m_filtered);
    // cv::imshow("Worker", m_filtered);

    auto loanedMsg = m_publisher->borrow_loaned_message();
    fill_loaned_message(loanedMsg, m_filtered);
    m_publisher->publish(std::move(loanedMsg));
  }

  void display(const cv::Mat &) {

    std::cout << std::fixed << std::setprecision(2);

    auto fps = m_fpsEstimator.fps();
    double loss = (100. * m_lost) / (m_count + m_lost);
    std::cout << "frame " << m_frameNum << " lost " << m_lost << " (" << loss
              << "%) fps " << fps << "\r" << std::flush;

    // cv::imshow("Worker", frame);
    cv::waitKey(1);
  }

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
    msg.offset = 0; // TODO alignment
    msg.count = m_count;

    // TODO: avoid if possible
    std::memcpy(msg.data.data(), frame.data, size);
  }
};

} // namespace demo

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::Worker>(options));
  rclcpp::shutdown();

  return 0;
}