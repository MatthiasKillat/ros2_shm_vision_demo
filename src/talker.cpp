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
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "shm_vision_demo/fps_estimator.hpp"
#include "shm_vision_demo/image_message.hpp"
#include "shm_vision_demo/msg_types.hpp"


using namespace std::chrono_literals;


namespace demo
{

class Talker : public rclcpp::Node
{
public:
  explicit Talker(const rclcpp::NodeOptions & options)
  : Node("shm_vision_demo_talker", options),
    m_thread(&Talker::loop, this, std::ref(m_keep_running)),
    m_fps(60),
    m_video_path("default_video.mp4")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    m_publisher = this->create_publisher<ImageMsg>("input_stream", qos);
  }

  Talker(
    const rclcpp::NodeOptions & options,
    const std::string & input_path,
    const uint32_t & input_fps)
  : Node("shm_vision_demo_talker", options),
    m_thread(&Talker::loop, this, std::ref(m_keep_running)),
    m_fps(input_fps),
    m_video_path(input_path)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    m_publisher = this->create_publisher<ImageMsg>("input_stream", qos);
  }

  void stop() {m_keep_running = false;}

private:
  FpsEstimator m_fpsEstimator;
  uint64_t m_count{0};
  rclcpp::Publisher<ImageMsg>::SharedPtr m_publisher;

  std::atomic<bool> m_keep_running{true};
  std::thread m_thread;
  uint32_t m_fps;
  std::string m_video_path;

private:
  void loop(std::atomic<bool> & keep_running)
  {
    cv::VideoCapture cap;

    std::chrono::milliseconds time_per_frame(1000 / m_fps);
    time_point_t next_time;

    if (m_video_path.empty()) {
      if (!cap.open(0)) {
        return;
      }
    } else {
      if (!cap.open(m_video_path.c_str())) {
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

      m_fpsEstimator.new_frame();

      auto fps = m_fpsEstimator.fps();
      auto avgfps = m_fpsEstimator.avgFps();
      ++m_count;

      std::cout << "frame " << m_count << " fps " << fps << " ( " << avgfps <<
        ") height " << frame.rows << " width " << frame.cols <<
        " channels " << frame.channels() << " bytes " <<
        frame.total() * frame.elemSize() << std::endl;
      //<< " cvtype "
      // << frame.type() << " elemSize " << frame.elemSize()
      // << " total bytes " << frame.total() * frame.elemSize()
      // << "\r"
      // << std::flush;

      // cv::imshow("Talker", frame);

      auto loanedMsg = m_publisher->borrow_loaned_message();
      fill_loaned_message(
        loanedMsg, frame, m_fpsEstimator.timestamp(),
        m_count);

      m_publisher->publish(std::move(loanedMsg));

      if (cv::waitKey(10) == 27) {
        break;
      }
      std::this_thread::sleep_until(next_time);
    }
  }
};

}  // namespace demo

int main(int argc, char * argv[])
{
  std::string input_path;
  uint32_t input_fps{60};
  if (argc > 1) {
    input_path = std::string(argv[1]);
    std::cout << "VIDEO: " << input_path << std::endl;
  }
  if (argc > 2) {
    input_fps = std::atoi(argv[2]);
  }
  std::cout << "INPUT FPS: " << input_fps << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  rclcpp::spin(
    std::make_shared<demo::Talker>(
      options, input_path, input_fps));
  rclcpp::shutdown();

  return 0;
}
