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

#include <cstring>
#include <iomanip>
#include <memory>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "shm_vision_demo/exchange_sync.hpp"
#include "shm_vision_demo/filter.hpp"
#include "shm_vision_demo/fps_estimator.hpp"
#include "shm_vision_demo/image_message.hpp"
#include "shm_vision_demo/msg_types.hpp"
#include "shm_vision_demo/object_detector.hpp"
#include "shm_vision_demo/perf_stats.hpp"

namespace demo
{

class ObjectDetectorNode : public rclcpp::Node
{
public:
  explicit ObjectDetectorNode(const rclcpp::NodeOptions & options)
  : Node("shm_demo_vision_object_detector", options)
  {
    // only work with the latest message and drop the others
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    m_publisher = this->create_publisher<ImageMsg>("objects_stream", qos);

    auto callback = [this](const ImageMsg::SharedPtr msg) -> void {
        receive_message(msg);
        // process_message(msg);
      };

    m_computationThread = std::thread(&ObjectDetectorNode::thread_main, this);

    m_subscription =
      create_subscription<ImageMsg>("input_stream", qos, callback);
    std::cout << "object detector ready" << std::endl;
  }

  ~ObjectDetectorNode()
  {
    m_keepRunning = false;
    if (m_computationThread.joinable()) {
      m_computationThread.join();
    }
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr m_subscription;
  rclcpp::Publisher<ImageMsg>::SharedPtr m_publisher;

  std::atomic_bool m_keepRunning{true};
  std::thread m_computationThread;

  struct ReceivedMsg
  {
    ReceivedMsg(const ImageMsg::SharedPtr & msg, uint64_t time)
    : msg(msg), receive_time(time) {}

    const ImageMsg::SharedPtr msg;
    uint64_t receive_time;
  };

  ExchangeBuffer<ReceivedMsg> m_buffer;

  PerfStats m_stats;

  Filter m_filter;
  ObjectDetector m_objectDetector{
    "./install/shm_vision_demo/share/shm_vision_demo/config/", 320, 320};
  cv::Mat m_result;

  void receive_message(const ImageMsg::SharedPtr & msg)
  {
    auto p = new ReceivedMsg(msg, m_stats.timestamp());
    p = m_buffer.write(p);
    delete p;
  }

  void thread_main()
  {
    while (m_keepRunning) {
      auto p = m_buffer.take();
      if (p) {
        auto & msg = p->msg;
        m_stats.new_frame(msg->count, msg->timestamp, p->receive_time);
        process_message(msg);
        delete p;
      }
    }
  }

  void process_message(const ImageMsg::SharedPtr & msg)
  {
    cv::Mat frame;
    from_message(msg, frame);

    if (frame.size().empty()) {
      return;
    }

    algorithm(frame);

    display();

    auto loanedMsg = m_publisher->borrow_loaned_message();
    fill_loaned_message(
      loanedMsg, m_result, m_stats.timestamp(),
      m_stats.count());
    m_publisher->publish(std::move(loanedMsg));
  }

  void display()
  {
    m_stats.print("object detector ");
    cv::waitKey(1);
  }

  void algorithm(cv::Mat & frame)
  {
    cv::Mat scaled;
    m_filter.downscale(frame, 2, scaled);
    m_objectDetector.process_frame(scaled);
    m_result = m_objectDetector.get_result();
    // cv::imshow("Object Detector", m_result);
  }
};

}  // namespace demo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::ObjectDetectorNode>(options));
  rclcpp::shutdown();

  return 0;
}
