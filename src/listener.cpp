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
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "shm_vision_demo/exchange_sync.hpp"
#include "shm_vision_demo/fps_estimator.hpp"
#include "shm_vision_demo/image_message.hpp"
#include "shm_vision_demo/msg_types.hpp"
#include "shm_vision_demo/perf_stats.hpp"


namespace demo
{


class Listener : public rclcpp::Node
{
public:
  explicit Listener(const rclcpp::NodeOptions & options)
  : Node("shm_demo_vision_listener", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));

    m_computationThread = std::thread(&Listener::thread_main, this);

    auto inputCallback = [this](const ImageMsg::SharedPtr msg) -> void {
        receive_message(msg, m_inputBuffer);
        // process_input_message(msg);
      };

    m_inputSubscription =
      create_subscription<ImageMsg>("input_stream", qos, inputCallback);

    auto filterCallback = [this](const ImageMsg::SharedPtr msg) -> void {
        receive_message(msg, m_filterBuffer);
        // process_filter_message(msg);
      };
    m_filterSubscription =
      create_subscription<ImageMsg>("filtered_stream", qos, filterCallback);

    auto edgesCallback = [this](const ImageMsg::SharedPtr msg) -> void {
        receive_message(msg, m_edgesBuffer);
        // process_edges_message(msg);
      };
    m_edgesSubscription =
      create_subscription<ImageMsg>("edges_stream", qos, edgesCallback);

    auto flowCallback = [this](const ImageMsg::SharedPtr msg) -> void {
        receive_message(msg, m_flowBuffer);
        // process_optical_flow_message(msg);
      };
    m_flowSubscription =
      create_subscription<ImageMsg>("optical_flow_stream", qos, flowCallback);

    // note: we could also just transfer the bounding boxes in a message
    auto objectsCallback = [this](const ImageMsg::SharedPtr msg) -> void {
        receive_message(msg, m_objectsBuffer);
        // process_objects_message(msg);
      };
    m_objectsSubscription =
      create_subscription<ImageMsg>("objects_stream", qos, objectsCallback);
    m_windowName = "ROS 2 Vision Demo";
  }

  ~Listener()
  {
    m_keepRunning = false;
    if (m_computationThread.joinable()) {
      m_computationThread.join();
    }
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr m_inputSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_filterSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_edgesSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_flowSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_objectsSubscription;

  std::atomic_bool m_keepRunning{true};
  std::thread m_computationThread;

  cv::Mat m_fusionResult;

  std::string m_windowName;

  struct ReceivedMsg
  {
    ReceivedMsg(const ImageMsg::SharedPtr & msg, uint64_t time)
    : msg(msg), receive_time(time) {}

    const ImageMsg::SharedPtr msg;
    uint64_t receive_time;
  };

  ExchangeBuffer<ReceivedMsg> m_inputBuffer;
  ExchangeBuffer<ReceivedMsg> m_filterBuffer;
  ExchangeBuffer<ReceivedMsg> m_edgesBuffer;
  ExchangeBuffer<ReceivedMsg> m_flowBuffer;
  ExchangeBuffer<ReceivedMsg> m_objectsBuffer;

  PerfStats m_stats;

  void receive_message(
    const ImageMsg::SharedPtr & msg,
    ExchangeBuffer<ReceivedMsg> & buffer)
  {
    auto p = new ReceivedMsg(msg, m_stats.timestamp());
    p = buffer.write(p);
    delete p;
  }

  void thread_main()
  {
    while (m_keepRunning) {
      auto p = m_inputBuffer.take();
      if (p) {
        auto & msg = p->msg;
        m_stats.new_frame(msg->count, msg->timestamp, p->receive_time);
        process_input_message(msg);
        delete p;
      }

      p = m_filterBuffer.take();
      if (p) {
        auto & msg = p->msg;
        process_filter_message(msg);
        delete p;
      }

      p = m_edgesBuffer.take();
      if (p) {
        auto & msg = p->msg;
        process_edges_message(msg);
        delete p;
      }

      p = m_flowBuffer.take();
      if (p) {
        auto & msg = p->msg;
        process_optical_flow_message(msg);
        delete p;
      }

      p = m_objectsBuffer.take();
      if (p) {
        auto & msg = p->msg;
        process_objects_message(msg);
        delete p;
      }

      if (fusionInitialized()) {
        cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
        cv::imshow(m_windowName, m_fusionResult);
      }
      cv::waitKey(1);
    }
  }

  void process_input_message(const ImageMsg::SharedPtr & msg)
  {
    cv::Mat frame;
    from_message(msg, frame);
    if (frame.size().empty()) {
      return;
    }
    m_stats.print("display ");
    cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
    cv::imshow(m_windowName, frame);  // display_input

    // TODO(matthiaskillat): brittle, improve setting the buffer
    // cannot deal with size change etc.
    if (m_fusionResult.rows != frame.rows) {
      m_fusionResult =
        cv::Mat::zeros(cv::Size(frame.cols, frame.rows), CV_8UC3);
    }
  }

  void process_filter_message(const ImageMsg::SharedPtr & msg)
  {
    cv::Mat frame;
    from_message(msg, frame);
    if (frame.size().empty()) {
      return;
    }

    // cv::imshow("Display: filter", frame);

    if (fusionInitialized()) {
      auto r = frame.rows / 2;
      auto c = frame.cols / 2;
      cv::Mat dstRoi(m_fusionResult, cv::Rect(0, r, c, r));
      cv::Mat srcRoi(frame, cv::Rect(c, r, c, r));
      srcRoi.copyTo(dstRoi);
    }
  }

  void process_edges_message(const ImageMsg::SharedPtr & msg)
  {
    cv::Mat frame;
    from_message(msg, frame);
    if (frame.size().empty()) {
      return;
    }
    // cv::imshow("Display: edges", frame);

    if (fusionInitialized()) {
      auto r = frame.rows / 2;
      auto c = frame.cols / 2;
      cv::Mat dstRoi(m_fusionResult, cv::Rect(c, r, c, r));
      cv::Mat srcRoi(frame, cv::Rect(c, r, c, r));
      srcRoi.copyTo(dstRoi);
    }
  }

  void process_optical_flow_message(const ImageMsg::SharedPtr & msg)
  {
    cv::Mat frame;
    from_message(msg, frame);
    if (frame.size().empty()) {
      return;
    }
    cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
    cv::imshow(m_windowName, frame);  // display_optical_flow

    if (fusionInitialized()) {
      auto r = frame.rows / 2;
      auto c = frame.cols / 2;
      cv::Mat dstRoi(m_fusionResult, cv::Rect(c, 0, c, r));
      cv::Mat srcRoi(frame, cv::Rect(c, 0, c, r));
      srcRoi.copyTo(dstRoi);
    }
  }

  void process_objects_message(const ImageMsg::SharedPtr & msg)
  {
    cv::Mat frame;
    from_message(msg, frame);
    if (frame.size().empty()) {
      return;
    }
    // cv::imshow("Display: objects", frame);

    if (fusionInitialized()) {
      cv::Mat dstRoi(m_fusionResult, cv::Rect(0, 0, frame.cols, frame.rows));
      frame.copyTo(dstRoi);
    }
  }

  bool fusionInitialized() {return m_fusionResult.rows > 0;}
};

}  // namespace demo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::Listener>(options));
  rclcpp::shutdown();

  return 0;
}
