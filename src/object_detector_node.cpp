#include <cstring>
#include <iomanip>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "exchange_sync.hpp"
#include "filter.hpp"
#include "fps_estimator.hpp"
#include "image_message.hpp"
#include "object_detector.hpp"
#include "perf_stats.hpp"
#include "ros2_shm_vision_demo/msg/image.hpp"

namespace demo {
class ObjectDetectorNode : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit ObjectDetectorNode(const rclcpp::NodeOptions &options)
      : Node("shm_demo_vision_object_detector", options) {

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

  ~ObjectDetectorNode() {
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

  struct ReceivedMsg {
    ReceivedMsg(const ImageMsg::SharedPtr &msg, uint64_t time)
        : msg(msg), receive_time(time) {}

    const ImageMsg::SharedPtr msg;
    uint64_t receive_time;
  };

  ExchangeBuffer<ReceivedMsg> m_buffer;

  PerfStats m_stats;

  Filter m_filter;
  ObjectDetector m_objectDetector{"./yolo_config/", 320, 320};
  cv::Mat m_result;

  void receive_message(const ImageMsg::SharedPtr &msg) {
    auto p = new ReceivedMsg(msg, m_stats.timestamp());
    p = m_buffer.write(p);
    delete p;
  }

  void thread_main() {
    while (m_keepRunning) {
      auto p = m_buffer.take();
      if (p) {
        auto &msg = p->msg;
        m_stats.new_frame(msg->count, msg->timestamp, p->receive_time);
        process_message(msg);
        delete p;
      }
    }
  }

  void process_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);

    algorithm(frame);

    display();

    auto loanedMsg = m_publisher->borrow_loaned_message();
    fill_loaned_message(loanedMsg, m_result, m_stats.timestamp(),
                        m_stats.count());
    m_publisher->publish(std::move(loanedMsg));
  }

  void display() {
    m_stats.print("object detector ");
    cv::waitKey(1);
  }

  void algorithm(cv::Mat &frame) {
    cv::Mat scaled;
    m_filter.scale(frame, 0.5, scaled);
    m_objectDetector.process_frame(scaled);
    m_result = m_objectDetector.get_result();
    // cv::imshow("Object Detector", m_result);
  }
};

} // namespace demo

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::ObjectDetectorNode>(options));
  rclcpp::shutdown();

  return 0;
}