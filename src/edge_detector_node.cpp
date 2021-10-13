#include <cstring>
#include <iomanip>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "edge_detector.hpp"
#include "exchange_sync.hpp"
#include "filter.hpp"
#include "fps_estimator.hpp"
#include "image_message.hpp"
#include "perf_stats.hpp"
#include "ros2_shm_vision_demo/msg/image.hpp"

namespace demo {
class EdgeDetectorNode : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit EdgeDetectorNode(const rclcpp::NodeOptions &options)
      : Node("shm_demo_vision_edge_detector", options) {

    // only work with the latest message and drop the others
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    m_publisher = this->create_publisher<ImageMsg>("edges_stream", qos);

    auto callback = [this](const ImageMsg::SharedPtr msg) -> void {
      receive_message(msg);
      // process_message(msg);
    };

    m_computationThread = std::thread(&EdgeDetectorNode::thread_main, this);

    m_subscription =
        create_subscription<ImageMsg>("input_stream", qos, callback);
  }

  ~EdgeDetectorNode() {
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
  EdgeDetector m_edgeDetector;
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

    display(frame);

    auto loanedMsg = m_publisher->borrow_loaned_message();
    fill_loaned_message(loanedMsg, m_result, m_stats.timestamp(),
                        m_stats.count());
    m_publisher->publish(std::move(loanedMsg));
  }

  void display(const cv::Mat &) {
    m_stats.print("edge detector ");
    cv::waitKey(1);
  }

  void algorithm(cv::Mat &frame) {
    cv::Mat scaled, gray, sobel, laplace, canny;

    m_filter.scale(frame, 0.5, scaled);
    m_filter.to_gray(scaled, gray);
    m_filter.blur(gray, 5, gray);

    m_edgeDetector.sobel(gray, sobel);
    m_edgeDetector.canny(gray, canny);
    m_edgeDetector.laplace(gray, laplace);
    cv::normalize(laplace, laplace, 0, 255, cv::NORM_MINMAX);

    cv::cvtColor(sobel, sobel, cv::COLOR_GRAY2BGR);
    cv::cvtColor(canny, canny, cv::COLOR_GRAY2BGR);
    cv::cvtColor(laplace, laplace, cv::COLOR_GRAY2BGR);

    cv::hconcat(scaled, laplace, laplace);
    cv::hconcat(canny, sobel, sobel);
    cv::vconcat(laplace, sobel, m_result);

    // cv::imshow("Edge Detector", m_result);
  }
};

} // namespace demo

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::EdgeDetectorNode>(options));
  rclcpp::shutdown();

  return 0;
}