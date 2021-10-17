#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>

#include <signal.h>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "background.hpp"
#include "exchange_sync.hpp"
#include "filter.hpp"
#include "fps_estimator.hpp"
#include "image_message.hpp"
#include "perf_stats.hpp"
#include "ros2_shm_vision_demo/msg/image.hpp"
#include "saliency.hpp"

namespace demo {
class FilterNode : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit FilterNode(const rclcpp::NodeOptions &options)
      : Node("shm_demo_vision_filter", options) {

    // only work with the latest message and drop the others
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    m_publisher = this->create_publisher<ImageMsg>("filtered_stream", qos);

    auto callback = [this](const ImageMsg::SharedPtr msg) -> void {
      receive_message(msg);
      // process_message(msg);
    };

    m_computationThread = std::thread(&FilterNode::thread_main, this);

    m_subscription =
        create_subscription<ImageMsg>("input_stream", qos, callback);
  }

  ~FilterNode() {
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
  BackgroundEstimator m_bgEstimator;
  SaliencyFilter m_saliency;
  cv::Mat m_result;

  void receive_message(const ImageMsg::SharedPtr &msg) {
    auto p = new ReceivedMsg(msg, m_stats.timestamp());
    p = m_buffer.write(p);
    delete p;
  }

  void thread_main() {
    // TODO: wait on semaphore
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

    if (frame.size().empty()) {
      return;
    }

    algorithm(frame);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    display(frame);

    auto loanedMsg = m_publisher->borrow_loaned_message();
    fill_loaned_message(loanedMsg, m_result, m_stats.timestamp(),
                        m_stats.count());
    m_publisher->publish(std::move(loanedMsg));
  }

  void display(const cv::Mat &) {
    m_stats.print("filter ");
    cv::waitKey(1);
  }

  void algorithm(cv::Mat &frame) {
    cv::Mat scaled, gray, bg, blurred, blended, saliency, blendFactor;

    m_filter.downscale(frame, 2, scaled);
    m_filter.to_gray(scaled, gray);

    // m_filter.blur(gray, 5, gray);
    // m_filter.blur(scaled, 50, blurred);

    m_bgEstimator.process_frame(gray);
    m_saliency.saliency(gray, saliency);
    // cv::dilate(saliency, saliency, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

    auto avg = m_bgEstimator.avg().clone();
    bg = m_bgEstimator.background_avg().clone();

    // cv::normalize(saliency, blendFactor, 0, 255, cv::NORM_MINMAX);
    // cv::threshold(blendFactor, blendFactor, 127, 255, cv::THRESH_BINARY);
    // m_filter.blend(scaled, blurred, blendFactor, blended);

    cv::cvtColor(avg, avg, cv::COLOR_GRAY2BGR);
    cv::cvtColor(gray, gray, cv::COLOR_GRAY2BGR);
    cv::cvtColor(bg, bg, cv::COLOR_GRAY2BGR);
    cv::cvtColor(saliency, saliency, cv::COLOR_GRAY2BGR);

    cv::hconcat(scaled, saliency, saliency);
    cv::hconcat(avg, bg, bg);
    cv::vconcat(saliency, bg, m_result);

    // cv::imshow("Filter", m_result);
  }
};

} // namespace demo

std::shared_ptr<demo::FilterNode> node;

// void sig_handler(int) {
//   node->stop_processing();
//   rclcpp::shutdown();
// }

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  node = std::make_shared<demo::FilterNode>(options);
  // signal(SIGINT, sig_handler);
  // signal(SIGTERM, sig_handler);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}