#include <cstring>
#include <iomanip>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

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
      process_message(msg);
    };

    m_subscription =
        create_subscription<ImageMsg>("input_stream", qos, callback);
    std::cout << "object detector ready" << std::endl;
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr m_subscription;
  rclcpp::Publisher<ImageMsg>::SharedPtr m_publisher;

  PerfStats m_stats;

  Filter m_filter;
  ObjectDetector m_objectDetector{"./yolo_config/", 320, 320};
  cv::Mat m_result;

  void process_message(const ImageMsg::SharedPtr &msg) {
    // executor thread is delayed so this timestamp is also
    // delayed hence the large latency in the beginning?
    m_stats.new_frame(msg->count, msg->timestamp);

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