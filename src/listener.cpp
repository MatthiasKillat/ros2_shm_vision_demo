#include <cstring>
#include <iomanip>
#include <memory>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "image_message.hpp"
#include "perf_stats.hpp"
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

    // TODO: we could also just transfer the bounding boxes in a message
    auto objectsCallback = [this](const ImageMsg::SharedPtr msg) -> void {
      process_objects_message(msg);
    };
    m_objectsSubscription =
        create_subscription<ImageMsg>("objects_stream", qos, objectsCallback);
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr m_inputSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_filterSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_edgesSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_flowSubscription;
  rclcpp::Subscription<ImageMsg>::SharedPtr m_objectsSubscription;

  PerfStats m_stats;

  void from_message(const ImageMsg::SharedPtr &msg, cv::Mat &frame) {
    auto buffer = (uint8_t *)msg->data.data();
    frame = cv::Mat(msg->rows, msg->cols, msg->type, buffer);
  }

  void process_input_message(const ImageMsg::SharedPtr &msg) {
    m_stats.new_frame(msg->count, msg->timestamp);

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

  void process_objects_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    cv::imshow("Listener: objects", frame);
    cv::waitKey(1);
  }

  void display(const cv::Mat &frame) {

    m_stats.print();

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