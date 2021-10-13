#include <cstring>
#include <iomanip>
#include <memory>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "exchange_sync.hpp"
#include "fps_estimator.hpp"
#include "image_message.hpp"
#include "perf_stats.hpp"
#include "ros2_shm_vision_demo/msg/image.hpp"

namespace demo {
class Listener : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit Listener(const rclcpp::NodeOptions &options)
      : Node("shm_demo_vision_listener", options) {

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
  }

  ~Listener() {
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

  struct ReceivedMsg {
    ReceivedMsg(const ImageMsg::SharedPtr &msg, uint64_t time)
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

  void receive_message(const ImageMsg::SharedPtr &msg,
                       ExchangeBuffer<ReceivedMsg> &buffer) {
    auto p = new ReceivedMsg(msg, m_stats.timestamp());
    p = buffer.write(p);
    delete p;
  }

  void thread_main() {
    while (m_keepRunning) {
      auto p = m_inputBuffer.take();
      if (p) {
        auto &msg = p->msg;
        m_stats.new_frame(msg->count, msg->timestamp, p->receive_time);
        process_input_message(msg);
        delete p;
      }

      p = m_filterBuffer.take();
      if (p) {
        auto &msg = p->msg;
        process_filter_message(msg);
        delete p;
      }

      p = m_edgesBuffer.take();
      if (p) {
        auto &msg = p->msg;
        process_edges_message(msg);
        delete p;
      }

      p = m_flowBuffer.take();
      if (p) {
        auto &msg = p->msg;
        process_optical_flow_message(msg);
        delete p;
      }

      p = m_objectsBuffer.take();
      if (p) {
        auto &msg = p->msg;
        process_objects_message(msg);
        delete p;
      }
      cv::waitKey(1);
    }
  }

  void process_input_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    m_stats.print("display ");
    cv::imshow("Display: input ", frame);
  }

  void process_filter_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    cv::imshow("Display: filter", frame);
    // cv::waitKey(1);
  }

  void process_edges_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    cv::imshow("Display: edges", frame);
    // cv::waitKey(1);
  }

  void process_optical_flow_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    cv::imshow("Display: optical flow", frame);
    // cv::waitKey(1);
  }

  void process_objects_message(const ImageMsg::SharedPtr &msg) {
    cv::Mat frame;
    from_message(msg, frame);
    cv::imshow("Display: objects", frame);
    // cv::waitKey(1);
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