#include <cstring>
#include <iomanip>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "filter.hpp"
#include "image_message.hpp"
#include "perf_stats.hpp"
#include "ros2_shm_vision_demo/msg/image.hpp"
#include "stop_watch.hpp"

namespace demo {
class OpticalFlowNode : public rclcpp::Node {
private:
  using ImageMsg = ros2_shm_vision_demo::msg::Image;

public:
  explicit OpticalFlowNode(const rclcpp::NodeOptions &options)
      : Node("shm_demo_vision_edge_detector", options) {

    // only work with the latest message and drop the others
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    m_publisher = this->create_publisher<ImageMsg>("optical_flow_stream", qos);

    auto callback = [this](const ImageMsg::SharedPtr msg) -> void {
      process_message(msg);
    };

    m_subscription =
        create_subscription<ImageMsg>("input_stream", qos, callback);
  }

private:
  rclcpp::Subscription<ImageMsg>::SharedPtr m_subscription;
  rclcpp::Publisher<ImageMsg>::SharedPtr m_publisher;

  PerfStats m_stats;

  Filter m_filter;
  cv::Mat m_prevFrameGray;
  cv::Mat m_result;

  void process_message(const ImageMsg::SharedPtr &msg) {
    m_stats.new_frame(msg->count, msg->timestamp);

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
    m_stats.print();
    cv::waitKey(1);
  }

  void algorithm(cv::Mat &frame) {
    cv::Mat scaled, gray, flow, angle, magnitude;
    m_filter.scale(frame, 0.5, scaled);
    m_filter.to_gray(scaled, gray);

    if (m_prevFrameGray.empty()) {
      m_prevFrameGray = gray;
    }

    cv::calcOpticalFlowFarneback(m_prevFrameGray, gray, flow, 0.4, 1, 12, 2, 8,
                                 1.2, 0);
    visualize_flow_hsv(flow, angle, magnitude);
    cv::vconcat(scaled, magnitude, magnitude);
    visualize_flow_field(flow, scaled, 3);
    cv::vconcat(scaled, angle, angle);
    cv::hconcat(magnitude, angle, m_result);

    // cv::imshow("Optical Flow", m_result);

    m_prevFrameGray = gray;
  }

  void visualize_flow_hsv(const cv::Mat &flow, cv::Mat &angleBgr,
                          cv::Mat &magnitudeBgr) {
    cv::Mat flowChannels[2];
    cv::split(flow, flowChannels);
    cv::Mat angle, magnitude;
    cv::cartToPolar(flowChannels[0], flowChannels[1], magnitude, angle, true);
    cv::normalize(magnitude, magnitude, 0.0f, 1.0f, cv::NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));

    cv::Mat hsvChannels[3], hsv, hsv8, bgr;
    hsvChannels[0] = angle;
    hsvChannels[1] = cv::Mat::ones(angle.size(), CV_32F);
    hsvChannels[2] = magnitude;
    cv::merge(hsvChannels, 3, hsv);

    hsv.convertTo(angle, CV_8U, 255.0);
    magnitude.convertTo(magnitude, CV_8U, 255.0);
    cv::cvtColor(angle, angleBgr, cv::COLOR_HSV2BGR);
    cv::cvtColor(magnitude, magnitudeBgr, cv::COLOR_GRAY2BGR);
  }

  void visualize_flow_field(const cv::Mat &flow, cv::Mat &original,
                            float amplification = 1, int gridSpacing = 10) {

    for (int y = gridSpacing; y < original.rows; y += gridSpacing) {
      for (int x = gridSpacing; x < original.cols; x += gridSpacing) {
        const cv::Point2f flowxy = flow.at<cv::Point2f>(y, x) * amplification;
        // draw line at flow direction
        cv::line(original, cv::Point(x, y),
                 cv::Point(cvRound(x + flowxy.x), cvRound(y + flowxy.y)),
                 cv::Scalar(255, 0, 0));
        // draw initial point
        cv::circle(original, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1);
      }
    }
  }
};

} // namespace demo

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<demo::OpticalFlowNode>(options));
  rclcpp::shutdown();

  return 0;
}