#pragma once

// C++ header
#include <queue>
#include <vector>
#include <mutex>

// OpenCV header
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace optical_flow
{

class OpticalFlow : public rclcpp::Node
{
public:
  OpticalFlow();
  ~OpticalFlow() = default;

private:
  void img_left_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  // void img_right_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_left_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_right_sub_;

  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Mat get_image_from_msg(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_left_buff_;
  // std::queue<sensor_msgs::msg::Image::SharedPtr> img_right_buff_;

  std::mutex mtx_;

  cv::Mat prev_img_;
  cv::Mat next_img_;

  rclcpp::Time prev_time_;
  rclcpp::Time next_time_;

  std::vector<cv::Point2f> prev_pt_;
  std::vector<cv::Point2f> next_pt_;

  cv::Ptr<cv::GFTTDetector> detector_;
};

} // namespace optical_flow
