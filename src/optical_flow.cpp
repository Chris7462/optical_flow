// C++ header
#include <chrono>

// ROS header
#include <cv_bridge/cv_bridge.h>

// OpenCV header
#include <opencv2/video.hpp>

// local header
#include "optical_flow/optical_flow.hpp"


namespace optical_flow
{
using namespace std::chrono_literals;

OpticalFlow::OpticalFlow()
: Node("optical_flow_node"), prev_img_(), next_img_(), prev_time_(),
  next_time_(), prev_pt_(), next_pt_()
{
  rclcpp::QoS qos(10);
  img_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //"kitti/camera/color/left/image_raw", qos, std::bind(
    "kitti/camera/gray/left/image_raw", qos, std::bind(
      &OpticalFlow::img_left_callback, this, std::placeholders::_1));

//img_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//  "kitti/camera/color/right/image_raw", qos, std::bind(
//    &OrbMatching::img_right_callback, this, std::placeholders::_1));

  img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "optical_flow/img_flow", qos);

  timer_ = this->create_wall_timer(
    25ms, std::bind(&OpticalFlow::timer_callback, this));

  //detector_ = cv::GFTTDetector::create(250, 0.01, 20);
  detector_ = cv::GFTTDetector::create(500, 0.01, 30.0);
}

void OpticalFlow::img_left_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  img_left_buff_.push(msg);
}

//void OrbMatching::img_right_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//{
//  std::lock_guard<std::mutex> lock(mtx_);
//  img_right_buff_.push(msg);
//}

cv::Mat OpticalFlow::get_image_from_msg(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImageConstPtr ptr;
  try {
    //ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exeception: %s", e.what());
  }
  return ptr->image;
}

void OpticalFlow::timer_callback()
{
  if (!img_left_buff_.empty()) {
    mtx_.lock();
    if (prev_img_.empty()) {  // First iteration
      prev_img_ = get_image_from_msg(img_left_buff_.front());
      prev_time_ = img_left_buff_.front()->header.stamp;
      img_left_buff_.pop();
      mtx_.unlock();
    } else {
      if (!next_img_.empty()) {
        prev_img_ = next_img_; // .clone();
        prev_time_ = next_time_;
      }
      next_img_ = get_image_from_msg(img_left_buff_.front());
      next_time_ = img_left_buff_.front()->header.stamp;
      img_left_buff_.pop();
      mtx_.unlock();
    }

    prev_pt_.clear();
    // run keypoint detection and store the keypoints
    std::vector<cv::KeyPoint> prev_keypoints;
    detector_->detect(prev_img_, prev_keypoints);
    for (auto & kp : prev_keypoints) {
      prev_pt_.push_back(kp.pt);
    }

    if (!next_img_.empty()) {
      next_pt_.clear();
      // use opencv's optical flow
      std::vector<uchar> status;
      std::vector<float> error;
      cv::calcOpticalFlowPyrLK(prev_img_, next_img_, prev_pt_, next_pt_, status, error);

      // publish the results
      cv::Mat next_img_cv;
      cv::cvtColor(next_img_, next_img_cv, cv::COLOR_GRAY2BGR);
      for (size_t i = 0; i < next_pt_.size(); ++i) {
        cv::circle(next_img_cv, next_pt_[i], 2, cv::Scalar(0, 250, 0), 2);
        cv::line(next_img_cv, prev_pt_[i], next_pt_[i], cv::Scalar(0, 250, 0));
      }

      cv_bridge::CvImage cv_img;
      cv_img.image = next_img_cv;
      cv_img.encoding = "bgr8";

      auto img_msg = cv_img.toImageMsg();
      img_pub_->publish(*img_msg);
    }
  }

//if (!img_left_buff_.empty()) {
//  mtx_.lock();
//  if (prev_img_.empty()) { // First iteration
//    prev_img_ = get_image_from_msg(img_left_buff_.front());
//    prev_time_ = img_left_buff_.front()->header.stamp;
//    img_left_buff_.pop();
//    mtx_.unlock();

//    prev_pt_.clear();
//    next_pt_.clear();

//    // run keypoint detection and store the keypoints
//    std::vector<cv::KeyPoint> prev_keypoints;
//    detector_->detect(prev_img_, prev_keypoints);
//    for (auto & kp : prev_keypoints) {
//      prev_pt_.push_back(kp.pt);
//    }
//    RCLCPP_INFO(get_logger(), "First iter");
//    RCLCPP_INFO(get_logger(), "PREV SIZE = %ld", prev_pt_.size());
//    RCLCPP_INFO(get_logger(), "NEXT SIZE = %ld", next_pt_.size());
//  } else {
//    if (next_img_.empty()) { // Second iteration
//      // do nothing here
//      RCLCPP_INFO(get_logger(), "Second iter");
//    } else {  // After the second iteration store the next image to previous image and get a new one
//      prev_img_ = next_img_;
//      prev_time_ = next_time_;
//      RCLCPP_INFO(get_logger(), "After iter");
//      prev_pt_.swap(next_pt_);
//      next_pt_.clear();
//    }
//    next_img_ = get_image_from_msg(img_left_buff_.front());
//    next_time_ = img_left_buff_.front()->header.stamp;
//    img_left_buff_.pop();
//    mtx_.unlock();

//    // check if two images have a gap
//    //double time_diff = (next_time_ - prev_time_).seconds();
//    //if (time_diff > 0.15) {
//    //  RCLCPP_INFO_STREAM(get_logger(), "Reset prev & next images. Sync error: " << time_diff);
//    //  prev_img_ = cv::Mat();
//    //  next_img_ = cv::Mat();
//    //  return;
//    //} else {  // No gap. Good to proceed
//      // use opencv's optical flow
//      std::vector<uchar> status;
//      std::vector<float> error;
//      cv::calcOpticalFlowPyrLK(prev_img_, next_img_, prev_pt_, next_pt_, status, error);
//      RCLCPP_INFO(get_logger(), "Size of prev_pt = %ld", prev_pt_.size());
//      RCLCPP_INFO(get_logger(), "Size of next_pt = %ld", next_pt_.size());

//      // publish the results
//      cv::Mat next_img_cv;
//      cv::cvtColor(next_img_, next_img_cv, cv::COLOR_GRAY2BGR);
//      for (size_t i = 0; i < next_pt_.size(); ++i) {
//        cv::circle(next_img_cv, next_pt_[i], 2, cv::Scalar(0, 250, 0), 2);
//        cv::line(next_img_cv, prev_pt_[i], next_pt_[i], cv::Scalar(0, 250, 0));
//      }

//      cv_bridge::CvImage cv_img;
//      cv_img.image = next_img_cv;
//      cv_img.encoding = "bgr8";

//      auto img_msg = cv_img.toImageMsg();
//      img_pub_->publish(*img_msg);
//    //}
//  }
//}
}

} // namespace optical_flow
