/*Copyright:
	This file copyright (C) 2017 by

	eYs3D an Etron company

	An unpublished work.  All rights reserved.

	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of eYs3D an Etron company.
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dm_preview/GetParams.h>
#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
using namespace configuru;  // NOLINT

namespace enc = sensor_msgs::image_encodings;

class EYS3DListener {
 public:
  EYS3DListener() : it_(nh_), color_count_(0) {
    color_sub_ =  it_.subscribe("eys3d_depth/left/image_color", 1,
        &EYS3DListener::colorCallback, this);
    depth_sub_ =  it_.subscribe("eys3d_depth/depth/image_raw", 1,
        &EYS3DListener::depthCallback, this);

    std::string img_intri = getCameraCalibInfo(0u);
    std::string img_extri = getCameraCalibInfo(1u);
    std::string imu_intri = getCameraCalibInfo(2u);
    std::string imu_extri = getCameraCalibInfo(3u);

    auto img_intri_info = parse_string(img_intri.c_str(), JSON, "log");
    auto img_extri_info = parse_string(img_extri.c_str(), JSON, "log");
    auto imu_intri_info = parse_string(imu_intri.c_str(), JSON, "log");
    auto imu_extri_info = parse_string(imu_extri.c_str(), JSON, "log");
    std::cout << "IMG_INTRINSICS:" << img_intri_info << std::endl
              << "IMG_EXTRINSICS_RTOL:" << img_extri_info << std::endl
              << "IMU_INTRINSICS:" << imu_intri_info << std::endl
              << "IMU_EXTRINSICS:" << imu_extri_info << std::endl;

    cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
  }

  ~EYS3DListener() {
    cv::destroyAllWindows();
  }

  std::string getCameraCalibInfo(unsigned int type_) {
    ros::ServiceClient client =
        nh_.serviceClient<dm_preview::GetParams>("/dm_preview/get_params");
    dm_preview::GetParams srv;
    srv.request.key = type_;
    if (client.call(srv)) {
      return srv.response.value;
    } else {
      ROS_ERROR("Failed to call service GetParams, make sure you have launch eys3d_depth device SDK nodelet");
      return "null";
    }
  }

  void colorCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, enc::RGB8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
      return;
    }
    ++color_count_;
    // ROS_INFO_STREAM("color: " << color_count_);

    cv::imshow("color", cv_ptr->image);
    cv::waitKey(3);
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      if (enc::isColor(msg->encoding)) {
        cv_ptr = cv_bridge::toCvShare(msg, enc::RGB8);
      } else if (msg->encoding == enc::MONO16) {
        cv_ptr = cv_bridge::toCvShare(msg, enc::MONO16);
      } else {
        cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
      }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
      return;
    }
    cv::imshow("depth", cv_ptr->image);
    cv::waitKey(3);
  }

  std::uint64_t colorCount() const {
    return color_count_;
  }

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber color_sub_;
  image_transport::Subscriber depth_sub_;

  std::uint64_t color_count_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "eys3d_depth_listener_d");

  EYS3DListener l;

  double time_beg = ros::Time::now().toSec();
  ros::spin();
  double time_end = ros::Time::now().toSec();

  double elapsed = time_end - time_beg;
  ROS_INFO_STREAM("time beg: " << std::fixed << time_beg << " s");
  ROS_INFO_STREAM("time end: " << std::fixed << time_end << " s");
  ROS_INFO_STREAM("time cost: " << elapsed << " s");
  ROS_INFO_STREAM("color count: " << l.colorCount() << ", "
      << (l.colorCount() / elapsed) << " fps");

  return 0;
}
