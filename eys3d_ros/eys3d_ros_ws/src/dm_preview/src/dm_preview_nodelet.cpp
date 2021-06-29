/*Copyright:
	This file copyright (C) 2017 by

	eYs3D an Etron company

	An unpublished work.  All rights reserved.

	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of eYs3D an Etron company.
*/
#define ROS_IMU

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <dm_preview/GetParams.h>

#include <unistd.h>
#include <vector>
#include <string>
#include <iomanip>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dm_preview/Temp.h> // NOLINT

//s:Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <dm_preview/dm_previewConfig.h>
#include <boost/thread.hpp>
//e:Dynamic Reconfigure

#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
using namespace configuru;  // NOLINT

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <csignal>

#include "EYS3DSystem.h"
#include "client.h"
#include "devices/CameraDevice.h"
#include "video/Frame.h"
#include "video/PCFrame.h"

namespace enc = sensor_msgs::image_encodings;

std::function<void()> close_function;

class DMPreviewNodelet : public nodelet::Nodelet {
 public:
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns;

  image_transport::CameraPublisher pub_left_color;
  image_transport::CameraPublisher pub_right_color;
  image_transport::CameraPublisher pub_depth;
  ros::Publisher pub_points;
  ros::Publisher pub_imu;
  ros::Publisher pub_imu_processed;

  ros::ServiceServer get_params_service_;

  sensor_msgs::CameraInfoPtr left_info_ptr;
  sensor_msgs::CameraInfoPtr right_info_ptr;
  sensor_msgs::CameraInfoPtr depth_info_ptr;

  sensor_msgs::PointCloud2 point_msg;
  std::shared_ptr<sensor_msgs::PointCloud2Modifier> point_modifier;

  std::string left_color_frame_id;
  std::string right_color_frame_id;
  std::string depth_frame_id;
  std::string points_frame_id;
  std::string imu_frame_id;
  std::string imu_frame_processed_id;

  enum ColorMode {
    Color_Left = 0,
    Color_Left_Right,
  };

  enum ColorFormat {
    Color_MJPEG = 0,
    Color_YUYV
  };

  enum DepthType {
    Depth_Raw = 0,
    Depth_Gray,
    Depth_Colorful,
  };

  struct DeviceParams {
    
    int framerate_;    
    
    int color_width_;
    int color_height_;
    ColorMode color_mode_;
    ColorFormat color_stream_format_;

    int depth_width_;
    int depth_height_;
    int depth_data_type_;
    DepthType depth_type_;

    int z_maximum_mm_;

    bool auto_exposure_;
    bool auto_white_balance_;

    int exposure_time_step_;

    int ir_intensity_;

    std::string serial_number_;
    std::string kernel_name_;
  };

  DeviceParams params_;

  typedef struct SubResult {
    bool left_color;
    bool right_color;
    bool depth;
    bool points;
    bool imu;
    bool imu_processed;
  } sub_result_t;

  sub_result_t sub_result;

  std::uint64_t unit_hard_time = 4294900000;

  std::shared_ptr<libeYs3D::EYS3DSystem> eYs3DSystem_;
  std::shared_ptr<libeYs3D::devices::CameraDevice> device_;

  DMPreviewNodelet() {
  }

  ~DMPreviewNodelet() {
    closeDevice();
  }

  //s:Dynamic Reconfigure
  void paramConfigCallback(dm_preview::dm_previewConfig &config, uint32_t level) {
    
  }

  void receiveParamter(){
    ros::NodeHandle nh;
    NODELET_INFO_STREAM("Callback Thread : " << boost::this_thread::get_id());

    dynamic_reconfigure::Server<dm_preview::dm_previewConfig> dr_srv;
    dynamic_reconfigure::Server<dm_preview::dm_previewConfig>::CallbackType cb;
    cb = boost::bind(&DMPreviewNodelet::paramConfigCallback,this, _1, _2);
    dr_srv.setCallback(cb);

    ros::Rate loop_rate(50);
    while (nh.ok()){
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  //e:Dynamic Reconfigure

  bool color_image_callback(const libeYs3D::video::Frame *frame) {
      
      if (0 == pub_left_color.getNumSubscribers()) return true;

      sensor_msgs::CameraInfoPtr info;
      std_msgs::Header header;
      header.stamp = ros::Time().now();;
      header.frame_id = left_color_frame_id;

      cv::Mat mat =
          cv::Mat(frame->height, frame->width, CV_8UC3, (void *)frame->rgbVec.data());

      auto &&msg = cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg();
      if (info) {
          info->header.stamp = msg->header.stamp;
          info->header.frame_id = left_color_frame_id;
      }

      pub_left_color.publish(msg, info);
      return true;
  }

  bool depth_image_callback(const libeYs3D::video::Frame *frame) {
      
      if (0 == pub_depth.getNumSubscribers()) return true;

      sensor_msgs::CameraInfoPtr info;
      std_msgs::Header header;
      header.stamp = ros::Time().now();
      ;
      header.frame_id = depth_frame_id;

      cv::Mat mat = cv::Mat(frame->height, frame->width, CV_8UC3,
                            (void *)frame->rgbVec.data());

      auto &&msg = cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg();
      if (info) {
          info->header.stamp = msg->header.stamp;
          info->header.frame_id = depth_frame_id;
      }

      pub_depth.publish(msg, info);
      return true;
  }

  bool pc_frame_callback(const libeYs3D::video::PCFrame *pcFrame) {

      sensor_msgs::PointCloud2Iterator<float> iter_x(point_msg, "x");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_msg, "rgb");

      for (int index = 0; index < pcFrame->width * pcFrame->height; ++index) {
          iter_x[0] = pcFrame->xyzDataVec[index * 3] / 1000.0f;
          iter_x[1] = pcFrame->xyzDataVec[index * 3 + 1] / 1000.0f;
          iter_x[2] = pcFrame->xyzDataVec[index * 3 + 2] / 1000.0f;

          iter_rgb[0] = pcFrame->rgbDataVec[index * 3 + 2];
          iter_rgb[1] = pcFrame->rgbDataVec[index * 3 + 1];
          iter_rgb[2] = pcFrame->rgbDataVec[index * 3];

          ++iter_x; ++iter_rgb;
      }

      point_msg.header.stamp = ros::Time().now();
      pub_points.publish(point_msg);

      return true;
  }

  bool imu_data_callback(const libeYs3D::sensors::SensorData *sensorData) {
      char buffer[2048];
      sensorData->toString(buffer, sizeof(buffer));

      return true;
  }

  void onInit() override {
    
    std::string dashes(30, '-');

    nh = getMTNodeHandle();// Get the node handle with the Multi Threaded callback queue.
    nh_ns = getMTPrivateNodeHandle();// Get the private node handle with the Multi Threaded callback queue.

    close_function = std::bind(&DMPreviewNodelet::deleteDevice, this);

    signal(SIGINT, [](int signal) {
      close_function();
      exit(signal);
    });

    //s:Dynamic Reconfigure
    boost::thread dynamic_reconfig_thread(&DMPreviewNodelet::receiveParamter,this);
    NODELET_INFO_STREAM("Main Thread id : " << boost::this_thread::get_id());
    //e:Dynamic Reconfigure

    getLanuchParams();
 
    // open device
    openDevice();

    // loop
    ros::Rate loop_rate(params_.framerate_);
    while (nh_ns.ok()) {
      loop_rate.sleep();
    }

  }

  void getLanuchParams() {
      
      nh_ns.getParamCached("framerate", params_.framerate_);

      int color_mode;
      nh_ns.getParamCached("color_mode", color_mode);
      params_.color_mode_ = (ColorMode) color_mode;

      nh_ns.getParamCached("color_width", params_.color_width_);
      nh_ns.getParamCached("color_height", params_.color_height_);

      int color_stream_format;
      nh_ns.getParamCached("color_stream_format", color_stream_format);
      params_.color_stream_format_ = (ColorFormat) color_stream_format;

      nh_ns.getParamCached("depth_width", params_.depth_width_);
      nh_ns.getParamCached("depth_height", params_.depth_height_);

      nh_ns.getParamCached("state_ae", params_.auto_exposure_);
      nh_ns.getParamCached("state_awb", params_.auto_white_balance_);
      nh_ns.getParamCached("exposure_time_step", params_.exposure_time_step_);
      nh_ns.getParamCached("ir_intensity", params_.ir_intensity_);

      int depth_type;
      nh_ns.getParamCached("depth_type", depth_type);
      params_.depth_type_ = (DepthType) depth_type;

      nh_ns.getParamCached("depth_data_type", params_.depth_data_type_);
      nh_ns.getParamCached("depth_maximum_mm", params_.z_maximum_mm_);

      nh_ns.getParamCached("dev_serial_number", params_.serial_number_);
      nh_ns.getParamCached("kernel_name", params_.kernel_name_);

      nh_ns.getParamCached("left_color_frame", left_color_frame_id);
      nh_ns.getParamCached("right_color_frame", right_color_frame_id);
      nh_ns.getParamCached("depth_frame", depth_frame_id);
      nh_ns.getParamCached("points_frame", points_frame_id);
      nh_ns.getParamCached("imu_frame", imu_frame_id);
      nh_ns.getParamCached("imu_frame_processed", imu_frame_processed_id);

      NODELET_INFO_STREAM("left_color_frame: " << left_color_frame_id);
      NODELET_INFO_STREAM("right_color_frame: " << right_color_frame_id);
      NODELET_INFO_STREAM("depth_frame: " << depth_frame_id);
      NODELET_INFO_STREAM("points_frame: " << points_frame_id);
      NODELET_INFO_STREAM("imu_frame: " << imu_frame_id);
      NODELET_INFO_STREAM("imu_frame_processed: " << imu_frame_processed_id);

      std::string left_color_topic;
      std::string right_color_topic;
      std::string depth_topic;
      std::string points_topic;
      std::string imu_topic;
      std::string imu_processed_topic;
      nh_ns.getParamCached("left_color_topic", left_color_topic);
      nh_ns.getParamCached("right_color_topic", right_color_topic);
      nh_ns.getParamCached("depth_topic", depth_topic);
      nh_ns.getParamCached("points_topic", points_topic);
      nh_ns.getParamCached("imu_topic", imu_topic);
      nh_ns.getParamCached("imu_processed_topic", imu_processed_topic);

      image_transport::ImageTransport it_eys3d_depth(nh);
      // left
      pub_left_color = it_eys3d_depth.advertiseCamera(left_color_topic, 1);
      NODELET_INFO_STREAM("Advertized on topic " << left_color_topic);
      // right
      pub_right_color = it_eys3d_depth.advertiseCamera(right_color_topic, 1);
      NODELET_INFO_STREAM("Advertized on topic " << right_color_topic);
      // depth
      pub_depth = it_eys3d_depth.advertiseCamera(depth_topic, 1);
      NODELET_INFO_STREAM("Advertized on topic " << depth_topic);
      // points
      pub_points = nh.advertise<sensor_msgs::PointCloud2>(points_topic, 1);
      NODELET_INFO_STREAM("Advertized on topic " << points_topic);
      // imu
      pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 100);
      NODELET_INFO_STREAM("Advertized on topic " << imu_topic);
      pub_imu_processed =
          nh.advertise<sensor_msgs::Imu>(imu_processed_topic, 100);  // NOLINT
      NODELET_INFO_STREAM("Advertized on topic " << imu_processed_topic);
  }

  ros::Time compatibleTimestamp(const int &frame_id) {
    static ros::Time timestamp;
    static int id = 0;
    if (id != frame_id) {
      timestamp = ros::Time().now();
      id = frame_id;
    }

    return timestamp;
  }

  void openDevice() {
      
      eYs3DSystem_ = libeYs3D::EYS3DSystem::getEYS3DSystem();

      int index = 0;
      if (!params_.serial_number_.empty() || !params_.kernel_name_.empty()) {
          index = -1;
          
          for (int i = 0; i < eYs3DSystem_->getCameraDeviceCount() ; ++i) {
              std::shared_ptr<libeYs3D::devices::CameraDevice> device =
                  eYs3DSystem_->getCameraDevice(i);

              if (!device) continue;

              if (!params_.serial_number_.empty()) {
                  if (strcmp(params_.serial_number_.c_str(),
                             device->getCameraDeviceInfo().serialNumber)) {
                      continue;
                  }
              }else {
                  if (strcmp(params_.kernel_name_.c_str(),
                             device->getCameraDeviceInfo().busInfo)) {
                      continue;
                  }
              }

              index = i;
              break;
          }
      }

      device_ = eYs3DSystem_->getCameraDevice(index);

      if (!device_) {
          exit(-1);
      }

      point_msg.width = params_.depth_width_;
      point_msg.height = params_.depth_height_;
      point_msg.is_dense = true;
      point_msg.header.frame_id = points_frame_id;

      point_modifier =
          std::make_shared<sensor_msgs::PointCloud2Modifier>(point_msg);
      point_modifier->setPointCloud2Fields(
          4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
          sensor_msgs::PointField::FLOAT32, "z", 1,
          sensor_msgs::PointField::FLOAT32, "rgb", 1,
          sensor_msgs::PointField::FLOAT32);
      point_modifier->setPointCloud2FieldsByString(2, "xyz", "rgb");

      libeYs3D::video::COLOR_RAW_DATA_TYPE color_type = 
          Color_YUYV == params_.color_stream_format_ ?
          libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2 :
          libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_MJPG;

      libeYs3D::devices::IRProperty property = device_->getIRProperty();
      property.setIRValue(params_.ir_intensity_);
      device_->setIRProperty(property);

      device_->setCameraDevicePropertyValue(
          libeYs3D::devices::CameraDeviceProperties::AUTO_EXPOSURE,
          params_.auto_exposure_);

      device_->setCameraDevicePropertyValue(
          libeYs3D::devices::CameraDeviceProperties::AUTO_WHITE_BLANCE,
          params_.auto_white_balance_);

      if (!params_.auto_exposure_){
          device_->setCameraDevicePropertyValue(
              libeYs3D::devices::CameraDeviceProperties::EXPOSURE_TIME,
              params_.exposure_time_step_);
      }

      int ret = device_->initStream(
          color_type, params_.color_width_, params_.color_height_,
          params_.framerate_,
          (libeYs3D::video::DEPTH_RAW_DATA_TYPE)params_.depth_data_type_,
          params_.depth_width_, params_.depth_height_,
          DEPTH_IMG_COLORFUL_TRANSFER, IMAGE_SN_SYNC, 0,
          std::bind(&DMPreviewNodelet::color_image_callback, this,
                    std::placeholders::_1),
          std::bind(&DMPreviewNodelet::depth_image_callback, this,
                    std::placeholders::_1),
          std::bind(&DMPreviewNodelet::pc_frame_callback, this,
                    std::placeholders::_1),
          std::bind(&DMPreviewNodelet::imu_data_callback, this,
                    std::placeholders::_1));

      device_->enableStream();
      
      uint16_t z_near, z_far;
      device_->getDepthOfField(&z_near, &z_far);
      device_->setDepthOfField(z_near, params_.z_maximum_mm_);
  }

  void deleteDevice() {
      closeDevice();
      device_.reset();
  }

  void closeDevice() {
      if (device_) device_->closeStream();
  }

  ros::Time hardTimeToSoftTime(std::uint64_t _hard_time) {
    static bool isInited = false;
    static uint64_t hard_time_begin(0);
    static double soft_time_begin(0);
    if (false == isInited) {
      soft_time_begin = ros::Time::now().toSec();
      hard_time_begin = _hard_time;
      isInited = true;
    }

    std::uint64_t time_ns_detal = (_hard_time - hard_time_begin);
    std::uint64_t time_ns_detal_s = time_ns_detal / 100000;
    std::uint64_t time_ns_detal_ns = time_ns_detal % 100000;
    double time_sec_double =
      ros::Time(time_ns_detal_s, time_ns_detal_ns * 10000).toSec();

    return ros::Time(soft_time_begin + time_sec_double);
  }

  inline bool is_overflow(
      std::uint64_t now, std::uint64_t pre) {

    return (now < pre) && ((pre - now) > (unit_hard_time / 2));
  }

  ros::Time checkUpImuTimeStamp(std::uint64_t _hard_time) {
    static std::uint64_t hard_time_now;
    static std::uint64_t imu_acc;

    if (is_overflow(_hard_time, hard_time_now)) {
      imu_acc++;
    }

    hard_time_now = _hard_time;

    return hardTimeToSoftTime(
        imu_acc * unit_hard_time + _hard_time);
  }
};

#include <pluginlib/class_list_macros.h> // NOLINT
PLUGINLIB_EXPORT_CLASS(DMPreviewNodelet, nodelet::Nodelet); // NOLINT
