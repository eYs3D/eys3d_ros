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
#include <tf2_ros/static_transform_broadcaster.h>

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
#include "version_utils.h"

// + camera info
#include "types_calib.h"
// - camera info
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

  std::string left_color_frame_id;
  std::string right_color_frame_id;
  std::string depth_frame_id;
  std::string points_frame_id;
  std::string imu_frame_id;
  std::string imu_frame_processed_id;

  int d_ir = 0;
  bool d_pcolor = false;
  bool d_pdepth = false;
  bool d_pstream = false;

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

    bool colorful_pointcloud;

    bool enable_depth_decimation_filter;
    int depth_decimation_filter_rate;
    
    int depth_data_type_;
    DepthType depth_type_;
    bool interleave_mode_;

    int z_maximum_mm_;

    bool auto_exposure_;
    bool auto_white_balance_;

    int exposure_time_step_;

    int ir_intensity_;

    std::string serial_number_;
    std::string kernel_name_;

    int zd_tbl_index_;
    
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
    if (config.ir_intensity != d_ir)
    {
      ROS_INFO("Dynamic Reconfigure : IR levle = %d", config.ir_intensity);
      libeYs3D::devices::IRProperty property = device_->getIRProperty();
      property.setIRValue(config.ir_intensity);
      device_->setIRProperty(property);
      d_ir = config.ir_intensity;
    }

    if (config.pause_stream != d_pstream)
    {
      if (config.pause_stream == true)
      {
        device_->pauseStream();
        d_pstream = true;
        ROS_INFO("Dynamic Reconfigure : Pause Stream");
      }
      else if (config.pause_stream == false)
      {
        device_->enableStream();
        d_pstream = false;
        ROS_INFO("Dynamic Reconfigure : Enable Stream");
      }
    }

    if (config.pause_color != d_pcolor)
    {
      if (config.pause_color == true)
      {
        device_->pauseColorStream();
        d_pcolor = true;
        ROS_INFO("Dynamic Reconfigure : Pause Color");
      }
      else if (config.pause_color == false)
      {
        device_->enableColorStream();
        d_pcolor = false;
        ROS_INFO("Dynamic Reconfigure : Enable Color");
      }
    }

    if (config.pause_depth != d_pdepth)
    {
      if (config.pause_depth == true)
      {
        device_->pauseDepthStream();
        d_pdepth = true;
        ROS_INFO("Dynamic Reconfigure : Pause Depth");
      }
      else if (config.pause_depth == false)
      {
        device_->enableDepthStream();
        d_pdepth = false;
        ROS_INFO("Dynamic Reconfigure : Enable Depth");
      }
    }
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

      auto timestamp = compatibleTimestamp(frame->serialNumber);

      sensor_msgs::CameraInfoPtr info;
//s:Split L/R Image
      std_msgs::Header headerLeft, headerRight;
      headerLeft.stamp = timestamp;
      headerRight.stamp = timestamp;
      headerLeft.frame_id = left_color_frame_id;
      headerRight.frame_id = right_color_frame_id;
      cv::Mat croppedLeftImg, croppedRightImg;
      bool bLeftOut = false, bRightOut = false;      
//e:Split L/R Image      

      cv::Mat mat =
          cv::Mat(frame->height, frame->width, CV_8UC3, (void *)frame->rgbVec.data());

//s:Split L/R Image
      if (params_.color_mode_ == 1) //color_Left = 0, color_Left_Right = 1
      {
        bLeftOut = true;
        bRightOut = true;
        mat(cv::Rect(0, 0, frame->width/2, frame->height)).copyTo(croppedLeftImg);
        mat(cv::Rect(frame->width/2, 0, frame->width/2, frame->height)).copyTo(croppedRightImg);       
      }
      else
      {
        bLeftOut = true;
        bRightOut = false;
        mat(cv::Rect(0, 0, frame->width, frame->height)).copyTo(croppedLeftImg);
      }

      auto &&msgLeft = cv_bridge::CvImage(headerLeft, enc::BGR8, croppedLeftImg).toImageMsg();
      auto &&msgRight = cv_bridge::CvImage(headerRight, enc::BGR8, croppedRightImg).toImageMsg();

      if(bLeftOut)
      {
          left_info_ptr->header.stamp = msgLeft->header.stamp;
          left_info_ptr->header.frame_id = left_color_frame_id;
          pub_left_color.publish(msgLeft, left_info_ptr);
      }

      if(bRightOut)  
      {
          right_info_ptr->header.stamp = msgRight->header.stamp;
          right_info_ptr->header.frame_id = right_color_frame_id;
          pub_right_color.publish(msgRight, right_info_ptr);
      }
 //e:Split L/R Image

      return true;
  }

  bool depth_image_callback(const libeYs3D::video::Frame *frame) {
      
      if (0 == pub_depth.getNumSubscribers()) return true;

      auto timestamp = compatibleTimestamp(frame->serialNumber);

      sensor_msgs::CameraInfoPtr info;
      std_msgs::Header header;
      header.stamp = timestamp;//ros::Time().now();

      header.frame_id = left_color_frame_id;

      cv::Mat mat = cv::Mat(frame->height, frame->width, CV_8UC3,
                            (void *)frame->rgbVec.data());
      auto &&msg = cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg();

      switch (params_.depth_type_){
        case DepthType::Depth_Raw: {
            mat = cv::Mat(frame->height, frame->width, CV_16UC1,
                            (void *)frame->dataVec.data());
            msg = cv_bridge::CvImage(header, enc::MONO16, mat).toImageMsg();
            break;
        }
        case DepthType::Depth_Gray: {
            mat = cv::Mat(frame->height, frame->width, CV_16UC1,
                            (void *)frame->zdDepthVec.data());
            mat.setTo(0,mat > params_.z_maximum_mm_);
            msg = cv_bridge::CvImage(header, enc::TYPE_16UC1, mat).toImageMsg();
            break;
        }
      }

       //if(depth_info_ptr)
      //{
      depth_info_ptr->header.stamp = msg->header.stamp;
      depth_info_ptr->header.frame_id = left_color_frame_id;
      //}
      pub_depth.publish(msg, depth_info_ptr);

      return true;
  }

  bool pc_frame_callback(const libeYs3D::video::PCFrame *pcFrame) {
      
      if (0 == pub_points.getNumSubscribers()) return true;
      point_msg.width = pcFrame->width;
      point_msg.height = pcFrame->height;
      point_msg.is_dense = true;
      point_msg.header.frame_id = left_color_frame_id;
      sensor_msgs::PointCloud2Modifier modifier(point_msg);
      unsigned int valid_count = 0;

      if (params_.colorful_pointcloud == false){

          modifier.setPointCloud2FieldsByString(1, "xyz");      
          sensor_msgs::PointCloud2Iterator<float>iter_x(point_msg, "x");

          for (int index = 0; index < point_msg.width * point_msg.height; ++index) {
              if (pcFrame->xyzDataVec[index * 3 + 2] > 0) {
                  
                  iter_x[0] = pcFrame->xyzDataVec[index * 3] / 1000.0f;
                  iter_x[1] = pcFrame->xyzDataVec[index * 3 + 1] / 1000.0f;
                  iter_x[2] = pcFrame->xyzDataVec[index * 3 + 2] / 1000.0f;

                  ++iter_x;
                  ++valid_count;
              }
          }}

      else{
          modifier.setPointCloud2FieldsByString(2, "xyz","rgb");
          sensor_msgs::PointCloud2Iterator<float>iter_x(point_msg, "x");
          sensor_msgs::PointCloud2Iterator<uint8_t>iter_rgb(point_msg, "rgb");

          for (int index = 0; index < point_msg.width * point_msg.height; ++index) {
              if (pcFrame->xyzDataVec[index * 3 + 2] > 0) {

                  iter_x[0] = pcFrame->xyzDataVec[index * 3] / 1000.0f;
                  iter_x[1] = pcFrame->xyzDataVec[index * 3 + 1] / 1000.0f;
                  iter_x[2] = pcFrame->xyzDataVec[index * 3 + 2] / 1000.0f;

                  iter_rgb[0] = pcFrame->rgbDataVec[index * 3];
                  iter_rgb[1] = pcFrame->rgbDataVec[index * 3 + 1];
                  iter_rgb[2] = pcFrame->rgbDataVec[index * 3 + 2];

                  ++iter_x;
                  ++valid_count;
                  ++iter_rgb;
              }
          }}

        

      modifier.resize(valid_count);
      point_msg.width = valid_count;
      point_msg.height = 1;
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
    
    //NODELET_INFO_STREAM("Wrapper version : " << WRAPPER_VERSION);
    //NODELET_INFO_STREAM("Git branch : " << GIT_BRANCH " / commit id : " << GIT_HASH);
    //NODELET_INFO_STREAM("Build time : " << BUILD_TIMESTAMP);

    std::string dashes(30, '-');

    nh = getMTNodeHandle();// Get the node handle with the Multi Threaded callback queue.
    nh_ns = getMTPrivateNodeHandle();// Get the private node handle with the Multi Threaded callback queue.

    close_function = std::bind(&DMPreviewNodelet::deleteDevice, this);

    signal(SIGINT, [](int signal) {
      close_function();
      exit(signal);
    });

    getLanuchParams();
 
    // open device
    openDevice();
    //++Calibration info
    bool in_ok;
    StreamMode mStreamMode = getStreamModeIndex();
    auto&& in = getCameraIntrinsics(mStreamMode, &in_ok);
    if (in_ok) {
      NODELET_INFO_STREAM("Camera info is created");
    } else {
      NODELET_WARN_STREAM("Camera info is null, use default parameters.");
    }
    left_info_ptr = createCameraInfo(in.left);
//s:New Camera Info  
    //in.right.isRightCam = true;
//e:New Camera Info    
    right_info_ptr = createCameraInfo(in.right);
    depth_info_ptr = createCameraInfo(in.left_d);
    //--Calibration info

    //s:Dynamic Reconfigure
    boost::thread dynamic_reconfig_thread(&DMPreviewNodelet::receiveParamter,this);
    NODELET_INFO_STREAM("Main Thread id : " << boost::this_thread::get_id());
    //e:Dynamic Reconfigure

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
      
      nh_ns.getParamCached("colorful_pointcloud", params_.colorful_pointcloud);

      nh_ns.getParamCached("enable_depth_decimation_filter", params_.enable_depth_decimation_filter);
      nh_ns.getParamCached("depth_decimation_filter_rate", params_.depth_decimation_filter_rate);

      nh_ns.getParamCached("state_ae", params_.auto_exposure_);
      nh_ns.getParamCached("state_awb", params_.auto_white_balance_);
      nh_ns.getParamCached("exposure_time_step", params_.exposure_time_step_);
      nh_ns.getParamCached("ir_intensity", params_.ir_intensity_);

      nh_ns.getParamCached("zd_tbl_index", params_.zd_tbl_index_);            

      int depth_type;
      nh_ns.getParamCached("depth_type", depth_type);
      params_.depth_type_ = (DepthType) depth_type;

      nh_ns.getParamCached("interleave_mode", params_.interleave_mode_);

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

      eYs3DSystem_ = std::make_shared<libeYs3D::EYS3DSystem>(libeYs3D::EYS3DSystem::COLOR_BYTE_ORDER::COLOR_RGB24);

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

      libeYs3D::video::COLOR_RAW_DATA_TYPE color_type = 
          Color_YUYV == params_.color_stream_format_ ?
          libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2 :
          libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_MJPG;

      device_->enableExtendIR(true);
      libeYs3D::devices::IRProperty property = device_->getIRProperty();
      property.setIRValue(params_.ir_intensity_);
      device_->setIRProperty(property);

      /*device_->setCameraDevicePropertyValue(
          libeYs3D::devices::CameraDeviceProperties::AUTO_EXPOSURE,
          params_.auto_exposure_);

      device_->setCameraDevicePropertyValue(
          libeYs3D::devices::CameraDeviceProperties::AUTO_WHITE_BLANCE,
          params_.auto_white_balance_);

      if (!params_.auto_exposure_){
          device_->setCameraDevicePropertyValue(
              libeYs3D::devices::CameraDeviceProperties::EXPOSURE_TIME,
              params_.exposure_time_step_);
      }*/

      auto& processOptions = device_->getPostProcessOptions();

        processOptions.enablePostProcess(false);      
        processOptions.setSpatialOutlierThreshold(20);// Smaller filter more points.
        processOptions.setSpatialFilterKernelSize(3); // Should be odd number for filter kernel size.
        
        processOptions.enableDepthDecimation(params_.enable_depth_decimation_filter);
        processOptions.setDecimationFactor(params_.depth_decimation_filter_rate);        // Ceil(resolution / factor / 4) * 4 will = resized resolution
        
        processOptions.enableColorPostProcess(false);
        processOptions.setColorResizeFactor(0.5);    //  resolution * factor = resized resolution
        device_->setPostProcessOptions(processOptions);
  
      int ret = device_->initStream(
          color_type, params_.color_width_, params_.color_height_,
          params_.framerate_,
          (libeYs3D::video::DEPTH_RAW_DATA_TYPE)params_.depth_data_type_,
          params_.depth_width_, params_.depth_height_,
          DEPTH_IMG_COLORFUL_TRANSFER, IMAGE_SN_SYNC, params_.zd_tbl_index_,
          std::bind(&DMPreviewNodelet::color_image_callback, this,
                    std::placeholders::_1),
          std::bind(&DMPreviewNodelet::depth_image_callback, this,
                    std::placeholders::_1),
          std::bind(&DMPreviewNodelet::pc_frame_callback, this,
                    std::placeholders::_1),
          std::bind(&DMPreviewNodelet::imu_data_callback, this,
                    std::placeholders::_1));

    bool isSupportInterLeaveMode = device_->isInterleaveModeSupported();
    device_->enableInterleaveMode(true);
    device_->enableInterleaveMode(false);
    NODELET_INFO_STREAM("isInterleaveModeSupported : " << isSupportInterLeaveMode);
    if (isSupportInterLeaveMode) {
        ret = device_->enableInterleaveMode(params_.interleave_mode_);
        if ( ret == APC_OK ) {
            NODELET_INFO_STREAM("is InterLeave Mode enabled: " << device_->isInterleaveModeEnabled());
        } else {
            NODELET_INFO_STREAM("enable Interleave Mode failure reason : " << ret);
        }
    }
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

      if (device_) {
          device_->closeStream();
      }
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

  //++Calibration info
StreamMode getStreamModeIndex(){
    StreamMode mStreamMode;
    if ( params_.color_width_ == 640 && params_.color_height_ == 480) {
        mStreamMode = StreamMode::STREAM_640x480;
    } else if (params_.color_width_ == 1280 && params_.color_height_ == 480){
        mStreamMode = StreamMode::STREAM_1280x480;
    }else if (params_.color_width_ == 1280 && params_.color_height_ == 720){
        mStreamMode = StreamMode::STREAM_1280x720;
    }else if (params_.color_width_ == 2560 && params_.color_height_ == 720){
        mStreamMode = StreamMode::STREAM_2560x720;
    }else if (params_.color_width_ == 640 && params_.color_height_ == 400){
        mStreamMode = StreamMode::STREAM_640x400;
    }else{
        mStreamMode = StreamMode::STREAM_CUSTOM;
    }
      return mStreamMode;
}

int getRectifyLogIndex(StreamMode stream_mode){
    int index;
    /*switch (stream_mode) {
        case StreamMode::STREAM_640x480:
        case StreamMode::STREAM_640x400:
        case StreamMode::STREAM_1280x480:  // 480p, vga
            index = 1; break;      
        case StreamMode::STREAM_1280x720:
        case StreamMode::STREAM_2560x720:  // 720p, hd
            index = 0; break;
        default:
            index =  1;
        break;
        // throw new std::runtime_error("StreamMode is unknown");
  }*/
    index = params_.zd_tbl_index_;
    return index;
}


StreamIntrinsics getCameraIntrinsics(
    const StreamMode& stream_mode, bool* ok) {

    auto in = getCameraRectifyLog(stream_mode, ok);
    if (*ok) return in;

    // if false, return default intrinsics
    // {w, h, fx, fy, cx, cy, coeffs[5]{k1,k2,p1,p2,k3}}
    CameraIntrinsics cam_in;

    switch (stream_mode) {
        case StreamMode::STREAM_640x480:
            cam_in = {640, 480, 979.8, 942.8, 682.3 / 2, 254.9, {0, 0, 0, 0, 0}};
            break;
        case StreamMode::STREAM_1280x480:
            cam_in = {640, 480, 979.8, 942.8, 682.3, 254.9, {0, 0, 0, 0, 0}};
            break;
        case StreamMode::STREAM_1280x720:
            cam_in = {1280, 720, 979.8, 942.8, 682.3, 254.9 * 2, {0, 0, 0, 0, 0}};
            break;
        case StreamMode::STREAM_2560x720:
            cam_in = {1280, 720, 979.8, 942.8, 682.3 * 2, 254.9 * 2, {0, 0, 0, 0, 0}};
            break;
        default:
            cam_in = {1280, 720, 979.8, 942.8, 682.3, 254.9 * 2, {0, 0, 0, 0, 0}};
            break;
        }
        return {cam_in, cam_in};
}

StreamIntrinsics getCameraRectifyLog(
    const StreamMode& stream_mode, bool* ok) {

    StreamIntrinsics in;
//s:Add ZD Table Index Setting Function    
    // int index = getRectifyLogIndex(stream_mode);
    int index = params_.zd_tbl_index_;
    NODELET_INFO_STREAM("ZD table Index: " << params_.zd_tbl_index_);    
//e:Add ZD Table Index Setting Function
    auto calib = device_->getRectifyLogData(index);
    if (calib == nullptr || calib->InImgWidth == 0) {
        *ok = false;
        return std::move(in);
    }

    in.left.width = calib->OutImgWidth/2;
    in.left.height = calib->OutImgHeight;

    in.left.fx = calib->CamMat1[0];
    in.left.fy = calib->CamMat1[4];
    in.left.cx = calib->CamMat1[2];
    in.left.cy = calib->CamMat1[5];
    
    in.left_d.width = params_.depth_width_;
    in.left_d.height = params_.depth_height_;

    in.left_d.fx = calib->CamMat1[0];
    in.left_d.fy = calib->CamMat1[4];
    in.left_d.cx = calib->CamMat1[2];
    in.left_d.cy = calib->CamMat1[5];
    for (int i = 0; i < 5; i++) { //rectify color image no need,raw need
      if (params_.depth_data_type_ == 0 || params_.depth_data_type_ == 9)
      {
        in.left.coeffs[i] = calib->CamDist1[i];
        in.left_d.coeffs[i] = calib->CamDist1[i];
      }
      else
      {
        in.left.coeffs[i] = 0;
        in.left_d.coeffs[i] = 0;
      }
    }
    for (int i = 0; i < 12; i++) {
        in.left.p[i] = calib->NewCamMat1[i];
        in.left_d.p[i] = calib->NewCamMat1[i];
    }
    for (int i = 0; i < 9; i++) {
        in.left.r[i] = calib->LRotaMat[i];
        in.left_d.r[i] = calib->LRotaMat[i];
    }
    in.right.width = calib->OutImgWidth/2;
    in.right.height = calib->OutImgHeight;
    in.right.fx = calib->CamMat2[0];
    in.right.fy = calib->CamMat2[4];
    in.right.cx = calib->CamMat2[2];
    in.right.cy = calib->CamMat2[5];
//s:New Camera Info   
    for (int i = 0; i < 5; i++) {
      if (params_.depth_data_type_ == 0 || params_.depth_data_type_ == 9)
      {
        in.right.coeffs[i] = calib->CamDist2[i];
      }
      else
      {
        in.right.coeffs[i] = 0;
      }      
    }
//e:New Camera Info
    for (int i = 0; i < 12; i++) {
        in.right.p[i] = calib->NewCamMat2[i];
        if (i==3)
        {
          in.right.p[3] = (float) in.right.p[3]/in.right.p[0];
        }
    }
    for (int i = 0; i < 9; i++) {
        in.right.r[i] = calib->RRotaMat[i];
    }

    in.ply_parameters.out_witdh = calib->OutImgWidth;
    in.ply_parameters.out_height = calib->OutImgHeight;
    memcpy(in.ply_parameters.ReProjectMat, calib->ReProjectMat, sizeof(float) * 16);

    *ok = true;
    return std::move(in);
}

sensor_msgs::CameraInfoPtr createCameraInfo(const CameraIntrinsics& in) {
    sensor_msgs::CameraInfo *camera_info = new sensor_msgs::CameraInfo();
    auto camera_info_ptr = sensor_msgs::CameraInfoPtr(camera_info);

    camera_info->width = in.width;
    camera_info->height = in.height;

//s:New Camera Info
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]    
    if (params_.depth_data_type_ == 0 || params_.depth_data_type_ == 9)
    {    
      camera_info->K.at(0) = in.fx;
      camera_info->K.at(2) = in.cx;
      camera_info->K.at(4) = in.fy;
      camera_info->K.at(5) = in.cy;
      camera_info->K.at(8) = 1;    
    }
    else
    {
      camera_info->K.at(0) = in.p[0];
      camera_info->K.at(2) = in.p[2];
      camera_info->K.at(4) = in.p[5];
      camera_info->K.at(5) = in.p[6];    
      camera_info->K.at(8) = 1;
    }

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    camera_info->P.at(0) = in.p[0];
    //camera_info->P.at(1) = (float) in.p[3]/in.p[0];
    camera_info->P.at(2) = in.p[2];
    //if (in.right) { //for right cam info   
    //}
    //else {
    camera_info->P.at(3) = in.p[3];
    //}
    camera_info->P.at(5) = in.p[5];
    camera_info->P.at(6) = in.p[6];
    camera_info->P.at(10) = in.p[10];
//e:New Camera Info

    camera_info->distortion_model = "plumb_bob";

    // D of plumb_bob: (k1, k2, t1, t2, k3)
    for (int i = 0; i < 5; i++) {
      camera_info->D.push_back(in.coeffs[i]);
    }

    // R to identity matrix
    camera_info->R.at(0) = in.r[0];
    camera_info->R.at(1) = in.r[1];
    camera_info->R.at(2) = in.r[2];
    camera_info->R.at(3) = in.r[3];
    camera_info->R.at(4) = in.r[4];
    camera_info->R.at(5) = in.r[5];
    camera_info->R.at(6) = in.r[6];
    camera_info->R.at(7) = in.r[7];
    camera_info->R.at(8) = in.r[8];

    return camera_info_ptr;
}
  //++Calibration info
};

#include <pluginlib/class_list_macros.h> // NOLINT
PLUGINLIB_EXPORT_CLASS(DMPreviewNodelet, nodelet::Nodelet); // NOLINT
