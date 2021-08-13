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
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
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

#include "eys3d_depth/camera.h"
#include "eys3d_depth/utils.h"

#include "pointcloud_generator.h" // NOLINT

//s:Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <dm_preview/dm_previewConfig.h>
#include <boost/thread.hpp>
//e:Dynamic Reconfigure

#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
using namespace configuru;  // NOLINT

#include <dbus/dbus.h>

#include <csignal>

#include "client.h"

#include "eys3d_depth/stubs/global_config.h"

#define EXTEND_IR_MAX 15
#define EXTEND_IR_DEFAULT 3

EYS3D_DEPTH_BEGIN_NAMESPACE

namespace {

void matrix_3x1(const double (*src1)[3], const double (*src2)[1],
    double (*dst)[1]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 1; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

void matrix_3x3(const double (*src1)[3], const double (*src2)[3],
    double (*dst)[3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

}  // namespace

namespace enc = sensor_msgs::image_encodings;

std::function<void()> close_function;

class DMPreviewNodelet : public nodelet::Nodelet {
 public:
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns;
  int skip_tag;
  int skip_tmp_left_tag;
  int skip_tmp_right_tag;
  Version spec_version;

  pthread_mutex_t mutex_sub_result;
  pthread_mutex_t mutex_color;

  image_transport::Publisher pub_left_mono;
  image_transport::CameraPublisher pub_left_color;
  image_transport::Publisher pub_right_mono;
  image_transport::CameraPublisher pub_right_color;
  image_transport::CameraPublisher pub_depth;
  ros::Publisher pub_points;
  ros::Publisher pub_temp;
  ros::Publisher pub_mesh_;  // < The publisher for camera mesh.
  #ifdef ROS_IMU
  ros::Publisher pub_imu;
  ros::Publisher pub_imu_processed;
  #endif

  visualization_msgs::Marker mesh_msg_;  // < Mesh message.
  ros::ServiceServer get_params_service_;

  sensor_msgs::CameraInfoPtr left_info_ptr;
  sensor_msgs::CameraInfoPtr right_info_ptr;
  sensor_msgs::CameraInfoPtr depth_info_ptr;

  // Launch params

  std::int32_t points_frequency;
  double points_factor;
  double gravity;

  std::string base_frame_id;
  std::string left_mono_frame_id;
  std::string left_color_frame_id;
  std::string right_mono_frame_id;
  std::string right_color_frame_id;
  std::string depth_frame_id;
  std::string points_frame_id;
  std::string temp_frame_id;
  #ifdef ROS_IMU
  std::string imu_frame_id;
  std::string imu_frame_processed_id;
  bool imu_timestamp_align = false;
  #endif

  // EYS3D objects
  OpenParams params;
  std::unique_ptr<Camera> eys3d_depth;
  int depth_type = 0;

  // Others

  std::unique_ptr<PointCloudGenerator> pointcloud_generator;

  std::shared_ptr<MotionIntrinsics> motion_intrinsics;
  bool motion_intrinsics_enabled;
  #ifdef ROS_IMU
  std::shared_ptr<ImuData> imu_accel;
  std::shared_ptr<ImuData> imu_gyro;
  #endif
  cv::Mat points_color;
  cv::Mat points_depth;

  typedef struct SubResult {
    bool left_mono;
    bool left_color;
    bool right_mono;
    bool right_color;
    bool depth;
    bool points;
    #ifdef ROS_IMU
    bool imu;
    bool imu_processed;
    #endif
    bool temp;
    bool left;
    bool right;
  } sub_result_t;

  sub_result_t sub_result;

  std::uint64_t unit_hard_time = 4294900000;

  DMPreviewNodelet() {
    skip_tag = -1;
    skip_tmp_left_tag = 0;
    skip_tmp_right_tag = 0;
    pthread_mutex_init(&mutex_sub_result, nullptr);
    pthread_mutex_init(&mutex_color, nullptr);

    DBusError dbus_error;
    ::dbus_error_init(&dbus_error);
    DBusConnection* dbus_conn = ::dbus_bus_get(DBUS_BUS_SYSTEM, &dbus_error);
    if (!dbus_conn) {
      NODELET_INFO_STREAM("Can't get bus connection.\n");
      return;
    }
    
    DBusMessage* dbus_msg = ::dbus_message_new_method_call(
      "org.freedesktop.login1", "/org/freedesktop/login1",
      "org.freedesktop.login1.Manager", "Inhibit");

    if (!dbus_msg) {
      NODELET_INFO_STREAM("Can't create new dbus method call from login1.\n");
      ::dbus_connection_unref(dbus_conn);
      return;
    }

    const char* what = "sleep:shutdown";
    const char* who = "ROS eYs3d node";
    const char* why = "Inhibit sleep:shutdown.";
    const char* mode = "block";
    ::dbus_message_append_args(dbus_msg, DBUS_TYPE_STRING, &what,
                               DBUS_TYPE_STRING, &who, DBUS_TYPE_STRING, &why,
                               DBUS_TYPE_STRING, &mode, DBUS_TYPE_INVALID);

    DBusMessage* dbus_reply = ::dbus_connection_send_with_reply_and_block(
        dbus_conn, dbus_msg, 1000, &dbus_error);
    if (!dbus_reply) {
        NODELET_INFO_STREAM("Can't reply from dbus method call.\n");
        ::dbus_connection_unref(dbus_conn);
        ::dbus_message_unref(dbus_msg);
        return;
    }

    int fd;
    if (dbus_message_get_args(dbus_reply, &dbus_error, DBUS_TYPE_UNIX_FD, &fd, DBUS_TYPE_INVALID)) {
      static int *block_fd = &fd;
      NODELET_INFO_STREAM("Inhibit sleep:shutdown.\n");
    }

    ::dbus_connection_unref(dbus_conn);
    ::dbus_message_unref(dbus_msg);
    ::dbus_message_unref(dbus_reply);

    dbus_conn = ::dbus_bus_get(DBUS_BUS_SESSION, &dbus_error);
    if (!dbus_conn) {
      NODELET_INFO_STREAM("Can't get bus connection.\n");
      return;
    }
    
    dbus_msg = ::dbus_message_new_method_call(
      "org.freedesktop.ScreenSaver", "/org/freedesktop/ScreenSaver",
      "org.freedesktop.ScreenSaver", "Inhibit");

    if (!dbus_msg) {
      NODELET_INFO_STREAM("Can't create new dbus method call from ScreenSaver.\n");
      ::dbus_connection_unref(dbus_conn);
      return;
    }

    const char* application_name = "ROS eYs3d node";
    const char* reason_for_inhibit = "Inhibit lock screen.";
    ::dbus_message_append_args(dbus_msg, DBUS_TYPE_STRING, &application_name,
                               DBUS_TYPE_STRING, &reason_for_inhibit, DBUS_TYPE_INVALID);

    dbus_reply = ::dbus_connection_send_with_reply_and_block(
        dbus_conn, dbus_msg, 1000, &dbus_error);
    if (!dbus_reply) {
        NODELET_INFO_STREAM("Can't reply from dbus method call.\n");
        ::dbus_connection_unref(dbus_conn);
        ::dbus_message_unref(dbus_msg);
        return;
    }

    unsigned int cookie;
    if (dbus_message_get_args(dbus_reply, &dbus_error, DBUS_TYPE_UINT32, &cookie, DBUS_TYPE_INVALID)) {
      static unsigned int *block_cookie = &cookie;
      NODELET_INFO_STREAM("nhibit lock screen.\n");
    }

    ::dbus_connection_unref(dbus_conn);
    ::dbus_message_unref(dbus_msg);
    ::dbus_message_unref(dbus_reply);

  }

  ~DMPreviewNodelet() {
    // workaroud : Avoid IR close incomplete when node exit.
    if (eys3d_depth)
    {
      eys3d_depth->SetIRIntensity(0);
    }
    closeDevice();
    eys3d_depth.reset(nullptr);
    motion_intrinsics = nullptr;
    motion_intrinsics_enabled = false;
  }

  //s:Dynamic Reconfigure
  void paramConfigCallback(dm_preview::dm_previewConfig &config, uint32_t level) {
    if (eys3d_depth->IsOpened()) {
      int state_ir, old_ir;
      bool state_ae, state_awb, old_ae, old_awb;

      state_ir = config.ir_intensity;
      state_ae = config.state_ae;
      state_awb = config.state_awb;

      printf("-- update settings\n");
      nh_ns.param<int>("ir_intensity", old_ir, EXTEND_IR_DEFAULT);
      nh_ns.param<bool>("state_ae", old_ae, true);
      nh_ns.param<bool>("state_awb", old_awb, true);
      if (old_ir != state_ir) {
        printf("-- set new IR : %d\n", state_ir);
        eys3d_depth->SetIRIntensity(state_ir);
      }
      if (old_ae != state_ae) {
        printf("-- set new AE state: %s\n", state_ae ? "enabled" : "disabled");
        eys3d_depth->AutoExposureControl(state_ae);
      }
      if (old_awb != state_awb) {
        printf("-- set new AWB state: %s\n", state_awb ? "enabled" : "disabled");
        eys3d_depth->AutoWhiteBalanceControl(state_awb);
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
    
  void onInit() override {

    NODELET_INFO_STREAM("Version : " << SW_VERSION);
    NODELET_INFO_STREAM("Git branch : " << GIT_BRANCH " / commit id : " << GIT_HASH);
    NODELET_INFO_STREAM("Build time : " << BUILD_TIMESTAMP);

    std::string dashes(30, '-');

    nh = getMTNodeHandle();// Get the node handle with the Multi Threaded callback queue.
    nh_ns = getMTPrivateNodeHandle();// Get the private node handle with the Multi Threaded callback queue.

    close_function = std::bind(&DMPreviewNodelet::deleteDevice, this);

    signal(SIGINT, [](int signal) {
      close_function();
      exit(signal);
    });

    // Launch params
    int dev_index = 0;
    int framerate = 10;
    int ros_output_framerate = -1;
    int low_framerate = 0;
    int dev_mode = 2;
    int color_mode = 0;
    int depth_mode = 0;
    int stream_mode = 0;
    int color_stream_format = 0;
    int depth_stream_format = 0;
    bool state_ae = true;
    bool state_awb = true;
    int ir_intensity = 0;
    bool ir_depth_only = true;
    
    int depth_data_type = 0;
    int depth_maximum_mm = 0;

    std::string color_resolution;
    int color_width = 1280;
    int color_height = 720;

    std::string depth_resolution;
    int depth_width = 1280;
    int depth_height = 720;    



    //s:Dynamic Reconfigure
    eys3d_depth.reset(new Camera);
    boost::thread dynamic_reconfig_thread(&DMPreviewNodelet::receiveParamter,this);
    NODELET_INFO_STREAM("Main Thread id : " << boost::this_thread::get_id());
    //e:Dynamic Reconfigure

    nh_ns.getParamCached("dev_index", dev_index);
    nh_ns.getParamCached("framerate", framerate);
    nh_ns.getParamCached("low_framerate", low_framerate);
    nh_ns.getParamCached("ros_output_framerate", ros_output_framerate);
    nh_ns.getParamCached("dev_mode", dev_mode);
    nh_ns.getParamCached("color_mode", color_mode);
    nh_ns.getParamCached("depth_mode", depth_mode);
    nh_ns.getParamCached("stream_mode", stream_mode);

    nh_ns.getParamCached("color_resolution", color_resolution);
    sscanf(color_resolution.c_str(), "%dx%d", &color_width, &color_height);

    nh_ns.getParamCached("depth_resolution", depth_resolution);
    sscanf(depth_resolution.c_str(), "%dx%d", &depth_width, &depth_height);

    nh_ns.getParamCached("color_stream_format", color_stream_format);
    nh_ns.getParamCached("depth_stream_format", depth_stream_format);
    nh_ns.getParamCached("state_ae", state_ae);
    nh_ns.getParamCached("state_awb", state_awb);
    nh_ns.getParamCached("ir_intensity", ir_intensity);
    nh_ns.getParamCached("ir_depth_only", ir_depth_only);
    nh_ns.getParamCached("depth_type", depth_type);
    nh_ns.getParamCached("depth_data_type", depth_data_type);
    nh_ns.getParamCached("depth_maximum_mm", depth_maximum_mm);
    #ifdef ROS_IMU
    nh_ns.getParamCached("imu_timestamp_align", imu_timestamp_align);
    #endif
    points_frequency = DEFAULT_POINTS_FREQUENCE;
    points_factor = DEFAULT_POINTS_FACTOR;
    gravity = 9.8;
    if (ros_output_framerate > 0 && ros_output_framerate < 7) {
      skip_tag = ros_output_framerate;
    }
    nh_ns.getParamCached("points_frequency", points_frequency);
    nh_ns.getParamCached("points_factor", points_factor);
    nh_ns.getParamCached("gravity", gravity);

    base_frame_id = "dm_preview_link";
    left_mono_frame_id = "dm_preview_left_mono_frame";
    left_color_frame_id = "dm_preview_left_color_frame";
    right_mono_frame_id = "dm_preview_right_mono_frame";
    right_color_frame_id = "dm_preview_right_color_frame";
    depth_frame_id = "dm_preview_depth_frame";
    points_frame_id = "dm_preview_points_frame";
    temp_frame_id = "dm_preview_temp_frame";
    #ifdef ROS_IMU
    imu_frame_id = "dm_preview_imu_frame";
    imu_frame_processed_id = "dm_preview_imu_frame_processed";
   #endif
    nh_ns.getParamCached("base_frame", base_frame_id);
    nh_ns.getParamCached("left_mono_frame", left_mono_frame_id);
    nh_ns.getParamCached("left_color_frame", left_color_frame_id);
    nh_ns.getParamCached("right_mono_frame", right_mono_frame_id);
    nh_ns.getParamCached("right_color_frame", right_color_frame_id);
    nh_ns.getParamCached("depth_frame", depth_frame_id);
    nh_ns.getParamCached("points_frame", points_frame_id);
    nh_ns.getParamCached("temp_frame", temp_frame_id);
    #ifdef ROS_IMU
    nh_ns.getParamCached("imu_frame", imu_frame_id);
    nh_ns.getParamCached("imu_frame_processed", imu_frame_processed_id);
   #endif
    NODELET_INFO_STREAM("base_frame: " << base_frame_id);
    NODELET_INFO_STREAM("left_mono_frame: " << left_mono_frame_id);
    NODELET_INFO_STREAM("left_color_frame: " << left_color_frame_id);
    NODELET_INFO_STREAM("right_mono_frame: " << right_mono_frame_id);
    NODELET_INFO_STREAM("right_color_frame: " << right_color_frame_id);
    NODELET_INFO_STREAM("depth_frame: " << depth_frame_id);
    NODELET_INFO_STREAM("points_frame: " << points_frame_id);
    NODELET_INFO_STREAM("temp_frame: " << temp_frame_id);
    #ifdef ROS_IMU
    NODELET_INFO_STREAM("imu_frame: " << imu_frame_id);
    NODELET_INFO_STREAM("imu_frame_processed: " << imu_frame_processed_id);
    #endif

    std::string left_mono_topic = "dm_preview/left/image_mono";
    std::string left_color_topic = "dm_preview/left/image_color";
    std::string right_mono_topic = "dm_preview/right/image_mono";
    std::string right_color_topic = "dm_preview/right/image_color";
    std::string depth_topic = "dm_preview/depth";
    std::string points_topic = "dm_preview/points";
     std::string temp_topic = "dm_preview/temp";
    #ifdef ROS_IMU
    std::string imu_topic = "dm_preview/imu";
    std::string imu_processed_topic = "dm_preview/imu_processed";
    #endif
    nh_ns.getParamCached("left_mono_topic", left_mono_topic);
    nh_ns.getParamCached("left_color_topic", left_color_topic);
    nh_ns.getParamCached("right_mono_topic", right_mono_topic);
    nh_ns.getParamCached("right_color_topic", right_color_topic);
    nh_ns.getParamCached("depth_topic", depth_topic);
    nh_ns.getParamCached("points_topic", points_topic);
    nh_ns.getParamCached("temp_topic", temp_topic);
    #ifdef ROS_IMU
    nh_ns.getParamCached("imu_topic", imu_topic);
    nh_ns.getParamCached("imu_processed_topic", imu_processed_topic);
    #endif
    // EYS3D objects
    std::vector<DeviceInfo> dev_infos;
    //eys3d_depth.reset(new Camera);
    {
      dev_infos = eys3d_depth->GetDeviceInfos();
      size_t n = dev_infos.size();
      if (n <= 0 || dev_index < 0 || dev_index >= n) {
        NODELET_ERROR_STREAM("Device not found, index: " << dev_index);
        return;
      }

      NODELET_INFO_STREAM(dashes);
      NODELET_INFO_STREAM("Device Information");
      NODELET_INFO_STREAM(dashes);
      for (auto &&info : dev_infos) {
        NODELET_INFO_STREAM(info.index << " | " << info);
      }
      NODELET_INFO_STREAM(dashes);

      params.dev_index = dev_index;
    }
    spec_version = eys3d_depth->GetDescriptors()->spec_version;
    {
      std::vector<StreamInfo> color_infos;
      std::vector<StreamInfo> depth_infos;
      eys3d_depth->GetStreamInfos(dev_index, &color_infos, &depth_infos);

      NODELET_INFO_STREAM("Color Stream Information");
      NODELET_INFO_STREAM(dashes);
      for (auto &&info : color_infos) {
        NODELET_INFO_STREAM(info.index << " | " << info);
      }
      NODELET_INFO_STREAM(dashes);

      NODELET_INFO_STREAM("Depth Stream Information");
      NODELET_INFO_STREAM(dashes);
      for (auto &&info : depth_infos) {
        NODELET_INFO_STREAM(info.index << " | " << info);
      }
      NODELET_INFO_STREAM(dashes);
    }

    pub_mesh_ = nh.advertise<visualization_msgs::Marker>("camera_mesh", 0 );
    // where to get the mesh from
    std::string mesh_file;
    if (nh_ns.getParamCached("mesh_file", mesh_file)) {
      mesh_msg_.mesh_resource = "package://dm_preview/mesh/"+mesh_file;
    } else {
      NODELET_INFO_STREAM("no mesh found for visualisation, set ros param mesh_file, if desired");
      mesh_msg_.mesh_resource = "";
    }

    const std::string DEVICE_PARAMS_SERVICE = "get_params";
    get_params_service_ = nh_ns.advertiseService(
        DEVICE_PARAMS_SERVICE, &DMPreviewNodelet::getParams, this);
    NODELET_INFO_STREAM("Advertized service " << DEVICE_PARAMS_SERVICE);

    params.framerate = framerate;
    params.low_framerate = low_framerate;
    params.dev_mode = static_cast<DeviceMode>(dev_mode);
    params.color_mode = static_cast<ColorMode>(color_mode);
    params.depth_mode = static_cast<DepthMode>(depth_mode);
    params.stream_mode = static_cast<StreamMode>(stream_mode);
    params.color_width = color_width;
    params.color_height = color_height;
    params.depth_width = depth_width;
    params.depth_height = depth_height;
    params.color_stream_format =
        static_cast<StreamFormat>(color_stream_format);
    params.depth_stream_format =
        static_cast<StreamFormat>(depth_stream_format);
    params.state_ae = state_ae;
    params.state_awb = state_awb;
    params.ir_intensity = ir_intensity;
    params.ir_depth_only = ir_depth_only;
    params.depth_data_type= static_cast<DepthDataType>(depth_data_type);
    params.colour_depth_value = depth_maximum_mm;
    
    eys3d_depth->EnableProcessMode(ProcessMode::PROC_NONE);

    // Image publishers

    image_transport::ImageTransport it_eys3d_depth(nh);
    // left
    pub_left_mono = it_eys3d_depth.advertise(left_mono_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_mono_topic);
    pub_left_color = it_eys3d_depth.advertiseCamera(left_color_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_color_topic);
    // right
    pub_right_mono = it_eys3d_depth.advertise(right_mono_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_mono_topic);
    pub_right_color = it_eys3d_depth.advertiseCamera(right_color_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_color_topic);
    // depth
    pub_depth = it_eys3d_depth.advertiseCamera(depth_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << depth_topic);
    // points
    pub_points = nh.advertise<sensor_msgs::PointCloud2>(points_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << points_topic);
   #ifdef ROS_IMU 
    // imu
    pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);
    pub_imu_processed = nh.advertise<sensor_msgs::Imu>(imu_processed_topic, 100); // NOLINT
    NODELET_INFO_STREAM("Advertized on topic " << imu_processed_topic);
    #endif
    // temp
    pub_temp = nh.advertise<dm_preview::Temp>(temp_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << temp_topic);
 
    // detect subscribers to enable/disable features
    detectSubscribers();
    // open device
    openDevice();

    // loop
    ros::Rate loop_rate(framerate);
    while (nh_ns.ok()) {
      publishMesh();
      detectSubscribers();
      loop_rate.sleep();
    }

    printf("Process exit\n");
  }

  void detectSubscribers() {
    bool left_mono_sub = pub_left_mono.getNumSubscribers() > 0;
    bool left_color_sub = pub_left_color.getNumSubscribers() > 0;
    bool right_mono_sub = pub_right_mono.getNumSubscribers() > 0;
    bool right_color_sub = pub_right_color.getNumSubscribers() > 0;
    bool depth_sub = pub_depth.getNumSubscribers() > 0;
    bool points_sub = pub_points.getNumSubscribers() > 0;
    bool temp_sub = pub_temp.getNumSubscribers() > 0;
    #ifdef ROS_IMU 
    bool imu_sub = pub_imu.getNumSubscribers() > 0;
    bool imu_processed_sub = pub_imu_processed.getNumSubscribers() > 0;
    #endif
    bool left_sub = left_mono_sub || left_color_sub;
    bool right_sub = right_mono_sub || right_color_sub;

    pthread_mutex_lock(&mutex_sub_result);
    if (left_sub != sub_result.left ||
        right_sub != sub_result.right ||
        depth_sub != sub_result.depth ||
        points_sub != sub_result.points) {
      if (left_sub || right_sub || depth_sub || points_sub) {
        if (eys3d_depth->IsImageInfoSupported()) {
          eys3d_depth->EnableImageInfo(false);
        }
      } else {
        eys3d_depth->DisableImageInfo();
      }
    }
     #ifdef ROS_IMU 
    if (imu_sub != sub_result.imu ||
        imu_processed_sub != sub_result.imu_processed ||
        temp_sub != sub_result.temp) {
      if (imu_sub || imu_processed_sub || temp_sub) {
        if (eys3d_depth->IsMotionDatasSupported()) {
          eys3d_depth->EnableMotionDatas(0);
        }
      } else {
        eys3d_depth->DisableMotionDatas();
      }
    }
    sub_result = {
      left_mono_sub, left_color_sub, right_mono_sub, right_color_sub,
      depth_sub, points_sub, imu_sub, temp_sub,
      imu_processed_sub, left_sub, right_sub,
    };
    #else
    eys3d_depth->DisableMotionDatas();
    sub_result = {
        left_mono_sub, left_color_sub, right_mono_sub, right_color_sub,
        depth_sub, points_sub,temp_sub,left_sub, right_sub,
    };
    #endif
    
    pthread_mutex_unlock(&mutex_sub_result);
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
    if (eys3d_depth->IsOpened()) return;

    // Set stream data callbacks
    std::vector<ImageType> types{
      ImageType::IMAGE_LEFT_COLOR,
      ImageType::IMAGE_RIGHT_COLOR,
      ImageType::IMAGE_DEPTH,
    };
    for (auto&& type : types) {
      eys3d_depth->SetStreamCallback(type, [this](const StreamData& data) {
        pthread_mutex_lock(&mutex_sub_result);
        bool sub_result_left = sub_result.left;
        bool sub_result_points = sub_result.points;
        bool sub_result_left_color = sub_result.left_color;
        bool sub_result_left_mono = sub_result.left_mono;
        bool sub_result_right = sub_result.right;
        bool sub_result_right_color = sub_result.right_color;
        bool sub_result_right_mono = sub_result.right_mono;
        bool sub_result_right_depth = sub_result.depth;
        pthread_mutex_unlock(&mutex_sub_result);

        auto timestamp = data.img_info
          ? checkUpTimeStamp(data.img_info->timestamp, data.img->type())
          : compatibleTimestamp(data.img->frame_id());

        switch (data.img->type()) {
          case ImageType::IMAGE_LEFT_COLOR: {
            if (sub_result_left || sub_result_points) {
              publishLeft(data, timestamp, sub_result_left_color,
                  sub_result_left_mono);
            }
          } break;
          case ImageType::IMAGE_RIGHT_COLOR: {
            if (sub_result_right) {
              publishRight(data, timestamp, sub_result_right_color,
                  sub_result_right_mono);
            }
          } break;
          case ImageType::IMAGE_DEPTH: {
            if (sub_result_right_depth || sub_result_points) {
              publishDepth(data, timestamp);
            }
          } break;
        }
      });
    }

    // Set motion data callback
    #ifdef ROS_IMU 
    eys3d_depth->SetMotionCallback([this](const MotionData& data) {
      pthread_mutex_lock(&mutex_sub_result);
      bool sub_result_imu = sub_result.imu;
      bool sub_result_imu_processed = sub_result.imu_processed;
      bool sub_result_temp = sub_result.temp;
      pthread_mutex_unlock(&mutex_sub_result);
      if (data.imu && (sub_result_imu ||
                       sub_result_imu_processed ||
                       sub_result_temp)) {
        if (data.imu->flag == EYS3D_IMU_ACCEL) {
          imu_accel = data.imu;
          if (imu_timestamp_align) {
            publishAlignImu(sub_result_imu, sub_result_imu_processed, sub_result_temp);
          } else {
            publishImu(sub_result_imu, sub_result_imu_processed, sub_result_temp);
          }
        } else if (data.imu->flag == EYS3D_IMU_GYRO) {
          imu_gyro = data.imu;
          if (imu_timestamp_align) {
            publishAlignImu(sub_result_imu, sub_result_imu_processed, sub_result_temp);
          } else {
            publishImu(sub_result_imu, sub_result_imu_processed, sub_result_temp);
          }
        } else if (data.imu->flag == EYS3D_IMU_ACCEL_GYRO_CALIB) {
          imu_accel = data.imu;
          imu_gyro = data.imu;
          if (0) {
            publishAlignImu(sub_result_imu, sub_result_imu_processed, sub_result_temp);
          } else {
            publishImu(sub_result_imu, sub_result_imu_processed, sub_result_temp);
          }
        }
      }
    });
#endif
    eys3d_depth->Open(params);
    if (!eys3d_depth->IsOpened()) {
      NODELET_ERROR_STREAM("Open camera failed");
      return;
    }
    NODELET_INFO_STREAM("Open camera success");
    eys3d_depth->SetIRMaxValue(EXTEND_IR_MAX); // set IR default value to 3
    dynamic_reconfigure::Client<dm_preview::dm_previewConfig> client("dm_preview_node",NULL);
    dm_preview::dm_previewConfig config;
    config.ir_intensity = params.ir_intensity;
    client.setConfiguration(config); // notify server and (rqt_reconfigure) dm_preview.cfg

    // camera infos
    bool in_ok;
    auto&& in = eys3d_depth->GetStreamIntrinsics(params.stream_mode, &in_ok);
    if (in_ok) {
      NODELET_INFO_STREAM("Camera info is created");
    } else {
      NODELET_WARN_STREAM("Camera info is null, use default parameters.");
    }
    left_info_ptr = createCameraInfo(in.left);
    right_info_ptr = createCameraInfo(in.right);
    depth_info_ptr = createCameraInfo(in.left);

    // motion intrinsics
     #ifdef ROS_IMU
    bool motion_ok;
    if (motion_intrinsics == nullptr) {
      motion_intrinsics = std::make_shared<MotionIntrinsics>();
    }
    *motion_intrinsics = eys3d_depth->GetMotionIntrinsics(&motion_ok);
    if (motion_ok) {
      motion_intrinsics_enabled = true;
    } else {
      motion_intrinsics_enabled = false;
      std::cout << "This device not supported to get motion intrinsics."
          << std::endl;
    }
    #else
        motion_intrinsics_enabled = false;
    #endif

    // pointcloud generator
    pointcloud_generator.reset(new PointCloudGenerator(
        [this](sensor_msgs::PointCloud2 &msg) {
          msg.header.frame_id = points_frame_id;
          pub_points.publish(msg);
        },
        in.ply_parameters,
        params,
        points_factor,
        points_frequency));
  }

  void deleteDevice()
  {
    if (eys3d_depth)
    {
      eys3d_depth->SetIRIntensity(0);
    }
    closeDevice();
    eys3d_depth.reset();
  }

  void closeDevice() {
    if (eys3d_depth) {
      eys3d_depth->Close();
    }
  }

  void publishLeft(const StreamData& data, const ros::Time &timestamp,
      bool color_sub, bool mono_sub) {
    if (skip_tag > 0) {
      if (skip_tmp_left_tag == 0) {
        skip_tmp_left_tag = skip_tag;
      } else {
        skip_tmp_left_tag--;
        return;
      }
    }
    publishColor(data, timestamp, left_info_ptr,
        pub_left_color, color_sub, left_color_frame_id,
        pub_left_mono, mono_sub, left_mono_frame_id, true);
  }

  void publishRight(const StreamData& data, const ros::Time &timestamp,
      bool color_sub, bool mono_sub) {
    if (skip_tag > 0) {
      if (skip_tmp_right_tag == 0) {
        skip_tmp_right_tag = skip_tag;
      } else {
        skip_tmp_right_tag--;
        return;
      }
    }
    publishColor(data, timestamp, right_info_ptr,
        pub_right_color, color_sub, right_color_frame_id,
        pub_right_mono, mono_sub, right_mono_frame_id, false);
  }

  void publishColor(const StreamData& data,
      const ros::Time &timestamp,
      const sensor_msgs::CameraInfoPtr& info,
      const image_transport::CameraPublisher& pub_color, bool color_sub,
      const std::string color_frame_id,
      const image_transport::Publisher& pub_mono, bool mono_sub,
      const std::string mono_frame_id, bool is_left) {

    auto&& mat = data.img->To(ImageFormat::COLOR_BGR)->ToMat();

    if (color_sub) {
      std_msgs::Header header;
      header.stamp = timestamp;
      header.frame_id = color_frame_id;

      auto&& msg = cv_bridge::CvImage(header, enc::BGR8, mat).toImageMsg();
      if (info) {
        info->header.stamp = msg->header.stamp;
        info->header.frame_id = color_frame_id;
      }
      pub_color.publish(msg, info);
    }
    if (mono_sub) {
      std_msgs::Header header;
      header.stamp = timestamp;
      header.frame_id = mono_frame_id;

      cv::Mat dst;
      cv::cvtColor(mat, dst, CV_BGR2GRAY);
      auto&& msg = cv_bridge::CvImage(header, enc::MONO8, dst).toImageMsg();
      pub_mono.publish(msg);
    }
    pthread_mutex_lock(&mutex_sub_result);
    bool sub_result_points = sub_result.points;
    pthread_mutex_unlock(&mutex_sub_result);
    if (is_left && sub_result_points) {
      pthread_mutex_lock(&mutex_color);
      points_color = mat;
      publishPoints(timestamp);
      pthread_mutex_unlock(&mutex_color);
    }
  }

  void publishDepth(const StreamData& data, const ros::Time &timestamp) {
    std_msgs::Header header;
    header.stamp = timestamp;
    header.frame_id = depth_frame_id;

    auto&& info = depth_info_ptr;
    if (info) info->header.stamp = header.stamp;
    if (info) info->header.frame_id = depth_frame_id;
    switch (params.depth_mode){
      case DepthMode::DEPTH_RAW: {
        if (depth_type == 0) {  // type_mono16
          auto &&mat = data.img->To(ImageFormat::DEPTH_RAW)->ToMat();
          pub_depth.publish(cv_bridge::CvImage(header, enc::MONO16, mat).toImageMsg(), info);
        } else if (depth_type == 1) {  // type_16uc1
          auto &&mat = data.img->To(ImageFormat::DEPTH_RAW)->ToMat();
          pub_depth.publish(cv_bridge::CvImage(header, enc::TYPE_16UC1, mat).toImageMsg(), info);
        } else if (depth_type == 2) {  // type_RGB
          Image::pointer depth_rgb = eys3d_depth->GetColorizer()->Process(data.img->To(ImageFormat::DEPTH_RAW), ImageFormat::DEPTH_BGR);
          auto &&mat = depth_rgb->To(ImageFormat::DEPTH_RGB)->ToMat();
          pub_depth.publish(cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg(), info);
        }

        // PLY
        pthread_mutex_lock(&mutex_sub_result);
        bool sub_result_points = sub_result.points;
        pthread_mutex_unlock(&mutex_sub_result);
        if (sub_result_points) {
          pthread_mutex_lock(&mutex_color);
          points_depth = data.img->To(ImageFormat::DEPTH_RAW)->ToMat();
          publishPoints(header.stamp);
          pthread_mutex_unlock(&mutex_color);
        }
        break;
      }
      case DepthMode::DEPTH_GRAY: {
        auto &&mat = data.img->To(ImageFormat::DEPTH_GRAY_24)->ToMat();
        pub_depth.publish(
          cv_bridge::CvImage(header, enc::TYPE_8UC3, mat).toImageMsg(), info);
        break;
      }
      case DepthMode::DEPTH_COLORFUL: {
        auto &&mat = data.img->To(ImageFormat::DEPTH_RGB)->ToMat();
        pub_depth.publish(
          cv_bridge::CvImage(header, enc::RGB8, mat).toImageMsg(), info);
        break;
      }
    }
  }

  void publishPoints(ros::Time stamp) {
    if (params.dev_mode == DeviceMode::DEVICE_ALL && (points_color.empty() || points_depth.empty()))
    {
      return;
    }

    if (params.dev_mode == DeviceMode::DEVICE_DEPTH && points_depth.empty())
    {
      return;
    }
    
    pointcloud_generator->Push(points_color, points_depth, stamp);
    points_color.release();
    points_depth.release();
  }

  void timestampAlign() {
     #ifdef ROS_IMU 
    static bool get_first_acc = false;
    static bool get_first_gyro = false;
    static bool has_gyro = false;
    static ImuData acc;
    static ImuData gyro;

    if (!get_first_acc && imu_accel != nullptr) {
      acc = *imu_accel;
      imu_accel = nullptr;
      get_first_acc = true;
      return;
    }

    if (!get_first_gyro && imu_gyro != nullptr) {
      gyro = *imu_gyro;
      imu_gyro = nullptr;
      get_first_gyro = true;
      return;
    }

    if (imu_accel != nullptr) {
      if (!has_gyro) {
        acc = *imu_accel;
        imu_accel = nullptr;
        return;
      }

      if (acc.timestamp <= gyro.timestamp) {
        ImuData acc_temp;
        acc_temp = *imu_accel;
        acc_temp.timestamp = gyro.timestamp;

        double k = static_cast<double>(imu_accel->timestamp - acc.timestamp);
        k = static_cast<double>(gyro.timestamp - acc.timestamp) / k;

        acc_temp.accel[0] = acc.accel[0] +
          (imu_accel->accel[0] - acc.accel[0]) * k;
        acc_temp.accel[1] = acc.accel[1] +
          (imu_accel->accel[1] - acc.accel[1]) * k;
        acc_temp.accel[2] = acc.accel[2] +
          (imu_accel->accel[2] - acc.accel[2]) * k;

        acc = *imu_accel;
        *imu_accel = acc_temp;
        imu_gyro.reset(new ImuData(gyro));
        has_gyro = false;
        return;
      } else {
        acc = *imu_accel;
        imu_accel = nullptr;
        return;
      }
    }

    if (imu_gyro != nullptr) {
      has_gyro = true;
      gyro = *imu_gyro;
      imu_gyro = nullptr;
      return;
    }
    #endif
  }

  void publishAlignImu(bool imu_sub,
      bool imu_processed_sub, bool temp_sub) {
     #ifdef ROS_IMU 
    if (spec_version <= Version(1, 0)) {
      timestampAlign();
    }

    if (imu_accel == nullptr || imu_gyro == nullptr) {
      return;
    }

    ros::Time stamp = checkUpImuTimeStamp(imu_accel->timestamp);

    if (imu_sub) {
      auto msg = getImuMsgFromData(ros::Time::now(),
          imu_frame_id, *imu_accel, *imu_gyro);
      msg.header.stamp = stamp;
      pub_imu.publish(msg);
    }

    if (motion_intrinsics_enabled && imu_processed_sub) {
      auto data_acc1 = ProcImuTempDrift(*imu_accel);
      auto data_gyr1 = ProcImuTempDrift(*imu_gyro);
      auto data_acc2 = ProcImuAssembly(data_acc1);
      auto data_gyr2 = ProcImuAssembly(data_gyr1);
      auto msg = getImuMsgFromData(ros::Time::now(), imu_frame_processed_id,
          data_acc2, data_gyr2);
      msg.header.stamp = stamp;
      pub_imu_processed.publish(msg);
    }

    if (temp_sub) {
      auto msg = getTempMsgFromData(ros::Time::now(), temp_frame_id,
                                    *imu_accel);
      msg.header.stamp = stamp;
      pub_temp.publish(msg);
    }

    imu_accel = nullptr;
    imu_gyro = nullptr;
    #else
        if (temp_sub) {
            auto msg = getTempMsgFromData(ros::Time::now(), temp_frame_id);
            pub_temp.publish(msg);
        }
    #endif
  }

  void publishImu(bool imu_sub,
      bool imu_processed_sub, bool temp_sub) {
       #ifdef ROS_IMU 
    if (imu_accel == nullptr || imu_gyro == nullptr) {
      return;
    }

    ros::Time stamp = checkUpImuTimeStamp(imu_accel->timestamp);

    if (imu_sub) {
      auto msg = getImuMsgFromData(ros::Time::now(),
          imu_frame_id, *imu_accel, *imu_gyro);
      msg.header.stamp = stamp;
      pub_imu.publish(msg);
    }

    if (motion_intrinsics_enabled && imu_processed_sub) {
      auto data_acc1 = ProcImuTempDrift(*imu_accel);
      auto data_gyr1 = ProcImuTempDrift(*imu_gyro);
      auto data_acc2 = ProcImuAssembly(data_acc1);
      auto data_gyr2 = ProcImuAssembly(data_gyr1);
      auto msg = getImuMsgFromData(ros::Time::now(), imu_frame_processed_id,
          data_acc2, data_gyr2);
      msg.header.stamp = stamp;
      pub_imu_processed.publish(msg);
    }

    if (temp_sub) {
      auto msg = getTempMsgFromData(ros::Time::now(), temp_frame_id,
                                    *imu_accel);
      msg.header.stamp = stamp;
      pub_temp.publish(msg);
    }

    imu_accel = nullptr;
    imu_gyro = nullptr;
    #else
        if (temp_sub) {
            auto msg = getTempMsgFromData(ros::Time::now(), temp_frame_id);
            pub_temp.publish(msg);
        }
    #endif
  }

  sensor_msgs::Imu getImuMsgFromData(ros::Time stamp,
      const std::string& frame_id,
      const ImuData& imu_accel, const ImuData& imu_gyro) {

    sensor_msgs::Imu msg;
#ifdef ROS_IMU 

    // msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.orientation.w = imu_accel.quaternion[0];
    msg.orientation.x = imu_accel.quaternion[1];
    msg.orientation.y = imu_accel.quaternion[2];
    msg.orientation.z = imu_accel.quaternion[3];

    // acceleration should be in m/s^2 (not in g's)
    msg.linear_acceleration.x = imu_accel.accel[0] * gravity;
    msg.linear_acceleration.y = imu_accel.accel[1] * gravity;
    msg.linear_acceleration.z = imu_accel.accel[2] * gravity;

    msg.linear_acceleration_covariance[0] = 0;
    msg.linear_acceleration_covariance[1] = 0;
    msg.linear_acceleration_covariance[2] = 0;

    msg.linear_acceleration_covariance[3] = 0;
    msg.linear_acceleration_covariance[4] = 0;
    msg.linear_acceleration_covariance[5] = 0;

    msg.linear_acceleration_covariance[6] = 0;
    msg.linear_acceleration_covariance[7] = 0;
    msg.linear_acceleration_covariance[8] = 0;

    // velocity should be in rad/sec
    msg.angular_velocity.x = imu_gyro.gyro[0] * M_PI / 180;
    msg.angular_velocity.y = imu_gyro.gyro[1] * M_PI / 180;
    msg.angular_velocity.z = imu_gyro.gyro[2] * M_PI / 180;

    msg.angular_velocity_covariance[0] = 0;
    msg.angular_velocity_covariance[1] = 0;
    msg.angular_velocity_covariance[2] = 0;

    msg.angular_velocity_covariance[3] = 0;
    msg.angular_velocity_covariance[4] = 0;
    msg.angular_velocity_covariance[5] = 0;

    msg.angular_velocity_covariance[6] = 0;
    msg.angular_velocity_covariance[7] = 0;
    msg.angular_velocity_covariance[8] = 0;
#endif
    return msg;
  }
  
#ifdef ROS_IMU 
  dm_preview::Temp getTempMsgFromData(ros::Time stamp,
      const std::string& frame_id, const ImuData& imu_accel) {
#else
    dm_preview::Temp getTempMsgFromData(ros::Time stamp,
          const std::string& frame_id) {
#endif
    dm_preview::Temp msg;
    msg.header.frame_id = frame_id;
     #ifdef ROS_IMU 
    msg.header.stamp = stamp;
    msg.data = imu_accel.temperature;
    #endif
    return msg;
  }

  sensor_msgs::CameraInfoPtr createCameraInfo(const CameraIntrinsics& in) {
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
    sensor_msgs::CameraInfo *camera_info = new sensor_msgs::CameraInfo();
    auto camera_info_ptr = sensor_msgs::CameraInfoPtr(camera_info);

    camera_info->width = in.width;
    camera_info->height = in.height;

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    camera_info->K.at(0) = in.fx;
    camera_info->K.at(2) = in.cx;
    camera_info->K.at(4) = in.fy;
    camera_info->K.at(5) = in.cy;
    camera_info->K.at(8) = 1;

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    camera_info->P.at(0) = in.p[0];
    camera_info->P.at(2) = in.p[2];
    camera_info->P.at(3) = in.p[3];
    camera_info->P.at(5) = in.p[5];
    camera_info->P.at(6) = in.p[6];
    camera_info->P.at(10) = in.p[10];

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

  ros::Time checkUpTimeStamp(std::uint64_t _hard_time, const ImageType &type) {
    static std::map<ImageType, std::uint64_t> hard_time_now;
    static std::map<ImageType, std::uint64_t> image_acc;

    if (is_overflow(_hard_time, hard_time_now[type])) {
      image_acc[type]++;
    }

    hard_time_now[type] = _hard_time;

    return hardTimeToSoftTime(
        image_acc[type] * unit_hard_time + _hard_time);
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

  ImuData ProcImuAssembly(const ImuData& data) const {
    ImuData res = data;
    if (nullptr == motion_intrinsics) {
      std::cout << "[WARNING!] Motion intrinsic "
                << "is not been supported at this device."
                << std::endl;
      return res;
    }
    double dst[3][3] = {0};
    if (data.flag == 1) {
      matrix_3x3(motion_intrinsics->accel.scale,
          motion_intrinsics->accel.assembly, dst);
      double s[3][1] = {0};
      double d[3][1] = {0};
      for (int i = 0; i < 3; i++) {
        s[i][0] = data.accel[i];
      }
      matrix_3x1(dst, s, d);
      for (int i = 0; i < 3; i++) {
        res.accel[i] = d[i][0];
      }
    } else if (data.flag == 2) {
      matrix_3x3(motion_intrinsics->gyro.scale,
          motion_intrinsics->gyro.assembly, dst);
      double s[3][1] = {0};
      double d[3][1] = {0};
      for (int i = 0; i < 3; i++) {
        s[i][0] = data.gyro[i];
      }
      matrix_3x1(dst, s, d);
      for (int i = 0; i < 3; i++) {
        res.gyro[i] = d[i][0];
      }
    } else if (res.flag == 11) {
      res.accel[0] = data.accel[0];
      res.accel[1] = data.accel[1];
      res.accel[2] = data.accel[2];
      res.gyro[0] = data.gyro[0];
      res.gyro[1] = data.gyro[1];
      res.gyro[2] = data.gyro[2];
    }
    return res;
  }

  ImuData ProcImuTempDrift(const ImuData& data) const {
    ImuData res = data;
    if (nullptr == motion_intrinsics) {
      std::cout << "[WARNING!] Motion intrinsic "
                << "is not been supported at this device."
                << std::endl;
      return res;
    }
    double temp = res.temperature;
    if (res.flag == 1) {
      res.accel[0] -= motion_intrinsics->accel.x[1] * temp
          + motion_intrinsics->accel.x[0];
      res.accel[1] -= motion_intrinsics->accel.y[1] * temp
          + motion_intrinsics->accel.y[0];
      res.accel[2] -= motion_intrinsics->accel.z[1] * temp
          + motion_intrinsics->accel.z[0];
    } else if (res.flag == 2) {
      res.gyro[0] -= motion_intrinsics->gyro.x[1] * temp
          + motion_intrinsics->gyro.x[0];
      res.gyro[1] -= motion_intrinsics->gyro.y[1] * temp
          + motion_intrinsics->gyro.y[0];
      res.gyro[2] -= motion_intrinsics->gyro.z[1] * temp
          + motion_intrinsics->gyro.z[0];
    } else if (res.flag == 11) {
      res.accel[0] -= motion_intrinsics->accel.x[1] * temp
          + motion_intrinsics->accel.x[0];
      res.accel[1] -= motion_intrinsics->accel.y[1] * temp
          + motion_intrinsics->accel.y[0];
      res.accel[2] -= motion_intrinsics->accel.z[1] * temp
          + motion_intrinsics->accel.z[0];
      res.gyro[0] -= motion_intrinsics->gyro.x[1] * temp
          + motion_intrinsics->gyro.x[0];
      res.gyro[1] -= motion_intrinsics->gyro.y[1] * temp
          + motion_intrinsics->gyro.y[0];
      res.gyro[2] -= motion_intrinsics->gyro.z[1] * temp
          + motion_intrinsics->gyro.z[0];
    }
    return res;
  }

  void publishMesh() {
    mesh_msg_.header.frame_id = temp_frame_id;
    mesh_msg_.header.stamp = ros::Time::now();
    mesh_msg_.type = visualization_msgs::Marker::MESH_RESOURCE;

    geometry_msgs::Quaternion q;
    double Pie = 3.1416;
    q = tf::createQuaternionMsgFromRollPitchYaw(Pie/2, 0.0, Pie);

    // fill orientation
    mesh_msg_.pose.orientation.x = q.x;
    mesh_msg_.pose.orientation.y = q.y;
    mesh_msg_.pose.orientation.z = q.z;
    mesh_msg_.pose.orientation.w = q.w;

    // fill position
    mesh_msg_.pose.position.x = 0;
    mesh_msg_.pose.position.y = 0;
    mesh_msg_.pose.position.z = 0;

    // scale -- needed
    mesh_msg_.scale.x = 0.02;
    mesh_msg_.scale.y = 0.02;
    mesh_msg_.scale.z = 0.02;

    mesh_msg_.action = visualization_msgs::Marker::ADD;
    mesh_msg_.color.a = 0.5;  // Don't forget to set the alpha!
    mesh_msg_.color.r = 1.0;
    mesh_msg_.color.g = 1.0;
    mesh_msg_.color.b = 1.0;

    // embedded material / colour
    // mesh_msg_.mesh_use_embedded_materials = true;
    if (!mesh_msg_.mesh_resource.empty())
      pub_mesh_.publish(mesh_msg_);  // publish stamped mesh
  }

  bool getParams(
      dm_preview::GetParams::Request &req,     // NOLINT
      dm_preview::GetParams::Response &res) {  // NOLINT
    using Request = dm_preview::GetParams::Request;
    bool in_ok_1, in_ok_2;
    // left_info_ptr
    // right_info_ptr
    switch (req.key) {
      case Request::IMG_INTRINSICS: {
        bool in_ok;
        auto&& in_vga = eys3d_depth->GetStreamIntrinsics(StreamMode::STREAM_1280x480, &in_ok_1);  // NOLINT
        auto&& in_hd = eys3d_depth->GetStreamIntrinsics(StreamMode::STREAM_2560x720, &in_ok_2);  // NOLINT
        if (params.stream_mode == StreamMode::STREAM_1280x480 && in_ok_1) {
          Config intrinsics {
            {"calib_model", "pinhole"},
            {"left", {
              {"width", in_vga.left.width},
              {"height", in_vga.left.height},
              {"fx", in_vga.left.fx},
              {"fy", in_vga.left.fy},
              {"cx", in_vga.left.cx},
              {"cy", in_vga.left.cy},
              {"coeffs", Config::array(
                  {in_vga.left.coeffs[0],
                  in_vga.left.coeffs[1],
                  in_vga.left.coeffs[2],
                  in_vga.left.coeffs[3],
                  in_vga.left.coeffs[4]})},
              {"p", Config::array(
                  {in_vga.left.p[0],in_vga.left.p[1],in_vga.left.p[2],  // NOLINT
                  in_vga.left.p[3],in_vga.left.p[4],in_vga.left.p[5],  // NOLINT
                  in_vga.left.p[6],in_vga.left.p[7],in_vga.left.p[8],  // NOLINT
                  in_vga.left.p[9],in_vga.left.p[10],in_vga.left.p[11]})}  // NOLINT
            }},
            {"right", {
              {"width", in_vga.right.width},
              {"height", in_vga.right.height},
              {"fx", in_vga.right.fx},
              {"fy", in_vga.right.fy},
              {"cx", in_vga.right.cx},
              {"cy", in_vga.right.cy},
              {"coeffs", Config::array(
                  {in_vga.right.coeffs[0],
                  in_vga.right.coeffs[1],
                  in_vga.right.coeffs[2],
                  in_vga.right.coeffs[3],
                  in_vga.right.coeffs[4]})},
              {"p", Config::array(
                  {in_vga.right.p[0],in_vga.right.p[1],in_vga.right.p[2],  // NOLINT
                  in_vga.right.p[3],in_vga.right.p[4],in_vga.right.p[5],  // NOLINT
                  in_vga.right.p[6],in_vga.right.p[7],in_vga.right.p[8],  // NOLINT
                  in_vga.right.p[9],in_vga.right.p[10],in_vga.right.p[11]})}  // NOLINT
            }}
          };
          std::string json = dump_string(intrinsics, JSON);
          res.value = json;
        } else if (params.stream_mode == StreamMode::STREAM_2560x720 &&
            in_ok_2) {
          Config intrinsics {
            {"calib_model", "pinhole"},
            {"left", {
              {"width", in_hd.left.width},
              {"height", in_hd.left.height},
              {"fx", in_hd.left.fx},
              {"fy", in_hd.left.fy},
              {"cx", in_hd.left.cx},
              {"cy", in_hd.left.cy},
              {"coeffs", Config::array(
                  {in_hd.left.coeffs[0],
                  in_hd.left.coeffs[1],
                  in_hd.left.coeffs[2],
                  in_hd.left.coeffs[3],
                  in_hd.left.coeffs[4]})},
              {"p", Config::array(
                  {in_hd.left.p[0],in_hd.left.p[1],in_hd.left.p[2],  // NOLINT
                  in_hd.left.p[3],in_hd.left.p[4],in_hd.left.p[5],  // NOLINT
                  in_hd.left.p[6],in_hd.left.p[7],in_hd.left.p[8],  // NOLINT
                  in_hd.left.p[9],in_hd.left.p[10],in_hd.left.p[11]})}  // NOLINT
            }},
            {"right", {
              {"width", in_hd.right.width},
              {"height", in_hd.right.height},
              {"fx", in_hd.right.fx},
              {"fy", in_hd.right.fy},
              {"cx", in_hd.right.cx},
              {"cy", in_hd.right.cy},
              {"coeffs", Config::array(
                  {in_hd.right.coeffs[0],
                  in_hd.right.coeffs[1],
                  in_hd.right.coeffs[2],
                  in_hd.right.coeffs[3],
                  in_hd.right.coeffs[4]})},
              {"p", Config::array(
                  {in_hd.right.p[0],in_hd.right.p[1],in_hd.right.p[2],  // NOLINT
                  in_hd.right.p[3],in_hd.right.p[4],in_hd.right.p[5],  // NOLINT
                  in_hd.right.p[6],in_hd.right.p[7],in_hd.right.p[8],  // NOLINT
                  in_hd.right.p[9],in_hd.right.p[10],in_hd.right.p[11]})}  // NOLINT
            }}
          };
          std::string json = dump_string(intrinsics, JSON);
          res.value = json;
        } else {
          res.value = "null";
        }
      }
      break;
      case Request::IMG_EXTRINSICS_RTOL: {
        bool ex_ok_1, ex_ok_2;
        auto vga_extrinsics = eys3d_depth->GetStreamExtrinsics(StreamMode::STREAM_1280x480, &ex_ok_1);  // NOLINT
        auto hd_extrinsics = eys3d_depth->GetStreamExtrinsics(StreamMode::STREAM_2560x720, &ex_ok_2);  // NOLINT
        if (params.stream_mode == StreamMode::STREAM_1280x480 && ex_ok_1) {
          Config extrinsics{
            {"rotation",     Config::array({vga_extrinsics.rotation[0][0], vga_extrinsics.rotation[0][1], vga_extrinsics.rotation[0][2],   // NOLINT
                                            vga_extrinsics.rotation[1][0], vga_extrinsics.rotation[1][1], vga_extrinsics.rotation[1][2],   // NOLINT
                                            vga_extrinsics.rotation[2][0], vga_extrinsics.rotation[2][1], vga_extrinsics.rotation[2][2]})},// NOLINT
            {"translation",  Config::array({vga_extrinsics.translation[0], vga_extrinsics.translation[1], vga_extrinsics.translation[2]})} // NOLINT
          };
          std::string json = dump_string(extrinsics, configuru::JSON);
          res.value = json;
        } else if (params.stream_mode == StreamMode::STREAM_2560x720 &&
            ex_ok_2) {
          Config extrinsics{
            {"rotation",     Config::array({hd_extrinsics.rotation[0][0], hd_extrinsics.rotation[0][1], hd_extrinsics.rotation[0][2],   // NOLINT
                                            hd_extrinsics.rotation[1][0], hd_extrinsics.rotation[1][1], hd_extrinsics.rotation[1][2],   // NOLINT
                                            hd_extrinsics.rotation[2][0], hd_extrinsics.rotation[2][1], hd_extrinsics.rotation[2][2]})},// NOLINT
            {"translation",  Config::array({hd_extrinsics.translation[0], hd_extrinsics.translation[1], hd_extrinsics.translation[2]})} // NOLINT
          };
          std::string json = dump_string(extrinsics, configuru::JSON);
          res.value = json;
        } else {
          res.value = "null";
        }
      }
      break;
      case Request::IMU_INTRINSICS:
      {
        bool is_ok;
        auto intri = eys3d_depth->GetMotionIntrinsics(&is_ok);
        if (is_ok) {
          Config intrinsics {
            {"accel", {
              {"scale",     Config::array({ intri.accel.scale[0][0], intri.accel.scale[0][1],  intri.accel.scale[0][2],   // NOLINT
                                            intri.accel.scale[1][0], intri.accel.scale[1][1],  intri.accel.scale[1][2],   // NOLINT
                                            intri.accel.scale[2][0], intri.accel.scale[2][1],  intri.accel.scale[2][2]})},// NOLINT
              {"assembly",  Config::array({ intri.accel.assembly[0][0], intri.accel.assembly[0][1],  intri.accel.assembly[0][2],   // NOLINT
                                            intri.accel.assembly[1][0], intri.accel.assembly[1][1],  intri.accel.assembly[1][2],   // NOLINT
                                            intri.accel.assembly[2][0], intri.accel.assembly[2][1],  intri.accel.assembly[2][2]})},// NOLINT
              {"drift",     Config::array({ intri.accel.drift[0],    intri.accel.drift[1],     intri.accel.drift[2]})}, // NOLINT
              {"noise",     Config::array({ intri.accel.noise[0],    intri.accel.noise[1],     intri.accel.noise[2]})}, // NOLINT
              {"bias",      Config::array({ intri.accel.bias[0],     intri.accel.bias[1],      intri.accel.bias[2]})}, // NOLINT
              {"x",         Config::array({ intri.accel.x[0],        intri.accel.x[1]})}, // NOLINT
              {"y",         Config::array({ intri.accel.y[0],        intri.accel.y[1]})}, // NOLINT
              {"z",         Config::array({ intri.accel.z[0],        intri.accel.z[1]})} // NOLINT
            }},
            {"gyro", {
              {"scale",     Config::array({ intri.gyro.scale[0][0], intri.gyro.scale[0][1],  intri.gyro.scale[0][2],   // NOLINT
                                            intri.gyro.scale[1][0], intri.gyro.scale[1][1],  intri.gyro.scale[1][2],   // NOLINT
                                            intri.gyro.scale[2][0], intri.gyro.scale[2][1],  intri.gyro.scale[2][2]})},// NOLINT
              {"assembly",  Config::array({ intri.gyro.assembly[0][0], intri.gyro.assembly[0][1],  intri.gyro.assembly[0][2],   // NOLINT
                                            intri.gyro.assembly[1][0], intri.gyro.assembly[1][1],  intri.gyro.assembly[1][2],   // NOLINT
                                            intri.gyro.assembly[2][0], intri.gyro.assembly[2][1],  intri.gyro.assembly[2][2]})},// NOLINT
              {"drift",     Config::array({ intri.gyro.drift[0],    intri.gyro.drift[1],     intri.gyro.drift[2]})}, // NOLINT
              {"noise",     Config::array({ intri.gyro.noise[0],    intri.gyro.noise[1],     intri.gyro.noise[2]})}, // NOLINT
              {"bias",      Config::array({ intri.gyro.bias[0],     intri.gyro.bias[1],      intri.gyro.bias[2]})}, // NOLINT
              {"x",         Config::array({ intri.gyro.x[0],        intri.gyro.x[1]})}, // NOLINT
              {"y",         Config::array({ intri.gyro.y[0],        intri.gyro.y[1]})}, // NOLINT
              {"z",         Config::array({ intri.gyro.z[0],        intri.gyro.z[1]})} // NOLINT
            }}
          };
          std::string json = dump_string(intrinsics, JSON);
          res.value = json;
        } else {
          NODELET_INFO_STREAM("INVALID IMU INTRINSICS");
          res.value = "null";
        }
      }
      break;
      case Request::IMU_EXTRINSICS:
      {
        bool is_ok;
        auto extri = eys3d_depth->GetMotionExtrinsics(&is_ok);
        if (is_ok) {
          Config extrinsics{
            {"rotation",     Config::array({extri.rotation[0][0], extri.rotation[0][1], extri.rotation[0][2],   // NOLINT
                                            extri.rotation[1][0], extri.rotation[1][1], extri.rotation[1][2],   // NOLINT
                                            extri.rotation[2][0], extri.rotation[2][1], extri.rotation[2][2]})},// NOLINT
            {"translation",  Config::array({extri.translation[0], extri.translation[1], extri.translation[2]})} // NOLINT
          };
          std::string json = dump_string(extrinsics, configuru::JSON);
          res.value = json;
        } else {
          NODELET_INFO_STREAM("INVALID IMU EXTRINSICS");
          res.value = "null";
        }
      }
      break;
      default:
        res.value = "null";
      break;
    }
    return true;
  }
};

EYS3D_DEPTH_END_NAMESPACE

#include <pluginlib/class_list_macros.h> // NOLINT
PLUGINLIB_EXPORT_CLASS(EYS3D_DEPTH_NAMESPACE::DMPreviewNodelet, nodelet::Nodelet); // NOLINT
