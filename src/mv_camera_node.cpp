// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include <CameraApi.h>

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace mindvision_camera
{
class MVCameraNode : public rclcpp::Node
{
public:
  explicit MVCameraNode(const rclcpp::NodeOptions & options) : Node("MVCamera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting MVCameraNode!");

    int i_camera_counts = 1;
    int i_status = -1;
    tSdkCameraDevInfo t_camera_enum_list;
    tSdkCameraCapbility t_capability;  // 设备描述信息

    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
    RCLCPP_INFO(this->get_logger(), "Enumerate state = %d", i_status);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", i_camera_counts);

    // 没有连接设备
    if (i_camera_counts == 0) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      return;
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

    // 初始化失败
    RCLCPP_INFO(this->get_logger(), "Init state = %d", i_status);
    if (i_status != CAMERA_STATUS_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Init failed!");
      return;
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(h_camera_, &t_capability);

    g_pRgbBuffer = (unsigned char *)malloc(
      t_capability.sResolutionRange.iHeightMax * t_capability.sResolutionRange.iWidthMax * 3);

    // 让SDK进入工作模式，开始接收来自相机发送的图像
    // 数据。如果当前相机是触发模式，则需要接收到
    // 触发帧以后才会更新图像。
    CameraPlay(h_camera_);

    CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_BGR8);

    // Create camera publisher
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "mv_camera");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter(
      "camera_info_url", "package://mindvision_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    capture_thread_ = std::thread{[this]() -> void {
      RCLCPP_INFO(this->get_logger(), "Publishing image!");
      while (rclcpp::ok()) {
        if (
          CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000) ==
          CAMERA_STATUS_SUCCESS) {
          CameraImageProcess(h_camera_, pby_buffer_, g_pRgbBuffer, &s_frame_info_);

          camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
          image_msg_.encoding = "rgb8";
          image_msg_.height = s_frame_info_.iHeight;
          image_msg_.width = s_frame_info_.iWidth;
          image_msg_.step = s_frame_info_.iWidth * 3;
          image_msg_.data = std::vector<uint8_t>(
            g_pRgbBuffer, g_pRgbBuffer + s_frame_info_.iWidth * s_frame_info_.iHeight * 3);

          camera_pub_.publish(image_msg_, camera_info_msg_);

          // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
          // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，
          // 直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
          CameraReleaseImageBuffer(h_camera_, pby_buffer_);
        }
      }
    }};
  }

  ~MVCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }

    CameraUnInit(h_camera_);
    free(g_pRgbBuffer);
  }

private:
  int h_camera_;
  uint8_t * pby_buffer_;
  uint8_t * g_pRgbBuffer;  // 处理后数据缓存区
  tSdkFrameHead s_frame_info_;

  image_transport::CameraPublisher camera_pub_;
  sensor_msgs::msg::Image image_msg_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::thread capture_thread_;
};

}  // namespace mindvision_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mindvision_camera::MVCameraNode)
