// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include <CameraApi.h>
#include <rmw/qos_profiles.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>

namespace mindvision_camera
{
class MVCameraNode : public rclcpp::Node
{
public:
  MVCameraNode(const rclcpp::NodeOptions & options) : Node("MVCamera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting MVCameraNode!");

    int i_camera_counts = 1;
    int i_status = -1;
    tSdkCameraDevInfo t_camera_enum_list;
    tSdkCameraCapbility t_capability;  //设备描述信息

    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
    RCLCPP_INFO(this->get_logger(), "state = %d\n", i_status);

    RCLCPP_INFO(this->get_logger(), "count = %d\n", i_camera_counts);
    // 没有连接设备
    if (i_camera_counts == 0) {
      return;
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

    // 初始化失败
    printf("state = %d\n", i_status);
    if (i_status != CAMERA_STATUS_SUCCESS) {
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

    camera_name_ = this->declare_parameter("camera_name", "MVCamera");
    camera_pub_ =
      image_transport::create_camera_publisher(this, camera_name_, rmw_qos_profile_sensor_data);

    capture_thread_ = std::thread{[this]() -> void {
      while (rclcpp::ok()) {
        if (
          CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000) ==
          CAMERA_STATUS_SUCCESS) {
          CameraImageProcess(h_camera_, pby_buffer_, g_pRgbBuffer, &s_frame_info_);

          image_msg_.header.stamp = this->now();
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

private:
  uint8_t * g_pRgbBuffer;  // 处理后数据缓存区
  int h_camera_;
  tSdkFrameHead s_frame_info_;
  BYTE * pby_buffer_;

  std::string camera_name_;
  image_transport::CameraPublisher camera_pub_;
  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::thread capture_thread_;
};

}  // namespace mindvision_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mindvision_camera::MVCameraNode)