#include <gl_depth_sim/simulator/depth_camera_plugin.h>
#include <gl_depth_sim/interfaces/pcl_interface.h>
#include <gl_depth_sim/interfaces/opencv_interface.h>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>

const static int TIMEOUT = 3;
static std::size_t SEQ = 0;

namespace gl_depth_sim
{
DepthCameraPlugin::DepthCameraPlugin()
  : node_(std::make_shared<rclcpp::Node>("depth_camera_node"))
  , clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
  , buffer_(clock_)
  , listener_(buffer_)
{
}

void DepthCameraPlugin::init(const YAML::Node &config)
{
  fixed_frame_ = config["fixed_frame"].as<std::string>();
  camera_frame_ = config["camera_frame"].as<std::string>();

  props_.width = config["width"].as<int>();
  props_.height = config["height"].as<int>();
  props_.z_near = config["z_near"].as<double>();
  props_.z_far = config["z_far"].as<double>();
  props_.fx = config["fx"].as<double>();
  props_.fy = config["fy"].as<double>();
  props_.cx = config["cx"].as<double>();
  props_.cy = config["cy"].as<double>();

  camera_info_.header.frame_id = camera_frame_;
  camera_info_.height = props_.height;
  camera_info_.width = props_.width;
  camera_info_.k = {props_.fx, 0.0, props_.cx,
                   0.0, props_.fy, props_.cy,
                   0.0, 0.0, 1.0};

  sim_ = std::make_unique<SimDepthCamera>(props_, camera_frame_);

  std::string cloud_topic_name = config["pt_cloud_topic"].as<std::string>();
  std::string depth_image_topic_name = config["depth_image_topic"].as<std::string>();
  std::string color_image_info_topic_name = config["color_image_topic"].as<std::string>();
  std::string color_image_topic_name = config["camera_info_topic"].as<std::string>();
  cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_name, 1);
  depth_img_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(depth_image_topic_name, 1);
  camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(color_image_info_topic_name, 1);
  color_img_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(color_image_topic_name, 1);
}

void DepthCameraPlugin::render(const std::map<std::string, RenderableObjectState> &scene,
                               const rclcpp::Time time)
{
  // Get camera tranform
  geometry_msgs::msg::TransformStamped camera_transform
    = buffer_.lookupTransform(fixed_frame_, camera_frame_, time, tf2::Duration(std::chrono::seconds(TIMEOUT)));

  Eigen::Isometry3d camera_pose = tf2::transformToEigen(camera_transform);

  // Render the depth image
  gl_depth_sim::DepthImage depth_img = sim_->render(camera_pose, scene);

  // Publish depth image
  cv::Mat cv_depth_img;
  gl_depth_sim::toCvImage16u(depth_img, cv_depth_img);
  sensor_msgs::msg::Image depth_img_msg;
  depth_img_msg.header.frame_id = camera_frame_;
  cv_bridge::CvImage cv_depth_img2(depth_img_msg.header, sensor_msgs::image_encodings::TYPE_16UC1, cv_depth_img);
  cv_depth_img2.toImageMsg(depth_img_msg);

  // Convert depth to color image
  sensor_msgs::msg::Image mono_img_msg = depth_img_msg;
  mono_img_msg.encoding = "mono16";
  cv_bridge::CvImagePtr mono_ptr = cv_bridge::toCvCopy(mono_img_msg, sensor_msgs::image_encodings::MONO16);
  cv::Mat img_n;
  cv::normalize(mono_ptr->image, img_n, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::Mat cv_color_img;
  cv::cvtColor(img_n, cv_color_img, CV_GRAY2BGR);
  sensor_msgs::msg::Image color_img_msg;
  color_img_msg.header.frame_id = camera_frame_;
  cv_bridge::CvImage cv_color_img2(color_img_msg.header, sensor_msgs::image_encodings::BGR8, cv_color_img);
  cv_color_img2.toImageMsg(color_img_msg);

  depth_img_msg.header.frame_id = camera_frame_;
  depth_img_msg.header.stamp = time;
  color_img_msg.header = depth_img_msg.header;
  camera_info_.header = depth_img_msg.header;

  depth_img_pub_->publish(depth_img_msg);
  color_img_pub_->publish(color_img_msg);

  // Publish camera info
  camera_info_pub_->publish(camera_info_);

  // Convert the depth image to point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  gl_depth_sim::toPointCloudXYZ(props_, depth_img, cloud);

  // Publish the cloud
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = depth_img_msg.header;
  cloud_pub_->publish(msg);
}

} // namespace gl_depth_sim

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gl_depth_sim::DepthCameraPlugin, gl_depth_sim::RenderPlugin);
