#include <gl_depth_sim/simulator/laser_scanner_plugin.h>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

const static int TIMEOUT = 3;
static std::size_t SEQ = 0;

namespace gl_depth_sim
{

LaserScannerPlugin::LaserScannerPlugin()
  : node_(std::make_shared<rclcpp::Node>("laser_scanner_node"))
  , clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
  , buffer_(clock_)
  , listener_(buffer_)
{

}

void LaserScannerPlugin::init(const YAML::Node& config)
{
  fixed_frame_ = config["fixed_frame"].as<std::string>();
  scanner_frame_ = config["camera_frame"].as<std::string>();

  props_.max_range = config["max_range"].as<double>();
  props_.min_range = config["min_range"].as<double>();
  props_.angular_resolution = config["angular_resolution"].as<double>();

  sim_ = std::make_unique<SimLaserScanner>(props_, scanner_frame_);

  std::string topic_name = config["topic"].as<std::string>();
  pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 1);
}

void LaserScannerPlugin::render(const std::map<std::string, RenderableObjectState>& scene,
                                const rclcpp::Time time)
{
  // Get camera tranform
  geometry_msgs::msg::TransformStamped scanner_transform
      = buffer_.lookupTransform(fixed_frame_, scanner_frame_, time, tf2::Duration(std::chrono::seconds(TIMEOUT)));

  Eigen::Isometry3d camera_pose = tf2::transformToEigen(scanner_transform);

  // Render the depth image
  pcl::PointCloud<pcl::PointXYZ> cloud = sim_->render(camera_pose, scene);

  // Publish the cloud
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = scanner_frame_;
  msg.header.stamp = node_->get_clock()->now();
  pub_->publish(msg);
}

}  // namespace gl_depth_sim

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gl_depth_sim::LaserScannerPlugin, gl_depth_sim::RenderPlugin);
