#ifndef GL_DEPTH_SIM_SIMULATOR_DEPTH_CAMERA_PLUGIN_H
#define GL_DEPTH_SIM_SIMULATOR_DEPTH_CAMERA_PLUGIN_H

#include <rclcpp/rclcpp.hpp>

#include <gl_depth_sim/simulator/simulator_plugins.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

namespace gl_depth_sim
{
/**
 * @brief Depth camera implementation of the render plugin
 */
class DepthCameraPlugin : public RenderPlugin
{
public:
  DepthCameraPlugin();

  virtual void init(const YAML::Node &config) override;
  virtual void render(const std::map<std::string, RenderableObjectState>& scene,
                      const rclcpp::Time time) override;

private:
  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  std::string fixed_frame_;
  std::string camera_frame_;

  gl_depth_sim::CameraProperties props_;
  std::unique_ptr<gl_depth_sim::SimDepthCamera> sim_;

  sensor_msgs::msg::CameraInfo camera_info_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_img_pub_;

  std::shared_ptr<rclcpp::Node> node_;
};

} // namespace gl_depth_sim

#endif // GL_DEPTH_SIM_SIMULATOR_DEPTH_CAMERA_PLUGIN_H
