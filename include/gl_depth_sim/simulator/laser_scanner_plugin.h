#ifndef GL_DEPTH_SIM_SIMULATOR_LASER_SCANNER_PLUGIN_H
#define GL_DEPTH_SIM_SIMULATOR_LASER_SCANNER_PLUGIN_H

#include <gl_depth_sim/simulator/simulator_plugins.h>
#include <gl_depth_sim/sim_laser_scanner.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace gl_depth_sim
{
/**
 * @brief Laser scanner implementation of the render plugin
 */
class LaserScannerPlugin : public RenderPlugin
{
public:
  LaserScannerPlugin();

  virtual void init(const YAML::Node &config) override;
  virtual void render(const std::map<std::string, RenderableObjectState>& scene,
                      const rclcpp::Time time) override;

private:
  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  std::string fixed_frame_;
  std::string scanner_frame_;

  gl_depth_sim::LaserScannerProperties props_;
  std::unique_ptr<gl_depth_sim::SimLaserScanner> sim_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::shared_ptr<rclcpp::Node> node_;
};

} // namespace gl_depth_sim

#endif // GL_DEPTH_SIM_SIMULATOR_LASER_SCANNER_PLUGIN_H
