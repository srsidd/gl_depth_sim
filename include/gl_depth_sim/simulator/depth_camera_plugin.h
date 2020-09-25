#ifndef GL_DEPTH_SIM_SIMULATOR_DEPTH_CAMERA_PLUGIN_H
#define GL_DEPTH_SIM_SIMULATOR_DEPTH_CAMERA_PLUGIN_H

#include <gl_depth_sim/simulator/simulator_plugins.h>
#include <tf2_ros/transform_listener.h>

namespace gl_depth_sim
{
/**
 * @brief Depth camera implementation of the render plugin
 */
class DepthCameraPlugin : public RenderPlugin
{
public:
  DepthCameraPlugin();

  virtual void init(const XmlRpc::XmlRpcValue &config) override;
  virtual void render(const std::map<std::string, RenderableObjectState>& scene) override;

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  std::string fixed_frame_;
  std::string camera_frame_;

  gl_depth_sim::CameraProperties props_;
  std::unique_ptr<gl_depth_sim::SimDepthCamera> sim_;

  ros::Publisher pub_;
};

} // namespace gl_depth_sim

#endif // GL_DEPTH_SIM_SIMULATOR_DEPTH_CAMERA_PLUGIN_H
