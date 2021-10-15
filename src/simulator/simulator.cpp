#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <gl_depth_sim/simulator/simulator_plugins.h>
#include <pluginlib/class_loader.hpp>
#include <gl_depth_sim/simulator/depth_camera_plugin.h>
#include <gl_depth_sim/simulator/laser_scanner_plugin.h>
#include <gl_depth_sim/simulator/ros_urdf_scene_updater.h>

#include <GLFW/glfw3.h>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class Simulator : public rclcpp::Node
{
public:
  Simulator()
    : Node("gl_depth_simulation")
    , scene_update_plugin_loader_("gl_depth_sim", "gl_depth_sim::SceneUpdaterPlugin")
    , render_plugin_loader_("gl_depth_sim", "gl_depth_sim::RenderPlugin")
  {
    this->declare_parameter("config_file_path");
    this->declare_parameter("urdf_file_path");

    std::string yaml_config_fp = this->get_parameter("config_file_path").as_string();
    YAML::Node yaml_config = YAML::LoadFile(yaml_config_fp);

    double render_rate = yaml_config["render_rate"].as<double>();

    std::string scene_plugin_type = yaml_config["scene_update_plugin"]["type"].as<std::string>();

    auto render_plugin = yaml_config["render_plugins"];

    scene_update_plugin_ = scene_update_plugin_loader_.createSharedInstance(scene_plugin_type);

    YAML::Node scene_plugin_params = yaml_config["scene_update_plugin"]["params"];
    YAML::Node urdf_file_path_yaml;
    std::string urdf_fp = this->get_parameter("urdf_file_path").as_string();
    urdf_file_path_yaml["urdf_file_path"] = urdf_fp;
    scene_update_plugin_->init(urdf_file_path_yaml);

    render_plugin_ = render_plugin_loader_.createSharedInstance(render_plugin["type"].as<std::string>());
    render_plugin_->init(render_plugin["params"]);

    // Create the scene
    scene_update_plugin_->createScene();

    // Create a timer for the updates
    render_timer_ = this->create_wall_timer(rclcpp::Rate(render_rate).period(), std::bind(&Simulator::renderTimerCallback, this));
  }

  private:

  void renderTimerCallback()
  {
      auto now = this->get_clock()->now();
      scene_update_plugin_->updateScene(now);
    render_plugin_->render(scene_update_plugin_->getScene(), now);
  }

  pluginlib::ClassLoader<gl_depth_sim::SceneUpdaterPlugin> scene_update_plugin_loader_;
  pluginlib::ClassLoader<gl_depth_sim::RenderPlugin> render_plugin_loader_;

  gl_depth_sim::SceneUpdaterPlugin::Ptr scene_update_plugin_;
  std::vector<gl_depth_sim::RenderPlugin::Ptr> render_plugins_;
  gl_depth_sim::RenderPlugin::Ptr render_plugin_;

  rclcpp::TimerBase::SharedPtr scene_timer_;
  rclcpp::TimerBase::SharedPtr render_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();

  return 0;
}

