#include <gl_depth_sim/simulator/ros_urdf_scene_updater.h>
#include <gl_depth_sim/mesh_loader.h>

#include <parallel/algorithm>
#include <regex>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/exceptions.h>
#include <fstream>

const static int TIMEOUT = 3;
const static std::string URDF_PARAM = "robot_description";

namespace
{
bool resolveURI(const std::string& in, const std::string& uri, std::string& out)
{
  std::regex expression(uri + "(\\w*)(\\/.*)");
  if (std::regex_match(in, expression))
  {
    std::smatch matches;
    std::regex_search(in, matches, expression);

    out = ament_index_cpp::get_package_share_directory(matches[1].str());
    out += matches[2].str();

    return true;
  }

  return false;
}

Eigen::Isometry3d poseURDFToEigen(const urdf::Pose& pose)
{
  Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);

  Eigen::Isometry3d out(Eigen::Isometry3d::Identity());
  out.translate(translation);
  out.rotate(rotation);

  return out;
}

}  // namespace

namespace gl_depth_sim
{
ROSURDFSceneUpdaterPlugin::ROSURDFSceneUpdaterPlugin()
  : node_(std::make_shared<rclcpp::Node>("URDF_scene_updater"))
  , clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
  , buffer_(clock_)
  , listener_(buffer_)
{
}

void ROSURDFSceneUpdaterPlugin::init(const YAML::Node &config)
{
  std::string urdf_fp = config["urdf_file_path"].as<std::string>();
  if (!model_.initFile(urdf_fp))
  {
    throw std::runtime_error("Failed to parse urdf file");
  }
  RCLCPP_INFO(node_->get_logger(),"Successfully parsed urdf file");
}

void ROSURDFSceneUpdaterPlugin::createScene(/*gl_depth_sim::SimDepthCamera &sim_*/)
{
  fixed_frame_ = model_.getRoot()->name;

  // Iterate through all of the links and add the visual geometries
  for (auto it = model_.links_.begin(); it != model_.links_.end(); it++)
  {
    const auto& link = model_.getLink(it->first);

    // Only include links with geometry
    if (link && link->visual_array.size() > 0)
    {
      // Create a container for all of the visuals defined in this link
      std::vector<std::unique_ptr<gl_depth_sim::Mesh>> visuals;
      visuals.reserve(link->visual_array.size());

      // Iterate over all of the visuals in this link
      for (std::size_t i = 0; i < link->visual_array.size(); ++i)
      {
        std::unique_ptr<gl_depth_sim::Mesh> mesh;
        switch (link->visual_array[i]->geometry->type)
        {
          case urdf::Geometry::MESH:
          {
            // Attempt to cast the visual pointer to a mesh
            const urdf::MeshConstSharedPtr tmp =
                urdf::dynamic_pointer_cast<const urdf::Mesh>(link->visual_array[i]->geometry);

            if (tmp)
            {
              // Get filepath and link name
              std::string filepath = tmp->filename;
              std::string link_name = link->name;  // This is also the name of the tf associated with this link

              // Exclude unsupported filetypes
              // dae files are still broken as of 8/21/18. The internal transforms are not properly imported by assimp
              // in gl_depth_sim
              if (filepath.substr(filepath.size() - 3) == "DAE" || filepath.substr(filepath.size() - 3) == "dae")
              {
                throw std::runtime_error("DAE files are currently unsupported");
              }

              // Load the object's mesh
              std::string mesh_filename;
              if (!resolveURI(filepath, "package://", mesh_filename))
              {
                if (!resolveURI(filepath, "file://", mesh_filename))
                {
                  mesh_filename = filepath;
                }
              }

              visuals.emplace_back(gl_depth_sim::loadMesh(mesh_filename));
            }

            break;
          }
          default:
            // TODO: add support for geometry primitives
            RCLCPP_WARN(node_->get_logger(), "Visual geometry other than meshes are not currently handled");
            break;
        }
      }

      // Create a single mesh from all of the vertices and indices of the visuals of this link
      std::unique_ptr<Mesh> mesh;
      {
        EigenAlignedVec<Eigen::Vector3f> vertices;
        std::vector<unsigned> indices;
        for (const auto& visual : visuals)
        {
          // Check that the visual was loaded correctly
          if (!visual)
            throw std::runtime_error("Failed to load visual mesh for link '" + link->name + "'");

          // Add the vertices directly
          vertices.insert(vertices.end(), visual->vertices().begin(), visual->vertices().end());

          // Offset this mesh's indices by the current size of the indices vector
          std::vector<unsigned> updated_mesh_indices(visual->indices());
          std::transform(updated_mesh_indices.begin(), updated_mesh_indices.end(),
                         updated_mesh_indices.begin(), [&indices](unsigned v) -> unsigned { return v + indices.size(); });

          indices.insert(indices.end(), updated_mesh_indices.begin(), updated_mesh_indices.end());
        }
        mesh = std::make_unique<gl_depth_sim::Mesh>(vertices, indices);
      }

      // Get the object's position relative to the fixed frame
      geometry_msgs::msg::TransformStamped mesh_transform =
          buffer_.lookupTransform(fixed_frame_, link->name, tf2::TimePointZero, tf2::Duration(std::chrono::seconds(TIMEOUT)));
      Eigen::Isometry3d pose = tf2::transformToEigen(mesh_transform);

      // Post-multiply the pose of the relative transform of the mesh points to its origin
      Eigen::Isometry3d relative_pose = poseURDFToEigen(link->visual->origin);
      relative_poses_.emplace(link->name, relative_pose);

      // Create the renderable object state
      gl_depth_sim::RenderableObjectState object;
      object.mesh = std::make_shared<gl_depth_sim::RenderableMesh>(*mesh);
      object.pose = pose * relative_pose;

      scene_.emplace(link->name, object);
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Added mesh for link '" << link->name << "'");
    }
  }
}

void ROSURDFSceneUpdaterPlugin::updateScene(const rclcpp::Time time)
{
  // Create a function that updates the position of each renderable object
  auto update_fn = [this, time](std::pair<const std::string, RenderableObjectState> &pair) -> void {
    // Look up the transform to the object
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = buffer_.lookupTransform(fixed_frame_,
                                          pair.first,
                                          time,
                                          tf2::Duration(std::chrono::seconds(TIMEOUT)));

      Eigen::Isometry3d pose = tf2::transformToEigen(transform);

      // Apply the relative pose of the visual geometry
      pair.second.pose = pose * relative_poses_.at(pair.first);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), ex.what());
    }
  };

  // Update the transforms of the meshes in the environment
  __gnu_parallel::for_each(scene_.begin(), scene_.end(), update_fn);
}


}  // namespace amsted_vision_processing

#include <pluginlib/class_list_macros.hpp>
//#include <class_loader/register_macro.hpp>
PLUGINLIB_EXPORT_CLASS(gl_depth_sim::ROSURDFSceneUpdaterPlugin, gl_depth_sim::SceneUpdaterPlugin);
