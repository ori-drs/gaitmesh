#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl/PclMeshSampling.h"  // Local copy

#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/auto_io.h>

#include <gaitmesh_ros/gaitmesh_ros.hpp>
#include <gaitmesh_ros/common.hpp>
#include <gaitmesh_ros/lvr2_reconstruction.hpp>
#include <gaitmesh_ros/meshlab_reconstruction.hpp>

using namespace gaitmesh;  // temporary

//--------------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "annotate_cloud_gaits");
  ros::NodeHandle nodeHandle("~");
  ros::Publisher pub1 = nodeHandle.advertise<sensor_msgs::PointCloud2>("step1", 1, true);
  ros::Publisher pub2 = nodeHandle.advertise<sensor_msgs::PointCloud2>("step2", 1, true);
  ros::Publisher pub3 = nodeHandle.advertise<sensor_msgs::PointCloud2>("step3", 1, true);
  ros::Publisher pub4 = nodeHandle.advertise<sensor_msgs::PointCloud2>("step4", 1, true);

  // parameters (input)
  bool input_clouds_not_mesh;
  bool mesh_labels_from_texture;
  double radius_normals;
  double radius_robot;
  double curvature_threshold;
  double mesh_sampling_resolution;
  std::string path_mesh;
  std::string path1;
  std::string path2;
  std::string path3;
  std::string path4;
  std::string path_cloud_mesh;
  nodeHandle.param("input_clouds_not_mesh", input_clouds_not_mesh, true);
  nodeHandle.param("mesh_labels_from_texture", mesh_labels_from_texture, false);
  nodeHandle.param("radius_normals", radius_normals, 0.20);
  nodeHandle.param("radius_robot", radius_robot, 0.60);
  nodeHandle.param("curvature_threshold", curvature_threshold, 0.06);
  nodeHandle.param("mesh_sampling_resolution", mesh_sampling_resolution, 0.05);
  nodeHandle.param("mesh", path_mesh, std::string(""));
  nodeHandle.param("cloud1", path1, std::string(""));
  nodeHandle.param("cloud2", path2, std::string(""));
  nodeHandle.param("cloud3", path3, std::string(""));
  nodeHandle.param("cloud4", path4, std::string(""));
  nodeHandle.param("cloud_mesh", path_cloud_mesh, std::string(""));

  std::vector<std::string> paths;
  if (input_clouds_not_mesh) {
    if (path1 != "") paths.push_back(path1);
    if (path2 != "") paths.push_back(path2);
    if (path3 != "") paths.push_back(path3);
    if (path4 != "") paths.push_back(path4);
    if (paths.size() == 0) {
      ROS_ERROR("No point cloud provided");
      return 0;
    }
  }

  // first step is getting the input cloud and mesh
  pcl::PointCloud<pcl::PointXYZ>::Ptr step1 ( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PolygonMesh mesh;

  if (input_clouds_not_mesh) {

    // load and join clouds
    ROS_INFO("Loading clouds...");
    for (unsigned int i = 0; i < paths.size(); i++) {
      ROS_INFO("Processing cloud %d", i);
      // load
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ>() );
      pcl::io::loadPLYFile (paths[i], *cloud);
      // join
      *step1 += *cloud;
    }

    // load or build cloud
    if (path_cloud_mesh != "") {
      // load mesh from file
      pcl::io::loadPolygonFileOBJ (path_cloud_mesh, mesh);
    } else {
      // mesh reconstruction
      // mesh = reconstructMeshLasVegasReconstruction(step1);
      mesh = reconstructMeshMeshlab(step1);
    }

  } else if (mesh_labels_from_texture) {

      // load textured mesh
      pcl::TextureMesh tex_mesh;
      pcl::io::load (path_mesh, tex_mesh);

      // init output mesh
      pcl::PolygonMesh mesh;
      mesh.header = tex_mesh.header;
      mesh.cloud = tex_mesh.cloud;

      // get labels from texture
      std::vector<char> mesh_labels;
      mesh_labels.reserve(mesh.polygons.size());
      for (int i = 0; i < tex_mesh.tex_polygons.size(); i++) {
        int id = (i % 19) + 1;
        ROS_INFO("Count[%d] = %d", id, (int)tex_mesh.tex_polygons[i].size());
        mesh.polygons.insert(mesh.polygons.end(), tex_mesh.tex_polygons[i].begin(), tex_mesh.tex_polygons[i].end());
        for (int j = 0; j < tex_mesh.tex_polygons[i].size(); j++) {
          mesh_labels.push_back((char)id);
        }
      }

      // save annotated mesh
      const std::string mesh_output_file  = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_mesh.obj";
      const std::string area_output_file  = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_mesh.dat";
      ROS_INFO("Saving mesh as OBJ...");
      pcl::io::saveOBJFile(mesh_output_file, mesh);
      ROS_INFO("Saving mesh annotations...");
      saveAreas(area_output_file, mesh_labels);
      return 0;

  } else {

    ROS_INFO("Loading mesh...");
    pcl::io::loadPolygonFile (path_mesh, mesh);
    ROS_INFO("Sampling mesh...");
    step1 = getCloudFromMesh (mesh, mesh_sampling_resolution);

  }

  // Annotate point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr step2(new pcl::PointCloud<pcl::PointXYZ>());  // outlier_removal_cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr step3(new pcl::PointCloud<pcl::PointNormal>());  // normal_cloud
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr step4(new pcl::PointCloud<pcl::PointXYZRGBNormal>());  // gait_cloud
  std::vector<char> mesh_labels = annotateCloud(step1, mesh, radius_normals, radius_robot, curvature_threshold, step2, step3, step4);

  // Save annotated mesh
  if (true) {
    const std::string mesh_output_file  = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_mesh.obj";
    const std::string area_output_file  = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_mesh.dat";
    ROS_INFO("Saving mesh as OBJ...");
    pcl::io::saveOBJFile(mesh_output_file, mesh);
    ROS_INFO("Saving mesh annotations...");
    saveAreas(area_output_file, mesh_labels);
  }

  // convert
  sensor_msgs::PointCloud2 msg1;
  pcl::toROSMsg(*step1.get(), msg1);
  msg1.header.frame_id = "map";

  sensor_msgs::PointCloud2 msg2;
  pcl::toROSMsg(*step2.get(), msg2);
  msg2.header.frame_id = "map";

  sensor_msgs::PointCloud2 msg3;
  pcl::toROSMsg(*step3.get(), msg3);
  msg3.header.frame_id = "map";

  sensor_msgs::PointCloud2 msg4;
  pcl::toROSMsg(*step4.get(), msg4);
  msg4.header.frame_id = "map";

  // publish loop
  ros::Rate rate(1.0); // hz
  while (nodeHandle.ok()) {
    // publish
    pub1.publish(msg1);
    pub2.publish(msg2);
    pub3.publish(msg3);
    pub4.publish(msg4);
    ROS_INFO("Published cloud");
    // wait
    rate.sleep();
  }

  return 0;
}

