#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

pcl::PolygonMesh reconstructMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PolygonMesh mesh;
  ROS_INFO("Reconstructing mesh (using meshlab server)...");
  const std::string cloud_output_file = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_cloud.ply";
  const std::string mesh_output_file  = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_mesh.ply";
  const std::string meshlab_server_file  = ros::package::getPath("gaitmesh_ros") + "/src/ballPivoting.mlx";
  // save cloud to file
  pcl::io::savePLYFile (cloud_output_file, *cloud);
  // run meshlab server
  std::stringstream cmd;
  cmd << "meshlabserver -i " << cloud_output_file << " -o " << mesh_output_file << " -s " << meshlab_server_file;
  int system_result = system(cmd.str().c_str());
  // load mesh from file
  pcl::io::loadPolygonFilePLY (mesh_output_file, mesh);
  return mesh;
}
