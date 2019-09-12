#include "PclMeshSampling.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

bool saveAreas(const std::string& path, const std::vector<char>& labels)
{
  std::ofstream FILE(path, std::ios::out | std::ofstream::binary);
  std::copy(labels.begin(), labels.end(), std::ostreambuf_iterator<char>(FILE));
  FILE.close();
  return true;
}

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
  std::string path_mesh;
  std::string path1;
  std::string path2;
  std::string path3;
  std::string path4;
  std::string path_cloud_mesh;
  nodeHandle.param("input_clouds_not_mesh", input_clouds_not_mesh, true);
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
      ROS_INFO("Reconstructing mesh (using lvr2_reconstruct)...");
      const std::string cloud_output_file = ros::package::getPath("sim_environments") + "/data/annotate_cloud_gaits_cloud.ply";
      const std::string mesh_output_file  = ros::package::getPath("sim_environments") + "/data/annotate_cloud_gaits_mesh.ply";
      // save cloud to file
      pcl::io::savePLYFile (cloud_output_file, *step1);
      // run lvr2_reconstruct
      std::stringstream cmd;
      cmd << "lvr2_reconstruct " << cloud_output_file << " -v 0.15 -r --outputFile " << mesh_output_file;
      int system_result = system(cmd.str().c_str());
      // load mesh from file
      pcl::io::loadPolygonFilePLY (mesh_output_file, mesh);
    }

  } else {

    ROS_INFO("Loading mesh...");
    pcl::io::loadPolygonFileOBJ (path_mesh, mesh);
    ROS_INFO("Sampling mesh...");
    step1 = getCloudFromMesh (mesh, 0.05);

  }

  // outlier filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr step2 ( new pcl::PointCloud<pcl::PointXYZ>() );
  if (false) {
    ROS_INFO("Removing outliers...");
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (step1);
    sor.setRadiusSearch (0.2);
    sor.setMinNeighborsInRadius (20);
    sor.filter (*step2);
  } else {
    *step2 = *step1;
  }

  // run terrain-related filters
  pcl::PointCloud<pcl::PointNormal>::Ptr step3 (new pcl::PointCloud<pcl::PointNormal>);
  {
    ROS_INFO("Estimating normals...");
    const float radius = 0.20;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (step2);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (radius);
    ne.compute (*normals);
    pcl::concatenateFields (*step2, *normals, *step3);
  }
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr step4 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  {
    ROS_INFO("Running filters...");
    const float radius = 0.60;
    step4->points.reserve(step3->points.size());
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
    tree->setInputCloud (step3);
    int pnumber = (int)step3->size ();
    for (int point_id = 0; point_id < pnumber; ++point_id) {
      // max curvature within radius
      std::vector<int> k_indices;
      std::vector<float> k_distances;
      tree->radiusSearch (point_id, radius, k_indices, k_distances);
      float max_curvature = 0;
      for (size_t n_id = 0; n_id < k_indices.size (); ++n_id) {
        float id = k_indices.at (n_id);
        float curvature = step3->points[id].curvature;
        if (curvature > max_curvature)
          max_curvature = curvature;
      }
      // add point
      pcl::PointXYZRGBNormal pt;
      pt.x = step3->points[point_id].x;
      pt.y = step3->points[point_id].y;
      pt.z = step3->points[point_id].z;
      pt.normal[0] = step3->points[point_id].normal[0];
      pt.normal[1] = step3->points[point_id].normal[1];
      pt.normal[2] = step3->points[point_id].normal[2];
      pt.curvature = step3->points[point_id].curvature;
      pt.r = (max_curvature < 0.06 ? 128 : 0); // walk
      pt.g = (max_curvature < 0.06 ? 128 : 0); // trot
      pt.b = 128; // step
      pt.a = 255;
      step4->points.push_back(pt);
    }
  }

  // back-project features to mesh
  std::vector<char> mesh_labels(mesh.polygons.size(), (char)0);
  {
    ROS_INFO("Back-projecting annotations to mesh...");
    // tree
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
    tree->setInputCloud (step4);
    // go through all mesh triangles
    pcl::PointCloud<pcl::PointXYZRGBNormal> mesh_cloud;
    pcl::fromPCLPointCloud2 (mesh.cloud, mesh_cloud);
    for (int i = 0; i < mesh.polygons.size(); i++) {
      // get closest annotated-cloud point
      int id = mesh.polygons[i].vertices[0];
      const pcl::PointXYZRGBNormal& pt = mesh_cloud.points[id];
      std::vector<int> k_indices;
      std::vector<float> k_distances;
      tree->nearestKSearch (pt, 1, k_indices, k_distances);
      // assign label to triangle
      const pcl::PointXYZRGBNormal& nn = step4->points[ k_indices[0] ];
      char label = 2;
      if (nn.g > 0)
        label = 1; // trot
      else if (nn.b > 0)
        label = 2; // step
      else
        label = 0; // unwalkable
      mesh_labels[i] = label;
    }
  }

  // save annotated mesh
  if (true) {
    const std::string mesh_output_file  = ros::package::getPath("sim_environments") + "/data/annotate_cloud_gaits_mesh.obj";
    const std::string area_output_file  = ros::package::getPath("sim_environments") + "/data/annotate_cloud_gaits_mesh.dat";
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

