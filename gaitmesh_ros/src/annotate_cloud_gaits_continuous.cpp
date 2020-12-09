#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <gaitmesh_ros/CloudMesh.h>
#include <recast_ros/InputMeshSrv.h>
#include <pcl_msgs/PolygonMesh.h>

#include <gaitmesh_ros/gaitmesh_ros.hpp>
#include <gaitmesh_ros/common.hpp>

using namespace gaitmesh;  // temporary

class ContinuousGaitMesh
{
public:
  ContinuousGaitMesh(const ros::NodeHandle& nh)
    : nh_(nh)
  {
    nh_.param("radius_normals", radius_normals_, 0.20);
    nh_.param("radius_robot", radius_robot_, 0.60);
    nh_.param("curvature_threshold", curvature_threshold_, 0.06);

    client_recast_ = nh_.serviceClient<recast_ros::InputMeshSrv>("/recast_node/input_mesh");
    cloud_mesh_sub_ = nh_.subscribe("/gaitmesh_ros/input_cloud_mesh", 1, &ContinuousGaitMesh::cloudMeshCallback, this);
  }

  void processCloudMesh()
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PolygonMesh mesh;
    pcl_conversions::toPCL(cloud_mesh_.cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *input_cloud);
    pcl_conversions::toPCL(cloud_mesh_.mesh, mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr step2(new pcl::PointCloud<pcl::PointXYZ>());  // outlier_removal_cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr step3(new pcl::PointCloud<pcl::PointNormal>());  // normal_cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr step4(new pcl::PointCloud<pcl::PointXYZRGBNormal>());  // gait_cloud
    std::vector<char> mesh_labels = annotateCloud(input_cloud, mesh, radius_normals_, radius_robot_, curvature_threshold_, step2, step3, step4);

    recast_ros::InputMeshSrv srv;
    srv.request.reference_point = cloud_mesh_.reference_point;
    srv.request.input_mesh = cloud_mesh_.mesh;
    srv.request.area_labels.resize(mesh_labels.size());
    for (size_t i = 0; i < mesh_labels.size(); i++)
      srv.request.area_labels[i] = mesh_labels[i];
    if (client_recast_.call(srv)) ROS_INFO("Map was sent");
    else ROS_ERROR("Failed to send map");
  }

protected:
  ros::NodeHandle nh_;  //!< ROS NodeHandle

  double radius_normals_;
  double radius_robot_;
  double curvature_threshold_;

  ros::ServiceClient client_recast_;
  ros::Subscriber cloud_mesh_sub_;

  gaitmesh_ros::CloudMesh cloud_mesh_;  //!< Most recently received CloudMesh - local copy, will be processed in processCloudMesh()

  void cloudMeshCallback(const gaitmesh_ros::CloudMesh::ConstPtr& msg)
  {
    ROS_INFO("Received mesh - storing internally");
    cloud_mesh_ = *msg;
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "annotate_cloud_gaits");
  ros::NodeHandle nh("~");

  ContinuousGaitMesh continuous_gait_mesh(nh);

  // Use an AsyncSpinner to always process incoming messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Use a rate to only process messages at a given rate (always the latest)
  ros::Rate rate(1.0); // hz
  while (ros::ok()) {
    continuous_gait_mesh.processCloudMesh();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}
