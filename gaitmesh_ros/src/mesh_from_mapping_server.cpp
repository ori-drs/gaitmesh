#include <ros/ros.h>
#include <gaitmesh_ros/CloudMesh.h>
#include "MeshLabReconstruction.h"
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg, ros::Publisher cloud_mesh_pub, bool* callback_time_elapsed) {
  ROS_INFO("Called this");
  if (*callback_time_elapsed) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloudMsg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *input_cloud);
    pcl::PolygonMesh mesh = reconstructMesh(input_cloud);

    gaitmesh_ros::CloudMesh cloud_mesh_msg;
    cloud_mesh_msg.cloud = *cloudMsg;
    pcl_conversions::fromPCL(mesh, cloud_mesh_msg.mesh);
    cloud_mesh_msg.reference_point = poseMsg->pose.pose.position;
    cloud_mesh_pub.publish(cloud_mesh_msg);
    *callback_time_elapsed = false;
  }
}

void timerCallback(bool* callback_time_elapsed) {
  *callback_time_elapsed = true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mesh_from_mapping_server");
  ros::NodeHandle nodeHandle("~");
  ros::Publisher cloud_mesh_pub = nodeHandle.advertise<gaitmesh_ros::CloudMesh>("/gaitmesh_ros/input_cloud_mesh", 1);

  bool callback_time_elapsed = false;
  ros::Timer timer = nodeHandle.createTimer(ros::Duration(30), boost::bind(&timerCallback, &callback_time_elapsed));

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nodeHandle, "/icp_odometry/map_cloud", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nodeHandle, "/icp_odometry/corrected_pose", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseWithCovarianceStamped> SyncPolicy;

  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud_sub, pose_sub);
  sync.registerCallback(boost::bind(&cloudCallback, _1, _2, cloud_mesh_pub, &callback_time_elapsed));

  ros::Rate rate(1.0); // hz
  while (nodeHandle.ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}
