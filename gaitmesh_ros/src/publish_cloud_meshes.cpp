#include <gaitmesh_ros/CloudMesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <gaitmesh_ros/meshlab_reconstruction.hpp>
#include "pcl/PclMeshSampling.h"
#include <boost/optional.hpp>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace gaitmesh;

pcl::PointCloud<pcl::PointXYZ>::Ptr publishInitialCloudMesh(ros::NodeHandle nh, ros::Publisher cloud_mesh_pub)
{
    // parameters (input)
    bool input_clouds_not_mesh;
    double mesh_sampling_resolution;
    std::string path_mesh;
    std::string path_cloud1;
    std::string path_cloud2;
    std::string path_cloud3;
    std::string path_cloud4;
    double reference_point_x;
    double reference_point_y;
    double reference_point_z;
    nh.param("input_clouds_not_mesh", input_clouds_not_mesh, true);
    nh.param("mesh_sampling_resolution", mesh_sampling_resolution, 0.05);
    nh.param("mesh", path_mesh, std::string(""));
    nh.param("cloud1", path_cloud1, std::string(""));  //Why send 4 clouds?
    nh.param("cloud2", path_cloud2, std::string(""));
    nh.param("cloud3", path_cloud3, std::string(""));
    nh.param("cloud4", path_cloud4, std::string(""));
    nh.param("reference_point_x", reference_point_x, 0.0);
    nh.param("reference_point_y", reference_point_y, 0.0);
    nh.param("reference_point_z", reference_point_z, 0.0);

    std::vector<std::string> cloud_paths;
    if (path_cloud1 != "") cloud_paths.push_back(path_cloud1);
    if (path_cloud2 != "") cloud_paths.push_back(path_cloud2);
    if (path_cloud3 != "") cloud_paths.push_back(path_cloud3);
    if (path_cloud4 != "") cloud_paths.push_back(path_cloud4);

    // first step is getting the input cloud and mesh
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PolygonMesh mesh;

    if (input_clouds_not_mesh)
    {
        // load and join clouds
        ROS_INFO("Loading clouds...");
        if (cloud_paths.size() == 0)
            return input_cloud;
        for (unsigned int i = 0; i < cloud_paths.size(); i++)
        {
            ROS_INFO("Processing cloud %d", i);
            // load
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::io::loadPLYFile(cloud_paths[i], *cloud);
            // join
            *input_cloud += *cloud;
        }
        // load or build cloud
        if (path_mesh != "")
        {
            pcl::io::loadPolygonFileOBJ(path_mesh, mesh);
        }
        else
        {
            mesh = reconstructMeshMeshlab(input_cloud);
        }
    }
    else
    {
        if (path_mesh == "")
            input_cloud;
        ROS_INFO("Loading mesh...");
        pcl::io::loadPolygonFile(path_mesh, mesh);
        ROS_INFO("Sampling mesh...");
        input_cloud = getCloudFromMesh(mesh, mesh_sampling_resolution);
    }

    gaitmesh_ros::CloudMesh initial_msg;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*input_cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, initial_msg.cloud);
    pcl_conversions::fromPCL(mesh, initial_msg.mesh);
    initial_msg.reference_point.x = reference_point_x;
    initial_msg.reference_point.y = reference_point_y;
    initial_msg.reference_point.z = reference_point_z;
    cloud_mesh_pub.publish(initial_msg);
    return input_cloud;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg, ros::Publisher cloud_mesh_pub, bool* callback_time_elapsed, pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud)
{
    if (*callback_time_elapsed)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloudMsg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *input_cloud);
        *input_cloud += *initial_cloud;

        // TODO: Subsample with VoxelGrid!
        ROS_WARN_STREAM("Input cloud size: " << input_cloud->size());

        if (input_cloud->size() == 0)
        {
            ROS_WARN_STREAM("Input point cloud empty, skipping processing.");
            return;
        }

        pcl::PolygonMesh mesh = reconstructMeshMeshlab(input_cloud);

        gaitmesh_ros::CloudMesh cloud_mesh_msg;
        cloud_mesh_msg.cloud = *cloudMsg;
        pcl_conversions::fromPCL(mesh, cloud_mesh_msg.mesh);
        cloud_mesh_msg.reference_point = poseMsg->pose.pose.position;
        cloud_mesh_pub.publish(cloud_mesh_msg);
        *callback_time_elapsed = false;
    }
}

void timerCallback(bool* callback_time_elapsed)
{
    *callback_time_elapsed = true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "publish_initial_cloud");
    ros::NodeHandle nh("~");
    ros::Publisher cloud_mesh_pub = nh.advertise<gaitmesh_ros::CloudMesh>("/gaitmesh_ros/input_cloud_mesh", 1);

    bool callback_time_elapsed = false;
    ros::Timer timer = nh.createTimer(ros::Duration(10), boost::bind(&timerCallback, &callback_time_elapsed));

    pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud = publishInitialCloudMesh(nh, cloud_mesh_pub);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/icp_odometry/map_cloud", 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, "/icp_odometry/corrected_pose", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseWithCovarianceStamped> SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud_sub, pose_sub);
    sync.registerCallback(boost::bind(&cloudCallback, _1, _2, cloud_mesh_pub, &callback_time_elapsed, initial_cloud));

    ros::Rate rate(1.0);  // hz
    while (nh.ok())
    {
        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
