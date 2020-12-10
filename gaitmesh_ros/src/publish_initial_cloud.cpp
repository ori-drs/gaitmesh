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

using namespace gaitmesh;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "publish_initial_cloud");
    ros::NodeHandle nodeHandle("~");
    ros::Publisher cloud_mesh_pub = nodeHandle.advertise<gaitmesh_ros::CloudMesh>("/gaitmesh_ros/input_cloud_mesh", 1);

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
    nodeHandle.param("input_clouds_not_mesh", input_clouds_not_mesh, true);
    nodeHandle.param("mesh_sampling_resolution", mesh_sampling_resolution, 0.05);
    nodeHandle.param("mesh", path_mesh, std::string(""));
    nodeHandle.param("cloud1", path_cloud1, std::string(""));  //Why send 4 clouds?
    nodeHandle.param("cloud2", path_cloud2, std::string(""));
    nodeHandle.param("cloud3", path_cloud3, std::string(""));
    nodeHandle.param("cloud4", path_cloud4, std::string(""));
    nodeHandle.param("reference_point_x", reference_point_x, 0.0);
    nodeHandle.param("reference_point_y", reference_point_y, 0.0);
    nodeHandle.param("reference_point_z", reference_point_z, 0.0);

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
        {
            ROS_ERROR("No point cloud provided");
        }
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

    ros::Rate rate(1 / 5.0);  // Hz
    rate.sleep();           // Sleep for 5s, then exit

    return EXIT_SUCCESS;
}
