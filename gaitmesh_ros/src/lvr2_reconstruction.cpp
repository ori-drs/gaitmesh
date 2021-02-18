#include <chrono>

#include <ros/package.h>  // To resolve packge paths to the Meshlab config
#include <ros/ros.h>

#include <pcl/io/ply_io.h>
// #include <pcl/io/vtk_lib_io.h>  // for loadPolygonFilePLY
#include "pcl/vtk_lib_io.h"

#include <gaitmesh_ros/lvr2_reconstruction.hpp>

namespace gaitmesh
{
pcl::PolygonMesh reconstructMeshLasVegasReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    auto s = std::chrono::high_resolution_clock::now();

    pcl::PolygonMesh mesh;
    ROS_INFO("Reconstructing mesh (using lvr2_reconstruct)...");
    const std::string cloud_output_file = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_cloud.ply";  // TODO: Use /tmp instead! Do not write to source!
    const std::string mesh_output_file = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_mesh.ply";    // TODO: Use /tmp instead! Do not write to source!
    // save cloud to file
    pcl::io::savePLYFile(cloud_output_file, *cloud);
    // run lvr2_reconstruct
    std::stringstream cmd;
    cmd << "lvr2_reconstruct " << cloud_output_file << " -v 0.15 -r --outputFile " << mesh_output_file;
    // TODO: Capture output
    int system_result = system(cmd.str().c_str());
    // load mesh from file
    pcl::io::loadPolygonFilePLY(mesh_output_file, mesh);

    auto e = std::chrono::high_resolution_clock::now();
    ROS_INFO("Mesh reconstruction using Las Vegas Reconstruction took: %.3f ms", 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(e - s).count());

    return mesh;
}

}  // namespace gaitmesh
