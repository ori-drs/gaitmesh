#include <chrono>

#include <ros/package.h>  // To resolve packge paths to the Meshlab config
#include <ros/ros.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>  // for loadPolygonFilePLY

#include <gaitmesh_ros/meshlab_reconstruction.hpp>

namespace gaitmesh
{
pcl::PolygonMesh reconstructMeshMeshlab(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    auto s = std::chrono::high_resolution_clock::now();

    pcl::PolygonMesh mesh;
    ROS_INFO("Reconstructing mesh (using meshlab server)...");
    const std::string cloud_output_file = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_cloud.ply";  // TODO: Use /tmp instead! Do not write to source!
    const std::string mesh_output_file = ros::package::getPath("gaitmesh_ros") + "/data/annotate_cloud_gaits_mesh.ply";    // TODO: Use /tmp instead! Do not write to source!
    const std::string meshlab_server_file = ros::package::getPath("gaitmesh_ros") + "/config/ballPivoting.mlx";
    // save cloud to file
    pcl::io::savePLYFile(cloud_output_file, *cloud);
    // run meshlab server
    std::stringstream cmd;
    cmd << "meshlabserver -i " << cloud_output_file << " -o " << mesh_output_file << " -s " << meshlab_server_file;
    // TODO: Capture output
    int system_result = system(cmd.str().c_str());
    // load mesh from file
    pcl::io::loadPolygonFilePLY(mesh_output_file, mesh);

    auto e = std::chrono::high_resolution_clock::now();
    ROS_INFO("Mesh reconstruction using ball pivoting took: %.3f ms", 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(e - s).count());

    return mesh;
}

}  // namespace gaitmesh
