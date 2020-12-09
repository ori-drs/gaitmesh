#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

namespace gaitmesh {

pcl::PolygonMesh reconstructMeshLasVegasReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

}  // namespace gaitmesh
