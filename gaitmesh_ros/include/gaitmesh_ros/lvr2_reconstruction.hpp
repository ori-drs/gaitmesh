#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gaitmesh
{
pcl::PolygonMesh reconstructMeshLasVegasReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

}  // namespace gaitmesh
