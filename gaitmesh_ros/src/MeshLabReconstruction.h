#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

pcl::PolygonMesh reconstructMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
