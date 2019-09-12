#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudFromMesh (pcl::PolygonMesh &mesh, float leaf_size);
pcl::PointCloud<pcl::PointNormal>::Ptr getCloudAndNormalsFromMesh (pcl::PolygonMesh &mesh, float leaf_size);

