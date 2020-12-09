#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include <gaitmesh_ros/common.hpp>

namespace gaitmesh {

void setWalkInBox(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, double x, double y, double z, double dx, double dy, double dz, double step);

void setWalkInCylinder(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, double x1, double y1, double z1, double x2, double y2, double z2, double r);

bool saveAreas(const std::string& path, const std::vector<char>& labels);

std::vector<char> annotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                pcl::PolygonMesh mesh,
                                double radius_normals,
                                double radius_robot,
                                double curvature_threshold,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removal_cloud,
                                pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud,
                                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr gait_cloud);

}  // namespace gaitmesh
