#pragma once

#include <pcl/point_types.h>
#include <Eigen/Core>

inline Eigen::Vector3d pcl2eig(const pcl::PointXYZRGBNormal& pt)
{
    return Eigen::Vector3d(pt.x, pt.y, pt.z);
}

inline pcl::PointXYZRGBNormal eig2pcl(const Eigen::Vector3d& vec)
{
    pcl::PointXYZRGBNormal pt;
    pt.x = vec(0);
    pt.y = vec(1);
    pt.z = vec(2);
    return pt;
}

inline bool pointsOnSameSideOfLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    Eigen::Vector3d n1 = (b - a).cross(p1 - a);
    Eigen::Vector3d n2 = (b - a).cross(p2 - a);
    return n1.dot(n2) >= 0;
}

inline bool pointInTriangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    return pointsOnSameSideOfLine(p, a, b, c) && pointsOnSameSideOfLine(p, b, a, c) && pointsOnSameSideOfLine(p, c, a, b);
}
