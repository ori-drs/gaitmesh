#include "std_msgs/String.h"
#include <gaitmesh_ros/CloudMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include "recast_ros/InputMeshSrv.h"

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
  Eigen::Vector3d n1 = (b-a).cross(p1-a);
  Eigen::Vector3d n2 = (b-a).cross(p2-a);
  return n1.dot(n2) >= 0;
}

inline bool pointInTriangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
  return pointsOnSameSideOfLine(p,a,b,c) && pointsOnSameSideOfLine(p,b,a,c) && pointsOnSameSideOfLine(p,c,a,b);
}

void setWalkInBox(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, double x, double y, double z, double dx, double dy, double dz, double step)
{
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
  tree->setInputCloud (cloud);
  for (double xx = x - dx; xx <= x + dx; xx += step) {
    for (double yy = y - dy; yy <= y + dy; yy += step) {
      for (double zz = z - dz; zz <= z + dz; zz += step) {
        pcl::PointXYZRGBNormal pt;
        pt.x = xx;
        pt.y = yy;
        pt.z = zz;
        std::vector<int> k_indices;
        std::vector<float> k_distances;
        tree->nearestKSearch (pt, 5, k_indices, k_distances);
        for (unsigned int i = 0; i < k_indices.size(); i++) {
          cloud->points[ k_indices[i] ].r = 0;   // walk
          cloud->points[ k_indices[i] ].g = 0;   // trot
          cloud->points[ k_indices[i] ].b = 128; // step
          cloud->points[ k_indices[i] ].a = 255;
        }
      }
    }
  }
}

void setWalkInCylinder(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, double x1, double y1, double z1, double x2, double y2, double z2, double r)
{
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
  tree->setInputCloud (cloud);
  double dist = sqrt( pow(x2-x1, 2.0) + pow(y2-y1, 2.0) + pow(y2-y1, 2.0) );
  double step = 0.10;
  double vx = step * (x2 - x1) / dist;
  double vy = step * (y2 - y1) / dist;
  double vz = step * (z2 - z1) / dist;
  for (unsigned int i = 0; i <= dist / step; i++) {
    double xx = x1 + vx * i;
    double yy = y1 + vy * i;
    double zz = z1 + vz * i;
    pcl::PointXYZRGBNormal pt;
    pt.x = xx;
    pt.y = yy;
    pt.z = zz;
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    tree->radiusSearch (pt, r, k_indices, k_distances);
    for (unsigned int i = 0; i < k_indices.size(); i++) {
      cloud->points[ k_indices[i] ].r = 0;   // walk
      cloud->points[ k_indices[i] ].g = 0;   // trot
      cloud->points[ k_indices[i] ].b = 128; // step
      cloud->points[ k_indices[i] ].a = 255;
    }
  }
}

bool saveAreas(const std::string& path, const std::vector<char>& labels)
{
  std::ofstream FILE(path, std::ios::out | std::ofstream::binary);
  std::copy(labels.begin(), labels.end(), std::ostreambuf_iterator<char>(FILE));
  FILE.close();
  return true;
}

//--------------------------------------------------------------------------------------------

std::vector<char> annotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PolygonMesh mesh, double radius_normals, double radius_robot, double curvature_threshold)
{
  // run terrain-related filters
  pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
  {
    ROS_INFO("Estimating normals...");
    const float radius = radius_normals;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (input_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (radius);
    ne.compute (*normals);
    pcl::concatenateFields (*input_cloud, *normals, *normal_cloud);
  }

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr gait_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  {
    ROS_INFO("Running filters...");
    const float radius = radius_robot;
    gait_cloud->points.reserve(normal_cloud->points.size());
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
    tree->setInputCloud (normal_cloud);
    int pnumber = (int)normal_cloud->size ();
    for (int point_id = 0; point_id < pnumber; ++point_id) {
      // max curvature within radius
      std::vector<int> k_indices;
      std::vector<float> k_distances;
      tree->radiusSearch (point_id, radius, k_indices, k_distances);
      float max_curvature = 0;
      for (size_t n_id = 0; n_id < k_indices.size (); ++n_id) {
        float id = k_indices.at (n_id);
        float curvature = normal_cloud->points[id].curvature;
        if (curvature > max_curvature)
          max_curvature = curvature;
      }
      // add point
      pcl::PointXYZRGBNormal pt;
      pt.x = normal_cloud->points[point_id].x;
      pt.y = normal_cloud->points[point_id].y;
      pt.z = normal_cloud->points[point_id].z;
      pt.normal[0] = normal_cloud->points[point_id].normal[0];
      pt.normal[1] = normal_cloud->points[point_id].normal[1];
      pt.normal[2] = normal_cloud->points[point_id].normal[2];
      pt.curvature = normal_cloud->points[point_id].curvature;
      pt.r = (max_curvature < curvature_threshold ? 128 : 0); // walk
      pt.g = (max_curvature < curvature_threshold ? 128 : 0); // trot
      pt.b = 128; // step
      pt.a = 255;
      gait_cloud->points.push_back(pt);
    }
  }

  // manually add a mobility hazard
  //setWalkInCylinder(gait_cloud, -1, 3.2, -1.1, 1.5, 2.9, -1.05, 0.90);

  // back-project features to mesh
  std::vector<char> mesh_labels(mesh.polygons.size(), (char)0);
  ROS_INFO("Back-projecting annotations to mesh...");
  // tree
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
  tree->setInputCloud (gait_cloud);
  // go through all mesh triangles
  pcl::PointCloud<pcl::PointXYZRGBNormal> mesh_cloud;
  pcl::fromPCLPointCloud2 (mesh.cloud, mesh_cloud);
  for (int i = 0; i < mesh.polygons.size(); i++) {
    // we will vote for each of the gait-controller options
    int votes_trot = 0;
    int votes_step = 0;
    // find all points around the triangle
    Eigen::Vector3d v0 = pcl2eig( mesh_cloud.points[ mesh.polygons[i].vertices[0] ] );
    Eigen::Vector3d v1 = pcl2eig( mesh_cloud.points[ mesh.polygons[i].vertices[1] ] );
    Eigen::Vector3d v2 = pcl2eig( mesh_cloud.points[ mesh.polygons[i].vertices[2] ] );
    Eigen::Vector3d vc = (v0+v1+v2) / 3;
    Eigen::Vector3d normal = (v1-v0).cross(v2-v1).normalized();
    pcl::PointXYZRGBNormal pt_center = eig2pcl(vc);
    double dist0 = (v0-vc).norm();
    double dist1 = (v1-vc).norm();
    double dist2 = (v2-vc).norm();
    double radius = std::max(std::max(dist0, dist1), dist2);
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    tree->radiusSearch (pt_center, radius, k_indices, k_distances);
    for (unsigned int k = 0; k < k_indices.size(); k++) {
      // check if point is inside triangle
      const pcl::PointXYZRGBNormal& pt = gait_cloud->points[ k_indices[k] ];
      Eigen::Vector3d v = pcl2eig(pt);
      if ( fabs((v-vc).dot(normal)) > 0.10  ) // far from plane
        continue;
      if ( !pointInTriangle(v,v0,v1,v2) ) // outside triangle
        continue;
      // vote
      if (pt.g > 0)
        votes_trot++;
      else if (pt.b > 0)
        votes_step++;
    }
    // assign label to triangle
    char label = 2;
    if (votes_trot > votes_step)
      label = 1; // trot
    else
      label = 2; // step
    if (votes_trot == 0 && votes_step == 0)
      label = 0; // none
    //if (votes_trot + votes_step < 10)
    //  label = 0;
    mesh_labels[i] = label;
  }

  return mesh_labels;
}

void cloudMeshCallback(const gaitmesh_ros::CloudMesh::ConstPtr& msg, ros::NodeHandle nodeHandle, ros::ServiceClient client_recast, double radius_normals, double radius_robot, double curvature_threshold)
{
  ROS_INFO("Received mesh...");
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PolygonMesh mesh;
  pcl_conversions::toPCL(msg->cloud, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *input_cloud);
  pcl_conversions::toPCL(msg->mesh, mesh);

  std::vector<char> mesh_labels = annotateCloud(input_cloud, mesh, radius_normals, radius_robot, curvature_threshold);
  recast_ros::InputMeshSrv srv;
  srv.request.reference_point = msg->reference_point;
  srv.request.input_mesh = msg->mesh;
  srv.request.area_labels.resize(mesh_labels.size());
  for (size_t i = 0; i < mesh_labels.size(); i++)
    srv.request.area_labels[i] = mesh_labels[i];
  if (client_recast.call(srv)) ROS_INFO("Map was sent");
  else ROS_ERROR("Failed to send map");
}

//--------------------------------------------------------------------------------------------


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "annotate_cloud_gaits");
  ros::NodeHandle nodeHandle("~");

  // parameters (input)
  double radius_normals;
  double radius_robot;
  double curvature_threshold;
  nodeHandle.param("radius_normals", radius_normals, 0.20);
  nodeHandle.param("radius_robot", radius_robot, 0.60);
  nodeHandle.param("curvature_threshold", curvature_threshold, 0.06);

  ros::ServiceClient client_recast = nodeHandle.serviceClient<recast_ros::InputMeshSrv>("/recast_node/input_mesh");

  typedef const boost::function<void(const gaitmesh_ros::CloudMesh::ConstPtr& msg)> callback;
  callback cb = boost::bind(&cloudMeshCallback, _1, nodeHandle, client_recast, radius_normals, radius_robot, curvature_threshold);
  ros::Subscriber cloudMeshSub = nodeHandle.subscribe("/gaitmesh_ros/input_cloud_mesh", 1, cb);

  ros::Rate rate(1.0); // hz
  while (nodeHandle.ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}


/* outlier filter (after loading cloud)
pcl::PointCloud<pcl::PointXYZ>::Ptr step2 ( new pcl::PointCloud<pcl::PointXYZ>() );
if (false) {
  ROS_INFO("Removing outliers...");
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (step1);
  sor.setRadiusSearch (0.2);
  sor.setMinNeighborsInRadius (20);
  sor.filter (*step2);
} else {
  *step2 = *step1;
}
*/


/* ALTERNATIVE BACK-PROJECTION STEP
if (false) {
  ROS_INFO("Back-projecting annotations to mesh...");
  // tree
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
  tree->setInputCloud (step4);
  // go through all mesh triangles
  pcl::PointCloud<pcl::PointXYZRGBNormal> mesh_cloud;
  pcl::fromPCLPointCloud2 (mesh.cloud, mesh_cloud);
  for (int i = 0; i < mesh.polygons.size(); i++) {
    // get closest annotated-cloud point
    Eigen::Vector3d v0 = pcl2eig( mesh_cloud.points[ mesh.polygons[i].vertices[0] ] );
    Eigen::Vector3d v1 = pcl2eig( mesh_cloud.points[ mesh.polygons[i].vertices[1] ] );
    Eigen::Vector3d v2 = pcl2eig( mesh_cloud.points[ mesh.polygons[i].vertices[2] ] );
    Eigen::Vector3d vc = (v0+v1+v2) / 3;
    pcl::PointXYZRGBNormal pt_center = eig2pcl(vc);
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    tree->nearestKSearch (pt_center, 1, k_indices, k_distances);
    // assign label to triangle
    const pcl::PointXYZRGBNormal& nn = step4->points[ k_indices[0] ];
    char label = 2;
    if (nn.g > 0)
      label = 1; // trot
    else if (nn.b > 0)
      label = 2; // step
    else
      label = 0; // unwalkable
    mesh_labels[i] = label;
  }
}
*/

/*
// publish loop
ros::Rate rate(1.0); // hz
while (nodeHandle.ok()) {
  // publish
  mesh_pub.publish(mesh_msg);
  mesh_labels_pub.publish(mesh_labels_msg);
  ROS_INFO("Published meshes");
  // wait
  rate.sleep();
}
*/
