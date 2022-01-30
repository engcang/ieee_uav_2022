#ifndef UTILITY_H
#define UTILITY_H

#include <chrono> 
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

using namespace std::chrono; 
using namespace std;

/////////// utils
//////////// common
#include <signal.h>
inline void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

inline high_resolution_clock::time_point start; //global, to use tic()
inline void tic(){
   start = high_resolution_clock::now();
}
inline void toc(){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%.3f ms spent", duration.count()/1000.0);
}
inline void toc(string text){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%s %.3f ms spent", text.c_str(), duration.count()/1000.0);
}

inline sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link")
{
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

inline pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
  pcl::PointCloud<pcl::PointXYZ> cloudresult;
  pcl::fromROSMsg(cloudmsg,cloudresult);
  return cloudresult;
}

inline geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose) {
    geometry_msgs::Pose geoPose;

    tf::Matrix3x3 m;
    m.setValue((double)pose(0,0),
            (double)pose(0,1),
            (double)pose(0,2),
            (double)pose(1,0),
            (double)pose(1,1),
            (double)pose(1,2),
            (double)pose(2,0),
            (double)pose(2,1),
            (double)pose(2,2));

    tf::Quaternion q;
    m.getRotation(q);
    geoPose.orientation.x = q.getX();
    geoPose.orientation.y = q.getY();
    geoPose.orientation.z = q.getZ();
    geoPose.orientation.w = q.getW();

    geoPose.position.x = pose(0,3);
    geoPose.position.y = pose(1,3);
    geoPose.position.z = pose(2,3);

    return geoPose;
}

inline Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose) {
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
  tf::Matrix3x3 m(q);
  result(0,0) = m[0][0];
  result(0,1) = m[0][1];
  result(0,2) = m[0][2];
  result(1,0) = m[1][0];
  result(1,1) = m[1][1];
  result(1,2) = m[1][2];
  result(2,0) = m[2][0];
  result(2,1) = m[2][1];
  result(2,2) = m[2][2];
  result(3,3) = 1;

  result(0,3) = geoPose.position.x;
  result(1,3) = geoPose.position.y;
  result(2,3) = geoPose.position.z;

  return result;
}

inline double interpolate(double x0, double x1, double ratio){
  return (1 - ratio) * x0 + ratio * x1;
}

inline geometry_msgs::Point interpolate(const geometry_msgs::Point& p0, const geometry_msgs::Point& p1, double ratio) {
  geometry_msgs::Point point_interpolated;
  point_interpolated.x = interpolate(p0.x, p1.x, ratio);
  point_interpolated.y = interpolate(p0.y, p1.y, ratio);
  point_interpolated.z = interpolate(p0.z, p1.z, ratio);
  return point_interpolated;
}

inline geometry_msgs::Quaternion interpolate(const geometry_msgs::Quaternion& quaternion0, const geometry_msgs::Quaternion& quaternion1, double ratio) {
  /**< SLERP denotes Spherical Linear interpolation  */

  tf::Quaternion q_interpolated;
  geometry_msgs::Quaternion q_out;

  tf::Quaternion q0;
  tf::quaternionMsgToTF(quaternion0, q0);
  tf::Quaternion q1;
  tf::quaternionMsgToTF(quaternion1, q1);

  // Normalize to avoid undefined behavior.
  q0.normalize();
  q1.normalize();

  double dot = q0.dot(q1);

  // If the dot product is negative, slerp won't take
  // the shorter path. Note that q1 and -q1 are equivalent when
  // the negation is applied to all four components. Fix by reversing one quaternion.
  if (dot < 0.0f) {
      q1 = q1 * (-1);
      dot = - dot;
  }
  const double DOT_THRESHOLD = 0.999;

  if (dot > DOT_THRESHOLD) {
      q_interpolated = q0 + (q1 - q0) * ratio;

  }else{
    // Since dot is in range [0, DOT_THRESHOLD], acos is safe
    q_interpolated = q0.slerp(q1, ratio);
  }

  q_interpolated.normalize();
  tf::quaternionTFToMsg(q_interpolated, q_out);
  return q_out;
}

inline geometry_msgs::Pose interpolate(const geometry_msgs::Pose& pose0, const geometry_msgs::Pose& pose1, double ratio) {
  geometry_msgs::Pose pose_interpolated;
  pose_interpolated.orientation = interpolate(pose0.orientation, pose1.orientation, ratio);
  pose_interpolated.position = interpolate(pose0.position, pose1.position, ratio);
  return pose_interpolated;
}

#endif
