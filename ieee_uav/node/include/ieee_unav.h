#ifndef IEEE_UAV_H
#define IEEE_UAV_H

#include "utility.hpp"

///// common headers
#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler
#include <tf2_msgs/TFMessage.h> //for tf between frames

///// headers for path planning and controller
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

///// headers for pcl and yolo
#include <yolo_ros_simple/bbox.h>
#include <yolo_ros_simple/bboxes.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, yolo_ros_simple::bboxes> db_sync_pol;

class ieee_uav_class{
private:

  std::string m_depth_topic, m_depth_base, m_body_base, m_fixed_frame, m_bbox_out_topic;
  bool m_bbox_check=false, m_depth_check=false, m_tf_check=false, m_body_t_cam_check=false;

  ///// for yolo and pcl
  cv_bridge::CvImagePtr m_depth_ptr;
  double m_scale_factor;
  double m_pcl_max_range=0.0, m_f_x, m_f_y, m_c_x, m_c_y;

  //// states
  Matrix4f m_map_t_cam = Matrix4f::Identity();
  Matrix4f m_map_t_body = Matrix4f::Identity();
  Matrix4f m_body_t_cam = Matrix4f::Identity();
  double m_cvt_quat_x=0.0, m_cvt_quat_y=0.0, m_cvt_quat_z=0.0, m_cvt_quat_w=1.0;

  //// for controller
  double m_altitude_fixed;

  ///// ros and tf
  ros::NodeHandle nh;
  ros::Subscriber m_new_path_sub;
  ros::Publisher m_best_branch_pub;
  ros::Publisher m_detected_target_pcl_pub;

  void getParam();
  void depb_callback(const sensor_msgs::ImageConstPtr& depthMsg,
                     const yolo_ros_simple::bboxes::ConstPtr& bboxMsg);
  void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);

public:
  ieee_uav_class();
  ~ieee_uav_class();
};

#endif
