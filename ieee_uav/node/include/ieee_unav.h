#ifndef IEEE_UAV_H
#define IEEE_UAV_H

#include "utility.hpp"
#include "predict.h"

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
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

///// headers for pcl and yolo
#include <yolo_ros_simple/bbox.h>
#include <yolo_ros_simple/bboxes.h>
#include <sensor_msgs/PointCloud2.h>
#include <ieee_uav/odom_array.h>

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

#include <image_transport/image_transport.h>

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, yolo_ros_simple::bboxes> db_sync_pol;

struct color_extraction_params { 
  std::string mode; 
  bool        verbose; 

  int         erosion_adaptive_size;
  int         erosion_small_kernel_size;
  int         erosion_large_kernel_size;

  int         sensitivity_adaptive_size;
  int         sensitivity;
  float       sensitivity_ratio;
};

class ieee_uav_class{
private:

  ///// ros and tf
  ros::NodeHandle nh;
  ros::Subscriber m_tf_sub;
  ros::Subscriber m_gt_sub;
  ros::Publisher m_goal_traj_pub;
  ros::Publisher m_detected_target_pcl_pub;
  
  image_transport::Publisher m_mask_pub;

  std::string m_depth_topic, m_depth_base, m_body_base, m_fixed_frame, m_bbox_out_topic;
  bool m_bbox_check=false, m_depth_check=false, m_gt_check=false, m_body_t_cam_check=false;

  ///// for color extraction
  color_extraction_params m_color_params;

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

  ///// target polynomial
  bezier_traj_class *target_poly_traj;
  double m_target_traj_hz;
  int m_traj_leng, m_traj_leng_past, m_target_predict_seg;

  void getParam();
  void depb_callback(const sensor_msgs::ImageConstPtr& depthMsg,
                     const yolo_ros_simple::bboxes::ConstPtr& bboxMsg);
  void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
  void gt_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);

  int get_hsv_mask(const cv::Mat& hsv_img, cv::Mat& mask, const color_extraction_params& params);
public:
  ieee_uav_class(ros::NodeHandle& n);
  ~ieee_uav_class();
};

#endif
