#include "../include/ieee_unav.h"


using namespace std;
using namespace std::chrono; 
using namespace Eigen;

ieee_uav_class::ieee_uav_class()
{
  ROS_WARN("Class generating...");
  ///// params
  getParam();

  ///// sub pub
  static ros::Subscriber m_tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &ieee_uav_class::tf_callback, this);

  static message_filters::Subscriber<sensor_msgs::Image> m_depth_sub;
  static message_filters::Subscriber<yolo_ros_simple::bboxes> m_bbox_sub;
  m_depth_sub.subscribe(nh,m_depth_topic,10);
  m_bbox_sub.subscribe(nh, m_bbox_out_topic, 10);

  static message_filters::Synchronizer<db_sync_pol> m_sub_DepBsynced(db_sync_pol(10),m_depth_sub,m_bbox_sub);
  m_sub_DepBsynced.registerCallback(&ieee_uav_class::depb_callback,this);

  m_detected_target_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/detected_target_pcl", 10);
  m_best_branch_pub = nh.advertise<nav_msgs::Path>("/best_path", 10);

  ROS_WARN("Class heritated, starting node...");
}

ieee_uav_class::~ieee_uav_class()
{

}

void ieee_uav_class::getParam()
{
  nh.param("/altitude_fixed", m_altitude_fixed, 0.8);
  nh.param("/pcl_max_range", m_pcl_max_range, 5.0);
  nh.param("/f_x", m_f_x, 320.0);
  nh.param("/f_y", m_f_y, 320.0);
  nh.param("/c_x", m_c_x, 320.5);
  nh.param("/c_y", m_c_y, 240.5);

  nh.param<std::string>("/bbox_out_topic", m_bbox_out_topic, "/bboxes");
  nh.param<std::string>("/depth_topic", m_depth_topic, "/d455/depth/image_raw");
  nh.param<std::string>("/depth_base", m_depth_base, "camera_link");
  nh.param<std::string>("/body_base", m_body_base, "body_base");
  nh.param<std::string>("/fixed_frame", m_fixed_frame, "map");
}
