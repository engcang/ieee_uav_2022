#include "ieee_unav.h"


using namespace std;
using namespace std::chrono; 
using namespace Eigen;

ieee_uav_class::ieee_uav_class(ros::NodeHandle& n) : nh(n)
{
  ROS_WARN("Class generating...");
  ///// params
  getParam();

  target_poly_traj = new bezier_traj_class(nh, m_target_traj_hz, m_traj_leng, m_traj_leng_past, m_fixed_frame);

  ///// sub pub
  m_tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &ieee_uav_class::tf_callback, this);
  m_gt_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &ieee_uav_class::gt_callback, this);

  static message_filters::Subscriber<sensor_msgs::Image> m_depth_sub;
  static message_filters::Subscriber<yolo_ros_simple::bboxes> m_bbox_sub;
  m_depth_sub.subscribe(nh,m_depth_topic,10);
  m_bbox_sub.subscribe(nh, m_bbox_out_topic, 10);

  static message_filters::Synchronizer<db_sync_pol> m_sub_DepBsynced(db_sync_pol(10),m_depth_sub,m_bbox_sub);
  m_sub_DepBsynced.registerCallback(&ieee_uav_class::depb_callback,this);

  m_detected_target_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/detected_target_pcl", 10);
  m_goal_traj_pub = nh.advertise<ieee_uav::odom_array>("/goal_pose", 10);
  
  image_transport::ImageTransport it(n);
  m_mask_pub = it.advertise("/binary_image", 10);

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
  nh.param<std::string>("/color_extraction/mode", m_color_extraction_mode, "red"); // "red", "white", and "both"
  nh.param("/color_extraction/white_color_sensitivity", m_white_color_sensitivity, 30);
  nh.param("/target_traj_hz", m_target_traj_hz, 12.0);
  nh.param("/traj_leng", m_traj_leng, 30);
  nh.param("/traj_leng_past", m_traj_leng_past, 300);
  nh.param("/target_predict_seg", m_target_predict_seg, 10);
}
