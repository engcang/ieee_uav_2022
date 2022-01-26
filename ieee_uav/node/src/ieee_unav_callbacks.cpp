
#include "../include/ieee_unav.h"


using namespace std;
using namespace std::chrono; 
using namespace Eigen;

void ieee_uav_class::depb_callback(const sensor_msgs::ImageConstPtr& depthMsg,
                                   const yolo_ros_simple::bboxes::ConstPtr& bboxMsg)
{
  sensor_msgs::Image depth=*depthMsg;
  try {
    pcl::PointXYZ p3d, p3d_empty;
    // tic();
    if (depth.encoding=="32FC1"){
      m_depth_ptr = cv_bridge::toCvCopy(depth, "32FC1"); // == sensor_msgs::image_encodings::TYPE_32FC1
      m_scale_factor=1.0;
    }
    else if (depth.encoding=="16UC1"){ // uint16_t (stdint.h) or ushort or unsigned_short
      m_depth_ptr = cv_bridge::toCvCopy(depth, "16UC1"); // == sensor_msgs::image_encodings::TYPE_16UC1
      m_scale_factor=1000.0;
    }
    m_depth_check=true;
    // toc();
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Error to cvt depth img");
    return;
  }

  if (bboxMsg->bboxes.size() < 1)
    return;

  yolo_ros_simple::bbox in_bbox = bboxMsg->bboxes[0];

  if (m_depth_check){
    pcl::PointXYZ p3d, p3d_center;
    int pcl_size=0;
    for (int i=in_bbox.y; i < in_bbox.y + in_bbox.height; i++){
      for (int j=in_bbox.x; j < in_bbox.x + in_bbox.width; j++){
        float temp_depth = 0.0;
        if (m_scale_factor==1.0){
          temp_depth = m_depth_ptr->image.at<float>(i,j); //float!!! double makes error here!!! because encoding is "32FC", float
        }
        else if (m_scale_factor==1000.0){
          temp_depth = m_depth_ptr->image.at<ushort>(i,j); //ushort!!! other makes error here!!! because encoding is "16UC"
        }
        if (std::isnan(temp_depth) or temp_depth==0.0){
          continue;
        }
        else if (temp_depth/m_scale_factor >= 0.1 and temp_depth/m_scale_factor <= m_pcl_max_range){
          p3d.z = (temp_depth/m_scale_factor);
          p3d.x = ( j - m_c_x ) * p3d.z / m_f_x;
          p3d.y = ( i - m_c_y ) * p3d.z / m_f_y;

          p3d_center.x += p3d.x;
          p3d_center.y += p3d.y;
          p3d_center.z += p3d.z;
          pcl_size++;
        }
      }
    }
    if (pcl_size>0){
      p3d_center.x /= (float)pcl_size;
      p3d_center.y /= (float)pcl_size;
      p3d_center.z /= (float)pcl_size;

      pcl::PointCloud<pcl::PointXYZ>::Ptr detected_center_pub(new pcl::PointCloud<pcl::PointXYZ>());
      detected_center_pub->push_back( pcl::PointXYZ(p3d_center.x, p3d_center.y, p3d_center.z) );
      m_detected_target_pcl_pub.publish(cloud2msg(*detected_center_pub, m_depth_base));
      m_bbox_check=true;

      if(m_tf_check){
        nav_msgs::Path path_for_control;
        path_for_control.header.stamp = ros::Time::now();
        path_for_control.header.frame_id = m_fixed_frame;

        geometry_msgs::PoseStamped starting_pose, end_pose;
        starting_pose.pose.position.x = m_map_t_body(0,3);
        starting_pose.pose.position.y = m_map_t_body(1,3);
        starting_pose.pose.position.z = m_map_t_body(2,3);
        starting_pose.pose.orientation.x = m_cvt_quat_x;
        starting_pose.pose.orientation.y = m_cvt_quat_y;
        starting_pose.pose.orientation.z = m_cvt_quat_z;
        starting_pose.pose.orientation.w = m_cvt_quat_w;

        MatrixXf center_before_tf(4,1), center_after_tf(4,1);
        center_before_tf << p3d_center.x, p3d_center.y, p3d_center.z, 1.0;
        center_after_tf = m_map_t_cam * center_before_tf;

        float target_yaw_ = atan2(center_after_tf(1)-starting_pose.pose.position.y, center_after_tf(0)-starting_pose.pose.position.x);
        tf::Quaternion target_quaternion_;
        target_quaternion_.setRPY(0, 0, target_yaw_);

        end_pose.pose.position.x = center_after_tf(0);
        end_pose.pose.position.y = center_after_tf(1);
        end_pose.pose.position.z = m_altitude_fixed;
        end_pose.pose.orientation.x = target_quaternion_.getX();
        end_pose.pose.orientation.y = target_quaternion_.getY();
        end_pose.pose.orientation.z = target_quaternion_.getZ();
        end_pose.pose.orientation.w = target_quaternion_.getW();

        path_for_control.poses.push_back(starting_pose);
        path_for_control.poses.push_back(end_pose);

        m_best_branch_pub.publish(path_for_control);
      }
    }

  }
}

void ieee_uav_class::tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg){
  for (int l=0; l < msg->transforms.size(); l++){
    if (msg->transforms[l].header.frame_id==m_fixed_frame && msg->transforms[l].child_frame_id==m_body_base){
      ///// for tf between map and body
      tf::Quaternion q(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m(q);
      m_map_t_body(0,0) = m[0][0];
      m_map_t_body(0,1) = m[0][1];
      m_map_t_body(0,2) = m[0][2];
      m_map_t_body(1,0) = m[1][0];
      m_map_t_body(1,1) = m[1][1];
      m_map_t_body(1,2) = m[1][2];
      m_map_t_body(2,0) = m[2][0];
      m_map_t_body(2,1) = m[2][1];
      m_map_t_body(2,2) = m[2][2];

      m_map_t_body(0,3) = msg->transforms[l].transform.translation.x;
      m_map_t_body(1,3) = msg->transforms[l].transform.translation.y;
      m_map_t_body(2,3) = msg->transforms[l].transform.translation.z;
      m_map_t_body(3,3) = 1.0;

      m_cvt_quat_x = q.getX();
      m_cvt_quat_y = q.getY();
      m_cvt_quat_z = q.getZ();
      m_cvt_quat_w = q.getW();
    }
    if (msg->transforms[l].child_frame_id==m_depth_base && !m_body_t_cam_check){
      tf::Quaternion q2(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m2(q2);
      m_body_t_cam(0,0) = m2[0][0];
      m_body_t_cam(0,1) = m2[0][1];
      m_body_t_cam(0,2) = m2[0][2];
      m_body_t_cam(1,0) = m2[1][0];
      m_body_t_cam(1,1) = m2[1][1];
      m_body_t_cam(1,2) = m2[1][2];
      m_body_t_cam(2,0) = m2[2][0];
      m_body_t_cam(2,1) = m2[2][1];
      m_body_t_cam(2,2) = m2[2][2];

      m_body_t_cam(0,3) = msg->transforms[l].transform.translation.x;
      m_body_t_cam(1,3) = msg->transforms[l].transform.translation.y;
      m_body_t_cam(2,3) = msg->transforms[l].transform.translation.z;
      m_body_t_cam(3,3) = 1.0;

      m_body_t_cam_check = true; // fixed!!!
    }
  }

  m_map_t_cam = m_map_t_body * m_body_t_cam;
  if (m_body_t_cam_check)
    m_tf_check=true;
}
