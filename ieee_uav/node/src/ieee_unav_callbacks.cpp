#include "ieee_unav.h"


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

      if(m_gt_check){

        MatrixXf center_before_tf(4,1), center_after_tf(4,1);
        center_before_tf << p3d_center.x, p3d_center.y, p3d_center.z, 1.0;
        center_after_tf = m_map_t_cam * center_before_tf;

        target_poly_traj->get_pose(center_after_tf(0), center_after_tf(1), center_after_tf(2), bboxMsg->header.stamp.toSec());
      
        std::vector<Eigen::Matrix<double,6,1>> predict_target_list = target_poly_traj->predict_state_list;
        if (predict_target_list.size() > m_target_predict_seg){
          ieee_uav::odom_array pub_odom_array;
          for (int i = 0; i < predict_target_list.size(); ++i){
            nav_msgs::Odometry odom_for_control;
            odom_for_control.pose.pose.position.x = predict_target_list[i](0);
            odom_for_control.pose.pose.position.y = predict_target_list[i](1);
            // odom_for_control.pose.pose.position.x = center_after_tf(0);
            // odom_for_control.pose.pose.position.y = center_after_tf(1);
            odom_for_control.pose.pose.position.z = m_altitude_fixed;
            odom_for_control.pose.pose.orientation.w = 1.0;

            odom_for_control.twist.twist.linear.x = predict_target_list[i](3);
            odom_for_control.twist.twist.linear.y = predict_target_list[i](4);
            odom_for_control.twist.twist.linear.z = predict_target_list[i](5);
            pub_odom_array.array.push_back(odom_for_control);
          }
          m_goal_traj_pub.publish(pub_odom_array);
        } 
      }
    }

  }
}

void ieee_uav_class::gt_callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  for (int i = 0; i < msg->pose.size(); ++i){
    if (msg->name[i] == "iris"){
      tf::Quaternion q(msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z, msg->pose[i].orientation.w);
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

      m_map_t_body(0,3) = msg->pose[i].position.x;
      m_map_t_body(1,3) = msg->pose[i].position.y;
      m_map_t_body(2,3) = msg->pose[i].position.z;
      m_map_t_body(3,3) = 1.0;

      m_cvt_quat_x = q.getX();
      m_cvt_quat_y = q.getY();
      m_cvt_quat_z = q.getZ();
      m_cvt_quat_w = q.getW();
    }
  }
  if (m_body_t_cam_check){
    m_gt_check = true;
    m_map_t_cam = m_map_t_body * m_body_t_cam;
  }

}
void ieee_uav_class::tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg){
  if(m_body_t_cam_check)
    return;
  for (int l=0; l < msg->transforms.size(); l++){
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
}
