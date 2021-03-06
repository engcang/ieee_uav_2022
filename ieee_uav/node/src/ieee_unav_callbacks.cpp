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
  auto cropped_msg = in_bbox.crop;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(cropped_msg, sensor_msgs::image_encodings::BGR8);
  
  cv::Mat hsv_img, color_mask;
  cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);
  int num_nonzero = get_hsv_mask(hsv_img, color_mask, m_color_params);
  // ToDo. What if num_nonzero == 0?
  if (m_color_params.verbose) { cout << "Total # of non-zero: " << num_nonzero << endl; }

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
          if (color_mask.at<uchar>(i - in_bbox.y, j - in_bbox.x) != 0) {
            p3d.z = (temp_depth/m_scale_factor);
            p3d.x = ( j - m_c_x ) * p3d.z / m_f_x;
            p3d.y = ( i - m_c_y ) * p3d.z / m_f_y;

            p3d_center.x += p3d.x;
            p3d_center.y += p3d.y;
            p3d_center.z += p3d.z;
            pcl_size++;
            
            cv_ptr->image.at<cv::Vec3b>(i - in_bbox.y, j - in_bbox.x)[0] = 0;
            cv_ptr->image.at<cv::Vec3b>(i - in_bbox.y, j - in_bbox.x)[1] = 255;
            cv_ptr->image.at<cv::Vec3b>(i - in_bbox.y, j - in_bbox.x)[2] = 0;
          }
        }
      }
    }
    if (pcl_size > 0){
      p3d_center.x /= (float)pcl_size;
      p3d_center.y /= (float)pcl_size;
      p3d_center.z /= (float)pcl_size;
      
      if (m_gt_check) {
        pcl::PointXYZ p3d_center_compensated;
        compensate_motion(p3d_center, p3d_center_compensated, bboxMsg->header.stamp.toSec());
        p3d_center = p3d_center_compensated;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr detected_center_pub(new pcl::PointCloud<pcl::PointXYZ>());
      detected_center_pub->push_back( pcl::PointXYZ(p3d_center.x, p3d_center.y, p3d_center.z) );
      m_detected_target_pcl_pub.publish(cloud2msg(*detected_center_pub, m_depth_base));
      sensor_msgs::ImagePtr binary_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
      m_mask_pub.publish(binary_img);
    
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

int ieee_uav_class::get_hsv_mask(const cv::Mat& hsv_img, cv::Mat& mask, const color_extraction_params& params) {
  if (params.verbose) { cout << "Curr. size - h: " << hsv_img.rows << ", w: " << hsv_img.cols << endl; }
  int min_size = min(hsv_img.cols, hsv_img.rows);
  // Color extraction
  cv::Mat red_mask, white_mask;
  if (params.mode == "red" || params.mode == "both") {
    // Extract red color regions
    // https://stackoverflow.com/questions/32522989/opencv-better-detection-of-red-color
    cv::Mat tmp_mask1, tmp_mask2;
    inRange(hsv_img, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), tmp_mask1);
    inRange(hsv_img, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), tmp_mask2);
    cv::bitwise_or(tmp_mask1, tmp_mask2, red_mask);
    
    // Erosion kernel size is changed in an adaptive way
    int k_size;
    if (min_size < m_color_params.erosion_adaptive_size) {
      k_size = params.erosion_small_kernel_size;
    } else {
      k_size = params.erosion_large_kernel_size;
    }
    if (params.verbose) { cout << min_size << " => kernel size is set to " << k_size << endl; }
    cv::erode(red_mask, red_mask, cv::Mat::ones(cv::Size(k_size, k_size), CV_8UC1), cv::Point(-1,-1), 2);
  } 

  if (params.mode == "white" || params.mode == "both") {
    // https://stackoverflow.com/questions/22588146/tracking-white-color-using-python-opencv
    inRange(hsv_img, cv::Scalar(0, 0, 255 - params.sensitivity), cv::Scalar(255, params.sensitivity, 255), white_mask);
  }	

  // Selection of corresponding mask
  if (params.mode == "red") {
    mask = red_mask;
  } else if (params.mode == "white") { 
    mask = white_mask;
  } else if (params.mode == "both") { 
    cv::bitwise_or(red_mask, white_mask, mask);
  } else { throw invalid_argument("Not implemented!"); }
  return cv::countNonZero(mask);
}

void ieee_uav_class::gt_callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  double time_sec = 0;
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
      
      // ToDo. gazebo_msgs::ModelStates doesn't provide timestamp...
      time_sec = ros::Time::now().toSec();
    }
  }
  if (m_body_t_cam_check){
    m_gt_check = true;
    m_map_t_cam = m_map_t_body * m_body_t_cam;
    
    m_map_t_cam_poses.push_back({time_sec, m_map_t_cam}); 
    while (m_map_t_cam_poses.size() > MAX_QUEUE_SIZE) { 
      m_map_t_cam_poses.pop_front();
    }
  }
}

void ieee_uav_class::compensate_motion(const pcl::PointXYZ& pt, pcl::PointXYZ& update, double t, bool verbose) {
  if (verbose) { cout << " ===== Motion Compensation start =====" << endl; }
  if (m_map_t_cam_poses.size() < 2) {
    update = pt;   
    return;
  }
  if (t < m_map_t_cam_poses[0].first) {
      update = pt;   
    return;
  }
  
  double ratio = 1.0; // For defensive programming
  while (m_map_t_cam_poses.size() > 2) {
    double t0 = m_map_t_cam_poses[0].first;
    double t1 = m_map_t_cam_poses[1].first; 
    if (t0 < t && t < t1) {
      ratio = (t - t0) / (t1 - t0);
      break; 
    } else {
      m_map_t_cam_poses.pop_front();
    } 
  }
  if (verbose) { cout << "Ratio: " << ratio << " || Queue size: " << m_map_t_cam_poses.size() << endl; }
  geometry_msgs::Pose p0 = eigen2geoPose(m_map_t_cam_poses[0].second);
  geometry_msgs::Pose p1 = eigen2geoPose(m_map_t_cam_poses[1].second);

  geometry_msgs::Pose p_interp = interpolate(p0, p1, ratio); 
  Matrix4f map_t_cam_interp = geoPose2eigen(p_interp);
  
  MatrixXf pt_h(4,1), pt_h_world(4,1), pt_h_updated(4,1);
  pt_h << pt.x, pt.y, pt.z, 1.0;
  pt_h_world = map_t_cam_interp * pt_h;
  pt_h_updated = m_map_t_cam_poses.back().second.inverse() * pt_h_world;
  
  update.x = pt_h_updated(0, 0);
  update.y = pt_h_updated(1, 0);
  update.z = pt_h_updated(2, 0);
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
