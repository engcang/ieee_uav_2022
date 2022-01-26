
// #include "../include/bezier_test/predict.h"
#include <bezier_test/predict.h>

using namespace std;
using namespace Eigen;

bezier_traj_class::bezier_traj_class(ros::NodeHandle& nh){
    ROS_WARN("Class generating...");
    ROS_INFO("Class generating...");
    cout << '3' << endl;
    ///// params
    // getParam();

    ///// sub pub
    pub_pre_traj = nh.advertise<visualization_msgs::Marker>("bezier/vis_pre_traj", 1);
    pub_past_traj = nh.advertise<visualization_msgs::Marker>("bezier/vis_past_traj", 1);
    sub_tar_pose = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &bezier_traj_class::get_pose, this);
    replan_timer = nh.createTimer(ros::Duration(1/replan_frequency), &bezier_traj_class::predict, this);
}

bezier_traj_class::~bezier_traj_class(){

}

void bezier_traj_class::get_pose(const geometry_msgs::PoseStamped msg)  {
    static bool flag_tar = false;
    static bool flag_past = false;
    Eigen::Vector4d state(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.header.stamp.toSec());
    if(!flag_tar)   {
        target_detect_list.push_back(state);
        if(target_detect_list.size() >= Max_L){
            flag_tar = 1;
        }
    }
    else{
        target_detect_list.erase(target_detect_list.begin());
        target_detect_list.push_back(state);
    }

    if(!flag_past)   {
        past_detect_list.push_back(state);
        if(past_detect_list.size() >= Max_past){
            flag_past = 1;
        }
    }
    else{
        past_detect_list.erase(past_detect_list.begin());
        past_detect_list.push_back(state);
    }
    
    visualize_past(past_detect_list);
    // last_rcvtime = ros::Time::now().toSec();
}

void bezier_traj_class::predict(const ros::TimerEvent& event)   {
    static std::vector<Eigen::Matrix<double,6,1>> predict_state_list;   //pos(3) + vel(3)
    static std::vector<Eigen::Vector3d> Sample_list;
    if(target_detect_list.size() < Max_L){
        return;
    }

    int bezier_flag = tgpredict.TrackingGeneration(5, 5, target_detect_list);   // max_vel, max_acc
    if(bezier_flag == 0){
        predict_state_list = tgpredict.getStateListFromBezier(_PREDICT_SEG);
        Sample_list = tgpredict.SamplePoslist_bezier(_PREDICT_SEG); // 0.05 * _PREDICT_SEG [sec]
    }
    else{
        ROS_WARN("bezier predict error");
    }
    if(predict_state_list.size() < 1) {
        ROS_ERROR("Bezier predict failed");    
        return;
    }

    visualize_sample(Sample_list);
}

void bezier_traj_class::visualize_sample(std::vector<Eigen::Vector3d> poslist) {
    visualization_msgs::Marker _pred_vis;
    _pred_vis.header.stamp       = ros::Time::now();
    _pred_vis.header.frame_id    = "map";
    _pred_vis.ns = "/tracking_pred";
    _pred_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _pred_vis.action = visualization_msgs::Marker::ADD;
    _pred_vis.scale.x = 0.1;
    _pred_vis.scale.y = 0.1;
    _pred_vis.scale.z = 0.1;
    _pred_vis.pose.orientation.x = 0.0;
    _pred_vis.pose.orientation.y = 0.0;
    _pred_vis.pose.orientation.z = 0.0;
    _pred_vis.pose.orientation.w = 1.0;
    _pred_vis.color.a = 1.0;
    _pred_vis.color.r = 0.0;
    _pred_vis.color.g = 1.0;
    _pred_vis.color.b = 0.0;//black
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;
    for(unsigned int i=0;i<poslist.size();i++){
        pos  = poslist[i];
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _pred_vis.points.push_back(pt);
    }
    pub_pre_traj.publish(_pred_vis);
}

void bezier_traj_class::visualize_past(std::vector<Eigen::Vector4d> poslist) {
    visualization_msgs::Marker _pred_vis;
    _pred_vis.header.stamp       = ros::Time::now();
    _pred_vis.header.frame_id    = "map";
    _pred_vis.ns = "/tracking_pred_past";
    _pred_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _pred_vis.action = visualization_msgs::Marker::ADD;
    _pred_vis.scale.x = 0.1;
    _pred_vis.scale.y = 0.1;
    _pred_vis.scale.z = 0.1;
    _pred_vis.pose.orientation.x = 0.0;
    _pred_vis.pose.orientation.y = 0.0;
    _pred_vis.pose.orientation.z = 0.0;
    _pred_vis.pose.orientation.w = 1.0;
    _pred_vis.color.a = 1.0;
    _pred_vis.color.r = 1.0;
    _pred_vis.color.g = 0.0;
    _pred_vis.color.b = 0.0;//black
    Eigen::Vector4d pos;
    geometry_msgs::Point pt;
    for(unsigned int i=0;i<poslist.size();i++){
        pos  = poslist[i];
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _pred_vis.points.push_back(pt);
    }
    pub_past_traj.publish(_pred_vis);
}



// void bezier_traj_class::visualize(std::vector<Eigen::Vector3d> poslist, int flag) {
//     visualization_msgs::Marker _pred_vis;
//     _pred_vis.header.stamp       = ros::Time::now();
//     _pred_vis.header.frame_id    = "map";
//     _pred_vis.ns = "/tracking_pred";
//     _pred_vis.type = visualization_msgs::Marker::SPHERE_LIST;
//     _pred_vis.action = visualization_msgs::Marker::ADD;
//     _pred_vis.scale.x = 0.1;
//     _pred_vis.scale.y = 0.1;
//     _pred_vis.scale.z = 0.1;
//     _pred_vis.pose.orientation.x = 0.0;
//     _pred_vis.pose.orientation.y = 0.0;
//     _pred_vis.pose.orientation.z = 0.0;
//     _pred_vis.pose.orientation.w = 1.0;
//     if(flag == SAMPLE)  {
//         _pred_vis.color.a = 1.0;
//         _pred_vis.color.r = 0.0;
//         _pred_vis.color.g = 1.0;
//         _pred_vis.color.b = 0.0;//green
//     }
//     else if(flag == PAST)   {
//         _pred_vis.color.a = 1.0;
//         _pred_vis.color.r = 1.0;
//         _pred_vis.color.g = 0.0;
//         _pred_vis.color.b = 0.0;//red
//     }
//     else{
//         _pred_vis.color.a = 1.0;
//         _pred_vis.color.r = 1.0;
//         _pred_vis.color.g = 1.0;
//         _pred_vis.color.b = 1.0;//black
//     }
//     Eigen::Vector3d pos;
//     geometry_msgs::Point pt;
//     for(unsigned int i=0;i<poslist.size();i++){
//         pos  = poslist[i];
//         pt.x = pos(0);
//         pt.y = pos(1);
//         pt.z = pos(2);
//         _pred_vis.points.push_back(pt);
//     }
//     if(flag == SAMPLE)  pub_pre_traj.publish(_pred_vis);
//     else if(flag == PAST)  pub_past_traj.publish(_pred_vis);
// }
