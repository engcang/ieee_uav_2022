#include "predict.h"

using namespace std;
using namespace Eigen;

bezier_traj_class::bezier_traj_class(ros::NodeHandle& nh, double replan_hz, int leng_1, int leng_2, string fixed_frame) : max_l(leng_1), max_p(leng_2), m_fixed_frame(fixed_frame)
{
    ROS_WARN("Class generating...");

    ///// sub pub
    pub_pre_traj = nh.advertise<nav_msgs::Path>("bezier/vis_pre_traj", 1);
    pub_past_traj = nh.advertise<nav_msgs::Path>("bezier/vis_past_traj", 1);
    replan_timer = nh.createTimer(ros::Duration(1/replan_hz), &bezier_traj_class::predict, this);
}

bezier_traj_class::~bezier_traj_class(){

}


void bezier_traj_class::get_pose(double x, double y, double z, double t){
    static bool flag_tar = false;
    static bool flag_past = false;
    Eigen::Vector4d state(x, y, z, t);
    if(!flag_tar)   {
        target_detect_list.push_back(state);
        if(target_detect_list.size() >= max_l){
            flag_tar = 1;
        }
    }
    else{
        target_detect_list.erase(target_detect_list.begin());
        target_detect_list.push_back(state);
    }

    // if(!flag_past)   {
    //     past_detect_list.push_back(state);
    //     if(past_detect_list.size() >= max_p){
    //         flag_past = 1;
    //     }
    // }
    // else{
    //     past_detect_list.erase(past_detect_list.begin());
    //     past_detect_list.push_back(state);
    // }
}

void bezier_traj_class::predict(const ros::TimerEvent& event){
    if(target_detect_list.size() < max_l){
        return;
    }

    int bezier_flag = tgpredict.TrackingGeneration(5, 5, target_detect_list);   // max_vel, max_acc
    if(bezier_flag == 0){
        predict_state_list = tgpredict.getStateListFromBezier(_PREDICT_SEG);
        past_fitted_list = tgpredict.getStateListFromBezier_past();
        Sample_list = tgpredict.SamplePoslist_bezier(_PREDICT_SEG); // 0.05 * _PREDICT_SEG [sec]
    }
    else{
        ROS_WARN("bezier predict error");
    }
    if(predict_state_list.size() < 1) {
        ROS_ERROR("Bezier predict failed");    
        return;
    }

    visualize_traj(Sample_list, pub_pre_traj);
    visualize_traj(past_fitted_list, pub_past_traj);
}


template <typename T>
void bezier_traj_class::visualize_traj(std::vector<T> poslist, ros::Publisher publisher) {
    nav_msgs::Path _pred_vis;
    _pred_vis.header.stamp       = ros::Time::now();
    _pred_vis.header.frame_id    = m_fixed_frame;

    for(unsigned int i=0;i<poslist.size();i++){
        geometry_msgs::PoseStamped pt;
        pt.pose.position.x = poslist[i](0);
        pt.pose.position.y = poslist[i](1);
        pt.pose.position.z = poslist[i](2);
        pt.pose.orientation.w = 1.0;
        _pred_vis.poses.push_back(pt);
    }

    publisher.publish(_pred_vis);
}