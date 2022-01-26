#ifndef _PREDICT_H
#define _PREDICT_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <bezier_test/bezier_predict.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <signal.h>
inline void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

#define Max_L 30
#define Max_past 300

#define SAMPLE 0 
#define PAST 1 

class bezier_traj_class   {
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_pre_traj;
        ros::Publisher pub_past_traj;
        ros::Subscriber sub_tar_pose;
        ros::Timer replan_timer;

        Bezierpredict tgpredict;

        std::vector<Eigen::Vector4d> target_detect_list;
        std::vector<Eigen::Vector4d> past_detect_list;

        double replan_frequency = 12.0;
        // double last_rcvtime;

        void get_pose(const geometry_msgs::PoseStamped msg);
        void predict(const ros::TimerEvent& event);
        void visualize(std::vector<Eigen::Vector3d> poslist, int flag);

    public:
        bezier_traj_class();
        ~bezier_traj_class();
};

#endif