#ifndef _PREDICT_H
#define _PREDICT_H

#include "bezier_predict.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <vector>

class bezier_traj_class{
    private:
        ros::Publisher pub_pre_traj;
        ros::Publisher pub_past_traj;
        ros::Timer replan_timer;

        Bezierpredict tgpredict;

        std::string m_fixed_frame;

        std::vector<Eigen::Vector4d> target_detect_list;
        std::vector<Eigen::Matrix<double,6,1>> past_fitted_list;
        // std::vector<Eigen::Vector4d> past_detect_list;

        void predict(const ros::TimerEvent& event);
        template <typename T>
        void visualize_traj(std::vector<T> poslist, ros::Publisher publisher);

        int max_l=30; // past observation numbers to estimate polynomial
        int max_p=300; // past observation numbers to visualize only

    public:
        void get_pose(double x, double y, double z, double t);
        std::vector<Eigen::Matrix<double,6,1>> predict_state_list;   //pos(3) + vel(3)
        std::vector<Eigen::Vector3d> Sample_list;

        bezier_traj_class(ros::NodeHandle& nh, double replan_hz, int leng_1, int leng_2, string fixed_frame);
        ~bezier_traj_class();
};

#endif