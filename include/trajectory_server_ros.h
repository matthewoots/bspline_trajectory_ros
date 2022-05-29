/*
* trajectory_server_ros.h
*
* ---------------------------------------------------------------------
* Copyright (C) 2022 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/
#ifndef TRAJECTORY_SERVER_ROS_H
#define TRAJECTORY_SERVER_ROS_H

#include "trajectory_server.h"

#include <string>
#include <thread>   
#include <mutex>
#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <random>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <mavros_msgs/PositionTarget.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <tf/tf.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace trajectory_server;
using namespace std;

class trajectory_server_ros
{
    private:

        struct pose
        {
            Eigen::Vector3d position;
            Eigen::Quaterniond q;
            Eigen::Vector3d euler;
        };

        ros::NodeHandle _nh;
        ros::Publisher _pos_raw_pub, _traj_pub;
        ros::Subscriber _odom_sub, _pose_sub;
        ros::Subscriber _goal_sub;
        ros::Subscriber _pcl_sensor_sub, _traj_sub;

        ros::Timer _trajectory_opt_timer, _cmd_timer;

        geometry_msgs::Point goal_pose; 

        std::string _id, _odom_or_pose;

        vector<Eigen::Vector3d> goal_vector;

        pose current_state;

        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

        void goal_callback(const nav_msgs::Path::ConstPtr &msg);

        std::mutex odometry_mutex;
        std::mutex trajectory_mutex;
        std::mutex goal_mutex;

        trajectory_server::bspline_server ts;
        bool restart_trajectory_server;
        double _traj_opt_update_hz, _cmd_update_hz;
        double _traj_duration_secs;
        int _order, _des_knot_div;

        void traj_optimization_update_timer(const ros::TimerEvent &);
        void command_update_timer_idx(const ros::TimerEvent &);

    public:
    
        trajectory_server::bspline_server::pva_cmd cmd;
        trajectory_msgs::JointTrajectoryPoint traj_vector_msg;
        geometry_msgs::PoseStamped pose;

        trajectory_server_ros(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            _nh.param<std::string>("agent_id", _id, "drone0");
            _nh.param<std::string>("odom_or_pose", _odom_or_pose, "pose");
            _nh.param<double>("traj_opt_update_hz", _traj_opt_update_hz, 2);
            _nh.param<double>("cmd_update_hz", _cmd_update_hz, 25);

            _nh.param<double>("traj_duration_secs", _traj_duration_secs, 5.0);
            _nh.param<int>("order", _order, 5);
            _nh.param<int>("des_knot_div", _des_knot_div, 5);

            /** @brief Subscriber that receives position data of uav */
            if (_odom_or_pose.compare("pose") == 0)
                _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
                    "/" + _id + "/mavros/local_position/pose", 20, &trajectory_server_ros::pose_callback, this);
            else if (_odom_or_pose.compare("odom") == 0)
                _pose_sub = _nh.subscribe<nav_msgs::Odometry>(
                    "/" + _id + "/mavros/local_position/odom", 20, &trajectory_server_ros::odom_callback, this);
            else
                throw std::logic_error("[ERROR] no pose data subscriber found");

            /** @brief Subscriber that receives goal vector */
            _goal_sub = _nh.subscribe<nav_msgs::Path>(
                    "/" + _id + "/goal", 20, &trajectory_server_ros::goal_callback, this);


            /** @brief Publisher that publishes control raw setpoints */
		    _pos_raw_pub = _nh.advertise<mavros_msgs::PositionTarget>(
			    "/" + _id + "/mavros/setpoint_raw/local", 20);
            _traj_pub = _nh.advertise<trajectory_msgs::JointTrajectory>(
                "/trajectory/points", 20);

            _trajectory_opt_timer = _nh.createTimer(
                ros::Duration(1/_traj_opt_update_hz), 
                &trajectory_server_ros::traj_optimization_update_timer, this, false, false);
            _cmd_timer = _nh.createTimer(
                ros::Duration(1/_cmd_update_hz), 
                &trajectory_server_ros::command_update_timer_idx, this, false, false);

        }

        // We will start the timers when we received a goal

        ~trajectory_server_ros()
        {
            _trajectory_opt_timer.stop();
            _cmd_timer.stop();
        }
        
};

#endif