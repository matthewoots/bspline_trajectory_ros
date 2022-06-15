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
#include "main_server.h"

#include <string>
#include <thread>   
#include <mutex>
#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <random>
#include <chrono>
#include <ctime>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <mavros_msgs/PositionTarget.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

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

        pose current_state;

        trajectory_server::bspline_server ts;
        trajectory_server::main_server ms;

        ros::NodeHandle _nh;
        ros::Publisher _pos_raw_pub, _traj_pub;
        
        ros::Subscriber _odom_sub, _pose_sub;
        ros::Subscriber _goal_sub, _local_pcl_sub;
        ros::Subscriber _pcl_sensor_sub, _traj_sub;

        ros::Timer _trajectory_opt_timer, _cmd_timer;

        geometry_msgs::Point goal_pose; 

        nav_msgs::Path path;

        std::string _id, _odom_or_pose;

        Eigen::Vector3d goal;

        pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud;

        bool _restart_trajectory_server;
        bool _completed;
        double _traj_opt_update_hz, _cmd_update_hz;
        double _traj_duration_secs;
        double _command_timer_time;
        double _max_vel, _max_acc;
        int _order, _des_knot_div;

        double _runtime_error, _sub_runtime_error, _search_interval;
        double min_height, max_height;
        double search_radius, obs_threshold;
        double _xybuffer, _zbuffer, _passage_size;

        vector<double> weight_vector{0.0, 0.0, 0.0, 0.0, 0.0};

        void traj_optimization_update_timer(const ros::TimerEvent &);
        
        void command_update_timer_idx(const ros::TimerEvent &);

        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

        void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

        void other_trajectory_callback(
            const trajectory_msgs::JointTrajectoryConstPtr& msg);


    public:
    
        trajectory_server::bspline_server::pva_cmd cmd;
        trajectory_msgs::JointTrajectoryPoint traj_vector_msg;
        geometry_msgs::PoseStamped pose;

        trajectory_server_ros(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            _nh.param<std::string>("agent_id", _id, "drone0");
            _nh.param<std::string>("odom_or_pose", _odom_or_pose, "pose");
            _nh.param<double>("traj_opt_update_hz", _traj_opt_update_hz, 1);
            _nh.param<double>("cmd_update_hz", _cmd_update_hz, 1);

            /** @brief Bspline parameters **/
            _nh.param<double>("traj_duration_secs", _traj_duration_secs, 5.0);
            _nh.param<double>("max_velocity", _max_vel, 1.0);
            _nh.param<int>("order", _order, 5);
            _nh.param<int>("des_knot_div", _des_knot_div, 5);

            /** @brief RRT parameters **/
            _nh.param<double>("runtime_error", _runtime_error, 0.1);
            _nh.param<double>("sub_runtime_error", _sub_runtime_error, 0.01);  
            _nh.param<double>("search_radius", search_radius, 5.0);
            _nh.param<double>("threshold", obs_threshold, 0.5); 
            _nh.param<double>("search_interval", _search_interval, 0.5);

            _nh.param<double>("xybuffer", _xybuffer, 1); 
            _nh.param<double>("zbuffer", _zbuffer, 1); 
            _nh.param<double>("passage_size", _passage_size, 1);

            _nh.param<double>("weight_smooth", weight_vector[0], 1); 
            _nh.param<double>("weight_feas", weight_vector[1], 1); 
            _nh.param<double>("weight_term", weight_vector[2], 1);  
            _nh.param<double>("weight_static", weight_vector[3], 1);  
            _nh.param<double>("weight_reci", weight_vector[4], 1);  

            _nh.param<double>("max_acc", _max_acc, 1);

            std::vector<double> height_list;
            _nh.getParam("height", height_list);
 
            min_height = height_list[0];
            max_height = height_list[1];

            /** @brief Subscriber that receives position data of uav */
            if (_odom_or_pose.compare("pose") == 0)
                _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
                    "/" + _id + "/mavros/local_position/pose", 20, &trajectory_server_ros::pose_callback, this);
            else if (_odom_or_pose.compare("odom") == 0)
                _pose_sub = _nh.subscribe<nav_msgs::Odometry>(
                    "/" + _id + "/mavros/local_position/pose", 20, &trajectory_server_ros::odom_callback, this);
            else
                throw std::logic_error("[ERROR] no pose data subscriber found");

            /** @brief Subscriber that receives goal vector */
            _goal_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
                    "/" + _id + "/goal", 10, &trajectory_server_ros::goal_callback, this);

            /** @brief Subscriber that receives pointcloud */
            _local_pcl_sub = _nh.subscribe<sensor_msgs::PointCloud2>(
                    "/" + _id + "/local_pcl", 5,  &trajectory_server_ros::pcl2_callback, this);

            _traj_sub = _nh.subscribe<trajectory_msgs::JointTrajectory>(
                "/trajectory/points", 100,  &trajectory_server_ros::other_trajectory_callback, this);


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

            std::string copy_id = _id; 
            std::string uav_id_char = copy_id.erase(0,5); // removes first 5 character
            int uav_id = stoi(uav_id_char);

            ms.initialize_opt_server(weight_vector, _max_acc, uav_id);
        }

        // We will start the timers when we received a goal

        ~trajectory_server_ros()
        {
            _trajectory_opt_timer.stop();
            _cmd_timer.stop();
        }

        /** @brief Convert point cloud from ROS sensor message to pcl point ptr **/
        pcl::PointCloud<pcl::PointXYZ>::Ptr 
            pcl2_converter(sensor_msgs::PointCloud2 _pc)
        {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(_pc, pcl_pc2);

            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
            
            return tmp_cloud;
        }

        nav_msgs::Path vector_3d_to_path(vector<Vector3d> path_vector)
        {
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "world";
            for (int i = 0; i < path_vector.size(); i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = path_vector[i][0];
                pose.pose.position.y = path_vector[i][1];
                pose.pose.position.z = path_vector[i][2];
                path.poses.push_back(pose);
            }

            return path;
        }
        
};

#endif