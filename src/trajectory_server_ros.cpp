/*
* trajectory_server_ros.cpp
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

#include "trajectory_server_ros.h"
#include "common_utils_ros.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;
using namespace common_utils_ros;

void trajectory_server_ros::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> odom_lock(odometry_mutex);
    Affine3d nwu_transform = Affine3d::Identity();
        
    // Local position in NWU frame
    nwu_transform.translation() = Vector3d(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    );
    // Local rotation in NWU frame
    nwu_transform.linear() = Quaterniond(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z).toRotationMatrix();

    current_state.euler = common_utils_ros::r_to_euler_rpy(nwu_transform.linear());
    current_state.position = nwu_transform.translation();
    current_state.q = nwu_transform.linear();
}

void trajectory_server_ros::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> odom_lock(odometry_mutex);
    Affine3d nwu_transform = Affine3d::Identity();
        
    // Local position in NWU frame
    nwu_transform.translation() = Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    // Local rotation in NWU frame
    nwu_transform.linear() = Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z).toRotationMatrix();

    current_state.euler = common_utils_ros::r_to_euler_rpy(nwu_transform.linear());
    current_state.position = nwu_transform.translation();
    current_state.q = nwu_transform.linear();
}

void trajectory_server_ros::goal_callback(const nav_msgs::Path::ConstPtr &msg)
{
    std::lock_guard<std::mutex> goal_lock(goal_mutex);
    nav_msgs::Path copy_msg = *msg;
    
    goal_vector.clear();
    _trajectory_opt_timer.stop();
    
    for (int i = 0; i < (int)copy_msg.poses.size(); i++)
    {
        goal_vector.push_back(
            common_utils_ros::point_to_vector(copy_msg.poses[i].pose.position));
    }

    restart_trajectory_server = true;
    _trajectory_opt_timer.start();
    _cmd_timer.start();
}

void trajectory_server_ros::traj_optimization_update_timer(const ros::TimerEvent &)
{
    if (restart_trajectory_server)
    {
        // Reinitialize and start the trajectory server
        // int _order, double _duration_secs, double _command_interval, int _knot_div) 
        ts.init_bspline_server(
            _order, _traj_duration_secs, 1/_cmd_update_hz, _des_knot_div);
        restart_trajectory_server = false;
    }

}

void trajectory_server_ros::command_update_timer_idx(const ros::TimerEvent &)
{
    // Trajectory server/optimization is still setting up, 
    // we have to wait first
    if (ts.get_running_time() > 1*60*60*24*31 ||
        ts.get_running_time() <= 0.0)
        return;
    
    bspline_server::pva_cmd cmd = ts.update_get_command_on_path_by_idx();
    // Check if pva_cmd is valid or else we will return
    if(cmd.t <= 0)
        return;
    
    mavros_msgs::PositionTarget pt;
    pt.header.stamp = ros::Time::now();
    pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pt.type_mask = 2496; // Ignore Acceleration
    pt.position = vector_to_point(cmd.p);
    pt.velocity = eigen_vector_to_vector3(cmd.v);
    pt.acceleration_or_force = eigen_vector_to_vector3(cmd.a);
    pt.yaw = cmd.yaw;

    _pos_raw_pub.publish(pt);
}

