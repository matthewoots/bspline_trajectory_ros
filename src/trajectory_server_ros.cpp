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

void trajectory_server_ros::pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    local_cloud = pcl2_converter(*msg);
    ms.set_local_cloud(local_cloud);
    return;
}

void trajectory_server_ros::other_trajectory_callback(
    const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
    std::string copy_id = msg->joint_names[0]; 
    std::string uav_id_char = copy_id.erase(0,5); // removes first 5 character
    int idx = stoi(uav_id_char);

    vector<Eigen::Vector3d> agent_control_points;
    vector<double> agent_knots;
    for (int i = 0; i < msg->points.size(); i++)
    {
        Eigen::Vector3d point = 
            Eigen::Vector3d(
            msg->points[i].positions[0],
            msg->points[i].positions[1],
            msg->points[i].positions[2]);
        agent_control_points.push_back(point);
        agent_knots.push_back(msg->points[i].effort[0]);
    }

    ms.update_other_agents(idx, agent_control_points, agent_knots);
    
    
    return;
}

void trajectory_server_ros::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
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

void trajectory_server_ros::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::PoseStamped copy_msg = *msg;
    std::cout << "[ROS] Received goal" << std::endl;

    // reset stime in the bspline trajectory server
    ms.reset_bspline_start_time();
    _trajectory_opt_timer.stop();
    _cmd_timer.stop();

    goal = common_utils_ros::point_to_vector(copy_msg.pose.position);
    
    _restart_trajectory_server = true;
    _trajectory_opt_timer.start();
    
    _completed = false;
    _command_timer_time = 0.0;
}

void trajectory_server_ros::traj_optimization_update_timer(const ros::TimerEvent &)
{
    // Do not update the trajectory when the path has ended
    if (_completed)
        return;

    if (_restart_trajectory_server)
    {
        ms.reset_goal_points(
            current_state.position, goal, obs_threshold, search_radius);
        // Reinitialize and start the trajectory server
        ms.initialize_bspline_server(
            _order, _traj_duration_secs, 1/_cmd_update_hz, 
            _des_knot_div, _max_vel);
        ms.initialize_rrt_server(
            _sub_runtime_error, _runtime_error,
            _xybuffer, _zbuffer, _passage_size);
    }

    std::cout << std::endl;
    time_point<std::chrono::system_clock> test_cycle_start = system_clock::now();
    
    ms.complete_path_generation();

    auto test_time_diff = duration<double>(system_clock::now() - test_cycle_start).count();
    std::cout << "[ROS] " << KRED << 
        "Completed complete_path_generation" << 
        KNRM << " in " << KBLU << test_time_diff*1000 << KNRM <<
        "ms" << std::endl;
    std::cout << std::endl;

    vector<Eigen::Vector3d> local_control_points =
        ms.get_bspline_control_points(_traj_duration_secs);

    vector<double> knots_points = 
        ms.get_bspline_knots(_traj_duration_secs);
    
    trajectory_msgs::JointTrajectory joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.joint_names.push_back(_id);

    for (int i = 0; i < local_control_points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(local_control_points[i].x());
        point.positions.push_back(local_control_points[i].y());
        point.positions.push_back(local_control_points[i].z());

        point.effort.push_back(knots_points[i]);

        joint_msg.points.push_back(point);
    }    

    _traj_pub.publish(joint_msg);

    // Before we end the timer process
    // Start the bspline time if starting a new path
    if (_restart_trajectory_server)
    {
        ms.start_module_timer();
        _cmd_timer.start();
        _restart_trajectory_server = false;
    }

}

void trajectory_server_ros::command_update_timer_idx(const ros::TimerEvent &)
{
    // Do not update the command when the path has ended
    _command_timer_time += 1/_cmd_update_hz;
    if (_command_timer_time >= ms.get_end_time() - 1/_cmd_update_hz)
    {
        // reset stime in the bspline trajectory server
        ms.reset_bspline_start_time();
        _completed = true;
        return;
    }

    // Trajectory server/optimization is still setting up, 
    // we have to wait first
    if (ms.get_bspline_time() > 1*60*60*24*31 ||
        ms.get_bspline_time() <= 0.0)
    {
        std::cout << "[ROS] Trajectory server not running" << std::endl;
        return;
    }

    bspline_server::pva_cmd cmd = ms.update_bs_path_get_command();
    // Check if pva_cmd is valid or else we will return
    if(cmd.t <= 0)
        return;
    // IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
    // std::cout << "[ROS] time: " << 
    //     KRED << std::setprecision(3) << ms.get_start_time()  << KNRM <<
    //     " " << 
    //     KBLU << std::setprecision(3) << cmd.t  << KNRM <<
    //     " " << 
    //     KRED << std::setprecision(3) << ms.get_end_time()  << KNRM <<
    //     " position: " << 
    //     KBLU << cmd.p.transpose().format(CleanFmt) << KNRM << 
    //     " total velocity: " << 
    //     KBLU << std::setprecision(3) << cmd.v.norm() << KNRM << std::endl;
    
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

