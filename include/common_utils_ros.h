/*
 * common_utils_ros.h
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
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

#ifndef COMMON_UTILS_ROS_H
#define COMMON_UTILS_ROS_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>

using namespace Eigen;
using namespace std;

#define dmax std::numeric_limits<double>::max();

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

namespace common_utils_ros
{
    inline Vector3d r_to_euler_rpy(Matrix3d R)
    {
        Vector3d euler_out;
        // Each vector is a row of the matrix
        Vector3d m_el[3];
        m_el[0] = Vector3d(R(0,0), R(0,1), R(0,2));
        m_el[1] = Vector3d(R(1,0), R(1,1), R(1,2));
        m_el[2] = Vector3d(R(2,0), R(2,1), R(2,2));

        // Check that pitch is not at a singularity
        if (abs(m_el[2].x()) >= 1)
        {
            euler_out.z() = 0;

            // From difference of angles formula
            double delta = atan2(m_el[2].y(),m_el[2].z());
            if (m_el[2].x() < 0)  //gimbal locked down
            {
                euler_out.y() = M_PI / 2.0;
                euler_out.x() = delta;
            }
            else // gimbal locked up
            {
                euler_out.y() = -M_PI / 2.0;
                euler_out.x() = delta;
            }
        }
        else
        {
            euler_out.y() = - asin(m_el[2].x());

            euler_out.x() = atan2(m_el[2].y()/cos(euler_out.y()), 
                m_el[2].z()/cos(euler_out.y()));

            euler_out.z() = atan2(m_el[1].x()/cos(euler_out.y()), 
                m_el[0].x()/cos(euler_out.y()));
        }

        return euler_out;
    }

    /** @brief Filter/Crop point cloud with the dimensions given */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr_box_crop(
        pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, 
        Eigen::Vector3d centroid, Eigen::Vector3d dimension)
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(
            new pcl::PointCloud<pcl::PointXYZ>);

        Eigen::Vector3d min = centroid - (dimension / 2);
        Eigen::Vector3d max = centroid + (dimension / 2);

        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(min.x(), min.y(), min.z(), 1.0));
        box_filter.setMax(Eigen::Vector4f(max.x(), max.y(), max.z(), 1.0));

        box_filter.setInputCloud(_pc);
        box_filter.filter(*output);

        return output;
    }

    inline geometry_msgs::Point vector_to_point(Vector3d v)
    {
        geometry_msgs::Point tmp;
        tmp.x = v.x(); 
        tmp.y = v.y(); 
        tmp.z = v.z();

        return tmp;
    }

    inline Vector3d point_to_vector(geometry_msgs::Point p)
    {
        Vector3d tmp;
        tmp.x() = p.x; 
        tmp.y() = p.y; 
        tmp.z() = p.z;

        return tmp;
    }

    inline Vector3d vector3_to_eigen_vector(geometry_msgs::Vector3 p)
    {
        Vector3d tmp;
        tmp.x() = p.x; 
        tmp.y() = p.y; 
        tmp.z() = p.z;

        return tmp;
    }

    inline geometry_msgs::Vector3 eigen_vector_to_vector3(Vector3d p)
    {
        geometry_msgs::Vector3 tmp;
        tmp.x = p.x(); 
        tmp.y = p.y(); 
        tmp.z = p.z();

        return tmp;
    }

    inline geometry_msgs::Quaternion quaternion_to_orientation(Quaterniond q)
    {
        geometry_msgs::Quaternion tmp;
        tmp.x = q.x(); 
        tmp.y = q.y(); 
        tmp.z = q.z();
        tmp.w = q.w();

        return tmp;
    }

    inline Quaterniond orientation_to_quaternion(geometry_msgs::Quaternion q)
    {
        Quaterniond tmp;
        tmp.x() = q.x; 
        tmp.y() = q.y; 
        tmp.z() = q.z;
        tmp.w() = q.w;

        return tmp;
    }
};

#endif