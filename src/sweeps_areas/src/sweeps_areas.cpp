//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"

#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include "sweeps_areas/sweeps.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>


#include "tf/transform_listener.h"
#include <tf/tf.h>

#include <algorithm>

using namespace std;

/**
 * @brief Map generation node.
 */
class SweepsAreas
{
  std::string p_target_frame_name_;
  std::string p_source_frame_name_;
  double p_sweeps_areas_update_rate_;
  double p_sweeps_areas_publish_rate_;
  int ii;
  int sweeps_point_num;
  geometry_msgs::PoseStamped pose_source_;
  int last_pose_index_x;
  int last_pose_index_y;
  int first_flag;
  float sweeps_areas_count;
  int start_sweeps_flags;

  ros::Timer update_sweeps_areas_timer_;
  ros::Timer publish_sweeps_areas_timer_;

  geometry_msgs::PoseStamped marker_pose_out;

  ros::Subscriber map_sub_;
  ros::Publisher sweeps_areas_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher sweeps_point_cloud_pub;
  sweeps_areas::sweeps sweeps_areas_;

  tf::TransformListener tf_;
  visualization_msgs::Marker line_strip;
  sensor_msgs::PointCloud sweeps_cloud;
public:
  SweepsAreas()
  {
    ros::NodeHandle private_nh("~");
    first_flag = 0;
    ii = 0;
    sweeps_point_num = 0;
    sweeps_areas_count = 0.0;
    private_nh.param("target_frame_name", p_target_frame_name_, std::string("map"));
    private_nh.param("source_frame_name", p_source_frame_name_, std::string("base_link"));
    private_nh.param("sweeps_areas_update_rate", p_sweeps_areas_update_rate_, 3.0);
    private_nh.param("sweeps_areas_publish_rate", p_sweeps_areas_publish_rate_, 2.0);
    //visualization_msgs::Marker points, line_strip;

    waitForTf();

    ros::NodeHandle nh;
    map_sub_ = nh.subscribe("/map", 1, &SweepsAreas::mapCallback, this) ;
    sweeps_areas_pub_ = nh.advertise<sweeps_areas::sweeps>("RobotSweepsAreas",1, true);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    sweeps_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("sweeps_point",50);

    update_sweeps_areas_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_sweeps_areas_update_rate_), &SweepsAreas::sweepsAreasUpdateTimerCallback, this, false);
    publish_sweeps_areas_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_sweeps_areas_publish_rate_), &SweepsAreas::publishSweepsAreasTimerCallback, this, false);

    pose_source_.pose.orientation.w = 1.0;
    pose_source_.header.frame_id = p_source_frame_name_;

    sweeps_areas_.sweeps_areas.header.frame_id = p_target_frame_name_;
  }

  void waitForTf()
  {
    ros::Time start = ros::Time::now();
    ROS_INFO("Waiting for tf transform data between frames %s and %s to become available", p_target_frame_name_.c_str(), p_source_frame_name_.c_str() );

    bool transform_successful = false;

    while (!transform_successful){
      transform_successful = tf_.canTransform(p_target_frame_name_, p_source_frame_name_, ros::Time());
      if (transform_successful) break;

      ros::Time now = ros::Time::now();

      if ((now-start).toSec() > 20.0){
        ROS_WARN_ONCE("No transform between frames %s and %s available after %f seconds of waiting. This warning only prints once.", p_target_frame_name_.c_str(), p_source_frame_name_.c_str(), (now-start).toSec());
      }
      
      if (!ros::ok()) return;
      ros::WallDuration(1.0).sleep();
    }

    ros::Time end = ros::Time::now();
    ROS_INFO("Finished waiting for tf, waited %f seconds", (end-start).toSec());
  }
  
  void mapCallback(const nav_msgs::OccupancyGrid& map_info_)
  {
    sweeps_areas_.sweeps_areas.header.stamp = ros::Time::now();
    sweeps_areas_.sweeps_areas.info = map_info_.info;
    if (sweeps_areas_.sweeps_areas.info.resolution != 0.0)
    {
	start_sweeps_flags = 1;
    }
  }


  void addCurrentTfPoseToSweepsAreas()
  {
    if (start_sweeps_flags == 1)
    {
        pose_source_.header.stamp = ros::Time(0);
        int left_sweeps_radius;
        int right_sweeps_radius;
        int up_sweeps_radius;
        int down_sweeps_radius;

        geometry_msgs::PoseStamped current_pose_out;

        tf_.transformPose(p_target_frame_name_, pose_source_, current_pose_out);

        int current_pose_index_x = (int)((current_pose_out.pose.position.x - sweeps_areas_.sweeps_areas.info.origin.position.x)/sweeps_areas_.sweeps_areas.info.resolution);
        int current_pose_index_y = (int)((current_pose_out.pose.position.y - sweeps_areas_.sweeps_areas.info.origin.position.y)/sweeps_areas_.sweeps_areas.info.resolution);
     
	    
    //初始化
   	 line_strip.header.frame_id = "/map";
   	 line_strip.header.stamp = ros::Time::now();
   	 line_strip.ns = "points_and_lines";
   	 line_strip.action = visualization_msgs::Marker::ADD;
    	 line_strip.pose.orientation.w = 1.0;

   	 line_strip.id = 1;


    //初始化形状
   	 line_strip.type = visualization_msgs::Marker::LINE_STRIP;

   	line_strip.scale.x = 0.36;

    // Line strip is green
    	line_strip.color.g = 1.0;
    	line_strip.color.a = 1.0;

	 geometry_msgs::Point p;
      	 p = current_pose_out.pose.position;
         line_strip.points.push_back(p);

         //frontier_point_sweeps_cloud_pub
  /*       int sweeps_size_all;
        // sweeps_size_all = std::max(sweeps_areas_.sweeps_areas.info.width,sweeps_areas_.sweeps_areas.info.height);
         sweeps_size_all = sweeps_areas_.sweeps_areas.info.width * sweeps_areas_.sweeps_areas.info.height;


                // max_all = frontier_list_new.size();
                 //ROS_INFO("sweeps_size_all = %d" , sweeps_size_all);

                 sweeps_cloud.header.stamp = ros::Time::now();
                 sweeps_cloud.header.frame_id = "/map";

                 sweeps_cloud.points.resize(sweeps_size_all);

                     //we'll also add an intensity channel to the sweeps_cloud
                 sweeps_cloud.channels.resize(1);
                 sweeps_cloud.channels[0].name = "intensities";
                 sweeps_cloud.channels[0].values.resize(sweeps_size_all);*/

	if(first_flag==0)
	{
	    last_pose_index_x = current_pose_index_x;
            last_pose_index_y = current_pose_index_y;
	    first_flag = 1;
	}
	else
	{
	    if ((current_pose_index_x!=last_pose_index_x) || (current_pose_index_y!=last_pose_index_y))
        {
	        int sweeps_radius = (int)(0.18/sweeps_areas_.sweeps_areas.info.resolution);

	        if(current_pose_index_x < sweeps_radius)
	        {
	    	    up_sweeps_radius = current_pose_index_x;
	        }
	        else 
	        {
		    up_sweeps_radius = sweeps_radius;
	        }
	        if(current_pose_index_y < sweeps_radius)
	        {
		    left_sweeps_radius = current_pose_index_y;
	        }
	        else 
	        {
		    left_sweeps_radius = sweeps_radius;
	        }
	        right_sweeps_radius = sweeps_radius;
	        down_sweeps_radius = sweeps_radius;
            sweeps_areas_.sweeps_poses.poses.resize(sweeps_areas_.sweeps_areas.info.width * sweeps_areas_.sweeps_areas.info.height);
            for (int i = (current_pose_index_x - left_sweeps_radius); i <= (current_pose_index_x + right_sweeps_radius ); i++ )
            {
                for(int j = (current_pose_index_y - up_sweeps_radius); j <= (current_pose_index_y + down_sweeps_radius); j++)
                 {
                int k = (int) sqrt((current_pose_index_x - i)*(current_pose_index_x - i) + (current_pose_index_y - j)*(current_pose_index_y - j));
                if (k <= sweeps_radius)
                    {
                        sweeps_areas_.sweeps_poses.poses[sweeps_point_num].position.x = i * sweeps_areas_.sweeps_areas.info.resolution + sweeps_areas_.sweeps_areas.info.origin.position.x;
                        sweeps_areas_.sweeps_poses.poses[sweeps_point_num].position.y = j * sweeps_areas_.sweeps_areas.info.resolution + sweeps_areas_.sweeps_areas.info.origin.position.y;
                        sweeps_point_num = sweeps_point_num + 1;
                        sweeps_areas_.num = sweeps_point_num;
                   /*     sweeps_cloud.points[ii].x = i * sweeps_areas_.sweeps_areas.info.resolution + sweeps_areas_.sweeps_areas.info.origin.position.x;
                        sweeps_cloud.points[ii].y = j * sweeps_areas_.sweeps_areas.info.resolution + sweeps_areas_.sweeps_areas.info.origin.position.y;
                        sweeps_cloud.points[ii].z = 0;
                        sweeps_cloud.channels[0].values[ii] = 100;
                        ii = ii + 1;
                        ROS_INFO("sweeps_cloud_ii = %d, i = %d, j = %d ", ii, i, j);*/
                    }
                 }

            }
        float diff_index_x = current_pose_index_x - last_pose_index_x;
        float diff_index_y = current_pose_index_y - last_pose_index_y;
		float diff_index = sqrt(diff_index_x * diff_index_x + diff_index_y * diff_index_y);
		sweeps_areas_count = sweeps_areas_count + 0.36*0.36*(diff_index/(4*sweeps_radius));
        sweeps_areas_.sweeps_areas_count = sweeps_areas_count;
        }
            last_pose_index_x = current_pose_index_x;
            last_pose_index_y = current_pose_index_y;
	}
     //sweeps_point_cloud_pub.publish(sweeps_cloud);
    }
  }

  void sweepsAreasUpdateTimerCallback(const ros::TimerEvent& event)
  {

    try{
      addCurrentTfPoseToSweepsAreas();
    }catch(tf::TransformException e)
    {
      ROS_WARN("Trajectory Server: Transform from %s to %s failed: %s \n", p_target_frame_name_.c_str(), pose_source_.header.frame_id.c_str(), e.what() );
    }
  }

  void publishSweepsAreasTimerCallback(const ros::TimerEvent& event)
  {
    sweeps_areas_pub_.publish(sweeps_areas_);
   // marker_pub_.publish(points);
    marker_pub_.publish(line_strip);
    sweeps_point_cloud_pub.publish(sweeps_cloud);
  }
   
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sweeps_areas");
  
  SweepsAreas sa;

  ros::spin();

  return 0;
}
