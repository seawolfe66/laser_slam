/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, PartnerX, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of PartnerX , Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Evan Zhu
* First building Date : 2017/9/15
* Last modified Date :  
*********************************************************************/

#ifndef CLEANER_LOCAL_PLANNER_H_
#define CLEANER_LOCAL_PLANNER_H_


#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/GetPlan.h>

#include <dynamic_reconfigure/server.h>
#include <cleaner_planner_plugin/CleanerLocalPlannerConfig.h>
#include <pluginlib/class_loader.h>
#include <cleaner_planner_plugin/ChangePlanner.h>
namespace cleaner_local_planner{

    /**
    * @class CleanerLocalPlanner
    * @brief Provides a simple local planner that will change predefined planner dynamically
    */
    class CleanerLocalPlanner : public nav_core::BaseLocalPlanner{
        public:
        /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid velocity command was found, false otherwise
       */
       bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
       bool isGoalReached() ;

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
       bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) ;

      /**
       * @brief  Constructs the local planner
       * @param name The name to give this instance of the local planner
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to local plans
       */
       void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) ;

 
        /**
        * @brief  Virtual destructor for the interface
        */
        virtual ~CleanerLocalPlanner(){}
        CleanerLocalPlanner();

        protected:
          /**
        *@brief Publish the global plan for visulatation.
        *@param path the position set to visualize 
        */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        /**
        *@brief Reconfigure config_
        */
        void reconfigureCB(cleaner_planner_plugin::CleanerLocalPlannerConfig &config, uint32_t level);
        bool changePlanner(std::string newPlanner);
        bool changePlannerService(cleaner_planner_plugin::ChangePlanner::Request &req, cleaner_planner_plugin::ChangePlanner::Response &res);
        private:
        
        dynamic_reconfigure::Server<cleaner_planner_plugin::CleanerLocalPlannerConfig> *dsrv_;/// the dynamic reconfigure server for modifing parameters 
        
        cleaner_planner_plugin::CleanerLocalPlannerConfig default_config_;///start config
        
        cleaner_planner_plugin::CleanerLocalPlannerConfig config_;///reconfigure config
        
        std::string             local_planner_;///current local planner
        
        pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;/// classloader to load local planner plugin
        
        ros::Publisher plan_pub_; /// a publisher for visualizing global plan path
         
        bool initialized_; /// the flag indicated the initialization status
        
        boost::shared_ptr<nav_core::BaseLocalPlanner> planner_;/// current local planner plugin 
        
         costmap_2d::Costmap2DROS* planner_costmap_ros_;///costmap 
        
        tf::TransformListener* tf_; ///used for transformation
        
        ros::ServiceServer plannerService_; /// server for accepting the new planner 
        
        boost::recursive_mutex configuration_mutex_;/// Mutex to avoid the confict of changing when moving to goal.
    };  
};


#endif

