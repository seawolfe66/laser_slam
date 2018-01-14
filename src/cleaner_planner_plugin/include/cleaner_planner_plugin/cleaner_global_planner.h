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

#ifndef CLEANER_GLOBAL_PLANNER_H_
#define CLEANER_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/GetPlan.h>

#include <dynamic_reconfigure/server.h>
#include <cleaner_planner_plugin/CleanerGlobalPlannerConfig.h>
#include <pluginlib/class_loader.h>
#include <cleaner_planner_plugin/ChangePlanner.h>
namespace cleaner_global_planner{
    /**
    * @class CleanerGlobalPlanner
    * @brief Provides a simple global planner that will generate a straight line from start to goal divided by segments and also be able to 
              compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
    */
    class CleanerGlobalPlanner : public nav_core::BaseGlobalPlanner{
        public:
        /**
        * @brief Given a goal pose in the world, compute a plan
        * @param[in] start The start pose 
        * @param[in] goal The goal pose 
        * @param[in] plan The plan... filled by the planner
        * @return True if a valid plan was found, false otherwise
        */
        virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        /**
        * @brief  Initialization function for the LinearGlobalPlanner
        * @param[in]  name The name of this planner
        * @param[in]  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
        */
        virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
        * @brief  Virtual destructor for the interface
        */
        virtual ~CleanerGlobalPlanner(){}
        CleanerGlobalPlanner();

        protected:
          /**
        *@brief Publish the global plan for visulatation.
        *@param[in] path the position set to visualize 
        */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        /**
        *@brief Reconfigure config_
        */
        void reconfigureCB(cleaner_planner_plugin::CleanerGlobalPlannerConfig &config, uint32_t level);
        /**
        *@brief change to new planner
        *@param[in] newPLanner the planner that need to be used 
        */
        bool changePlanner(std::string newPlanner);
        /**
        *@brief the service callback to receive the request of new planner and response with old planner.
        *
        */
         bool changePlannerService(cleaner_planner_plugin::ChangePlanner::Request &req, cleaner_planner_plugin::ChangePlanner::Response &res);
      private:
        
        dynamic_reconfigure::Server<cleaner_planner_plugin::CleanerGlobalPlannerConfig> *dsrv_;/// the dynamic reconfigure server for modifing parameters 
       
        cleaner_planner_plugin::CleanerGlobalPlannerConfig default_config_; ///start config
        
        cleaner_planner_plugin::CleanerGlobalPlannerConfig config_; ///reconfigure config
        
        std::string             global_planner_; ///current global planner
        
        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_; ///class loader to load global planner that movebase will use 
        
        ros::Publisher plan_pub_; /// a publisher for visualizing global plan path 
         
        bool initialized_; /// the flag indicated the initialization status
        
        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;/// The planner pointor that indicates the actual global planner .
        
         costmap_2d::Costmap2DROS* planner_costmap_ros_;/// costmap ros that passed from move base
         
         ros::ServiceServer plannerService_;/// The servet to accept new planner
         
         boost::recursive_mutex configuration_mutex_;/// The mutex for avoiding the confiction when changing planner while the planner pointor is called.
    };  
};


#endif