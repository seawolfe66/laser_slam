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
* First building Date : 06/23/2017
* Last modified Date :  07/27/2017
*********************************************************************/

#ifndef LINEAR_GLOBAL_PLANNER_H_
#define LINEAR_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/GetPlan.h>

#include <dynamic_reconfigure/server.h>
#include <linear_global_planner/LinearPlannerConfig.h>

namespace linear_global_planner{
    /**
    * @class LinearGlobalPlanner
    * @brief Provides a simple global planner that will generate a straight line from start to goal divided by segments and also be able to 
              compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
    */
    class LinearGlobalPlanner : public nav_core::BaseGlobalPlanner{
        public:
        /**
        * @brief Given a goal pose in the world, compute a plan
        * @param start The start pose 
        * @param goal The goal pose 
        * @param plan The plan... filled by the planner
        * @return True if a valid plan was found, false otherwise
        */
        virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        /**
        * @brief  Initialization function for the LinearGlobalPlanner
        * @param  name The name of this planner
        * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
        */
        virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
        * @brief  Virtual destructor for the interface
        */
        virtual ~LinearGlobalPlanner(){}
        LinearGlobalPlanner();

        protected:
        /**
        * @brief  Find the nearest available position from the passed goal 
        * @param  start The start position 
        * @param  goal  The goal position and get the available position if return true 
        * @return True if an available position was found , false otherwise  
        */
        bool findAvailableGoal(const geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal);

        /**
        * @brief  Check the footprint of robot at the given position is legal in the worldModel  
        * @param  x_i The x value of position 
        * @param  y_i The y value of position 
        * @param theta_i The yaw value of position 
        * @return -1 if the footprint is illegal , >0 otherwise  
        */
        double footprintCost(double x_i, double y_i, double theta_i);

        /**
        *@brief Publish the global plan for visulatation.
        *@param path the position set to visualize 
        */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        /**
        * @brief Given a goal pose in the world, compute a linear compenstation path 
        * @param start The start pose 
        * @param goal The goal pose 
        * @param plan The plan... filled by the planner
        * @return True if a valid path is created , otherwise false
        */
        bool linearPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
        
        /**
        *@brief Reconfigure config_
        */
        void reconfigureCB(LinearPlannerConfig &config, uint32_t level);
        
        private:
        // Global costmap ros interface 
            costmap_2d::Costmap2DROS* costmap_ros_;
            // the distance every step for finding available goal
            double step_size_;
            // the minimum distance from robot considered as valid position
            double min_dist_from_robot_;
            // the costmap data from Costmap2DROS interface 
            costmap_2d::Costmap2D* costmap_;
            // the world model from local planner 
            base_local_planner::WorldModel* world_model_;
            // the flag indicated the initialization status 
            bool initialized_;
            // a publisher for visualizing global plan path 
            ros::Publisher plan_pub_;
            // the dynamic reconfigure server for modifing parameters 
            dynamic_reconfigure::Server<LinearPlannerConfig> *dsrv_;
            //start config
            linear_global_planner::LinearPlannerConfig default_config_;
            //reconfigure config
            linear_global_planner::LinearPlannerConfig config_;
    };  
};


#endif