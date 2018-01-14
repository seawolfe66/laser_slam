
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
* Log:
     07/27 Modify scale to 1 when min_offset is set  zero ;
*********************************************************************/


#include <linear_global_planner/linearGlobalPlanner.h>
#include <pluginlib/class_list_macros.h>

#define max(x,y)  ( x>y?x:y )
#define min(x,y)  ( x<y?x:y )
//register linear global planner 
PLUGINLIB_EXPORT_CLASS(linear_global_planner::LinearGlobalPlanner,nav_core::BaseGlobalPlanner)


namespace linear_global_planner {

    LinearGlobalPlanner::LinearGlobalPlanner() {}

    void LinearGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
            private_nh.param("div_offset", default_config_.div_offset, 0.10);
            world_model_ = new base_local_planner::CostmapModel(*costmap_); 
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("linear_global_plan", 1);
            initialized_ = true;

            // set the default value of config_
          //  default_config_.div_offset = 0.1;

            //Parameter for dynamic reconfigure
            dsrv_ = new dynamic_reconfigure::Server<LinearPlannerConfig>(private_nh);
            dynamic_reconfigure::Server<LinearPlannerConfig>::CallbackType cb = boost::bind(&LinearGlobalPlanner::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            //
            ROS_INFO("The linear global planner initialized");
        }
        else
            ROS_WARN("The linear global planner has already been initialized... doing nothing");
    }

    bool LinearGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        if(!initialized_){
        ROS_ERROR("The linear global planner has not been initialized, please call initialize() to use the planner");
        return false;
        }

        plan.clear();

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
            ROS_ERROR("The linear global planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }
        geometry_msgs::PoseStamped newGoal = goal;

        bool done = true;
        //findAvailableGoal(start,newGoal);

        linearPlan(start,newGoal,plan);
        publishPlan(plan);
        return done;
      }
    // following functions are local class memeber functions //

    double LinearGlobalPlanner::footprintCost(double x_i, double y_i, double theta_i){
        if(!initialized_){
        ROS_ERROR("The linear global planner has not been initialized, please call initialize() to use the planner");
        return -1.0;
        }
        
        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
        //if we have no footprint... do nothing
        if(footprint.size() < 3)
            return -1.0;

        //check if the footprint is legal
        double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
        return footprint_cost;
    }
    bool LinearGlobalPlanner::linearPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan)
    {
        tf::Stamped<tf::Pose> goal_tf;
        tf::Stamped<tf::Pose> start_tf;

        poseStampedMsgToTF(goal,goal_tf);
        poseStampedMsgToTF(start,start_tf);

        double useless_pitch, useless_roll, goal_yaw, start_yaw;
        start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
        goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);
        
        double goal_x = goal.pose.position.x;
        double goal_y = goal.pose.position.y;
        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;

        double diff_x = goal_x - start_x;
        double diff_y = goal_y - start_y;
        double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

        double target_x = goal_x;
        double target_y = goal_y;
        double target_yaw = goal_yaw;

        bool done = false;
        double scale = 0.0;
        double dScale = 0.01;
        double min_offset = config_.div_offset;

        if(min_offset == 0)
            dScale = 1;
        else 
            dScale = min(fabs(min_offset/diff_x),fabs(min_offset/diff_y));
        if(dScale > 1)
            dScale = 1;

        while(scale <= 1)
        {
            target_x = start_x + scale * diff_x;
            target_y = start_y + scale * diff_y;
         //   target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

            geometry_msgs::PoseStamped new_goal = start;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

            new_goal.pose.position.x = target_x;
            new_goal.pose.position.y = target_y;

            new_goal.pose.orientation.x = goal_quat.x();
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);

            scale +=dScale;
        }
        ROS_INFO("LinearGlobalPlanner : linearPlan size is %d", (int)plan.size());
    }
    bool LinearGlobalPlanner::findAvailableGoal(const geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal)
    {
        tf::Stamped<tf::Pose> goal_tf;
        tf::Stamped<tf::Pose> start_tf;

        poseStampedMsgToTF(goal,goal_tf);
        poseStampedMsgToTF(start,start_tf);

        double useless_pitch, useless_roll, goal_yaw, start_yaw;
        start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
        goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);
        
        double goal_x = goal.pose.position.x;
        double goal_y = goal.pose.position.y;
        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;

        double diff_x = goal_x - start_x;
        double diff_y = goal_y - start_y;
        double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

        double target_x = goal_x;
        double target_y = goal_y;
        double target_yaw = goal_yaw;

        bool done = false;
        double scale = 1.0;
        double dScale = 0.01;

        while(!done)
        {
            if(scale < 0)
            {
                target_x = start_x;
                target_y = start_y;
                target_yaw = start_yaw;
                ROS_WARN("The linear global planner could not find an available position for this goal");
                break;
            }
            target_x = start_x + scale * diff_x;
            target_y = start_y + scale * diff_y;
            target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);
            
            double footprint_cost = footprintCost(target_x, target_y, target_yaw);
            if(footprint_cost >= 0)
            {
                done = true;
            }
            scale -=dScale;
        }
        if(done)
        {
            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

            new_goal.pose.position.x = target_x;
            new_goal.pose.position.y = target_y;

            new_goal.pose.orientation.x = goal_quat.x();
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            goal = new_goal;
        }
        return done;
    }
    void LinearGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

    void LinearGlobalPlanner::reconfigureCB(LinearPlannerConfig &config, uint32_t level)
    {
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }
        config_ = config;
    }
};

