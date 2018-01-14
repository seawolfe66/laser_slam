#ifndef HANDLER_HECTOR_PLANNING_H_
#define HANDLER_HECTOR_PLANNING_H_

#include <hector_nav_core/hector_move_base_handler.h>
#include <hector_nav_core/exploration_planner.h>
#include <pluginlib/class_loader.h>

namespace hector_move_base_handler {

class HectorPlanningHandler : public HectorMoveBaseHandler {
private:
    costmap_2d::Costmap2DROS* costmap_;

    bool isGoalIDEqual(const actionlib_msgs::GoalID& firstGoalID, const actionlib_msgs::GoalID& secondGoalID)
    {
        return ((firstGoalID.stamp == secondGoalID.stamp) && (firstGoalID.id == secondGoalID.id));
    }

    // ------------------------------------------------------------------------------------

    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
    boost::shared_ptr<nav_core::BaseGlobalPlanner> trajectory_planner_;
    // ------------------------------------------------------------------------------------

public:
    HectorPlanningHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface)
      , bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
    {
        costmap_ = interface->getCostmap();
        ros::NodeHandle private_nh("~");

        // ------------------------------------------------------------------------------------

        std::string path_planner = "SBPLLatticePlanner";
        private_nh.param("path_planner", path_planner, path_planner);
        ROS_INFO("Now the trajectory_planner use %s as path_planner",path_planner.c_str());

        try
        {
            //check if a non fully qualified name has potentially been passed in
            if(!bgp_loader_.isClassAvailable(path_planner)){
                std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
                for(unsigned int i = 0; i < classes.size(); ++i){
                    if(path_planner == bgp_loader_.getName(classes[i])){
                        //if we've found a match... we'll get the fully qualified name and break out of the loop
                        ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                 path_planner.c_str(), classes[i].c_str());
                        path_planner = classes[i];
                        break;
                    }
                }
            }
            trajectory_planner_ = bgp_loader_.createInstance(path_planner);
            trajectory_planner_->initialize(bgp_loader_.getName(path_planner), costmap_);
        }
        catch (const pluginlib::PluginlibException& ex)
        {
            ROS_FATAL("[move_base] [handler_planning] Failed to create the %s planner. "
                      "Are you sure it is properly registered and the containing library is built? "
                      "Exception: %s", path_planner.c_str(), ex.what());
            exit(1);
        }
        costmap_->start();
        // ------------------------------------------------------------------------------------
    }

    hector_move_base::RESULT handle()
    {
        ROS_DEBUG("[move_base] [planning_handler] Planning started.");

        hector_move_base::handlerActionGoal current_goal = hectorMoveBaseInterface->getCurrentGoal();
        std::vector<geometry_msgs::PoseStamped> plan;
        if(costmap_ == NULL)
        {
            ROS_ERROR("[move_base] [planning_handler] makePlan failed, costmap is NULL");
            return hector_move_base::FAIL;
        }


        // lock costmap
#ifdef LAYERED_COSTMAP_H_
        boost::unique_lock< boost::shared_mutex > lock(*(costmap_->getCostmap()->getLock()));
#endif // LAYERED_COSTMAP_H_

        //get the starting pose of the robot
        tf::Stamped<tf::Pose> global_pose;
        if(!costmap_->getRobotPose(global_pose)){
            ROS_ERROR("[move_base] [planning_handler] makePlan failed, pose not retrievable from costmap.");
            return hector_move_base::FAIL;
        }

        geometry_msgs::PoseStamped start;
        tf::poseStampedTFToMsg(global_pose, start);

        bool fixed = false;
     // ------------------------------------------------------------------------------------

        geometry_msgs::PoseStamped goalForTrajectory = current_goal.target_pose;

        std::vector<geometry_msgs::PoseStamped> trajectory;
        if(!(trajectory_planner_->makePlan(start, goalForTrajectory, trajectory)))
        {
            ROS_WARN("[hector_move_base] [planning_handler]: [sbpl_only] Execution of hector planner failed for goal (%.2f, %.2f)",
                     goalForTrajectory.pose.position.x, goalForTrajectory.pose.position.y);
            if (hectorMoveBaseInterface->getGlobalGoal().do_exploration)
            {
                ROS_INFO("[planning_handler]: In Exploration. Looking for new frontier.");
                return hector_move_base::ALTERNATIVE;
            }
            return hector_move_base::FAIL;
        }
        ROS_INFO("Hello, Hector_move_base generate global plan successfully~!");
        ROS_INFO("Hello, now the size of current generated path is %lu",trajectory.size());
        plan = trajectory;
        fixed = true;
            // ------------------------------------------------------------------------------------


        hector_move_base_msgs::MoveBaseActionPath new_path = hector_move_base_msgs::MoveBaseActionPath();
        new_path.goal_id = current_goal.goal_id;
        new_path.header.frame_id = current_goal.target_pose.header.frame_id;
        new_path.header.stamp = current_goal.target_pose.header.stamp;
        new_path.goal.fixed = fixed;
        new_path.goal.speed = current_goal.speed;
        new_path.goal.target_path.header.frame_id = new_path.header.frame_id ;
        new_path.goal.target_path.header.stamp = ros::Time::now() ;
        new_path.goal.target_path.poses = plan;
        new_path.reverse_allowed = current_goal.reverse_allowed;

	// Gabriel Hüttenberger: Test for easier robot orientation in front of Victims
	// new_path.goal.target_path.poses[(unsigned int)(plan.size()-1)].orientation = new_path.goal.target_path.poses[(unsigned int)(plan.size()-2)].orientation;

        hectorMoveBaseInterface->setActionPath(new_path);

        ROS_DEBUG("[move_base] [planning_handler] Generated plan, continue with NEXT.");

        if (isGoalIDEqual(current_goal.goal_id, hectorMoveBaseInterface->getCurrentGoal().goal_id)) {
            //hectorMoveBaseInterface->popCurrentGoal();
            //current_goal.target_pose.pose = plan.back().pose;
            //hectorMoveBaseInterface->pushCurrentGoal(current_goal);
        }

        return hector_move_base::NEXT;
    }

    void abort()
    {
        ROS_WARN("[move_base] [planning_handler] Abort was called in planning, this seams to lead to unresponsive behavior");
    }
};
}
#endif
