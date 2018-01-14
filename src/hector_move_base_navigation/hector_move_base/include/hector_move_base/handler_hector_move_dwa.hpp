#ifndef HANDLER_HECTOR_MOVE_DWA_HPP
#define HANDLER_HECTOR_MOVE_DWA_HPP


#include <hector_nav_core/hector_move_base_handler.h>
#include <nav_core/base_local_planner.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <pluginlib/class_loader.h>


namespace hector_move_base_handler {

class HectorMoveDwaHandler : public HectorMoveBaseHandler {

private:
    costmap_2d::Costmap2DROS* local_costmap_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner>  dwa_loader_;
    boost::shared_ptr<nav_core::BaseLocalPlanner> dwa_planner_;
    //tf::TransformListener& tf_;
    ros::Publisher vel_pub_;
    double controller_patience_;
    bool new_global_plan_;


    bool isGoalIDEqual(const actionlib_msgs::GoalID& firstGoalID, const actionlib_msgs::GoalID& secondGoalID)
    {
        return ((firstGoalID.stamp == secondGoalID.stamp) && (firstGoalID.id == secondGoalID.id));
    }

    ros::Time last_valid_control_;

    // ------------------------------------------------------------------------------------

    // TODO: For ftc_local_planner.
    //pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
    //boost::shared_ptr<nav_core::BaseGlobalPlanner> trajectory_planner_;
public:

    // ------------------------------------------------------------------------------------
    HectorMoveDwaHandler(hector_move_base::IHectorMoveBase* interface) : HectorMoveBaseHandler(interface)
      , dwa_loader_("nav_core", "nav_core::BaseLocalPlanner")
    //  , bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
    {
        //costmap_ = interface->getCostmap();
        local_costmap_=interface->getControlCostmap();

        ros::NodeHandle private_nh("~");

        std::string local_planner_name = "dwa_local_planner/DWAPlannerROS";

        private_nh.param("hector_local_planner", local_planner_name, std::string("dwa_local_planner/DWAPlannerROS"));
        private_nh.param("controller_patience", controller_patience_, 15.0);
        //tf_=interface->getTransformListener();
        vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        new_global_plan_=true;


        try {
            //check if a non fully qualified name has potentially been passed in
            if(!dwa_loader_.isClassAvailable(local_planner_name))
            {
                ROS_INFO("The local_planner %s you want doesn't exist", local_planner_name.c_str());
                std::vector<std::string> classes = dwa_loader_.getDeclaredClasses();
                for(unsigned int i = 0; i < classes.size(); ++i){
                    if(local_planner_name == dwa_loader_.getName(classes[i])){
                        //if we've found a match... we'll get the fully qualified name and break out of the loop
                        ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                 local_planner_name.c_str(), classes[i].c_str());
                        local_planner_name = classes[i];
                        break;
                    }
                }
            }


            dwa_planner_ = dwa_loader_.createInstance(local_planner_name);
            ROS_INFO("Created local_planner %s", local_planner_name.c_str());
            dwa_planner_->initialize(dwa_loader_.getName(local_planner_name),&(interface->getTransformListener()), local_costmap_);
            local_costmap_->start();
        } catch (const pluginlib::PluginlibException& ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner_name.c_str(), ex.what());
            exit(1);
        }

    }


    hector_move_base::RESULT handle(){

        ROS_DEBUG("[move_base] [move_dwa_handler] Dwa moving started.");
        geometry_msgs::Twist cmd_vel;

        hector_move_base_msgs::MoveBaseActionPath dwa_path_= hectorMoveBaseInterface->getCurrentActionPath();
        ROS_DEBUG("[move_base] [move_dwa_handler] first, now the size of current path implemented by dwa is %lu", dwa_path_.goal.target_path.poses.size());
//        if(dwa_path_.goal.target_path.poses.size()==0)
//            new_global_plan_=false;
//        else
//            new_global_plan_=true;

        if(new_global_plan_)
        {
            ROS_DEBUG("[move_base] [move_dwa_handler] second, now the size of current path implemented by dwa is %lu", dwa_path_.goal.target_path.poses.size());
            if(dwa_path_.goal.target_path.poses.size()!=0)
            {

                if(!dwa_planner_->setPlan(dwa_path_.goal.target_path.poses))
                {
                    //TODO: if faild to pass global plan to local_planner
                    return hector_move_base::FAIL;

                }
                new_global_plan_=false;
            }
            else
            {
                new_global_plan_=true;
                return hector_move_base::FAIL;
            }

        }
//        if(!dwa_planner_->setPlan(dwa_path_.goal.target_path.poses))
//        {
//            //TODO: if faild to pass global plan to local_planner
//            return hector_move_base::FAIL;

//        }

        //check to see if we've reached our goal
        if(dwa_planner_->isGoalReached()){
          ROS_DEBUG_NAMED("[move_base]""[dwa_move_base]","Goal reached!");
          //resetState();
          new_global_plan_=true;

          return hector_move_base::SUCCESS;
        }

        if(!local_costmap_->isCurrent()){
          ROS_INFO("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
          //publishZeroVelocity();
          return hector_move_base::FAIL;
        }

        if(dwa_planner_->computeVelocityCommands(cmd_vel) && !new_global_plan_)
        {
            ROS_INFO("Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                             cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
            last_valid_control_ = ros::Time::now();
            //ROS_INFO("Hello, Publish new cmd_vel!");
            //make sure that we send the velocity command to the base
            vel_pub_.publish(cmd_vel);
            return hector_move_base::WAIT;

          }
          else {
            ROS_INFO("The local planner could not find a valid plan.");
            ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

            //check if we've tried to find a valid control for longer than our time limit
            if(ros::Time::now() > attempt_end){
              //TODO: we'll move into our obstacle clearing mode
//              publishZeroVelocity();
//              state_ = CLEARING;
//              recovery_trigger_ = CONTROLLING_R;
            }
            else{
              //TODO: otherwise, if we can't find a valid control, we'll go back to planning
              //last_valid_plan_ = ros::Time::now();
//              planning_retries_ = 0;
//              state_ = PLANNING;
//              publishZeroVelocity();
        }



//        hectorMoveBaseInterface->setActionServerCanceled();
//        hector_move_base_msgs::MoveBaseActionResult action_result;
//        action_result.header.stamp = ros::Time::now();
//        action_result.status.goal_id = hectorMoveBaseInterface->getCurrentGoal().goal_id;
//        action_result.status.status = actionlib_msgs::GoalStatus::ABORTED;
//        result_pub_.publish(action_result);
            return hector_move_base::FAIL;

    }
    }

    void abort(){
        ROS_INFO("Hello, going to abort dwa_state!");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
        new_global_plan_=true;
    }
};
}

#endif // HANDLER_HECTOR_MOVE_DWA_HPP
