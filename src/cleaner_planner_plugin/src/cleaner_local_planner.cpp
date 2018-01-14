#include <cleaner_planner_plugin/cleaner_local_planner.h>
#include <pluginlib/class_list_macros.h>

namespace cleaner_local_planner{
    CleanerLocalPlanner::CleanerLocalPlanner():
     blp_loader_("nav_core", "nav_core::BaseLocalPlanner"){

    }
    bool CleanerLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
    {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);
        if(planner_)
        {
           return planner_->computeVelocityCommands(cmd_vel);
        }
        ROS_WARN("cleaner_local_planner did not create an available planner");
        return false;
    }
    bool CleanerLocalPlanner::isGoalReached() 
    {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);
        if(planner_)
        {
           return planner_->isGoalReached();
        }
        ROS_WARN("cleaner_local_planner did not create an available planner");
        return false;
    }
    bool CleanerLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) 
    {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);
        if(planner_)
        {
           return planner_->setPlan(plan);
        }
        ROS_WARN("cleaner_local_planner did not create an available planner");
        return false;
    }
    void CleanerLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) 
    {
        if(!initialized_){
            tf_ = tf;
            planner_costmap_ros_ = costmap_ros;

            default_config_.local_planner = std::string("base_local_planner/TrajectoryPlannerROS");
            std::string             local_planner_;
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("local_planner", local_planner_,default_config_.local_planner);

            //initialize the global planner
            try {
            planner_ = blp_loader_.createInstance(local_planner_);
            planner_->initialize(blp_loader_.getName(local_planner_),tf, costmap_ros);
            } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner_.c_str(), ex.what());
            exit(1);
            }
            
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("cleaner_local_plan", 1);
            initialized_ = true;

            //Parameter for dynamic reconfigure
            dsrv_ = new dynamic_reconfigure::Server<cleaner_planner_plugin::CleanerLocalPlannerConfig>(private_nh);
            dynamic_reconfigure::Server<cleaner_planner_plugin::CleanerLocalPlannerConfig>::CallbackType cb = boost::bind(&CleanerLocalPlanner::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            plannerService_ = private_nh.advertiseService("change_planner",
                &CleanerLocalPlanner::changePlannerService, this);
            //
            ROS_INFO("The cleaner local planner initialized");
        }
        else
            ROS_WARN("The cleaner local planner has already been initialized... doing nothing");
    }
    bool CleanerLocalPlanner::changePlannerService(cleaner_planner_plugin::ChangePlanner::Request &req, cleaner_planner_plugin::ChangePlanner::Response &res)
    {
        ROS_INFO("local planner : changePlannerService");
        res.oldPlanner = config_.local_planner;
        if(req.newPlanner == "")
            return true;
        return changePlanner(req.newPlanner);
    }
     bool CleanerLocalPlanner::changePlanner(std::string newPlanner)
     {
         boost::recursive_mutex::scoped_lock l(configuration_mutex_);
         if(config_.local_planner != newPlanner) {
             boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = planner_;
            //initialize the global planner
            ROS_INFO("Loading local planner %s", newPlanner.c_str());
            try {
                planner_ = blp_loader_.createInstance(newPlanner);
                // Clean up before initializing the new planner
                planner_->initialize(blp_loader_.getName(newPlanner), tf_,planner_costmap_ros_);
                config_.local_planner = newPlanner;
            } catch (const pluginlib::PluginlibException& ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                        containing library is built? Exception: %s", newPlanner.c_str(), ex.what());
                planner_ = old_planner;
                return false;
            }
        }   
        return true;
     }
    void CleanerLocalPlanner::reconfigureCB(cleaner_planner_plugin::CleanerLocalPlannerConfig &config, uint32_t level)
    {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }
        if(config.local_planner != config_.local_planner) {
            boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = planner_;
            //initialize the local planner
            ROS_INFO("Loading local planner %s", config.local_planner.c_str());
            try {
                planner_ = blp_loader_.createInstance(config.local_planner);
                // Clean up before initializing the new planner
                planner_->initialize(blp_loader_.getName(config.local_planner), tf_,planner_costmap_ros_);
            } catch (const pluginlib::PluginlibException& ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                        containing library is built? Exception: %s", config.local_planner.c_str(), ex.what());
                planner_ = old_planner;
                config.local_planner = config_.local_planner;
            }
        }
        config_ = config;
    }
    void CleanerLocalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = planner_costmap_ros_->getGlobalFrameID();
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }
};

PLUGINLIB_EXPORT_CLASS(cleaner_local_planner::CleanerLocalPlanner, nav_core::BaseLocalPlanner)