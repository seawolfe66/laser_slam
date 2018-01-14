#include <cleaner_planner_plugin/cleaner_global_planner.h>
#include <pluginlib/class_list_macros.h>


namespace cleaner_global_planner{

    CleanerGlobalPlanner::CleanerGlobalPlanner() : 
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner") {}

    void CleanerGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
                        
            planner_costmap_ros_ = costmap_ros;

            default_config_.global_planner = std::string("navfn/NavfnROS");

            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("global_planner", global_planner_,default_config_.global_planner);

            //initialize the global planner
            try {
            planner_ = bgp_loader_.createInstance(global_planner_);
            planner_->initialize(bgp_loader_.getName(global_planner_), costmap_ros);
            } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner_.c_str(), ex.what());
            exit(1);
            }
            
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("cleaner_global_plan", 1);
            initialized_ = true;

            //Parameter for dynamic reconfigure
            dsrv_ = new dynamic_reconfigure::Server<cleaner_planner_plugin::CleanerGlobalPlannerConfig>(private_nh);
            dynamic_reconfigure::Server<cleaner_planner_plugin::CleanerGlobalPlannerConfig>::CallbackType cb = boost::bind(&CleanerGlobalPlanner::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            plannerService_ = private_nh.advertiseService("change_planner",
                &CleanerGlobalPlanner::changePlannerService, this);
            //
            ROS_INFO("The cleaner global planner initialized");
        }
        else
            ROS_WARN("The cleaner global planner has already been initialized... doing nothing");
    }

    bool CleanerGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
          boost::recursive_mutex::scoped_lock l(configuration_mutex_);
        if(planner_)
        {
            if(planner_->makePlan(start,goal,plan))
            {
                publishPlan(plan);
                return true;
            }
            else 
                return false;
        }
         ROS_WARN("cleaner_global_planner did not create an available planner");
        return false;
      }
      bool CleanerGlobalPlanner::changePlannerService(cleaner_planner_plugin::ChangePlanner::Request &req, cleaner_planner_plugin::ChangePlanner::Response &res)
    {
        ROS_INFO("global planner : changePlannerService");
        res.oldPlanner = config_.global_planner;
        if(req.newPlanner == "")
            return true;
        return changePlanner(req.newPlanner);
    }
    bool CleanerGlobalPlanner::changePlanner(std::string newPlanner)
     {
         boost::recursive_mutex::scoped_lock l(configuration_mutex_);
         if(config_.global_planner != newPlanner) {
             boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
            //initialize the global planner
            ROS_INFO("Loading global planner %s", newPlanner.c_str());
            try {
                planner_ = bgp_loader_.createInstance(newPlanner);
                planner_->initialize(bgp_loader_.getName(newPlanner),planner_costmap_ros_);
                config_.global_planner = newPlanner;
            } catch (const pluginlib::PluginlibException& ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                        containing library is built? Exception: %s", newPlanner.c_str(), ex.what());
                planner_ = old_planner;
                return false;
            }
            
        }
        return true;
     }
    void CleanerGlobalPlanner::reconfigureCB(cleaner_planner_plugin::CleanerGlobalPlannerConfig &config, uint32_t level)
    {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
            ROS_INFO("Set default configureation");
        }
        if(config.global_planner != config_.global_planner) {
            boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
            //initialize the global planner
            ROS_INFO("Loading global planner %s", config.global_planner.c_str());
            try {
                planner_ = bgp_loader_.createInstance(config.global_planner);
                planner_->initialize(bgp_loader_.getName(config.global_planner), planner_costmap_ros_);
            } catch (const pluginlib::PluginlibException& ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                        containing library is built? Exception: %s", config.global_planner.c_str(), ex.what());
                planner_ = old_planner;
                config.global_planner = config_.global_planner;
            }
        }
        config_ = config;
    }
    void CleanerGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
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

//register linear global planner 
PLUGINLIB_EXPORT_CLASS(cleaner_global_planner::CleanerGlobalPlanner,nav_core::BaseGlobalPlanner)


