#include <cleaner_planner_plugin/cleaner_planner_plugin.h>

cleaner_planner_plugin::CleanerGlobalPlannerConfig global_planner_config_;
void globalConfigCallback(const cleaner_planner_plugin::CleanerGlobalPlannerConfig& config)
{
    global_planner_config_ = config;
    ROS_INFO("config call back");
}


int main(int argc, char** argv) {
    // in order to replace the original move_base seamlessly we use its name
    ros::init(argc, argv, "cleaner_planner");
    ros::NodeHandle private_nh_("~");

    // configure global planner 
    // std::string name = "CleanerGlobalPlanner";
    // dynamic_reconfigure::Client< cleaner_planner_plugin::CleanerGlobalPlannerConfig> client("/move_base/" + name,globalConfigCallback);
    // cleaner_planner_plugin::CleanerGlobalPlannerConfig config;
    // if(!client.getCurrentConfiguration(config,ros::Duration(1)))
    //     config  = global_planner_config_;
    // ROS_INFO("current configuration global planner is %s ",config.global_planner.c_str());
    // config.global_planner = "global_planner/GlobalPlanner";
    // config.restore_defaults = false;
    // if(client.setConfiguration(config))
    // {     
    //      ROS_INFO("Set configuration with global planner of %s  successfully",config.global_planner.c_str());
    //      global_planner_config_ = config;
    // }
    
    
    // local planner change service

    ros::ServiceClient localPlanner = private_nh_.serviceClient<cleaner_planner_plugin::ChangePlanner>("/move_base/CleanerLocalPlanner/change_planner");
    cleaner_planner_plugin::ChangePlanner srv_local;
    srv_local.request.newPlanner = "ftc_local_planner/FTCPlanner";
    if(localPlanner.call(srv_local))
         ROS_INFO("Change local planner to %s from %s successfully",srv_local.request.newPlanner.c_str(),srv_local.response.oldPlanner.c_str());
    
    // global planner change service
    ros::ServiceClient globalPlanner = private_nh_.serviceClient<cleaner_planner_plugin::ChangePlanner>("/move_base/CleanerGlobalPlanner/change_planner");
     cleaner_planner_plugin::ChangePlanner srv_global;
     srv_global.request.newPlanner = "global_planner/GlobalPlanner";
    if(globalPlanner.call(srv_global))
         ROS_INFO("Change global planner to %s from %s successfully",srv_global.request.newPlanner.c_str(),srv_global.response.oldPlanner.c_str());
    //
    ros::Rate rate = ros::Rate(10.0);
    while (ros::ok()) {
        
        ros::spinOnce();
        rate.sleep();
    }

    return(0);

}
