#ifndef HANDLER_COVERAGE_EXPLORATION_H_
#define HANDLER_COVERAGE_EXPLORATION_H_

#include <cleaner_state_machine/cleaner_state_handler.h>
 #include <coverage_exploration/coverage_exploration.h>

#include <cleaner_planner_plugin/ChangePlanner.h>
#include <std_msgs/String.h>
namespace cleaner_state_handler {

class CleanerCoverageExplorationHandler : public CleanerStateHandler {
private:
    coverage_exploration::CoverageAction* coverage_action_;
     ros::NodeHandle private_nh_;


public:
    
    CleanerCoverageExplorationHandler(cleaner_state_machine::ICleanerDataInterface* interface) : CleanerStateHandler(interface),private_nh_("~")
    {

        coverage_action_ = new coverage_exploration::CoverageAction(private_nh_,"/map","/map","base_link");
        CLOG_INFO("Coverage exploration handler is created");
       
    }


    cleaner_state_machine::RESULT handle()
    {   
    
        CLOG_INFO("Coverage exploration handler is running");
    
        geometry_msgs::PoseStamped pose_orig;
        pose_orig.header.frame_id = "/map";
        pose_orig.header.stamp =  ros::Time::now();
        coverage_action_->initialize(pose_orig);
        geometry_msgs::Pose pose_dest;
        geometry_msgs::PolygonStamped output;

        // Change planner for move_base 

        ros::ServiceClient localPlanner = private_nh_.serviceClient<cleaner_planner_plugin::ChangePlanner>("/move_base/CleanerLocalPlanner/change_planner");
        cleaner_planner_plugin::ChangePlanner srv_local;
        srv_local.request.newPlanner = "base_local_planner/TrajectoryPlannerROS";
       // srv_local.request.newPlanner = "dwa_local_planner/DWAPlannerROS";
        if(localPlanner.call(srv_local))
            ROS_INFO("Change local planner to %s from %s successfully",srv_local.request.newPlanner.c_str(),srv_local.response.oldPlanner.c_str());
    
        // global planner change service
        ros::ServiceClient globalPlanner = private_nh_.serviceClient<cleaner_planner_plugin::ChangePlanner>("/move_base/CleanerGlobalPlanner/change_planner");
        cleaner_planner_plugin::ChangePlanner srv_global;
        srv_global.request.newPlanner = "global_planner/GlobalPlanner";
        if(globalPlanner.call(srv_global))
            ROS_INFO("Change global planner to %s from %s successfully",srv_global.request.newPlanner.c_str(),srv_global.response.oldPlanner.c_str());
        //
        int ret ;

        ret = coverage_action_->exploreNextArea(pose_dest,output);
        
        if(ret == coverage_exploration::CoverageAction::C_SUCCESS)
        {
            cleanerDataInterface->setCurrentArea(output);
           
            // ros::ServiceClient updateCostmapBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::CostmapPolygon>("/explore_server/explore_costmap/explore_boundary/get_costmap_polygon");
            // if(!updateCostmapBoundaryPolygon.waitForExistence()){
            // }

            // if(ros::ok()){
            //     frontier_exploration::CostmapPolygon srv;
            //     srv.request.costmap_boundary = output;
            // //input_rectangle_polygon_
            //     // sweeps_areas_pub_.publish(sweeps_areas_);

            //     if(updateCostmapBoundaryPolygon.call(srv)){
            //         ROS_INFO("Region boundary set");
            //         polygon_pub_.publish(srv.response.sweeps_boundary);
            //         //
            //         geometry_msgs::PolygonStamped validPolygon = srv.response.sweeps_boundary;
            //         double average_distance = polygonPerimeter(validPolygon.polygon) / validPolygon.polygon.points.size();
            //         ROS_INFO("Valid polygon: average_distance = %.4f",average_distance);
            //         if (validPolygon.polygon.points.size() < 3)
            //         {
            //             ROS_ERROR("Not a valid polygon,");
            //         }
                    
            //         else 
            //         {
            //             bDone = false;
            //             heatmap_pub_.publish(validPolygon);
                        
            //             while(ros::ok())
            //             {
            //                 ros::Duration(0.1).sleep();
            //                 ros::spinOnce();
            //                 if(bDone)
            //                     break;
            //                 if(bObstacle)
            //                     return cleaner_state_machine::ALTERNATIVE;
            //             }
                        
            //         }
                    
            //     }else{
            //         ROS_ERROR("Failed to set region boundary");
            //     }
            // }
            ROS_INFO("CoverageExplore finish one area");
            return cleaner_state_machine::NEXT; 
        }
        else
            return cleaner_state_machine::FAIL;
    }

    void abort()
    {
        CLOG_WARN("[move_base] [planning_handler] Abort was called in planning, but is not implemented.");
    }


};
}
#endif
