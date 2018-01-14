#ifndef HANDLER_CLEANER_PATH_PLAN_H_
#define HANDLER_CLEANER_PATH_PLAN_H_

#include <cleaner_state_machine/cleaner_state_handler.h>
#include <ros/ros.h>
#include <frontier_exploration/CostmapPolygon.h>
#include <std_msgs/String.h>
namespace cleaner_state_handler {

class CleanerPathPlanHandler : public CleanerStateHandler {
     
     ros::Publisher polygon_pub_,heatmap_pub_;
     ros::Subscriber heatmap_done_sub_;
     bool bDone,bObstacle;
     ros::NodeHandle private_nh_;
public:
    CleanerPathPlanHandler(cleaner_state_machine::ICleanerDataInterface* interface) : CleanerStateHandler(interface),private_nh_("~"){

        bObstacle = bDone = false;
        polygon_pub_ = private_nh_.advertise<geometry_msgs::PolygonStamped>("/coverage_output_polygon",50);
        heatmap_pub_ = private_nh_.advertise<geometry_msgs::PolygonStamped>("/heatmap_area",50);
        
        heatmap_done_sub_ = private_nh_.subscribe("/heatmap_area_done", 10,  &CleanerPathPlanHandler::heatmapDone , this);
    }
        
    void heatmapDone(const std_msgs::String& msg)
    {
        if(msg.data == "Done")
            bDone = true;
        else if(msg.data == "Obstacle")
            bObstacle =  true;
    }
    /**
	* @brief Thread-safe implementation for coverage_exploration.
	*/
	template<typename T, typename S>
	double pointsDistance(const T &one, const S &two){
		return sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0) + pow(one.z-two.z,2.0));
	}
    /**
	* @brief Calculate polygon perimeter
	* @param polygon Polygon to process
	* @return Perimeter of polygon
	*/
	double polygonPerimeter(const geometry_msgs::Polygon &polygon){

		double perimeter = 0;
		if(polygon.points.size()   > 1)
		{
			for (int i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
			{
			perimeter += pointsDistance(polygon.points[i], polygon.points[j]);
			}
		}
		return perimeter;
	}
    cleaner_state_machine::RESULT handle()
    {
        ROS_WARN("Enter path plan state");
        ros::ServiceClient updateCostmapBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::CostmapPolygon>("/explore_server/explore_costmap/explore_boundary/get_costmap_polygon");
        if(!updateCostmapBoundaryPolygon.waitForExistence()){
        }

        if(ros::ok()){
            frontier_exploration::CostmapPolygon srv;
            geometry_msgs::PolygonStamped curPolygon;
            if(cleanerDataInterface->getCurrentArea(curPolygon) && curPolygon.polygon.points.size() >= 3)
                srv.request.costmap_boundary = curPolygon;
            else
            {
                ROS_WARN("path plan - input an invalid polygon ");
                return cleaner_state_machine::FAIL;
            }    

            if(updateCostmapBoundaryPolygon.call(srv)){
                ROS_INFO("Region boundary set");
                polygon_pub_.publish(srv.response.sweeps_boundary);
                //
                geometry_msgs::PolygonStamped validPolygon = srv.response.sweeps_boundary;
                double perimeter = polygonPerimeter(validPolygon.polygon);
                double average_distance =  perimeter/ validPolygon.polygon.points.size();
                ROS_INFO("Valid polygon: perimeter = %.2f average_distance = %.4f",perimeter,average_distance);
                if (validPolygon.polygon.points.size() < 3)
                {
                    ROS_ERROR("Not a valid polygon,");
                }
                
                else 
                {
                    // Change planner for move_base 

                    // ros::ServiceClient localPlanner = private_nh_.serviceClient<cleaner_planner_plugin::ChangePlanner>("/move_base/CleanerLocalPlanner/change_planner");
                    // cleaner_planner_plugin::ChangePlanner srv_local;
                    // srv_local.request.newPlanner = "ftc_local_planner/FTCPlanner";
                    // if(localPlanner.call(srv_local))
                    //     ROS_INFO("Change local planner to %s from %s successfully",srv_local.request.newPlanner.c_str(),srv_local.response.oldPlanner.c_str());
                
                    // // global planner change service
                    // ros::ServiceClient globalPlanner = private_nh_.serviceClient<cleaner_planner_plugin::ChangePlanner>("/move_base/CleanerGlobalPlanner/change_planner");
                    // cleaner_planner_plugin::ChangePlanner srv_global;
                    // srv_global.request.newPlanner = "linear_global_planner/LinearGlobalPlanner";
                    // if(globalPlanner.call(srv_global))
                    //     ROS_INFO("Change global planner to %s from %s successfully",srv_global.request.newPlanner.c_str(),srv_global.response.oldPlanner.c_str());
                    // //
                    
                    bObstacle = bDone = false;
                    heatmap_pub_.publish(validPolygon);
                    
                    while(ros::ok())
                    {
                        ros::Duration(0.1).sleep();
                        ros::spinOnce();
                        if(bDone)
                            break;
                        if(bObstacle)
                         {
                             ROS_WARN("path plan state - obstacle found"); 
                             return cleaner_state_machine::ALTERNATIVE;
                            break;
                         }  
                    }
                    
                }
                
            }else{
                ROS_ERROR("Failed to set region boundary");
               return cleaner_state_machine::FAIL;
            }
        }
        return cleaner_state_machine::NEXT;
    }

    void abort(){
    }
};
}
#endif
