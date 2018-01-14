#ifndef HANDLER_DEM_WALL_FOLLOWING_H_
#define HANDLER_DEM_WALL_FOLLOWING_H_

#include <cleaner_state_machine/cleaner_state_handler.h>
#include "node_wallFollowing.h"

namespace cleaner_state_handler {

class CleanerWallFollowingHandler: public CleanerStateHandler {
private:
   NodeWallFollowing* wallFollowing_action_;
public:

    CleanerWallFollowingHandler(cleaner_state_machine::ICleanerDataInterface* interface) : CleanerStateHandler(interface){

    }
    
    cleaner_state_machine::RESULT handle()
    {    
        ROS_WARN("Enter wall following state"); 
        wallFollowing_action_=new NodeWallFollowing();
        ROS_INFO("dem wall following  handle is created"); 
        int ret=NodeWallFollowing::WAIT;
        ros::Rate loop_rate(10);
        int i=0;
        while(ret==NodeWallFollowing::WAIT){
            ret=wallFollowing_action_->finishtask();
            i++;
            //ROS_INFO("dem wall following  handler is running....... i:%d",i);
            if(ret==NodeWallFollowing::SUCCESS){
                ROS_INFO("finish!...");
                delete wallFollowing_action_;
                return cleaner_state_machine::NEXT;
                //break;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }  
        #if 0    
            while(ret==NodeWallFollowing::WAIT){
                
                ret=wallFollowing_action_->finishtask();
                ROS_INFO(" dem wall following  handler is running  ret :%d",ret);
                if(ret ==NodeWallFollowing::WAIT){
                    return cleaner_state_machine::WAIT; 
                }
                else if (ret==NodeWallFollowing::SUCCESS){
                    delete wallFollowing_action_;
                    return cleaner_state_machine::NEXT;
                }
                else
                    return cleaner_state_machine::FAIL;
                ros::Rate loop_rate(10);
                ros::spinOnce();
            }
        #endif 
    }

    void abort()
    {
        ROS_WARN("[move_base] [planning_handler] Abort was called in planning, but is not implemented.");
    }


};
}
#endif
