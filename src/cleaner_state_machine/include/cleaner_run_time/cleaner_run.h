#ifndef CLEANER_STATE_MACHINE_RUN_H_
#define CLEANER_STATE_MACHINE_RUN_H_

#include <cleaner_state_machine/cleaner_state_machine.h>
#include <cleaner_run_time/cleaner_data.h>
#include <cleaner_run_time/cleaner_log.h>

#include <cleaner_handler/handler_cleaner_idle.h>
#include <cleaner_handler/handler_cleaner_coverage_exploration.h>
#include <cleaner_handler/handler_cleaner_dem_wall_following.h>
#include <cleaner_handler/handler_cleaner_path_plan.h>
#include <cleaner_handler/handler_cleaner_autodocking.h>


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <kobuki_msgs/PowerSystemEvent.h>

namespace cleaner_state_machine
{

    /**
    * @class CleanerRun
    * @brief Provides a class for configuring and running state machine 
    */
    class CleanerRun
    {
     public: 
        
            /**
        * @brief  Constructor for the actions
        * @param name The name of the action
        * @param tf A reference to a TransformListener
        */
        CleanerRun(std::string name, tf::TransformListener& tf);

        /**
        * @brief  Destructor - Cleans up
        */
        virtual ~CleanerRun();

        /**
        * @brief  running by step calling
        */
        void runStep();
        void BatteryCallback(const  kobuki_msgs::PowerSystemEventConstPtr &msg);
        void setCurrentState(TStateHandlerPtr state);
    protected:
        void setupStateMachine();
       
        void MapState(TStateHandlerPtr state_from,TMapResult& map, RESULT result, TStateHandlerPtr state_to);
    private:
        boost::shared_ptr<CleanerStateMachine> stateMachine_;
        ros::NodeHandle nh_, private_nh_;
        tf::TransformListener& tf_;
        ICleanerDataInterface* dataInterface_;

        ros::Subscriber battery_subscriber;     
        TStateHandlerPtr    idleState_,coverageState_,wallFollowState_,pathPlanState_, autodockingState_;
        //boost::shared_ptr<hector_move_base_handler::HectorMoveBaseHandler> currentState_, nextState_, startState_, autodockingState_;
        //typedef boost::shared_ptr<cleaner_state_handler::CleanerStateHandler>   TStateHandlerPtr;
        TStateHandlerPtr    currentState_, nextState_, startState_;
    };
}

#endif 

