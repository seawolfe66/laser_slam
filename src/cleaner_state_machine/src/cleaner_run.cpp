#include <cleaner_run_time/cleaner_run.h>

namespace cleaner_state_machine
{

    CleanerRun::CleanerRun(std::string name ,  tf::TransformListener& tf) : 
                nh_(""),
                private_nh_("~"),
                tf_(tf),
                stateMachine_(new CleanerStateMachine),
                dataInterface_(new CleanerData)
    {
        google::InitGoogleLogging("cleaner_log");
        google::SetLogDestination(google::GLOG_INFO, "log/Info_");
        FLAGS_colorlogtostderr = true;
        FLAGS_v = 3;

        CleanerLog::EnableGlog();


        battery_subscriber = private_nh_.subscribe<kobuki_msgs::PowerSystemEvent>("/mobile_base/events/power_system", 10, boost::bind(&CleanerRun::BatteryCallback, this, _1));


        setupStateMachine();
        
    }
    CleanerRun::~CleanerRun()
    {

    }

void CleanerRun::setCurrentState(TStateHandlerPtr state){

    currentState_= state;
}


void CleanerRun::BatteryCallback(const  kobuki_msgs::PowerSystemEventConstPtr &msg){


       if(msg->event == kobuki_msgs::PowerSystemEvent::BATTERY_LOW){

           setCurrentState(autodockingState_);

       }else if(msg->event == kobuki_msgs::PowerSystemEvent::PLUGGED_TO_DOCKBASE){

           setCurrentState(idleState_);
       }

}


    void CleanerRun::runStep()
    {
        RESULT result = currentState_->handle();

        //we should check if the autodocking failed; if failed, it should notice user.--chenrui
       if(currentState_ == autodockingState_ && result == cleaner_state_machine::FAIL){

        }



        if (currentState_ != nextState_) {
            currentState_ = nextState_;
            ROS_INFO("[cleaner_state_machine]: nextState_ was set, ignoring statemachine mapping");
            return;
        }
        switch (result) {
        case WAIT:
            ROS_DEBUG("[cleaner_state_machine]: result is WAIT, currentState_ will be kept");
            return;

        default:
            ROS_INFO("[cleaner_state_machine]: nextState from result %d",result);
            currentState_ = stateMachine_->getNextActionForHandler(currentState_, result);
            nextState_ = currentState_;
            return;
        }
    }
    void CleanerRun::setupStateMachine()
    {
        idleState_.reset(new cleaner_state_handler::CleanerIdleHandler(dataInterface_));
        coverageState_.reset(new cleaner_state_handler::CleanerCoverageExplorationHandler(dataInterface_));
        wallFollowState_.reset(new cleaner_state_handler::CleanerWallFollowingHandler(dataInterface_));
        pathPlanState_.reset(new cleaner_state_handler::CleanerPathPlanHandler(dataInterface_));
        //
        TMapResult mappingForState;
        MapState(idleState_,mappingForState,NEXT,coverageState_);

        //
        mappingForState.clear();
        MapState(coverageState_,mappingForState,FAIL,idleState_);
        MapState(coverageState_,mappingForState,NEXT,pathPlanState_);
       
        mappingForState.clear();
        MapState(pathPlanState_,mappingForState,ALTERNATIVE,wallFollowState_);
        MapState(pathPlanState_,mappingForState,NEXT,coverageState_);
        MapState(pathPlanState_,mappingForState,FAIL,idleState_);
        
        mappingForState.clear();
        MapState(wallFollowState_,mappingForState,NEXT,pathPlanState_);
        //

        mappingForState.clear();
        MapState(autodockingState_,mappingForState,NEXT,idleState_);


        startState_ = idleState_;
        currentState_ = idleState_;
        nextState_ = currentState_;
    }
    void CleanerRun::MapState(TStateHandlerPtr state_from, TMapResult& map,RESULT result, TStateHandlerPtr state_to)
    {
        
        map.insert(std::make_pair(result, state_to));
        stateMachine_->addHandlerMapping(state_from, map);
    }
}

