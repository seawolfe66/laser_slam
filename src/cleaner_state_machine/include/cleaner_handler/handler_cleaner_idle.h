#ifndef HANDLER_CLEANER_IDLE_H_
#define HANDLER_CLEANER_IDLE_H_

#include <cleaner_state_machine/cleaner_state_handler.h>
#include <ros/ros.h>

namespace cleaner_state_handler {

class CleanerIdleHandler : public CleanerStateHandler {
public:
    CleanerIdleHandler(cleaner_state_machine::ICleanerDataInterface* interface) : CleanerStateHandler(interface){
	 
    }

    cleaner_state_machine::RESULT handle()
    {
        CLOG_INFO("Now, the statemachine is in IDLE_STATE ");
        return cleaner_state_machine::NEXT;
    }

    void abort(){
    }
};
}
#endif
