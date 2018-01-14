#ifndef CLEANER_STATE_MACHINE_STATE_HANDLER_H_
#define CLEANER_STATE_MACHINE_STATE_HANDLER_H_

#include <cleaner_state_machine/cleaner_data_interface.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <map>

namespace cleaner_state_handler
{

/**
 * @abstract CleanerStateHandler
 * @brief Provides an abstract class for implementation of state modules
 */
class CleanerStateHandler
{
protected:
    cleaner_state_machine::ICleanerDataInterface* cleanerDataInterface;

    CleanerStateHandler(cleaner_state_machine::ICleanerDataInterface* interface)
    {
        cleanerDataInterface = interface;
    }

public:
    virtual cleaner_state_machine::RESULT handle() = 0;

    virtual void abort() = 0;
};

}

#endif