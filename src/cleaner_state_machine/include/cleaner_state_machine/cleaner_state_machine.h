#ifndef CLEANER_STATE_MACHINE_H_
#define CLEANER_STATE_MACHINE_H_

#include <cleaner_state_machine/cleaner_state_handler.h>

#include <boost/shared_ptr.hpp>

#include <deque>
#include <map>

namespace cleaner_state_machine {



typedef boost::shared_ptr<cleaner_state_handler::CleanerStateHandler>   TStateHandlerPtr;
typedef std::map<RESULT, TStateHandlerPtr>  TMapResult;
typedef std::map<TStateHandlerPtr,TMapResult>  TMapTransition;
struct TPair
    {
        TPair(RESULT r,TStateHandlerPtr s)
        {
            result = r;
            state = s;
        }
        RESULT result;
        TStateHandlerPtr state;
    } ;
class CleanerStateMachine {
private:
    TMapTransition nextStateMapping;

public:
    CleanerStateMachine();
    virtual ~CleanerStateMachine();

    void addHandlerMapping(const TStateHandlerPtr,
                           const TMapResult);

    void overrideMapping(TMapTransition);

    TStateHandlerPtr getNextActionForHandler(const TStateHandlerPtr, const RESULT);

    void setNextActionForHandler(const TStateHandlerPtr,
                                 const RESULT, const TStateHandlerPtr);

};

}

#endif
