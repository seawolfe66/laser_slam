#include <cleaner_state_machine/cleaner_state_machine.h>

namespace cleaner_state_machine {

CleanerStateMachine::CleanerStateMachine() {
    nextStateMapping = TMapTransition();
}

CleanerStateMachine::~CleanerStateMachine() {

}

void CleanerStateMachine::addHandlerMapping(const TStateHandlerPtr handler,
                                                const TMapResult mappingForHandler) {
    nextStateMapping[handler] = mappingForHandler;
}

void CleanerStateMachine::overrideMapping(TMapTransition mapping) {
    nextStateMapping = mapping;
}

TStateHandlerPtr CleanerStateMachine::getNextActionForHandler(
        const TStateHandlerPtr handler, const RESULT result) {
    return nextStateMapping[handler][result];
}

void CleanerStateMachine::setNextActionForHandler(const TStateHandlerPtr handler,
                                                         const RESULT result, const TStateHandlerPtr nextHandler) {
    nextStateMapping[handler][result] = nextHandler;
}

}
