#ifndef SATUNOL_SM_H
#define SATUNOL_SM_H

#include "StateMachine.h"
#include "evt-queue.h"

class SatunolSM : public StateMachine
{
public:
    SatunolSM(const char* task_name, uint32_t stackSize, int priority, uint8_t maxStates);
    virtual ~SatunolSM();

    void stop();
protected:
    bool sendEvent(std::pair<std::function<void(EventData*)>, EventData*>);
private:
    EvtQueue* _pEventQueue;
};

#endif /* SATUNOL_SM_H */