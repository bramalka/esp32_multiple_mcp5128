#include "satunol-sm.h"
#include <esp_log.h>

void sm_task(void* arg)
{
    EvtQueue* pEventQueue = static_cast<EvtQueue*>(arg);
    while (!pEventQueue->getStop()) {
        std::pair<std::function<void(EventData*)>, EventData*> eventPair = pEventQueue->wait();
        if (!pEventQueue->getStop()) {
            eventPair.first(eventPair.second);
        }
    }
}

SatunolSM::SatunolSM(const char* task_name, uint32_t stackSize, int priority, uint8_t maxStates) : StateMachine(maxStates)
{
    _pEventQueue = new EvtQueue(32);
    xTaskCreate(sm_task, task_name, stackSize, _pEventQueue, priority, &(_pEventQueue->eventQueue_TaskHandle));
    // xTaskCreatePinnedToCore(sm_task, task_name, stackSize, _pEventQueue, priority, &(_pEventQueue->eventQueue_TaskHandle), 1);
};

SatunolSM::~SatunolSM()
{
    delete _pEventQueue;
}

void SatunolSM::stop()
{
    _pEventQueue->setStop(true);
}

bool SatunolSM::sendEvent(std::pair<std::function<void(EventData*)>, EventData*> eventPair)
{
    return _pEventQueue->sendEvent(eventPair);
}