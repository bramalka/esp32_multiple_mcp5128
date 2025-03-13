#ifndef _EVENT_QUEUE_HPP_
#define _EVENT_QUEUE_HPP_

#include "StateMachine.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <functional>
#include <queue>
#include <utility>

class EvtQueue {
public:
    explicit EvtQueue(uint8_t maxQueueLength);
    ~EvtQueue();

    bool sendEvent(std::pair<std::function<void(EventData*)>, EventData*>);
    std::pair<std::function<void(EventData*)>, EventData*> wait();
    void setStop(bool stop);
    bool getStop();

    TaskHandle_t eventQueue_TaskHandle = NULL;

private:
    bool _isStop = { false };
    uint8_t _qLength { 0 };
    uint8_t _maxQueueLength { 20 };
    std::queue<std::pair<std::function<void(EventData*)>, EventData*>> _equeue;
};

#endif // _EVENT_QUEUE_HPP_