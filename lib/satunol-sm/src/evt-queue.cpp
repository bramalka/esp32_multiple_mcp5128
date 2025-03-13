#include "evt-queue.h"

EvtQueue::EvtQueue(uint8_t maxQueueLength) : _maxQueueLength(maxQueueLength)
{
}

EvtQueue::~EvtQueue()
{
}

bool EvtQueue::sendEvent(std::pair<std::function<void(EventData*)>, EventData*> eventPair)
{
    bool retVal { false };

    if (_qLength < _maxQueueLength) {
        _equeue.push(eventPair);
        _qLength++;
        retVal = true;
        xTaskResumeFromISR(eventQueue_TaskHandle);
    }

    return retVal;
}

std::pair<std::function<void(EventData*)>, EventData*> EvtQueue::wait()
{
    std::pair<std::function<void(EventData*)>, EventData*> retVal;

    if (_equeue.size() == 0) {
        vTaskSuspend(NULL);
    }

    retVal = _equeue.front();
    _equeue.pop();
    _qLength--;

    return retVal;
}

void EvtQueue::setStop(bool stop)
{
    _isStop = stop;
}

bool EvtQueue::getStop()
{
    return _isStop;
}