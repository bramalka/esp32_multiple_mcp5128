#ifndef CANFD_DRIVER_H
#define CANFD_DRIVER_H

#include "satunol-sm.h"
#include "drv_canfdspi_api.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>

class CanFdData : public EventData {
public:
    uint32_t msg_id;
    uint8_t length;
    bool extd;
    uint32_t extd_id;
    uint8_t data[MAX_DATA_BYTES];
};

class CanFdDClient {
public:
    virtual void canDataReady(uint32_t msg_id, const uint8_t* data, uint8_t length, bool extd = false, uint32_t extd_id = 0) = 0;
    virtual void canError() = 0; 
};

class CanFdDriver : public SatunolSM {
public:
    CanFdDriver();

    // External events taken by this state machine
    void init(can_fd_t canfd);
    // msg_id : max 29 bit (based on init)
    // length : max 64 bytes
    void sendData(uint32_t msg_id, const uint8_t* data, uint8_t length, bool extd = false, uint32_t extd_id = 0);

    can_fd_t* getCtx() { return &_canfd; }
    CanFdDClient* getClient() { return _pClient; }
    void setClient(CanFdDClient* pClient) { _pClient = pClient; }

private:
    CanFdDClient* _pClient;
    can_fd_t _canfd;
    
    // State enumeration order must match the order of state method entries in the state map.
    enum States {
        ST_IDLE,
        ST_INIT,
        ST_WAIT,
        ST_SEND,
        ST_ERROR,
        ST_MAX_STATES
    };

    // Define the state machine state functions with event data type
    STATE_DECLARE(CanFdDriver, Idle, NoEventData)
    STATE_DECLARE(CanFdDriver, Init, NoEventData)
    STATE_DECLARE(CanFdDriver, Wait, NoEventData)
    STATE_DECLARE(CanFdDriver, Send, CanFdData)
    STATE_DECLARE(CanFdDriver, Error, NoEventData)

    BEGIN_STATE_MAP_EX
        STATE_MAP_ENTRY_EX(&Idle)
        STATE_MAP_ENTRY_EX(&Init)
        STATE_MAP_ENTRY_EX(&Wait)
        STATE_MAP_ENTRY_EX(&Send)
        STATE_MAP_ENTRY_EX(&Error)
    END_STATE_MAP_EX
};

#endif    // CANFD_DRIVER_H