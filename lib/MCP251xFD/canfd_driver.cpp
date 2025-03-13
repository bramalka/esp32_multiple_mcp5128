#include "canfd_driver.h"

#include <esp_log.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <string.h>

#define APP_RX_FIFO         CAN_FIFO_CH1
#define APP_TX_FIFO         CAN_FIFO_CH2
#define TX_WAIT_TIMEOUT     pdMS_TO_TICKS(1000) // 1000 ms = 1 second
#define RX_WAIT_TIMEOUT     pdMS_TO_TICKS(5000) // 5000 ms = 5 second

QueueHandle_t gpio_evt_queue = NULL;
SemaphoreHandle_t rx_sem = NULL;
SemaphoreHandle_t tx_sem = NULL;

static const char* TAG = "CanFdDriver";

static void IRAM_ATTR int_isr_handler(void* arg)
{
    int gpio_num = (int) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR rx_int_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(rx_sem, &xHigherPriorityTaskWoken);
    // If giving the semaphore unblocked a higher priority task, request a context switch
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void rx_task(void* arg)
{
    CanFdDriver* pCanFd = (CanFdDriver*)arg;
    uint8_t rx_data[MAX_DATA_BYTES];
    CAN_RX_FIFO_EVENT rx_flags = CAN_RX_FIFO_NO_EVENT;

    while (true) {
        DRV_CANFDSPI_ReceiveChannelEventGet(pCanFd->getCtx(), APP_RX_FIFO, &rx_flags);
        // ESP_LOGI(TAG, "RX_EVENT - RX_FIFO_EVENT: 0x%04X", rx_flags);

        // Check if FIFO is not empty
        if (rx_flags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
            CAN_RX_MSGOBJ rxObj;
            DRV_CANFDSPI_ReceiveMessageGet(pCanFd->getCtx(), APP_RX_FIFO, &rxObj, rx_data, MAX_DATA_BYTES);
            uint8_t len = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);

            if (rxObj.bF.ctrl.IDE) {
                pCanFd->getClient()->canDataReady(rxObj.bF.id.SID, rx_data, len, true, rxObj.bF.id.EID);
            } else {
                pCanFd->getClient()->canDataReady(rxObj.bF.id.SID, rx_data, len);
            }
        }
        else {
            if (gpio_get_level(pCanFd->getCtx()->int1_pin)) {
                // ESP_LOGE(TAG, "No data available");
                if (xSemaphoreTake(rx_sem, RX_WAIT_TIMEOUT) == pdTRUE) {}
            }
        }
    }
}

static void int_task(void* arg)
{
    CanFdDriver* pCanFd = (CanFdDriver*)arg;
    int io_num;
    CAN_MODULE_EVENT flags;
    
    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            DRV_CANFDSPI_ModuleEventGet(pCanFd->getCtx(), &flags);
            ESP_LOGI(TAG, "Intr from GPIO[%d] - Module event flags: 0x%04X", io_num, flags);

            if ((flags & CAN_TX_EVENT) == CAN_TX_EVENT) {
                DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_TX_EVENT);
                CAN_TX_FIFO_EVENT tx_flags = CAN_TX_FIFO_NO_EVENT;
                DRV_CANFDSPI_TransmitChannelEventGet(pCanFd->getCtx(), APP_TX_FIFO, &tx_flags);
                ESP_LOGI(TAG, "TX_EVENT - TX_FIFO_EVENT: 0x%04X", tx_flags);
                xSemaphoreGive(tx_sem);
            }

            if (flags & 0xFFFE) {
                if (flags == CAN_ALL_EVENTS) {
                    ESP_LOGI(TAG, "CAN_ALL_EVENTS");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_ALL_EVENTS);
                } else if ((flags & CAN_TIME_BASE_COUNTER_EVENT) == CAN_TIME_BASE_COUNTER_EVENT) {
                    ESP_LOGI(TAG, "CAN_TIME_BASE_COUNTER_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_TIME_BASE_COUNTER_EVENT);
                } else if ((flags & CAN_OPERATION_MODE_CHANGE_EVENT) == CAN_OPERATION_MODE_CHANGE_EVENT) {
                    ESP_LOGI(TAG, "CAN_OPERATION_MODE_CHANGE_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_OPERATION_MODE_CHANGE_EVENT);
                } else if ((flags & CAN_TEF_EVENT) == CAN_TEF_EVENT) {
                    ESP_LOGI(TAG, "CAN_TEF_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_TEF_EVENT);
                } else if ((flags & CAN_RAM_ECC_EVENT) == CAN_RAM_ECC_EVENT) {
                    ESP_LOGI(TAG, "CAN_RAM_ECC_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_RAM_ECC_EVENT);
                } else if ((flags & CAN_SPI_CRC_EVENT) == CAN_SPI_CRC_EVENT) {
                    ESP_LOGI(TAG, "CAN_SPI_CRC_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_SPI_CRC_EVENT);
                } else if ((flags & CAN_TX_ATTEMPTS_EVENT) == CAN_TX_ATTEMPTS_EVENT) {
                    ESP_LOGI(TAG, "CAN_TX_ATTEMPTS_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_TX_ATTEMPTS_EVENT);
                } else if ((flags & CAN_RX_OVERFLOW_EVENT) == CAN_RX_OVERFLOW_EVENT) {
                    ESP_LOGI(TAG, "CAN_RX_OVERFLOW_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_RX_OVERFLOW_EVENT);
                } else if ((flags & CAN_SYSTEM_ERROR_EVENT) == CAN_SYSTEM_ERROR_EVENT) {
                    ESP_LOGI(TAG, "CAN_SYSTEM_ERROR_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_SYSTEM_ERROR_EVENT);
                } else if ((flags & CAN_BUS_ERROR_EVENT) == CAN_BUS_ERROR_EVENT) {
                    ESP_LOGI(TAG, "CAN_BUS_ERROR_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_BUS_ERROR_EVENT);
                } else if ((flags & CAN_BUS_WAKEUP_EVENT) == CAN_BUS_WAKEUP_EVENT) {
                    ESP_LOGI(TAG, "CAN_BUS_WAKEUP_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_BUS_WAKEUP_EVENT);
                } else if ((flags & CAN_RX_INVALID_MESSAGE_EVENT) == CAN_RX_INVALID_MESSAGE_EVENT) {
                    ESP_LOGI(TAG, "CAN_RX_INVALID_MESSAGE_EVENT");
                    DRV_CANFDSPI_ModuleEventClear(pCanFd->getCtx(), CAN_RX_INVALID_MESSAGE_EVENT);
                }
            }
        }
    }
}

CanFdDriver::CanFdDriver() : SatunolSM("canfd_sm", 1024 * 4, 10, ST_MAX_STATES)
{
}

void CanFdDriver::init(can_fd_t canfd)
{
    _canfd = canfd;

    std::function<void(EventData*)> event = [&](EventData* data) {
        BEGIN_TRANSITION_MAP                            // - Current State -
            TRANSITION_MAP_ENTRY (ST_INIT)              // ST_IDLE
            TRANSITION_MAP_ENTRY (EVENT_IGNORED)        // ST_INIT
            TRANSITION_MAP_ENTRY (EVENT_IGNORED)        // ST_WAIT
            TRANSITION_MAP_ENTRY (EVENT_IGNORED)        // ST_SEND
            TRANSITION_MAP_ENTRY (EVENT_IGNORED)        // ST_ERROR
        END_TRANSITION_MAP(NULL)
    };

    std::pair<std::function<void(EventData*)>, EventData*> eventPair(event, nullptr);
    sendEvent(eventPair);
}

void CanFdDriver::sendData(uint32_t msg_id, const uint8_t* data, uint8_t len, bool extd, uint32_t extd_id) {

    if (len > MAX_DATA_BYTES) {
        ESP_LOGE(TAG, "Data length must be <= %d", MAX_DATA_BYTES);
        return;
    }

    CanFdData* pData = new CanFdData();
    pData->msg_id = msg_id;
    pData->length = len;
    pData->extd = extd;
    pData->extd_id = extd_id;
    memcpy(pData->data, data, len);
    
    std::function<void(EventData*)> event = [&](EventData* data) {
        BEGIN_TRANSITION_MAP                            // - Current State -
            TRANSITION_MAP_ENTRY (EVENT_IGNORED)        // ST_IDLE
            TRANSITION_MAP_ENTRY (EVENT_IGNORED)        // ST_INIT
            TRANSITION_MAP_ENTRY (ST_SEND)              // ST_WAIT
            TRANSITION_MAP_ENTRY (EVENT_IGNORED)        // ST_SEND
            TRANSITION_MAP_ENTRY (EVENT_IGNORED)        // ST_ERROR
        END_TRANSITION_MAP(data)
    };

    std::pair<std::function<void(EventData*)>, EventData*> eventPair(event, pData);
    sendEvent(eventPair);
}

STATE_DEFINE(CanFdDriver, Idle, NoEventData)
{
    ESP_LOGI(TAG, "CanFdDriver::ST_IDLE");
}

STATE_DEFINE(CanFdDriver, Init, NoEventData)
{
    ESP_LOGI(TAG, "CanFdDriver::ST_INIT");

    gpio_set_direction(_canfd.chip_select, GPIO_MODE_OUTPUT);
    gpio_set_level(_canfd.chip_select, 1);

    // Reset device
    DRV_CANFDSPI_Reset(&_canfd);

    // Enable ECC and initialize RAM
    DRV_CANFDSPI_EccEnable(&_canfd);

    DRV_CANFDSPI_RamInit(&_canfd, 0xff);

    // Configure device
    CAN_CONFIG config;
    DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;
    DRV_CANFDSPI_Configure(&_canfd, &config);

    // Setup TX FIFO
    CAN_TX_FIFO_CONFIG txConfig;
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
    txConfig.FifoSize = _canfd.tx_config.FifoSize;
    txConfig.PayLoadSize = _canfd.tx_config.PayLoadSize;
    txConfig.TxPriority = _canfd.tx_config.TxPriority;
    DRV_CANFDSPI_TransmitChannelConfigure(&_canfd, APP_TX_FIFO, &txConfig);

    // Setup RX FIFO
    CAN_RX_FIFO_CONFIG rxConfig;
    DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
    rxConfig.FifoSize = _canfd.rx_config.FifoSize;
    rxConfig.PayLoadSize = _canfd.rx_config.PayLoadSize;
    DRV_CANFDSPI_ReceiveChannelConfigure(&_canfd, APP_RX_FIFO, &rxConfig);

    // Setup RX Filter
    REG_CiFLTOBJ fObj;
    fObj.word = 0;
    // fObj.bF.SID11 = _canfd.filter_obj.bF.SID11;
    fObj.bF.SID = _canfd.filter_obj.bF.SID;
    fObj.bF.EXIDE = _canfd.filter_obj.bF.EXIDE;
    fObj.bF.EID = _canfd.filter_obj.bF.EID;
    DRV_CANFDSPI_FilterObjectConfigure(&_canfd, CAN_FILTER0, &fObj.bF);

    // Setup RX Mask
    REG_CiMASK mObj;
    mObj.word = 0;
    // mObj.bF.MSID11 = _canfd.filter_mask.bF.MSID11;
    mObj.bF.MSID = _canfd.filter_mask.bF.MSID;
    mObj.bF.MIDE = _canfd.filter_mask.bF.MIDE;
    mObj.bF.MEID = _canfd.filter_mask.bF.MEID;
    DRV_CANFDSPI_FilterMaskConfigure(&_canfd, CAN_FILTER0, &mObj.bF);

    // Link FIFO and Filter
    DRV_CANFDSPI_FilterToFifoLink(&_canfd, CAN_FILTER0, APP_RX_FIFO, true);

    // Setup Bit Time
    DRV_CANFDSPI_BitTimeConfigure(&_canfd, _canfd.bit_time, CAN_SSP_MODE_AUTO, _canfd.sysclk);

    // Setup Transmit and Receive Interrupts
    DRV_CANFDSPI_GpioModeConfigure(&_canfd, GPIO_MODE_GPIO, GPIO_MODE_INT);
    DRV_CANFDSPI_GpioDirectionConfigure(&_canfd, GPIO_OUTPUT, GPIO_OUTPUT);
    DRV_CANFDSPI_GpioPinSet(&_canfd, GPIO_PIN_0, GPIO_HIGH);
    DRV_CANFDSPI_TransmitChannelEventEnable(&_canfd, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
    DRV_CANFDSPI_ReceiveChannelEventEnable(&_canfd, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
    DRV_CANFDSPI_ModuleEventEnable(&_canfd, CAN_TX_EVENT | CAN_RX_EVENT);

    // Select Normal Mode
    DRV_CANFDSPI_OperationModeSelect(&_canfd, _canfd.mode);

    rx_sem = xSemaphoreCreateBinary();
    if (rx_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create RX semaphore.");
        return;
    }
    tx_sem = xSemaphoreCreateBinary();
    if (tx_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create TX semaphore.");
        return;
    }

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(int_task, "int_task", 1024 * 2, this, 10, NULL);
    xTaskCreate(rx_task, "canfd_rx_task", 1024 * 3, this, 10, NULL);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // falling edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((1ULL<<_canfd.int_pin) | (1ULL<<_canfd.int1_pin));
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(_canfd.int_pin, int_isr_handler, (void*) _canfd.int_pin);
    gpio_isr_handler_add(_canfd.int1_pin, rx_int_isr_handler, (void*) _canfd.int1_pin);

    ESP_LOGI(TAG, "Driver started");
    InternalEvent(ST_WAIT);
}

STATE_DEFINE(CanFdDriver, Wait, NoEventData)
{
    ESP_LOGI(TAG, "CanFdDriver::ST_WAIT");
}

STATE_DEFINE(CanFdDriver, Send, CanFdData)
{
    ESP_LOGI(TAG, "CanFdDriver::ST_SEND");

    // Check if FIFO is not full
    CAN_TX_FIFO_EVENT tx_flags;
    DRV_CANFDSPI_TransmitChannelEventGet(&_canfd, APP_TX_FIFO, &tx_flags);
    if (!(tx_flags & CAN_TX_FIFO_NOT_FULL_EVENT)) {
        ESP_LOGE(TAG, "TX FIFO is full");
        _pClient->canError();
        if (xSemaphoreTake(tx_sem, TX_WAIT_TIMEOUT) == pdTRUE) {} // Wait until TX FIFO is not full, timeout in 1 second
        else {
            ESP_LOGE(TAG, "Failed to take TX semaphore.");
            InternalEvent(ST_WAIT);
            return;
        }
    }

    CAN_DLC can_dlc = DRV_CANFDSPI_DataBytesToDlc(data->length);

    // Configure transmit message
    CAN_TX_MSGOBJ txObj;
    txObj.word[0] = 0;
    txObj.word[1] = 0;

    txObj.bF.id.SID = data->msg_id;
    if (data->extd) {
        txObj.bF.id.EID = data->extd_id;
        txObj.bF.ctrl.IDE = 1;
    } else {
        txObj.bF.id.EID = 0;
        txObj.bF.ctrl.IDE = 0;
    }

    txObj.bF.ctrl.BRS = 1;
    txObj.bF.ctrl.DLC = can_dlc;
    txObj.bF.ctrl.FDF = 1;

    // Load message and transmit
    uint8_t data_len = DRV_CANFDSPI_DlcToDataBytes(can_dlc);

    DRV_CANFDSPI_TransmitChannelLoad(&_canfd, APP_TX_FIFO, &txObj, (uint8_t*)data->data, data_len, true);
    
    InternalEvent(ST_WAIT);
}

STATE_DEFINE(CanFdDriver, Error, NoEventData)
{
    ESP_LOGE(TAG, "CanFdDriver::ST_ERROR");
}