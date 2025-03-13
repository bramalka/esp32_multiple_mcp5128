#include "canfd_driver.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pthread.h>

#include <string.h>

#ifdef MAIN_BOARD
#define SPI_HOST_NUM    SPI2_HOST
#define SPI_MOSI_PIN    GPIO_NUM_3
#define SPI_MISO_PIN    GPIO_NUM_20
#define SPI_SCLK_PIN    GPIO_NUM_9

#define CANFD_CS_PIN_1      GPIO_NUM_19
#define CANFD_INT_PIN_1     GPIO_NUM_12
#define CANFD_INT1_PIN_1    GPIO_NUM_10

#define CANFD_CS_PIN_2      GPIO_NUM_14
#define CANFD_INT_PIN_2     GPIO_NUM_48
#define CANFD_INT1_PIN_2    GPIO_NUM_47

#define PWR_CTRL_PIN      GPIO_NUM_8
#define LED_BUILTIN       GPIO_NUM_0

#else
#define SPI_HOST_NUM        SPI2_HOST
#define SPI_MOSI_PIN        GPIO_NUM_14
#define SPI_MISO_PIN        GPIO_NUM_12
#define SPI_SCLK_PIN        GPIO_NUM_27

#define CANFD_CS_PIN        GPIO_NUM_25
#define CANFD_INT_PIN       GPIO_NUM_13
#define CANFD_INT1_PIN      GPIO_NUM_26

#define LED_BUILTIN GPIO_NUM_0
#endif

#define USE_CANFD

static const char TAG[] = "MAIN RX";

static spi_device_handle_t can_handle_1, can_handle_2;
static pthread_mutex_t spi_mutex;

void printTaskInfo(char* buffer)
{
    TaskStatus_t* taskStatusArray;
    UBaseType_t uxArraySize;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    uxArraySize = uxTaskGetNumberOfTasks();
    if (uxArraySize > 0u) {
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
        int len = sprintf(buffer, "FreeRTOS Free/Total heap: %d / %d (%d%%)\n", free_heap, total_heap, free_heap * 100 / total_heap);
        
        taskStatusArray = (TaskStatus_t*)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
        if (taskStatusArray != NULL) {
            uxArraySize = uxTaskGetSystemState(taskStatusArray, uxArraySize, &ulTotalRunTime);
            ulTotalRunTime /= 100UL;

            /* Avoid divide by zero errors. */
            if (ulTotalRunTime > 0UL) {
                len += sprintf(buffer + len, "%-16s\tRun Time (%%)\tFree Stack\r\n", "Taskname");
                for (UBaseType_t i = 0; i < uxArraySize; i++) {
                    ulStatsAsPercentage = taskStatusArray[i].ulRunTimeCounter / ulTotalRunTime;
                    len += sprintf(buffer + len, "%-16s\t", taskStatusArray[i].pcTaskName);
                    if (ulStatsAsPercentage > 0UL) {
                        len += sprintf(buffer + len, "   %lu%%\t\t", ulStatsAsPercentage);
                    } else {
                        len += sprintf(buffer + len, "   <1%%\t\t");
                    }
                    len += sprintf(buffer + len, "   %lu\n", taskStatusArray[i].usStackHighWaterMark);
                }
            }
        }
        vPortFree(taskStatusArray);
    }
}

esp_err_t initSpiMaster(spi_host_device_t host_id, int mosi_pin, int miso_pin, int sclk_pin)
{
    esp_err_t err;
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = mosi_pin;
    buscfg.miso_io_num = miso_pin;
    buscfg.sclk_io_num = sclk_pin;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 0;   // 0 means that max transfer size is 4k bytes

    err = spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE("CAN", "SPI bus initialization failed, err = %d", err);
        return err;
    }
    
    // Add two separate devices to the same bus with different CS pins
    spi_device_interface_config_t devcfg1 = {};
    devcfg1.command_bits = 0;
    devcfg1.address_bits = 0;
    devcfg1.dummy_bits = 0;
    devcfg1.clock_speed_hz = 10 * 1000 * 1000;
    devcfg1.mode = 0;
    devcfg1.spics_io_num = CANFD_CS_PIN_1;
    devcfg1.queue_size = 1;
    devcfg1.cs_ena_posttrans = 3;  // Keep CS active longer after transaction
    err = spi_bus_add_device(host_id, &devcfg1, &can_handle_1);
    if (err != ESP_OK) {
        ESP_LOGE("CAN", "SPI bus device 1 add failed, err = %d", err);
        return err;
    }

    spi_device_interface_config_t devcfg2 = {};
    devcfg2.command_bits = 0;
    devcfg2.address_bits = 0;
    devcfg2.dummy_bits = 0;
    devcfg2.clock_speed_hz = 10 * 1000 * 1000;
    devcfg2.mode = 0;
    devcfg2.spics_io_num = CANFD_CS_PIN_2;
    devcfg2.queue_size = 1;
    devcfg2.cs_ena_posttrans = 3;  // Keep CS active longer after transaction
    err = spi_bus_add_device(host_id, &devcfg2, &can_handle_2);
    if (err != ESP_OK) {
        ESP_LOGE("CAN", "SPI bus device 2 add failed, err = %d", err);
        return err;
    }

    return err;
}

int extractNumberAfterColon(const char* str) {
    // Find the position of the colon
    const char* colonPos = strchr(str, ':');
    if (colonPos == NULL) {
        // Colon not found
        return -1;
    }

    // Extract the substring after the colon
    const char* numberStr = colonPos + 1;

    // Convert the substring to an integer
    int number = atoi(numberStr);

    return number;
}

uint32_t combine_can_ids(uint16_t std_id, uint32_t ext_id) {
    // Ensure std_id fits in 11 bits
    std_id &= 0x7FF;
    
    // Ensure ext_id fits in 18 bits
    ext_id &= 0x3FFFF;
    
    // Combine: std_id goes in bits 18-28, ext_id goes in bits 0-17
    uint32_t combined_id = ((uint32_t)std_id << 18) | ext_id;
    
    // Set the extended ID bit (bit 31)
    // combined_id |= CAN_EXTD_ID_MASK;
    
    return combined_id;
}

class ClientDummy : public CanFdDClient {
public:
    uint32_t _lastReceive = 0;
    uint32_t _frameCounter = 0;
    uint32_t _fdFrameCounter = 0;
    
    void canDataReady(uint32_t msg_id, const uint8_t* data, uint8_t length, bool extd, uint32_t extd_id) override
    {
        // Ignore empty frames (likely spurious interrupts)
        if (msg_id == 0 && extd_id == 0) {
            // ESP_LOGD(TAG, "Ignoring empty frame");
            return;
        }

        uint32_t current = esp_log_timestamp();
        uint32_t duration = current - _lastReceive;
        _frameCounter++;
        
        // Check if this is likely a CAN FD frame based on message length
        bool is_canfd = length > 8;
        if (is_canfd) _fdFrameCounter++;
        
        const char* frame_type = is_canfd ? "CAN FD" : "CAN";
        
        // For J1939 messages, reconstruct the full 29-bit ID
        uint32_t j1939_id = combine_can_ids(msg_id, extd_id);
        if (extd) {
            // Extract J1939 components
            uint8_t priority = (j1939_id >> 26) & 0x7;
            uint32_t pgn = (j1939_id >> 8) & 0xFFFF;
            uint8_t source_addr = j1939_id & 0xFF;
            
            // Reconstruct the full ID in J1939 format
            // j1939_id = (priority << 26) | (pgn << 8) | source_addr;
            
            ESP_LOGI(TAG, "------------- %s FRAME RECEIVED -------------", frame_type);
            ESP_LOGI(TAG, "MSG ID : 0x%08lX || EXT ID : 0x%08lX", msg_id, extd_id);
            ESP_LOGI(TAG, "CAN ID : 0x%08lX", j1939_id);
            ESP_LOGI(TAG, "Priority=%d, PGN=0x%04lX, Source=0x%02X", 
                     priority, pgn, source_addr);
        } else {
            ESP_LOGI(TAG, "------------- %s FRAME RECEIVED -------------", frame_type);
            ESP_LOGI(TAG, "MSG ID : 0x%08lX || EXT ID : 0x%08lX", msg_id, extd_id);
        }
        
        // Add ASCII representation if the data might be text
        if (isTextData(data, length)) {
            char text_buffer[65];
            memcpy(text_buffer, data, length);
            text_buffer[length] = '\0';
            ESP_LOGI(TAG, "ASCII: %s", text_buffer);
        }
        
        _lastReceive = current;
    }
    
    void canError() override {
        ESP_LOGE(TAG, "CAN Error");
    }
    
private:
    int last_number = 0;
    uint32_t packet_counter = 0;
    uint32_t skip_counter = 0;
    
    // Helper to determine if data might be text
    bool isTextData(const uint8_t* data, uint8_t length) {
        int printable_chars = 0;
        for (int i = 0; i < length; i++) {
            if (data[i] >= 32 && data[i] <= 126) {
                printable_chars++;
            }
        }
        return (printable_chars > length * 0.7); // If >70% are printable ASCII
    }
};

void can_config(can_fd_t& can_fd_config, uint16_t pgn_filter, 
               gpio_num_t cs_pin, gpio_num_t int_pin, gpio_num_t int1_pin) {
    can_fd_config.p_spi = (cs_pin == CANFD_CS_PIN_1) ? &can_handle_1 : &can_handle_2;
    can_fd_config.p_spi_mutex = &spi_mutex;
    can_fd_config.chip_select = cs_pin;
    can_fd_config.int_pin = int_pin;
    can_fd_config.int1_pin = int1_pin;

    can_fd_config.bit_time = CAN_500K_2M;   // first value is NOMINAL (CAN 2.0), second is DATA (CAN FD)
    can_fd_config.sysclk = CAN_SYSCLK_20M;  // Based on used oscillator clock

    // Changing below config needs experiment (see skip counter from receiver)
    can_fd_config.tx_config.FifoSize = 7;
    can_fd_config.tx_config.PayLoadSize = CAN_PLSIZE_64;
    can_fd_config.tx_config.TxPriority = 1;

    can_fd_config.rx_config.FifoSize = 16;
    can_fd_config.rx_config.PayLoadSize = CAN_PLSIZE_64;


    if(pgn_filter != 0) {
        ESP_LOGW(TAG, "PGN filter set");
    
        // Set up filter for PGN 0xF004
        can_fd_config.filter_obj.word = 0; // Clear all bits first
        can_fd_config.filter_obj.bF.EXIDE = 1; // We want extended ID frames
        can_fd_config.filter_obj.bF.EID = (pgn_filter << 8);
    
        // Set up the mask to only consider PGN bits for matching
        can_fd_config.filter_mask.word = 0; // Clear all bits first
        can_fd_config.filter_mask.bF.MIDE = 1; // Force matching of ID type (extended only)
        can_fd_config.filter_mask.bF.MEID = 0x00FFFF00; // Match only bits 8-23 (PGN), ignore priority and source
    }

    can_fd_config.mode = CAN_NORMAL_MODE;
}

extern "C" void app_main()
{
    #ifdef MAIN_BOARD
    gpio_set_direction(PWR_CTRL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_CTRL_PIN, 0);
    ESP_LOGI(TAG, "PWR_CTRL_PIN set to 0");
    #endif

    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_BUILTIN, 0);

    ESP_LOGW(TAG, "RECEIVER MODE");

    if (initSpiMaster(SPI_HOST_NUM, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCLK_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Init SPI Master failed");
        for (;;)
            ;
    }

    if (pthread_mutex_init(&spi_mutex, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to initialize the spi_mutex");
        for (;;)
            ;
    }

    ClientDummy client1, client2;

    can_fd_t can_fd_conf1 = {}, can_fd_conf2 = {};
    can_config(can_fd_conf1, 0xE002, CANFD_CS_PIN_1, CANFD_INT_PIN_1, CANFD_INT1_PIN_1);
    can_config(can_fd_conf2, 0xF002, CANFD_CS_PIN_2, CANFD_INT_PIN_2, CANFD_INT1_PIN_2);

    CanFdDriver canfdDriver1, canfdDriver2;
    canfdDriver1.setClient(&client1);
    canfdDriver1.init(can_fd_conf1);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    canfdDriver2.setClient(&client2);
    canfdDriver2.init(can_fd_conf2);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (true) {
        // printTaskInfo(stack_info);
        // printf("%s\n", stack_info);

        // printf("FreeRTOS Free/Total heap: %d / %d (%d%%)\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_total_size(MALLOC_CAP_8BIT), heap_caps_get_free_size(MALLOC_CAP_8BIT) * 100 / heap_caps_get_total_size(MALLOC_CAP_8BIT));

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}