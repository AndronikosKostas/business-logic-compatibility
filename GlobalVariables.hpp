#pragma once

#include "ErrorMaps.hpp"
#include "StateMachineTask.hpp"
#include "String.hpp"

#include <ECSS_Definitions.hpp>
#include <ErrorDefinitions.hpp>
#include <FreeRTOS.h>
#include <queue.h>

#define OBC_APPLICATION_ID 1
#define TTC_APPLICATION_ID 2

#define MAX_PARAMETERS 128
#define NUM_OF_TMP_PARAMS 3
#define NUM_OF_PWR_PARAMS 3

#define EPS_BUF_SIZE 500
#define MAX_CW_CHARS 50


#define FIXED_LENGTH_AT_SIZE_TX 512
#define FIXED_LENGTH_AT_SIZE_RX 512

#define CRC_LENGTH 4
#define PREAMBLE_LENGTH 32

static uint8_t LUT[255] = {
    0xff, 0x48, 0x0e, 0xc0, 0x9a, 0x0d, 0x70, 0xbc, 0x8e, 0x2c, 0x93, 0xad,
    0xa7, 0xb7, 0x46, 0xce, 0x5a, 0x97, 0x7d, 0xcc, 0x32, 0xa2, 0xbf, 0x3e,
    0x0a, 0x10, 0xf1, 0x88, 0x94, 0xcd, 0xea, 0xb1, 0xfe, 0x90, 0x1d, 0x81,
    0x34, 0x1a, 0xe1, 0x79, 0x1c, 0x59, 0x27, 0x5b, 0x4f, 0x6e, 0x8d, 0x9c,
    0xb5, 0x2e, 0xfb, 0x98, 0x65, 0x45, 0x7e, 0x7c, 0x14, 0x21, 0xe3, 0x11,
    0x29, 0x9b, 0xd5, 0x63, 0xfd, 0x20, 0x3b, 0x02, 0x68, 0x35, 0xc2, 0xf2,
    0x38, 0xb2, 0x4e, 0xb6, 0x9e, 0xdd, 0x1b, 0x39, 0x6a, 0x5d, 0xf7, 0x30,
    0xca, 0x8a, 0xfc, 0xf8, 0x28, 0x43, 0xc6, 0x22, 0x53, 0x37, 0xaa, 0xc7,
    0xfa, 0x40, 0x76, 0x04, 0xd0, 0x6b, 0x85, 0xe4, 0x71, 0x64, 0x9d, 0x6d,
    0x3d, 0xba, 0x36, 0x72, 0xd4, 0xbb, 0xee, 0x61, 0x95, 0x15, 0xf9, 0xf0,
    0x50, 0x87, 0x8c, 0x44, 0xa6, 0x6f, 0x55, 0x8f, 0xf4, 0x80, 0xec, 0x09,
    0xa0, 0xd7, 0x0b, 0xc8, 0xe2, 0xc9, 0x3a, 0xda, 0x7b, 0x74, 0x6c, 0xe5,
    0xa9, 0x77, 0xdc, 0xc3, 0x2a, 0x2b, 0xf3, 0xe0, 0xa1, 0x0f, 0x18, 0x89,
    0x4c, 0xde, 0xab, 0x1f, 0xe9, 0x01, 0xd8, 0x13, 0x41, 0xae, 0x17, 0x91,
    0xc5, 0x92, 0x75, 0xb4, 0xf6, 0xe8, 0xd9, 0xcb, 0x52, 0xef, 0xb9, 0x86,
    0x54, 0x57, 0xe7, 0xc1, 0x42, 0x1e, 0x31, 0x12, 0x99, 0xbd, 0x56, 0x3f,
    0xd2, 0x03, 0xb0, 0x26, 0x83, 0x5c, 0x2f, 0x23, 0x8b, 0x24, 0xeb, 0x69,
    0xed, 0xd1, 0xb3, 0x96, 0xa5, 0xdf, 0x73, 0x0c, 0xa8, 0xaf, 0xcf, 0x82,
    0x84, 0x3c, 0x62, 0x25, 0x33, 0x7a, 0xac, 0x7f, 0xa4, 0x07, 0x60, 0x4d,
    0x06, 0xb8, 0x5e, 0x47, 0x16, 0x49, 0xd6, 0xd3, 0xdb, 0xa3, 0x67, 0x2d,
    0x4b, 0xbe, 0xe6, 0x19, 0x51, 0x5f, 0x9f, 0x05, 0x08, 0x78, 0xc4, 0x4a,
    0x66, 0xf5, 0x58};

#define REPORT_ERROR_WITH_CONTEXT(code, isr, ms_to_wait_if_queue_is_full)        \
do {                                                                         \
char context_buffer[128];                                                \
buildErrorContext(context_buffer, sizeof(context_buffer), __FILE__, __LINE__, __func__); \
reportError(code, isr, ms_to_wait_if_queue_is_full, context_buffer);     \
} while (0)


enum class Component : uint8_t;
struct PacketHandler {
    uint8_t buf[FIXED_LENGTH_AT_SIZE_TX];
    int16_t data_length;
    bool beacon_active = false;

    // Constructor
    PacketHandler(const uint8_t* init_data = nullptr, int16_t length = 0) : data_length(length) {
        if (init_data && length <= FIXED_LENGTH_AT_SIZE_TX) {
            memcpy(buf, init_data, length);
        } else {
            memset(buf, 0, sizeof(buf));
            data_length = 0;
        }
    }
};

struct TimersHandler {
    uint32_t timer;
    uint32_t period_ms;

    TimersHandler() {
        timer = 0;
        period_ms = 0;
    }
};



struct PacketHandlerCW {
    uint16_t sequence_length;
    bool active;
    char sequence_buf[MAX_CW_CHARS];

    // Constructor
    PacketHandlerCW(const char* init_data = nullptr, int16_t length = 0)
        : sequence_length(0), active(false) {
        if (init_data && length > 0 && length <= MAX_CW_CHARS) {
            memcpy(sequence_buf, init_data, length);
            sequence_length = length;
        } else {
            memset(sequence_buf, 0, sizeof(sequence_buf));
        }
    }
};

namespace InternalFunctionManagement {
    enum functionID : uint16_t {

        // GNSS Task States (5-8)
        GNSS_DISABLED = 5,
        GNSS_NOMINAL = 6,
        GNSS_SAFE = 7,
        GNSS_SCIENCE = 8,

        // RF_RX Task States (9-11)
        RF_RX_DISABLED = 9,
        RF_RX_NOMINAL = 10,
        RF_RX_SAFE = 11,

        // RF_TX Task States (12-14)
        RF_TX_DISABLED = 12,
        RF_TX_NOMINAL = 13,
        RF_TX_SAFE = 14,

        // RF_CW Task States (15-17)
        RF_CW_DISABLED = 15,
        RF_CW_NOMINAL = 16,
        RF_CW_SAFE = 17,

        // RF_ISR Task States (18-20)
        RF_ISR_NOMINAL = 18,
        RF_ISR_SAFE = 19,
        RF_ISR_DISABLED = 20,

        // CAN_GATEKEEPER Task States (21-23)
        CAN_GATEKEEPER_DISABLED = 21,
        CAN_GATEKEEPER_NOMINAL = 22,
        CAN_GATEKEEPER_SAFE = 23,

        // TM Task States (24-26)
        TM_DISABLED = 24,
        TM_NOMINAL = 25,


        PMON_DISABLED = 26,
        PMON_NOMINAL = 27,

        TIMEKEEPING_DISABLED = 28,
        TIMEKEEPING_NOMINAL = 29,
        TIMEKEEPING_SAFE = 30,

        EPS_DISABLED = 31,
        EPS_NOMINAL = 32,
        EPS_SAFE = 33,

        COMPONENT_RF_AVAILABLE = 34,

        COMPONENT_EMMC_AVAILABLE = 36,

        COMPONENT_GNSS_AVAILABLE = 38,

        COMPONENT_INT_TMP_AVAILABLE = 40,

        COMPONENT_TMP_PCB_AVAILABLE = 42,

        COMPONENT_TMP_PA_AVAILABLE = 44,

        COMPONENT_TMP_GNSS_AVAILABLE = 46,

        COMPONENT_TMP_ALL_AVAILABLE = 48,

        COMPONENT_INA_AVAILABLE = 49,

        COMPONENT_CAN_1_AVAILABLE = 51,
        COMPONENT_CAN_1_UNAVAILABLE = 52,

        COMPONENT_CAN_2_AVAILABLE = 53,
        COMPONENT_CAN_2_UNAVAILABLE = 54,

        GNSS_RESTART = 58,

        PMON_PA_TMP_ABOVE_HIGH_LIMIT = 59,

        PMON_RF_POWER_BELOW_LOW_LIMIT = 62,

        HEARTBEAT_TIMEOUT = 80,
        CAN_AVAILABLE = 81,
        CAN_UNAVAILABLE = 82,
        OBC_RECOVERED = 83,

        HW_INIT_MODE_EXIT = 84,
        COMPONENTS_STATUS_OK = 86,

        COMMISSIONING_MODE_EXIT = 89,
        RF_PASS_ACTIVE = 90,
        RF_TRANSCEIVER_RESTART = 91,

    };
}

struct EventHandler {
    struct TaskState {
        uint8_t task_id;
        uint8_t state;
    }task;

    SpacecraftErrorCode spacecraft_error;
    uint16_t function_id;

    bool internal_function_active;
    bool external_function_active;

    bool is_component_failure = false;
    bool is_component_recovery = false;
    Component component = Component::NONE;

    bool pmon_active = false;
    uint8_t pmon_event = 0;

    char function_data_chars[50];
    uint16_t function_data_length_chars;
    String<ECSSEventDataAuxiliaryMaxSize> event_message;

    bool debug_report_active;

    explicit EventHandler(SpacecraftErrorCode spacecraft_error_code = GENERIC_ERROR_NONE,
                          uint16_t id = 0,
                          bool pmon = false,
                          const String<ECSSEventDataAuxiliaryMaxSize> msg = "")
        : task(), spacecraft_error(spacecraft_error_code),
          function_id(id),
          internal_function_active(false),
          external_function_active(false),
          function_data_length_chars(0),
          event_message(msg),
          debug_report_active(false) {
        std::fill(std::begin(function_data_chars), std::end(function_data_chars), 0);
    }
};

struct CCSDSHEADER {
    uint8_t packet_version_number;
    uint8_t packet_type;
    uint8_t secondary_header_flag;
    uint16_t application_process_ID;
    uint16_t packet_data_length;
    uint8_t sequence_flags;
    uint16_t sequence_count;
    uint16_t ccsds_length;


    explicit CCSDSHEADER(uint8_t version, uint8_t type, uint8_t header_flag, uint16_t app_id, uint16_t packet_data_length)
        : packet_version_number(version),
          packet_type(type),
          secondary_header_flag(header_flag),
          application_process_ID(app_id),
          packet_data_length(packet_data_length) {}
};

inline bool heartbeatReceived = true;


inline QueueHandle_t event_handler_queue;
inline StaticQueue_t event_handler_buff;
constexpr uint8_t event_handler_item_num = 50;
constexpr size_t event_handler_item_size = sizeof(EventHandler);
inline uint8_t event_handler_storage_area[event_handler_item_num * event_handler_item_size];

// TX Queue configuration
constexpr uint8_t TX_QUEUE_ITEM_NUM = 40;
constexpr size_t TX_ITEM_SIZE = sizeof(PacketHandler);
inline QueueHandle_t g_tx_queue;
inline StaticQueue_t g_tx_queue_buffer;
inline uint8_t g_tx_queue_storage_area[TX_QUEUE_ITEM_NUM * TX_ITEM_SIZE] __attribute__((section(".dtcmram_outgoingTMQueueStorageArea")));


// TX Queue configuration
constexpr uint8_t TX_QUEUE_ITEM_NUM_CW = 3;
constexpr size_t TX_ITEM_SIZE_CW = sizeof(PacketHandlerCW);
inline QueueHandle_t g_tx_queue_cw;
inline StaticQueue_t g_tx_queue_buffer_cw;
inline uint8_t g_tx_queue_storage_area_cw[TX_QUEUE_ITEM_NUM * TX_ITEM_SIZE_CW];

inline QueueHandle_t TCQueue;
inline StaticQueue_t TCQueueBuffer;
constexpr uint8_t TCQueueItemNum = 5;
constexpr size_t TCItemSize = sizeof(PacketHandler);
inline uint8_t TCQueueStorageArea[TCQueueItemNum * TCItemSize];

const char* shortenPath(const char* path);
void buildErrorContext(char* out, size_t size, const char* file, int line, const char* func);
void reportError(const UnifiedModuleError& err, bool isr, uint16_t ms_to_wait_if_queue_is_full);
void reportComponentFailure(Component failed_component, bool isr = false, uint16_t ms_to_wait_if_queue_is_full = 50);
void performInternalFunction(InternalFunctionManagement::functionID function_id, bool isr = false, uint16_t ms_to_wait_if_queue_is_full = 50);
void reportError(SpacecraftErrorCode code, bool isr, uint16_t ms_to_wait_if_queue_is_full);
void reportError(SpacecraftErrorCode code, bool isr, uint16_t ms_to_wait_if_queue_is_full, const char* context);

inline uint8_t EPSBuffer[EPS_BUF_SIZE]{};

inline uint8_t g_rx_buf[FIXED_LENGTH_AT_SIZE_RX];
