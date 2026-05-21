#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <Arduino.h>
#include <LoRa_E32.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"

// ════════════════════════════════════════════════════════════
// ĐỊA CHỈ NODE  ← ĐỔI cho từng node trước khi flash
// ════════════════════════════════════════════════════════════
#define NODE_ADDH     0x00
#define NODE_ADDL     0x02    

#define GW_ADDH       0x00
#define GW_ADDL       0x00

// ════════════════════════════════════════════════════════════
// LORA E32
// ════════════════════════════════════════════════════════════
#define LORA_TX_PIN   34
#define LORA_RX_PIN   25
#define LORA_BAUD     9600
#define LORA_CH       20

// ════════════════════════════════════════════════════════════
// GIAO THỨC
// ════════════════════════════════════════════════════════════
#define CMD_POLL          0x01
#define CMD_DATA          0x02
#define CMD_ACK           0x03
#define CMD_OTA           0x10
#define CMD_HEARTBEAT     0x20
#define CMD_ERROR         0xFF

#define POLL_FRAME_LEN    4     // [CMD_POLL][GW_ADDH][GW_ADDL][XOR]
#define OTA_FRAME_LEN     7     // [CMD_OTA][NODE_ADDH][NODE_ADDL][CH][OTA_ADDH][OTA_ADDL][XOR]
#define ACK_FRAME_LEN     4     // [CMD_ACK][GW_ADDH][GW_ADDL][frag_idx]
                                // E32 tự bóc 3 bytes prefix → node nhận đúng 4 bytes body
#define FRAG_PAYLOAD_MAX  49    // 55 - 6 byte header
#define ACK_TIMEOUT_MS    8000
#define POLL_WAIT_MS      10
#define FRAG_TX_RETRY     5
#define CHUNK_SIZE        50
#define RB_SIZE           16

// ════════════════════════════════════════════════════════════
// STRUCTS
// ════════════════════════════════════════════════════════════
typedef struct __attribute__((packed)) {
    uint16_t index;
    uint16_t total;
    uint8_t  len;
    uint8_t  data[CHUNK_SIZE];
} LoraChunk;   // 55 bytes

typedef struct __attribute__((packed)) {
    uint8_t  status;
    uint16_t index;
} AckPkt;      // 3 bytes

typedef enum {
    STREAM_TYPE_PART,   // đọc từ flash partition
    STREAM_TYPE_OTA,    // ghi vào OTA partition
} StreamType;

typedef struct {
    StreamType             type;
    // Partition stream
    const esp_partition_t* partition;
    long int               offset;
    size_t                 part_size;
    // OTA stream
    esp_ota_handle_t       ota_handle;
    uint8_t                ota_buf[512];
    size_t                 ota_buf_pos;
    long int               pos;
} UnifiedStream;

#define JANPATCH_STREAM UnifiedStream

// ════════════════════════════════════════════════════════════
// EXTERN — định nghĩa thực sự nằm trong main.ino
// ════════════════════════════════════════════════════════════
extern HardwareSerial loraSerial;
extern LoRa_E32       e32;

// Task handles — định nghĩa trong main.ino, dùng chung giữa các file
extern TaskHandle_t g_pollingTask;
extern TaskHandle_t g_rxTask;
extern TaskHandle_t g_processTask;

// Ring buffer OTA — định nghĩa trong update_ota.cpp
extern LoraChunk         ring[];
extern volatile int      rbHead;
extern volatile int      rbTail;
extern SemaphoreHandle_t rbMutex;
extern SemaphoreHandle_t dataReady;

#endif // LORA_CONFIG_H
