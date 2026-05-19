#ifndef UPDATE_OTA_H
#define UPDATE_OTA_H

#include "lora_config.h"

// ════════════════════════════════════════════════════════════
// FLAG OTA — định nghĩa thực sự trong polling.cpp
// ════════════════════════════════════════════════════════════
extern volatile bool g_ota_in_progress;

// ════════════════════════════════════════════════════════════
// RING BUFFER
// ════════════════════════════════════════════════════════════
bool rbFull();
bool rbEmpty();
bool pushChunk(const LoraChunk* c);
bool popChunk(LoraChunk* out);

// ════════════════════════════════════════════════════════════
// OTA HELPERS
// ════════════════════════════════════════════════════════════
void sendAck(uint8_t status, uint16_t index);
bool applyDeltaOTA(size_t patchSize);

// ════════════════════════════════════════════════════════════
// OTA TASKS
// ════════════════════════════════════════════════════════════
void LoRaRxTask(void* pv);
void ProcessTask(void* pv);

// ════════════════════════════════════════════════════════════
// KHỞI ĐỘNG 2 TASK OTA — gọi từ PollingTask khi nhận CMD_OTA
// ════════════════════════════════════════════════════════════
void start_ota_tasks();

#endif // UPDATE_OTA_H