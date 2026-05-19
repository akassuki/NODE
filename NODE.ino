/**
 * main.ino
 * ══════════════════════════════════════════════════════════════
 * Entry point: setup() + loop()
 * Định nghĩa các biến global dùng chung (extern trong lora_config.h)
 *
 * Cấu trúc file:
 *   lora_config.h   — defines, structs, extern declarations
 *   update_ota.h/cpp — ring buffer, delta OTA, LoRaRxTask, ProcessTask
 *   polling.h/cpp    — PollingTask, send_data, wait_ack, build_json
 *   main.ino         — setup(), loop(), định nghĩa global
 */

#include "polling.h"  

// ════════════════════════════════════════════════════════════
// ĐỊNH NGHĨA GLOBAL — extern khai báo trong lora_config.h
// ════════════════════════════════════════════════════════════
HardwareSerial loraSerial(1);
LoRa_E32       e32(LORA_TX_PIN, LORA_RX_PIN, &loraSerial, UART_BPS_RATE_9600);

TaskHandle_t g_pollingTask  = NULL;
TaskHandle_t g_rxTask       = NULL;
TaskHandle_t g_processTask  = NULL;

// ════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    randomSeed(esp_random());

    Serial.printf("\n[Node 0x%02X] Khởi động...\n", NODE_ADDL);

    loraSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    e32.begin();

    ResponseStructContainer csc = e32.getConfiguration();
    if (csc.status.code == SUCCESS) {
        Configuration cfg = *(Configuration*)csc.data;
        cfg.ADDH = NODE_ADDH;
        cfg.ADDL = NODE_ADDL;
        cfg.CHAN  = LORA_CH;
        cfg.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
        e32.setConfiguration(cfg, WRITE_CFG_PWR_DWN_SAVE);
        Serial.printf("[Node 0x%02X] LoRa OK\n", NODE_ADDL);
    } else {
        Serial.printf("[Node 0x%02X] LoRa config FAIL\n", NODE_ADDL);
    }
    csc.close();

    // Semaphore cho ring buffer OTA — init tại đây vì update_ota.cpp dùng
    rbMutex   = xSemaphoreCreateMutex();
    dataReady = xSemaphoreCreateCounting(RB_SIZE, 0);

    // Chỉ tạo PollingTask lúc đầu
    // OTA tasks (LoRaRxTask + ProcessTask) tạo sau khi nhận CMD_OTA
    xTaskCreatePinnedToCore(
        PollingTask, "PollingTask", 8192, NULL, 2, &g_pollingTask, 0
    );

    Serial.printf("[Node 0x%02X] Sẵn sàng\n", NODE_ADDL);
}

// ════════════════════════════════════════════════════════════
// LOOP — không làm gì, mọi logic chạy trong FreeRTOS tasks
// ════════════════════════════════════════════════════════════
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
