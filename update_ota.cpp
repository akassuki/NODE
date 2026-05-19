#include "update_ota.h"
#include "janpatch.h"

// ════════════════════════════════════════════════════════════
// RING BUFFER — định nghĩa thực sự (extern trong lora_config.h)
// ════════════════════════════════════════════════════════════
LoraChunk         ring[RB_SIZE];
volatile int      rbHead = 0;
volatile int      rbTail = 0;
SemaphoreHandle_t rbMutex   = NULL;
SemaphoreHandle_t dataReady = NULL;

bool rbFull()  { return ((rbHead + 1) % RB_SIZE) == rbTail; }
bool rbEmpty() { return rbHead == rbTail; }

bool pushChunk(const LoraChunk* c) {
    if (xSemaphoreTake(rbMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    if (rbFull()) { xSemaphoreGive(rbMutex); return false; }
    memcpy(&ring[rbHead], c, sizeof(LoraChunk));
    rbHead = (rbHead + 1) % RB_SIZE;
    xSemaphoreGive(rbMutex);
    xSemaphoreGive(dataReady);
    return true;
}

bool popChunk(LoraChunk* out) {
    if (xSemaphoreTake(rbMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    if (rbEmpty()) { xSemaphoreGive(rbMutex); return false; }
    memcpy(out, &ring[rbTail], sizeof(LoraChunk));
    rbTail = (rbTail + 1) % RB_SIZE;
    xSemaphoreGive(rbMutex);
    return true;
}

// ════════════════════════════════════════════════════════════
// GỬI ACK CHO OTA CHUNK
// ════════════════════════════════════════════════════════════
void sendAck(uint8_t status, uint16_t index) {
    vTaskDelay(pdMS_TO_TICKS(300));
    AckPkt ack = { status, index };
    e32.sendFixedMessage(GW_ADDH, GW_ADDL, LORA_CH, &ack, sizeof(ack));
}

// ════════════════════════════════════════════════════════════
// DELTA OTA — fread / fwrite / fseek / ftell
// ════════════════════════════════════════════════════════════
static size_t my_fread(void* buf, size_t size, size_t count, UnifiedStream* s) {
    size_t bytes = size * count;
    if (s->type == STREAM_TYPE_PART) {
        if (esp_partition_read(s->partition, (size_t)s->offset, buf, bytes) != ESP_OK)
            return 0;
        s->offset += bytes;
        s->pos    += bytes;
        return count;
    }
    return 0;
}

static size_t my_fwrite(const void* buf, size_t size, size_t count, UnifiedStream* s) {
    size_t bytes = size * count;
    if (s->type == STREAM_TYPE_OTA) {
        const uint8_t* src = (const uint8_t*)buf;
        size_t written = 0;
        while (written < bytes) {
            size_t space = 512 - s->ota_buf_pos;
            size_t copy  = (space < bytes - written) ? space : bytes - written;
            memcpy(s->ota_buf + s->ota_buf_pos, src + written, copy);
            s->ota_buf_pos += copy;
            written        += copy;
            if (s->ota_buf_pos == 512) {
                if (esp_ota_write(s->ota_handle, s->ota_buf, 512) != ESP_OK) {
                    Serial.println("[FWRITE] ota_write 512 FAIL");
                    return 0;
                }
                s->ota_buf_pos = 0;
            }
        }
        s->pos += bytes;
        return count;
    }
    if (s->type == STREAM_TYPE_PART) {
        s->pos += bytes;
        return count;
    }
    return 0;
}

static int my_fseek(UnifiedStream* s, long int offset, int whence) {
    if      (whence == SEEK_SET) { s->offset = offset;  s->pos = offset; }
    else if (whence == SEEK_CUR) { s->offset += offset; s->pos += offset; }
    return 0;
}

static long my_ftell(UnifiedStream* s) { return s->pos; }

static bool ota_flush(UnifiedStream* s) {
    if (s->ota_buf_pos > 0) {
        if (esp_ota_write(s->ota_handle, s->ota_buf, s->ota_buf_pos) != ESP_OK)
            return false;
        s->ota_buf_pos = 0;
    }
    return true;
}

// ════════════════════════════════════════════════════════════
// APPLY DELTA OTA
// ════════════════════════════════════════════════════════════
bool applyDeltaOTA(size_t patchSize) {
    Serial.println("[DELTA] Bắt đầu apply patch...");

    const esp_partition_t* oldPart   = esp_ota_get_running_partition();
    const esp_partition_t* patchPart = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "patch");
    const esp_partition_t* newPart   = esp_ota_get_next_update_partition(NULL);

    if (!oldPart || !patchPart || !newPart) {
        Serial.println("[DELTA] ERROR: Không tìm thấy partition");
        return false;
    }

    esp_ota_handle_t otaHandle = 0;
    esp_err_t err = esp_ota_begin(newPart, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle);
    if (err != ESP_OK) {
        Serial.printf("[DELTA] ERROR: ota_begin: %s\n", esp_err_to_name(err));
        return false;
    }

    UnifiedStream src, pat, tgt;

    memset(&src, 0, sizeof(src));
    src.type      = STREAM_TYPE_PART;
    src.partition = oldPart;
    src.part_size = oldPart->size;

    memset(&pat, 0, sizeof(pat));
    pat.type      = STREAM_TYPE_PART;
    pat.partition = patchPart;
    pat.part_size = patchPart->size;

    memset(&tgt, 0, sizeof(tgt));
    tgt.type       = STREAM_TYPE_OTA;
    tgt.ota_handle = otaHandle;

    uint8_t buf_src[512], buf_pat[512], buf_tgt[512];

    janpatch_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.fread  = my_fread;
    ctx.fwrite = my_fwrite;
    ctx.fseek  = my_fseek;
    ctx.ftell  = my_ftell;

    ctx.source_buffer.buffer = buf_src;
    ctx.source_buffer.size   = sizeof(buf_src);
    ctx.source_buffer.stream = &src;

    ctx.patch_buffer.buffer  = buf_pat;
    ctx.patch_buffer.size    = sizeof(buf_pat);
    ctx.patch_buffer.stream  = &pat;

    ctx.target_buffer.buffer = buf_tgt;
    ctx.target_buffer.size   = sizeof(buf_tgt);
    ctx.target_buffer.stream = &tgt;

    ctx.max_file_size = (long)(oldPart->size + patchSize);

    Serial.println("[DELTA] Đang apply janpatch...");
    int ret = janpatch(ctx, &src, &pat, &tgt);
    Serial.printf("[DELTA] janpatch ret=%d | output=%ld bytes\n", ret, tgt.pos);

    if (ret != 0) {
        Serial.printf("[DELTA] ERROR: janpatch failed: %d\n", ret);
        esp_ota_abort(otaHandle);
        return false;
    }
    if (!ota_flush(&tgt)) {
        Serial.println("[DELTA] ERROR: flush fail");
        esp_ota_abort(otaHandle);
        return false;
    }

    err = esp_ota_end(otaHandle);
    if (err != ESP_OK) {
        Serial.printf("[DELTA] ota_end FAIL: %s\n", esp_err_to_name(err));
        esp_ota_abort(otaHandle);
        return false;
    }
    err = esp_ota_set_boot_partition(newPart);
    if (err != ESP_OK) {
        Serial.printf("[DELTA] ERROR: set_boot: %s\n", esp_err_to_name(err));
        return false;
    }

    Serial.println("[DELTA] THÀNH CÔNG!");
    return true;
}

// ════════════════════════════════════════════════════════════
// OTA TASK 1 — LoRaRxTask (Core 0)
// Nhận LoraChunk, push ring buffer, gửi ACK
// ════════════════════════════════════════════════════════════
void LoRaRxTask(void* pv) {
    Serial.println("[RxTask] Bắt đầu nhận chunk OTA...");
    uint16_t lastReceivedIdx = 0xFFFF;

    while (true) {
        if (e32.available() < (int)sizeof(LoraChunk)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        ResponseStructContainer rsc = e32.receiveMessage(sizeof(LoraChunk));
        if (rsc.status.code != SUCCESS) { rsc.close(); continue; }

        LoraChunk* pkt = (LoraChunk*)rsc.data;

        if (pkt->len == 0 || pkt->len > CHUNK_SIZE) {
            sendAck(0xFF, pkt->index); rsc.close(); continue;
        }
        if (pkt->index == lastReceivedIdx) {
            // Duplicate: ACK lại nhưng không push
            sendAck(0xAA, pkt->index); rsc.close(); continue;
        }
        if (!pushChunk(pkt)) {
            sendAck(0xFF, pkt->index); rsc.close(); continue;
        }
        lastReceivedIdx = pkt->index;
        sendAck(0xAA, pkt->index);
        rsc.close();
    }
}

// ════════════════════════════════════════════════════════════
// OTA TASK 2 — ProcessTask (Core 1)
// Pop chunk, ghi patch partition, apply delta OTA
// Thành công → restart | Thất bại → clear flag → PollingTask tự tiếp tục
// ════════════════════════════════════════════════════════════
void ProcessTask(void* pv) {
    Serial.println("[ProcessTask] Bắt đầu xử lý patch...");

    LoraChunk c;
    bool     firstChunk  = true;
    uint16_t lastIndex   = 0;
    size_t   patchSize   = 0;
    uint32_t totalRecv   = 0;
    uint32_t totalUnique = 0;
    uint32_t totalDup    = 0;
    uint32_t totalMiss   = 0;

    const esp_partition_t* patchPart = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "patch");

    if (!patchPart) {
        Serial.println("[ProcessTask] ERROR: Không tìm thấy patch partition");
        goto ota_fail;
    }

    Serial.println("[ProcessTask] Xóa patch partition...");
    esp_partition_erase_range(patchPart, 0, patchPart->size);

    while (true) {
        xSemaphoreTake(dataReady, portMAX_DELAY);
        if (!popChunk(&c)) continue;

        totalRecv++;

        if (!firstChunk && c.index == lastIndex) {
            totalDup++;
            Serial.printf("[DUP]  idx=%-5u\n", c.index);
            continue;
        }
        if (!firstChunk && c.index < lastIndex) {
            Serial.printf("[OOO]  idx=%-5u\n", c.index);
            continue;
        }
        if (!firstChunk && c.index > (uint16_t)(lastIndex + 1)) {
            uint16_t missed = c.index - lastIndex - 1;
            totalMiss += missed;
            Serial.printf("[MISS] Chunk %u..%u bị mất\n", lastIndex + 1, c.index - 1);
        }

        firstChunk = false;
        lastIndex  = c.index;
        totalUnique++;

        size_t offset = (size_t)c.index * CHUNK_SIZE;
        esp_err_t err = esp_partition_write(patchPart, offset, c.data, c.len);
        if (err != ESP_OK) {
            Serial.printf("[ProcessTask] ERROR: write patch idx=%u: %s\n",
                          c.index, esp_err_to_name(err));
            continue;
        }
        patchSize += c.len;

        Serial.println("----------------------------------------");
        Serial.printf("[OK]  Chunk thứ   : %u (tổng nhận=%u)\n", totalUnique, totalRecv);
        Serial.printf("      Số thứ tự   : idx=%u / total=%u\n",  c.index, c.total);
        Serial.printf("      Độ dài data : %u bytes\n",            c.len);
        Serial.printf("      Patch size  : %u bytes\n",            patchSize);
        Serial.printf("      Bị mất      : %u chunk\n",            totalMiss);
        Serial.printf("      Trùng lặp   : %u lần\n",              totalDup);
        Serial.println("      Data (hex):");
            for (uint8_t i = 0; i < c.len; i++) {
                if (i % 16 == 0) Serial.printf("        %02X: ", i);
                    Serial.printf("%02X ", c.data[i]);
                if ((i % 16 == 15) || (i == c.len - 1)) Serial.println();
            }
        Serial.flush();

        // ── Chunk cuối → apply delta OTA ─────────────────────
        if (c.index == c.total - 1) {
            Serial.printf("[PATCH] Nhận đủ %u chunks | size=%u bytes\n",
                          c.total, patchSize);
            if (applyDeltaOTA(patchSize)) {
                vTaskDelay(pdMS_TO_TICKS(3000));
                esp_restart();  // firmware mới, khởi động lại
            } else {
                Serial.println("[DELTA] FAIL — giữ nguyên firmware cũ");
                goto ota_fail;
            }
        }
    }

ota_fail:
    Serial.println("[ProcessTask] OTA thất bại, khôi phục polling...");

    // Xóa LoRaRxTask
    if (g_rxTask) {
        vTaskDelete(g_rxTask);
        g_rxTask = NULL;
    }

    // Reset ring buffer
    if (xSemaphoreTake(rbMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        rbHead = 0;
        rbTail = 0;
        xSemaphoreGive(rbMutex);
    }

    // Clear flag → PollingTask tự tiếp tục, KHÔNG dùng vTaskResume
    g_ota_in_progress = false;
    Serial.println("[ProcessTask] PollingTask tự tiếp tục polling");

    g_processTask = NULL;
    vTaskDelete(NULL);
}

// ════════════════════════════════════════════════════════════
// KHỞI ĐỘNG 2 TASK OTA — gọi từ PollingTask khi nhận CMD_OTA
// g_ota_in_progress đã được set = true trước khi gọi hàm này
// ════════════════════════════════════════════════════════════
void start_ota_tasks() {
    Serial.println("[OTA] Khởi động LoRaRxTask và ProcessTask...");

    // Reset ring buffer
    if (xSemaphoreTake(rbMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        rbHead = 0;
        rbTail = 0;
        xSemaphoreGive(rbMutex);
    }

    xTaskCreatePinnedToCore(LoRaRxTask,  "RX",   4096, NULL, 2, &g_rxTask,      0);
    xTaskCreatePinnedToCore(ProcessTask, "PROC", 8192, NULL, 1, &g_processTask, 1);

    Serial.println("[OTA] 2 task OTA đã tạo, PollingTask tự chờ qua flag");
}
