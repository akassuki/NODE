#include "polling.h"

// ════════════════════════════════════════════════════════════
// FLAG OTA — định nghĩa thực sự tại đây (extern trong polling.h)
// ════════════════════════════════════════════════════════════
volatile bool g_ota_in_progress = false;

// ════════════════════════════════════════════════════════════
// TIỆN ÍCH
// ════════════════════════════════════════════════════════════
static uint8_t xor_chk(const uint8_t* buf, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) c ^= buf[i];
    return c;
}

static float sim_val(float base, float range) {
    return base + (float)(millis() % (uint32_t)(range * 1000)) / 1000.0f;
}

// ════════════════════════════════════════════════════════════
// BUILD JSON
// ════════════════════════════════════════════════════════════
static int build_json(int test_case, char* buf, size_t buf_size) {
    if (test_case == CASE_SMALL) {
        return snprintf(buf, buf_size,
            "{\"temp\":%.1f,\"hum\":%.1f,\"volt\":%.2f,\"uptime\":%lu}",
            sim_val(20.0f,5.0f), sim_val(55.0f,5.0f),
            sim_val(3.60f,0.4f), millis()/1000UL);

    } else if (test_case == CASE_MEDIUM) {
        return snprintf(buf, buf_size,
            "{\"t1\":%.1f,\"t2\":%.1f,\"t3\":%.1f,\"t4\":%.1f,"
            "\"t5\":%.1f,\"t6\":%.1f,\"t7\":%.1f,\"t8\":%.1f,"
            "\"h1\":%.1f,\"h2\":%.1f,\"h3\":%.1f,\"h4\":%.1f,"
            "\"h5\":%.1f,\"h6\":%.1f,\"h7\":%.1f,\"h8\":%.1f,"
            "\"v1\":%.2f,\"v2\":%.2f,\"v3\":%.2f,\"v4\":%.2f,"
            "\"v5\":%.2f,\"v6\":%.2f,\"v7\":%.2f,\"v8\":%.2f}",
            sim_val(20.0f,5.0f),sim_val(21.0f,5.0f),
            sim_val(22.0f,5.0f),sim_val(23.0f,5.0f),
            sim_val(24.0f,5.0f),sim_val(25.0f,5.0f),
            sim_val(26.0f,5.0f),sim_val(27.0f,5.0f),
            sim_val(60.0f,5.0f),sim_val(61.0f,5.0f),
            sim_val(62.0f,5.0f),sim_val(63.0f,5.0f),
            sim_val(64.0f,5.0f),sim_val(65.0f,5.0f),
            sim_val(66.0f,5.0f),sim_val(67.0f,5.0f),
            sim_val(3.60f,0.4f),sim_val(3.61f,0.4f),
            sim_val(3.62f,0.4f),sim_val(3.63f,0.4f),
            sim_val(3.64f,0.4f),sim_val(3.65f,0.4f),
            sim_val(3.66f,0.4f),sim_val(3.67f,0.4f));

    } else {
        return snprintf(buf, buf_size,
            "{\"t1\":%.1f,\"t2\":%.1f,\"t3\":%.1f,\"t4\":%.1f,"
            "\"t5\":%.1f,\"t6\":%.1f,\"t7\":%.1f,\"t8\":%.1f,"
            "\"t9\":%.1f,\"t10\":%.1f,"
            "\"h1\":%.1f,\"h2\":%.1f,\"h3\":%.1f,\"h4\":%.1f,"
            "\"h5\":%.1f,\"h6\":%.1f,\"h7\":%.1f,\"h8\":%.1f,"
            "\"h9\":%.1f,\"h10\":%.1f,"
            "\"v1\":%.2f,\"v2\":%.2f,\"v3\":%.2f,\"v4\":%.2f,"
            "\"v5\":%.2f,\"v6\":%.2f,\"v7\":%.2f,\"v8\":%.2f,"
            "\"v9\":%.2f,\"v10\":%.2f}",
            sim_val(20.0f,5.0f),sim_val(21.0f,5.0f),
            sim_val(22.0f,5.0f),sim_val(23.0f,5.0f),
            sim_val(24.0f,5.0f),sim_val(25.0f,5.0f),
            sim_val(26.0f,5.0f),sim_val(27.0f,5.0f),
            sim_val(28.0f,5.0f),sim_val(29.0f,5.0f),
            sim_val(60.0f,5.0f),sim_val(61.0f,5.0f),
            sim_val(62.0f,5.0f),sim_val(63.0f,5.0f),
            sim_val(64.0f,5.0f),sim_val(65.0f,5.0f),
            sim_val(66.0f,5.0f),sim_val(67.0f,5.0f),
            sim_val(68.0f,5.0f),sim_val(69.0f,5.0f),
            sim_val(3.60f,0.4f),sim_val(3.61f,0.4f),
            sim_val(3.62f,0.4f),sim_val(3.63f,0.4f),
            sim_val(3.64f,0.4f),sim_val(3.65f,0.4f),
            sim_val(3.66f,0.4f),sim_val(3.67f,0.4f),
            sim_val(3.68f,0.4f),sim_val(3.69f,0.4f));
    }
}

// ════════════════════════════════════════════════════════════
// CHỜ ACK (dùng trong gửi DATA fragment)
// E32 fixed transmission tự bóc 3 bytes prefix [DST_ADDH][DST_ADDL][CH]
// → node nhận đúng body: [CMD_ACK][GW_ADDH][GW_ADDL][frag_idx] = 4 bytes
// ════════════════════════════════════════════════════════════
static bool wait_ack(uint8_t expected_idx) {
    vTaskDelay(pdMS_TO_TICKS(30));
    uint32_t start = millis();
    while (millis() - start < ACK_TIMEOUT_MS) {
        if (e32.available() >= ACK_FRAME_LEN) {
            ResponseStructContainer rsc = e32.receiveMessage(ACK_FRAME_LEN);
            if (rsc.status.code != SUCCESS) {
                rsc.close();
                vTaskDelay(pdMS_TO_TICKS(POLL_WAIT_MS));
                continue;
            }
            uint8_t* p = (uint8_t*)rsc.data;
            Serial.printf("[0x%02X] ACK bytes: %02X %02X %02X %02X\n",
                          NODE_ADDL, p[0], p[1], p[2], p[3]);

            if (p[0] != CMD_ACK)                    { rsc.close(); continue; }
            if (p[1] != GW_ADDH || p[2] != GW_ADDL) { rsc.close(); continue; }

            if (p[3] == expected_idx) {
                // ACK đúng fragment
                Serial.printf("[0x%02X] ACK(%d) OK\n", NODE_ADDL, expected_idx);
                rsc.close();
                return true;
            }

            if (p[3] < expected_idx) {
                // ACK cũ từ retry của gateway — reset timer, tiếp tục chờ
                Serial.printf("[0x%02X] ACK(%d) cũ, đang chờ ACK(%d) — reset timer\n",
                              NODE_ADDL, p[3], expected_idx);
                rsc.close();
                start = millis();  // ← reset timer, không tính là timeout
                continue;
            }

            // p[3] > expected_idx: bất thường
            Serial.printf("[0x%02X] ACK(%d) > expected(%d), bỏ qua\n",
                          NODE_ADDL, p[3], expected_idx);
            rsc.close();
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_WAIT_MS));
    }
    Serial.printf("[0x%02X] ACK(%d) timeout!\n", NODE_ADDL, expected_idx);
    return false;
}

// ════════════════════════════════════════════════════════════
// GỬI DATA FRAGMENTS
// ════════════════════════════════════════════════════════════
static void send_data(int test_case) {
    static char json[512];
    int jlen = build_json(test_case, json, sizeof(json));
    if (jlen <= 0 || jlen >= (int)sizeof(json)) {
        Serial.println("[ERR] JSON lỗi");
        return;
    }

    const char* cases[] = {"SMALL(4)", "MEDIUM(24)", "LARGE(30)"};
    uint8_t frag_total = (jlen + FRAG_PAYLOAD_MAX - 1) / FRAG_PAYLOAD_MAX;
    Serial.printf("[0x%02X] Case=%s  JSON=%dB  frags=%d\n",
                  NODE_ADDL, cases[test_case], jlen, frag_total);

    for (uint8_t idx = 0; idx < frag_total; idx++) {
        uint16_t offset  = idx * FRAG_PAYLOAD_MAX;
        uint8_t  pay_len = (uint8_t)min((int)FRAG_PAYLOAD_MAX, jlen - (int)offset);

        uint8_t frame[55];
        frame[0] = CMD_DATA;
        frame[1] = NODE_ADDH;
        frame[2] = NODE_ADDL;
        frame[3] = idx;
        frame[4] = frag_total;
        frame[5] = pay_len;
        memcpy(&frame[6], json + offset, pay_len);
        uint8_t frame_len = 6 + pay_len;

        bool ack_ok = false;
        for (uint8_t tx_try = 0; tx_try < FRAG_TX_RETRY; tx_try++) {
            ResponseStatus rs = e32.sendFixedMessage(
                GW_ADDH, GW_ADDL, LORA_CH, frame, frame_len);
            if (rs.code != SUCCESS) {
                Serial.printf("[0x%02X] TX frag %d FAIL code=%d\n",
                              NODE_ADDL, idx, rs.code);
                break;
            }
            Serial.printf("[0x%02X] TX frag %d/%d  len=%d  try=%d\n",
                          NODE_ADDL, idx, frag_total-1, pay_len, tx_try+1);

            if (idx == frag_total - 1) { ack_ok = true; break; }
            if (wait_ack(idx))         { ack_ok = true; break; }

            Serial.printf("[0x%02X] Retry frag %d (lần %d/%d)\n",
                          NODE_ADDL, idx, tx_try+1, FRAG_TX_RETRY);
        }

        if (!ack_ok) {
            Serial.printf("[0x%02X] Huỷ tại frag %d sau %d lần thử\n",
                          NODE_ADDL, idx, FRAG_TX_RETRY);
            return;
        }
    }
    Serial.printf("[0x%02X] Gửi xong\n", NODE_ADDL);
}

// ════════════════════════════════════════════════════════════
// POLLING TASK — task chính (Core 0)
//
// Bình thường: chờ CMD_POLL → gửi DATA fragments
// Khi có OTA:  nhận CMD_OTA → set flag → tạo 2 task OTA
//              → loop tiếp nhưng skip (chờ flag clear)
// Sau OTA:     ProcessTask clear flag → polling tự tiếp tục
// ════════════════════════════════════════════════════════════
void PollingTask(void* pv) {
    Serial.printf("[PollingTask] Bắt đầu, chờ POLL...\n");

    while (true) {
        if (g_ota_in_progress) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (e32.available() < POLL_FRAME_LEN) {
            vTaskDelay(pdMS_TO_TICKS(POLL_WAIT_MS));
            continue;
        }

        ResponseStructContainer rsc = e32.receiveMessage(POLL_FRAME_LEN);
        if (rsc.status.code != SUCCESS) { rsc.close(); continue; }

        uint8_t* p   = (uint8_t*)rsc.data;
        uint8_t  cmd = p[0];

        Serial.printf("[0x%02X] RX cmd=0x%02X: %02X %02X %02X %02X\n",
                      NODE_ADDL, cmd, p[0], p[1], p[2], p[3]);

        // ── Bỏ qua ACK cũ còn sót sau khi gửi xong fragment cuối ──
        if (cmd == CMD_ACK) {
            Serial.printf("[0x%02X] ACK cũ (idx=%d) còn sót — flush\n",
                          NODE_ADDL, p[3]);
            rsc.close();
            continue;
        }

        if (cmd == CMD_POLL) {
            if (p[1] != GW_ADDH || p[2] != GW_ADDL || xor_chk(p, 3) != p[3]) {
                Serial.printf("[0x%02X] POLL frame lỗi\n", NODE_ADDL);
                rsc.close();
                continue;
            }
            rsc.close();

            Serial.printf("[0x%02X] POLL hợp lệ\n", NODE_ADDL);
            int test_case = random(0, 3);
            const char* cases[] = {"SMALL(4)", "MEDIUM(24)", "LARGE(30)"};
            Serial.printf("[0x%02X] Case = %d (%s)\n",
                          NODE_ADDL, test_case, cases[test_case]);
            send_data(test_case);
        }

        else if (cmd == CMD_OTA) {
            if (p[1] != NODE_ADDH || p[2] != NODE_ADDL) {
                Serial.printf("[0x%02X] CMD_OTA không phải cho node này\n", NODE_ADDL);
                rsc.close();
                continue;
            }
            rsc.close();

            Serial.printf("[0x%02X] CMD_OTA hợp lệ — bắt đầu OTA\n", NODE_ADDL);
            g_ota_in_progress = true;
            start_ota_tasks();
        }

        else {
            Serial.printf("[0x%02X] Frame lạ cmd=0x%02X, bỏ qua\n",
                          NODE_ADDL, cmd);
            rsc.close();
        }
    }
}
