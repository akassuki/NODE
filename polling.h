#ifndef POLLING_H
#define POLLING_H

#include "update_ota.h"

// ════════════════════════════════════════════════════════════
// TEST CASES
// ════════════════════════════════════════════════════════════
#define CASE_SMALL    0
#define CASE_MEDIUM   1
#define CASE_LARGE    2

// ════════════════════════════════════════════════════════════
// FLAG OTA — set true khi đang OTA, false khi xong
// Định nghĩa thực sự trong polling.cpp
// ════════════════════════════════════════════════════════════
extern volatile bool g_ota_in_progress;

// ════════════════════════════════════════════════════════════
// POLLING FUNCTIONS
// ════════════════════════════════════════════════════════════
void PollingTask(void* pv);

#endif // POLLING_H
