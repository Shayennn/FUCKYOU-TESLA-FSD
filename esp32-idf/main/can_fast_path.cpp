#include "can_fast_path.h"
#include "can_logger.h"

#include <cstring>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kLogTag[] = "tesla_fsd";
constexpr gpio_num_t kStatusLedPin = GPIO_NUM_2;
constexpr uint8_t kTxFailureLimit = 8;
constexpr size_t kRingBufSize = 64;
constexpr size_t kRingBufMask = kRingBufSize - 1;
constexpr UBaseType_t kCanTaskPriority = configMAX_PRIORITIES - 2;
constexpr uint32_t kCanTaskStackSize = 4096;
constexpr BaseType_t kCanTaskCore = 1;

struct RingBuf {
    tesla_fsd::can_frame buf[kRingBufSize];
    volatile uint32_t head = 0;
    volatile uint32_t tail = 0;
    uint32_t rx_timestamp[kRingBufSize];

    bool push(const tesla_fsd::can_frame& f, uint32_t ts_us) {
        uint32_t next = (head + 1) & kRingBufMask;
        if (next == tail) return false;
        buf[head] = f;
        rx_timestamp[head] = ts_us;
        head = next;
        return true;
    }

    bool pop(tesla_fsd::can_frame& f, uint32_t& ts_us) {
        if (tail == head) return false;
        f = buf[tail];
        ts_us = rx_timestamp[tail];
        tail = (tail + 1) & kRingBufMask;
        return true;
    }
};

struct HandlerStorage {
    int type = 1;
    union {
        tesla_fsd::LegacyHandler legacy;
        tesla_fsd::HW3Handler hw3;
        tesla_fsd::HW4Handler hw4;
    };

    HandlerStorage() : type(1), hw3{} {}
};

twai_node_handle_t gNode = nullptr;
RingBuf gRingBuf;
TaskHandle_t gCanTask = nullptr;
HandlerStorage gHandler;
can_fast::Stats gStats;
can_fast::RecoveryFn gRecoveryFn = nullptr;

bool gRecoveryArmed = false;
int64_t gRecoveryArmedTime = 0;
bool gRecoveryFsdWasEngaged = false;

uint8_t gTxData[8] = {};

bool sendFrame(void*, const tesla_fsd::can_frame& frame) {
    twai_frame_t tx = {};
    tx.header.id = frame.can_id;
    tx.header.dlc = frame.can_dlc;
    std::memcpy(gTxData, frame.data, sizeof(frame.data));
    tx.buffer = gTxData;
    tx.buffer_len = frame.can_dlc;
    return twai_node_transmit(gNode, &tx, 0) == ESP_OK;
}

tesla_fsd::HandleResult dispatch(tesla_fsd::can_frame& frame, const tesla_fsd::FrameSink& sink) {
    switch (gHandler.type) {
        case 0: return gHandler.legacy.handleMessage(frame, sink);
        case 1: return gHandler.hw3.handleMessage(frame, sink);
        case 2: return gHandler.hw4.handleMessage(frame, sink);
        default: return {};
    }
}

void checkRecovery(const tesla_fsd::can_frame& frame) {
    if (!gRecoveryArmed) return;
    if (frame.can_id != tesla_fsd::UI_DRIVER_ASSIST_CONTROL_ID) return;

    bool engaged = tesla_fsd::isFSDSelectedInUI(frame);

    if (!gRecoveryFsdWasEngaged && engaged) {
        gRecoveryFsdWasEngaged = true;
    } else if (gRecoveryFsdWasEngaged && !engaged) {
        gRecoveryArmed = false;
        gStats.recovery_armed = false;
        gRecoveryFsdWasEngaged = false;
        if (gRecoveryFn) gRecoveryFn();
        return;
    }

    int64_t elapsed = esp_timer_get_time() - gRecoveryArmedTime;
    if (elapsed > 30000000) {
        gRecoveryArmed = false;
        gStats.recovery_armed = false;
        gRecoveryFsdWasEngaged = false;
    }
}

void processFrame(tesla_fsd::can_frame& frame, uint32_t rx_ts_us) {
    (void)rx_ts_us;
    if (gStats.fail_safe) return;

    gStats.rx_count++;
    can_logger::push(frame, can_logger::Dir::RX);
    checkRecovery(frame);

    tesla_fsd::FrameSink sink = {nullptr, &sendFrame};
    int32_t t0 = (int32_t)esp_timer_get_time();

    tesla_fsd::HandleResult result = dispatch(frame, sink);

    if (!result.attemptedSend) {
        gStats.filtered_count++;
        return;
    }

    int32_t latency = (int32_t)esp_timer_get_time() - t0;

    if (result.sent) {
        gStats.tx_count++;
        gStats.consecutive_tx_fail = 0;
        gStats.last_latency_us = latency;
        if (latency > gStats.max_latency_us) gStats.max_latency_us = latency;
        if (latency < gStats.min_latency_us) gStats.min_latency_us = latency;
        can_logger::push(frame, can_logger::Dir::TX);
    } else {
        gStats.tx_fail_count++;
        if (gStats.consecutive_tx_fail < 0xFFFFFFFF) gStats.consecutive_tx_fail++;
        if (gStats.consecutive_tx_fail >= kTxFailureLimit) {
            gStats.fail_safe = true;
            gpio_set_level(kStatusLedPin, 1);
            ESP_LOGE(kLogTag, "Fail-safe: %lu TX failures (%u consecutive)",
                     (unsigned long)gStats.tx_fail_count,
                     (unsigned)gStats.consecutive_tx_fail);
        }
    }
}

bool IRAM_ATTR onRxDone(twai_node_handle_t handle,
                        const twai_rx_done_event_data_t*, void*) {
    uint8_t buf[8] = {};
    twai_frame_t rx = {};
    rx.buffer = buf;
    rx.buffer_len = sizeof(buf);
    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) return false;

    tesla_fsd::can_frame f{};
    f.can_id = rx.header.id;
    f.can_dlc = (uint8_t)rx.header.dlc;
    std::memcpy(f.data, buf, sizeof(buf));

    uint32_t ts = (uint32_t)esp_timer_get_time();
    gRingBuf.push(f, ts);

    BaseType_t woken = pdFALSE;
    if (gCanTask) vTaskNotifyGiveFromISR(gCanTask, &woken);
    return woken == pdTRUE;
}

bool IRAM_ATTR onStateChange(twai_node_handle_t,
                              const twai_state_change_event_data_t* edata, void*) {
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        gStats.fail_safe = true;
        gpio_set_level(kStatusLedPin, 1);
    }
    return false;
}

[[noreturn]] void canTaskFn(void*) {
    tesla_fsd::can_frame frame{};
    uint32_t ts = 0;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (gRingBuf.pop(frame, ts)) {
            processFrame(frame, ts);
        }
    }
}

}  // namespace

namespace can_fast {

void init(twai_node_handle_t node) {
    gNode = node;
    gpio_reset_pin(kStatusLedPin);
    gpio_set_direction(kStatusLedPin, GPIO_MODE_OUTPUT);
    gpio_set_level(kStatusLedPin, 0);
}

void start() {
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = &onRxDone;
    cbs.on_state_change = &onStateChange;
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(gNode, &cbs, nullptr));

    xTaskCreatePinnedToCore(canTaskFn, "can_fast", kCanTaskStackSize,
                            nullptr, kCanTaskPriority, &gCanTask, kCanTaskCore);
    ESP_ERROR_CHECK(twai_node_enable(gNode));
    ESP_LOGI(kLogTag, "CAN fast path started on core %d", (int)kCanTaskCore);
}

Stats get_stats() { return gStats; }

int get_handler_type() { return gHandler.type; }

void set_handler_type(int type) {
    gHandler.type = type;
    switch (type) {
        case 0: gHandler.legacy = {}; break;
        case 1: gHandler.hw3 = {}; break;
        case 2: gHandler.hw4 = {}; break;
        default: gHandler.type = 1; gHandler.hw3 = {}; break;
    }
}

void set_recovery_callback(RecoveryFn fn) { gRecoveryFn = fn; }

void arm_recovery() {
    gRecoveryArmed = true;
    gStats.recovery_armed = true;
    gRecoveryArmedTime = esp_timer_get_time();
    gRecoveryFsdWasEngaged = false;
}

}  // namespace can_fast
