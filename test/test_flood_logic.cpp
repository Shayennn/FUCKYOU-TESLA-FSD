/*
 * Desktop simulation test for flood scheduling logic.
 *
 *   cd test && make test_flood_logic && ./test_flood_logic
 */

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <vector>

struct can_frame {
  uint32_t can_id;
  uint8_t  can_dlc;
  uint8_t  data[8];
};

enum MessageKeyType : uint8_t {
  KEY_INVALID = 0,
  KEY_ID_ONLY,
  KEY_LOW3_MUX,
  KEY_FULL_BYTE_MUX,
};

struct MessageKey {
  uint32_t can_id = 0;
  uint8_t selector = 0;
  MessageKeyType type = KEY_INVALID;
};

struct FloodSlot {
  bool active = false;
  MessageKey key{};
  can_frame frame{};
  uint32_t rx_count = 0;
  uint32_t last_rx_us = 0;
  uint32_t source_period_us = 0;
  uint32_t flood_period_us = 0;
  uint32_t next_tx_us = 0;
  uint32_t flood_tx_count = 0;
  uint32_t last_flood_log_us = 0;
};

static constexpr size_t kFloodSlotCount = 8;
static constexpr uint32_t kFloodRateMultiplier = 100;
static constexpr int UI_DRIVING_SIDE_BIT = 40;
static constexpr int UI_DRIVING_SIDE_LEN = 2;
static constexpr int UI_APMV3_BRANCH_BIT = 40;
static constexpr int UI_APMV3_BRANCH_LEN = 3;
static constexpr uint8_t UI_DRIVING_SIDE_RIGHT = 1;
static constexpr uint8_t UI_APMV3_BRANCH_DEV = 2;

static std::vector<can_frame> busWrites;
static FloodSlot floodSlots[kFloodSlotCount];
static uint32_t simulatedNowUs = 0;

inline uint8_t readMuxID(const can_frame& frame) {
  return frame.data[0] & 0x07;
}

inline bool isFSDSelectedInUI(const can_frame& frame) {
  return (frame.data[4] >> 6) & 0x01;
}

inline void setSpeedProfileV12V13(can_frame& frame, int profile) {
  frame.data[6] &= ~0x06;
  frame.data[6] |= (profile << 1);
}

inline void setBit(can_frame& frame, int bit, bool value) {
  int byteIndex = bit / 8;
  int bitIndex = bit % 8;
  uint8_t mask = static_cast<uint8_t>(1U << bitIndex);
  if (value) {
    frame.data[byteIndex] |= mask;
  } else {
    frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
  }
}

inline uint32_t readField(const can_frame& frame, int startBit, int length) {
  uint32_t value = 0;
  for (int i = 0; i < length; ++i) {
    const int absoluteBit = startBit + i;
    const int byteIndex = absoluteBit / 8;
    const int bitIndex = absoluteBit % 8;
    if ((frame.data[byteIndex] >> bitIndex) & 0x01) {
      value |= (1UL << i);
    }
  }
  return value;
}

inline void setField(can_frame& frame, int startBit, int length, uint32_t value) {
  for (int i = 0; i < length; ++i) {
    setBit(frame, startBit + i, (value >> i) & 0x01);
  }
}

inline uint8_t readDrivingSide(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_DRIVING_SIDE_BIT, UI_DRIVING_SIDE_LEN));
}

inline void setDrivingSide(can_frame& frame, uint8_t value) {
  setField(frame, UI_DRIVING_SIDE_BIT, UI_DRIVING_SIDE_LEN, value);
}

inline uint8_t readApmv3Branch(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_APMV3_BRANCH_BIT, UI_APMV3_BRANCH_LEN));
}

inline void setApmv3Branch(can_frame& frame, uint8_t value) {
  setField(frame, UI_APMV3_BRANCH_BIT, UI_APMV3_BRANCH_LEN, value);
}

inline bool sameMessageKey(const MessageKey& lhs, const MessageKey& rhs) {
  return lhs.can_id == rhs.can_id &&
         lhs.selector == rhs.selector &&
         lhs.type == rhs.type;
}

MessageKey makeMessageKey(const can_frame& frame) {
  MessageKey key{};
  key.can_id = frame.can_id;

  switch (frame.can_id) {
  case 1006:
  case 1021:
    key.selector = readMuxID(frame);
    key.type = KEY_LOW3_MUX;
    break;
  case 1016:
    key.type = KEY_ID_ONLY;
    break;
  case 2047:
    key.selector = frame.data[0];
    key.type = KEY_FULL_BYTE_MUX;
    break;
  default:
    break;
  }

  return key;
}

inline bool isTimeDue(uint32_t now_us, uint32_t deadline_us) {
  return static_cast<int32_t>(now_us - deadline_us) >= 0;
}

inline uint32_t sanitizePeriodUs(uint32_t period_us) {
  return period_us == 0 ? 1 : period_us;
}

inline uint32_t computeFloodPeriodUs(uint32_t source_period_us) {
  return sanitizePeriodUs(source_period_us / kFloodRateMultiplier);
}

void resetFloodSlot(FloodSlot& slot) {
  slot.active = false;
  slot.key = MessageKey{};
  memset(&slot.frame, 0, sizeof(slot.frame));
  slot.rx_count = 0;
  slot.last_rx_us = 0;
  slot.source_period_us = 0;
  slot.flood_period_us = 0;
  slot.next_tx_us = 0;
  slot.flood_tx_count = 0;
  slot.last_flood_log_us = 0;
}

void resetState() {
  busWrites.clear();
  simulatedNowUs = 0;
  for (auto& slot : floodSlots) {
    resetFloodSlot(slot);
  }
}

FloodSlot* findOrCreateSlot(const MessageKey& key) {
  FloodSlot* emptySlot = nullptr;

  for (auto& slot : floodSlots) {
    if (slot.active && sameMessageKey(slot.key, key)) {
      return &slot;
    }
    if (!slot.active && !emptySlot) {
      emptySlot = &slot;
    }
  }

  if (!emptySlot) {
    return nullptr;
  }

  resetFloodSlot(*emptySlot);
  emptySlot->active = true;
  emptySlot->key = key;
  return emptySlot;
}

void updateFloodSlot(const can_frame& frame, uint32_t now_us) {
  MessageKey key = makeMessageKey(frame);
  if (key.type == KEY_INVALID) {
    return;
  }

  FloodSlot* slot = findOrCreateSlot(key);
  if (!slot) {
    return;
  }

  const uint32_t previous_rx_us = slot->last_rx_us;

  slot->frame = frame;
  slot->last_rx_us = now_us;
  slot->rx_count += 1;

  if (slot->rx_count < 2) {
    slot->source_period_us = 0;
    slot->flood_period_us = 0;
    slot->next_tx_us = 0;
    return;
  }

  slot->source_period_us = sanitizePeriodUs(now_us - previous_rx_us);
  slot->flood_period_us = computeFloodPeriodUs(slot->source_period_us);
  slot->next_tx_us = now_us + slot->flood_period_us;
}

void canSendImmediate(const can_frame& frame) {
  busWrites.push_back(frame);
}

void canSend(can_frame& frame) {
  canSendImmediate(frame);
  updateFloodSlot(frame, simulatedNowUs);
}

bool serviceFloodSlots(uint32_t now_us) {
  bool sentAny = false;

  for (auto& slot : floodSlots) {
    if (!slot.active || slot.rx_count < 2 || slot.flood_period_us == 0) {
      continue;
    }
    if (!isTimeDue(now_us, slot.next_tx_us)) {
      continue;
    }

    canSendImmediate(slot.frame);
    sentAny = true;
    slot.flood_tx_count += 1;

    slot.next_tx_us += slot.flood_period_us;
    while (isTimeDue(now_us, slot.next_tx_us)) {
      slot.next_tx_us += slot.flood_period_us;
    }
  }

  return sentAny;
}

struct CarManagerBase {
  int speedProfile = 1;
  virtual bool handelMessage(can_frame& frame) = 0;
  virtual ~CarManagerBase() = default;
};

struct LegacyHandler : public CarManagerBase {
  bool handelMessage(can_frame& frame) override {
    switch (frame.can_id) {
    case 1006:
      switch (readMuxID(frame)) {
      case 0: {
        auto off = static_cast<uint8_t>((frame.data[3] >> 1) & 0x3F) - 30;
        switch (off) {
        case 2: speedProfile = 2; break;
        case 1: speedProfile = 1; break;
        case 0: speedProfile = 0; break;
        default: break;
        }
        setBit(frame, 46, true);
        setSpeedProfileV12V13(frame, speedProfile);
        canSend(frame);
        break;
      }
      case 1:
        setBit(frame, 19, false);
        canSend(frame);
        break;
      }
      return true;
    case 2047:
      if (frame.data[0] != 2) return false;
      frame.data[5] &= ~(0x07 << 2);
      frame.data[5] |= (3 & 0x07) << 2;
      canSend(frame);
      return true;
    default:
      return false;
    }
  }
};

struct HW3Handler : public CarManagerBase {
  int speedOffset = 0;

  bool handelMessage(can_frame& frame) override {
    switch (frame.can_id) {
    case 1016: {
      uint8_t followDistance = (frame.data[5] & 0b11100000) >> 5;
      switch (followDistance) {
      case 1: speedProfile = 2; break;
      case 2: speedProfile = 1; break;
      case 3: speedProfile = 0; break;
      default: break;
      }
      setBit(frame, 5, true);
      setBit(frame, 14, true);
      setDrivingSide(frame, UI_DRIVING_SIDE_RIGHT);
      canSend(frame);
      return true;
    }
    case 1021: {
      auto index = readMuxID(frame);
      switch (index) {
      case 0: {
        int rawOff = static_cast<uint8_t>((frame.data[3] >> 1) & 0x3F) - 30;
        speedOffset = std::max(std::min(rawOff * 5, 100), 0);
        setBit(frame, 38, true);
        setBit(frame, 46, true);
        setSpeedProfileV12V13(frame, speedProfile);
        canSend(frame);
        break;
      }
      case 1:
        setApmv3Branch(frame, UI_APMV3_BRANCH_DEV);
        setBit(frame, 19, false);
        setBit(frame, 43, false);
        setBit(frame, 45, true);
        canSend(frame);
        break;
      case 2:
        frame.data[0] &= ~(0b11000000);
        frame.data[1] &= ~(0b00111111);
        frame.data[0] |= (speedOffset & 0x03) << 6;
        frame.data[1] |= (speedOffset >> 2);
        canSend(frame);
        break;
      }
      return true;
    }
    case 2047:
      if (frame.data[0] != 2) return false;
      frame.data[5] &= ~(0x07 << 2);
      frame.data[5] |= (3 & 0x07) << 2;
      canSend(frame);
      return true;
    default:
      return false;
    }
  }
};

struct HW4Handler : public CarManagerBase {
  bool handelMessage(can_frame& frame) override {
    switch (frame.can_id) {
    case 1016: {
      auto fd = (frame.data[5] & 0b11100000) >> 5;
      switch (fd) {
      case 1: speedProfile = 3; break;
      case 2: speedProfile = 2; break;
      case 3: speedProfile = 1; break;
      case 4: speedProfile = 0; break;
      case 5: speedProfile = 4; break;
      }
      setBit(frame, 5, true);
      setBit(frame, 14, true);
      setDrivingSide(frame, UI_DRIVING_SIDE_RIGHT);
      canSend(frame);
      return true;
    }
    case 1021: {
      auto index = readMuxID(frame);
      switch (index) {
      case 0:
        setBit(frame, 38, true);
        setBit(frame, 46, true);
        setBit(frame, 60, true);
        canSend(frame);
        break;
      case 1:
        setApmv3Branch(frame, UI_APMV3_BRANCH_DEV);
        setBit(frame, 19, false);
        setBit(frame, 43, false);
        setBit(frame, 45, true);
        setBit(frame, 47, true);
        canSend(frame);
        break;
      case 2:
        frame.data[7] &= ~(0x07 << 4);
        frame.data[7] |= (speedProfile & 0x07) << 4;
        canSend(frame);
        break;
      }
      return true;
    }
    case 2047:
      if (frame.data[0] != 2) return false;
      frame.data[5] &= ~(0x07 << 2);
      frame.data[5] |= (4 & 0x07) << 2;
      canSend(frame);
      return true;
    default:
      return false;
    }
  }
};

static int testsPassed = 0;
static int testsFailed = 0;

#define RUN(fn) do { \
  resetState(); \
  printf("  %-60s ", #fn); \
  fn(); \
  printf("PASS\n"); \
  testsPassed++; \
} while (0)

#define ASSERT_TRUE(expr) do { \
  if (!(expr)) { \
    printf("FAIL\n    line %d: %s\n", __LINE__, #expr); \
    testsFailed++; return; \
  } \
} while (0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

#define ASSERT_EQ(a, b) do { \
  auto _va = (a); auto _vb = (b); \
  using _ct = typename std::common_type<decltype(_va), decltype(_vb)>::type; \
  if (static_cast<_ct>(_va) != static_cast<_ct>(_vb)) { \
    printf("FAIL\n    line %d: %s == %lld, expected %s == %lld\n", \
           __LINE__, #a, (long long)static_cast<_ct>(_va), \
           #b, (long long)static_cast<_ct>(_vb)); \
    testsFailed++; return; \
  } \
} while (0)

static can_frame makeFrame(uint32_t id,
    uint8_t d0=0, uint8_t d1=0, uint8_t d2=0, uint8_t d3=0,
    uint8_t d4=0, uint8_t d5=0, uint8_t d6=0, uint8_t d7=0) {
  can_frame f{};
  f.can_id = id;
  f.can_dlc = 8;
  f.data[0] = d0;
  f.data[1] = d1;
  f.data[2] = d2;
  f.data[3] = d3;
  f.data[4] = d4;
  f.data[5] = d5;
  f.data[6] = d6;
  f.data[7] = d7;
  return f;
}

FloodSlot* findSlot(const MessageKey& key) {
  for (auto& slot : floodSlots) {
    if (slot.active && sameMessageKey(slot.key, key)) {
      return &slot;
    }
  }
  return nullptr;
}

int countActiveSlots() {
  int count = 0;
  for (const auto& slot : floodSlots) {
    if (slot.active) {
      count++;
    }
  }
  return count;
}

void test_makeMessageKey_distinguishes_mux_rules() {
  can_frame mux0 = makeFrame(1021, 0x00);
  can_frame mux1 = makeFrame(1021, 0x01);
  can_frame carCfg2 = makeFrame(2047, 0x02);
  can_frame carCfg3 = makeFrame(2047, 0x03);
  can_frame idOnly = makeFrame(1016, 0x07);

  MessageKey keyMux0 = makeMessageKey(mux0);
  MessageKey keyMux1 = makeMessageKey(mux1);
  MessageKey keyCfg2 = makeMessageKey(carCfg2);
  MessageKey keyCfg3 = makeMessageKey(carCfg3);
  MessageKey keyIdOnly = makeMessageKey(idOnly);

  ASSERT_EQ(keyMux0.type, KEY_LOW3_MUX);
  ASSERT_EQ(keyMux1.selector, 1);
  ASSERT_FALSE(sameMessageKey(keyMux0, keyMux1));
  ASSERT_EQ(keyCfg2.type, KEY_FULL_BYTE_MUX);
  ASSERT_FALSE(sameMessageKey(keyCfg2, keyCfg3));
  ASSERT_EQ(keyIdOnly.type, KEY_ID_ONLY);
  ASSERT_EQ(keyIdOnly.selector, 0);
}

void test_first_capture_only_arms_slot_without_flood() {
  simulatedNowUs = 100000;
  can_frame frame = makeFrame(1021, 0x01);
  canSend(frame);

  FloodSlot* slot = findSlot(makeMessageKey(frame));
  ASSERT_TRUE(slot != nullptr);
  ASSERT_EQ((int)busWrites.size(), 1);
  ASSERT_EQ(slot->rx_count, 1);
  ASSERT_EQ(slot->flood_period_us, 0);
  ASSERT_EQ(slot->next_tx_us, 0);

  ASSERT_FALSE(serviceFloodSlots(200000));
  ASSERT_EQ((int)busWrites.size(), 1);
}

void test_second_capture_learns_100x_rate_for_that_slot() {
  can_frame frame = makeFrame(1021, 0x00, 0, 0x12);

  simulatedNowUs = 100000;
  canSend(frame);
  simulatedNowUs = 150000;
  frame.data[2] = 0x34;
  canSend(frame);

  FloodSlot* slot = findSlot(makeMessageKey(frame));
  ASSERT_TRUE(slot != nullptr);
  ASSERT_EQ(slot->rx_count, 2);
  ASSERT_EQ(slot->source_period_us, 50000);
  ASSERT_EQ(slot->flood_period_us, 500);
  ASSERT_EQ(slot->next_tx_us, 150500);
  ASSERT_EQ(slot->frame.data[2], 0x34);

  ASSERT_FALSE(serviceFloodSlots(150499));
  ASSERT_EQ((int)busWrites.size(), 2);
  ASSERT_TRUE(serviceFloodSlots(150500));
  ASSERT_EQ((int)busWrites.size(), 3);
}

void test_each_message_key_tracks_its_own_rate_counter() {
  can_frame msgA = makeFrame(1021, 0x00);
  can_frame msgB = makeFrame(1021, 0x01);

  simulatedNowUs = 100000;
  canSend(msgA);
  simulatedNowUs = 150000;
  canSend(msgA);

  FloodSlot* slotA = findSlot(makeMessageKey(msgA));
  ASSERT_TRUE(slotA != nullptr);
  ASSERT_EQ(slotA->rx_count, 2);
  ASSERT_EQ(slotA->source_period_us, 50000);

  simulatedNowUs = 200000;
  canSend(msgB);
  FloodSlot* slotB = findSlot(makeMessageKey(msgB));
  ASSERT_TRUE(slotB != nullptr);
  ASSERT_EQ(slotB->rx_count, 1);
  ASSERT_EQ(slotA->rx_count, 2);

  simulatedNowUs = 300000;
  canSend(msgB);
  ASSERT_EQ(slotB->rx_count, 2);
  ASSERT_EQ(slotB->source_period_us, 100000);
  ASSERT_EQ(slotB->flood_period_us, 1000);
  ASSERT_EQ(slotA->source_period_us, 50000);
  ASSERT_EQ(slotA->flood_period_us, 500);
}

void test_new_source_frame_updates_only_matching_slot() {
  can_frame msgA = makeFrame(1021, 0x00, 0, 0x11);
  can_frame msgB = makeFrame(1021, 0x01, 0, 0x22);

  simulatedNowUs = 100000;
  canSend(msgA);
  simulatedNowUs = 150000;
  canSend(msgA);

  simulatedNowUs = 200000;
  canSend(msgB);
  simulatedNowUs = 260000;
  canSend(msgB);

  FloodSlot* slotA = findSlot(makeMessageKey(msgA));
  FloodSlot* slotB = findSlot(makeMessageKey(msgB));
  ASSERT_TRUE(slotA != nullptr);
  ASSERT_TRUE(slotB != nullptr);
  ASSERT_EQ(slotA->flood_period_us, 500);
  ASSERT_EQ(slotB->flood_period_us, 600);

  simulatedNowUs = 230000;
  msgA.data[2] = 0x44;
  canSend(msgA);

  ASSERT_EQ(slotA->rx_count, 3);
  ASSERT_EQ(slotA->frame.data[2], 0x44);
  ASSERT_EQ(slotA->source_period_us, 80000);
  ASSERT_EQ(slotA->flood_period_us, 800);
  ASSERT_EQ(slotB->rx_count, 2);
  ASSERT_EQ(slotB->frame.data[2], 0x22);
  ASSERT_EQ(slotB->flood_period_us, 600);
}

void test_indefinite_flooding_continues_without_new_rx() {
  can_frame frame = makeFrame(1021, 0x00);

  simulatedNowUs = 100000;
  canSend(frame);
  simulatedNowUs = 150000;
  canSend(frame);

  ASSERT_TRUE(serviceFloodSlots(150500));
  ASSERT_TRUE(serviceFloodSlots(151000));
  ASSERT_TRUE(serviceFloodSlots(151500));
  ASSERT_EQ((int)busWrites.size(), 5);
}

void test_late_service_sends_once_without_burst() {
  can_frame frame = makeFrame(1021, 0x00);

  simulatedNowUs = 100000;
  canSend(frame);
  simulatedNowUs = 150000;
  canSend(frame);

  FloodSlot* slot = findSlot(makeMessageKey(frame));
  ASSERT_TRUE(slot != nullptr);
  ASSERT_EQ(slot->next_tx_us, 150500);

  ASSERT_TRUE(serviceFloodSlots(185000));
  ASSERT_EQ((int)busWrites.size(), 3);
  ASSERT_EQ(slot->next_tx_us, 185500);

  ASSERT_FALSE(serviceFloodSlots(185000));
  ASSERT_EQ((int)busWrites.size(), 3);
  ASSERT_TRUE(serviceFloodSlots(190000));
  ASSERT_EQ((int)busWrites.size(), 4);
}

void test_legacy_handler_registers_expected_targets() {
  LegacyHandler handler;

  simulatedNowUs = 100000;
  can_frame m0 = makeFrame(1006, 0x00, 0, 0, 62);
  handler.handelMessage(m0);

  simulatedNowUs = 200000;
  can_frame m1 = makeFrame(1006, 0x01);
  handler.handelMessage(m1);

  simulatedNowUs = 300000;
  can_frame cfg = makeFrame(2047, 0x02, 0, 0, 0, 0, 0xFF);
  handler.handelMessage(cfg);

  ASSERT_EQ(countActiveSlots(), 3);
  ASSERT_TRUE(findSlot(makeMessageKey(m0)) != nullptr);
  ASSERT_TRUE(findSlot(makeMessageKey(m1)) != nullptr);
  ASSERT_TRUE(findSlot(makeMessageKey(cfg)) != nullptr);
}

void test_hw3_handler_registers_expected_targets() {
  HW3Handler handler;

  simulatedNowUs = 100000;
  can_frame id1016 = makeFrame(1016, 0, 0, 0, 0, 0, 2u << 5);
  handler.handelMessage(id1016);

  simulatedNowUs = 200000;
  can_frame m0 = makeFrame(1021, 0x00, 0, 0, 60);
  handler.handelMessage(m0);

  simulatedNowUs = 300000;
  can_frame m1 = makeFrame(1021, 0x01, 0, 0, 0, 0, 0x08);
  handler.handelMessage(m1);

  simulatedNowUs = 400000;
  handler.speedOffset = 25;
  can_frame m2 = makeFrame(1021, 0x02, 0xFF);
  handler.handelMessage(m2);

  simulatedNowUs = 500000;
  can_frame cfg = makeFrame(2047, 0x02, 0, 0, 0, 0, 0xFF);
  handler.handelMessage(cfg);

  ASSERT_EQ(countActiveSlots(), 5);
  ASSERT_TRUE(findSlot(makeMessageKey(id1016)) != nullptr);
  ASSERT_TRUE(findSlot(makeMessageKey(m0)) != nullptr);
  FloodSlot* slotM1 = findSlot(makeMessageKey(m1));
  ASSERT_TRUE(slotM1 != nullptr);
  ASSERT_FALSE((slotM1->frame.data[5] >> 3) & 0x01);
  ASSERT_TRUE((slotM1->frame.data[5] >> 5) & 0x01);
  ASSERT_EQ(readApmv3Branch(slotM1->frame), UI_APMV3_BRANCH_DEV);
  ASSERT_EQ(readDrivingSide(findSlot(makeMessageKey(id1016))->frame), UI_DRIVING_SIDE_RIGHT);
  ASSERT_TRUE(findSlot(makeMessageKey(m2)) != nullptr);
  ASSERT_TRUE(findSlot(makeMessageKey(cfg)) != nullptr);
}

void test_hw4_handler_registers_expected_targets() {
  HW4Handler handler;

  simulatedNowUs = 100000;
  can_frame id1016 = makeFrame(1016, 0, 0, 0, 0, 0, 2u << 5);
  handler.handelMessage(id1016);

  simulatedNowUs = 200000;
  can_frame m0 = makeFrame(1021, 0x00);
  handler.handelMessage(m0);

  simulatedNowUs = 300000;
  can_frame m1 = makeFrame(1021, 0x01, 0, 0, 0, 0, 0x08);
  handler.handelMessage(m1);

  simulatedNowUs = 400000;
  handler.speedProfile = 4;
  can_frame m2 = makeFrame(1021, 0x02);
  handler.handelMessage(m2);

  simulatedNowUs = 500000;
  can_frame cfg = makeFrame(2047, 0x02, 0, 0, 0, 0, 0xFF);
  handler.handelMessage(cfg);

  ASSERT_EQ(countActiveSlots(), 5);
  ASSERT_TRUE(findSlot(makeMessageKey(id1016)) != nullptr);
  ASSERT_TRUE(findSlot(makeMessageKey(m0)) != nullptr);
  FloodSlot* slotM1 = findSlot(makeMessageKey(m1));
  ASSERT_TRUE(slotM1 != nullptr);
  ASSERT_FALSE((slotM1->frame.data[5] >> 3) & 0x01);
  ASSERT_TRUE((slotM1->frame.data[5] >> 5) & 0x01);
  ASSERT_EQ(readApmv3Branch(slotM1->frame), UI_APMV3_BRANCH_DEV);
  ASSERT_EQ(readDrivingSide(findSlot(makeMessageKey(id1016))->frame), UI_DRIVING_SIDE_RIGHT);
  ASSERT_TRUE(findSlot(makeMessageKey(m2)) != nullptr);
  ASSERT_TRUE(findSlot(makeMessageKey(cfg)) != nullptr);
}

int main() {
  printf("=== Flood key tests ===\n");
  RUN(test_makeMessageKey_distinguishes_mux_rules);

  printf("\n=== Flood scheduler tests ===\n");
  RUN(test_first_capture_only_arms_slot_without_flood);
  RUN(test_second_capture_learns_100x_rate_for_that_slot);
  RUN(test_each_message_key_tracks_its_own_rate_counter);
  RUN(test_new_source_frame_updates_only_matching_slot);
  RUN(test_indefinite_flooding_continues_without_new_rx);
  RUN(test_late_service_sends_once_without_burst);

  printf("\n=== Handler integration tests ===\n");
  RUN(test_legacy_handler_registers_expected_targets);
  RUN(test_hw3_handler_registers_expected_targets);
  RUN(test_hw4_handler_registers_expected_targets);

  printf("\n========================================\n");
  printf("Results: %d passed, %d failed\n", testsPassed, testsFailed);
  return testsFailed > 0 ? 1 : 0;
}
