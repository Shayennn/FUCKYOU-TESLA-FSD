/*
 * Desktop simulation test for CAN frame handler logic.
 * Compiles with any C++14 compiler — no Arduino dependencies.
 *
 *   cd test && make
 */

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <vector>

// ============================================================
// Types & stubs
// ============================================================

struct can_frame {
  uint32_t can_id;
  uint8_t  can_dlc;
  uint8_t  data[8];
};

static std::vector<can_frame> sentFrames;

constexpr uint32_t UI_CHASSIS_CONTROL_ID = 659;
constexpr int UI_DRIVER_SIDE_BIT = 40;
constexpr int UI_DRIVER_SIDE_LEN = 2;
constexpr int UI_APMV3_BRANCH_BIT = 40;
constexpr int UI_APMV3_BRANCH_LEN = 3;
constexpr int UI_AUTO_LANE_CHANGE_ENABLE_BIT = 24;
constexpr int UI_AUTO_LANE_CHANGE_ENABLE_LEN = 2;
constexpr uint8_t UI_DRIVER_SIDE_RIGHT = 1;
constexpr uint8_t UI_APMV3_BRANCH_DEV = 2;
constexpr uint8_t UI_AUTO_LANE_CHANGE_ENABLE_OFF = 0;

void canSend(can_frame& frame) {
  sentFrames.push_back(frame);
}

// ============================================================
// Pure logic (extracted from CanFeather.ino, no Arduino deps)
// ============================================================

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
  int bitIndex  = bit % 8;
  uint8_t mask  = static_cast<uint8_t>(1U << bitIndex);
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

inline uint8_t readDriverSide(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_DRIVER_SIDE_BIT, UI_DRIVER_SIDE_LEN));
}

inline uint8_t readApmv3Branch(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_APMV3_BRANCH_BIT, UI_APMV3_BRANCH_LEN));
}

inline void setField(can_frame& frame, int startBit, int length, uint32_t value) {
  for (int i = 0; i < length; ++i) {
    setBit(frame, startBit + i, (value >> i) & 0x01);
  }
}

inline void setDriverSide(can_frame& frame, uint8_t value) {
  setField(frame, UI_DRIVER_SIDE_BIT, UI_DRIVER_SIDE_LEN, value);
}

inline void setApmv3Branch(can_frame& frame, uint8_t value) {
  setField(frame, UI_APMV3_BRANCH_BIT, UI_APMV3_BRANCH_LEN, value);
}

inline uint8_t readAutoLaneChangeEnable(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_AUTO_LANE_CHANGE_ENABLE_BIT, UI_AUTO_LANE_CHANGE_ENABLE_LEN));
}

inline void setAutoLaneChangeEnable(can_frame& frame, uint8_t value) {
  setField(frame, UI_AUTO_LANE_CHANGE_ENABLE_BIT, UI_AUTO_LANE_CHANGE_ENABLE_LEN, value);
}

inline bool handleCommonUiChassisControl(can_frame& frame) {
  if (frame.can_id != UI_CHASSIS_CONTROL_ID) return false;
  setAutoLaneChangeEnable(frame, UI_AUTO_LANE_CHANGE_ENABLE_OFF);
  canSend(frame);
  return true;
}

struct CarManagerBase {
  int speedProfile = 1;
  virtual bool handelMessage(can_frame& frame) = 0;
  virtual ~CarManagerBase() = default;
};

// ---- LegacyHandler ----

struct LegacyHandler : public CarManagerBase {
  bool handelMessage(can_frame& frame) override {
    if (handleCommonUiChassisControl(frame)) return true;

    switch (frame.can_id) {
    case 1006: {
      switch (readMuxID(frame)) {
      case 0: {
        auto off = (uint8_t)((frame.data[3] >> 1) & 0x3F) - 30;
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

// ---- HW3Handler ----

struct HW3Handler : public CarManagerBase {
  int speedOffset = 0;
  bool handelMessage(can_frame& frame) override {
    if (handleCommonUiChassisControl(frame)) return true;

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
      setBit(frame, 13, true);
      setBit(frame, 48, true);
      setBit(frame, 49, true);
      setDriverSide(frame, UI_DRIVER_SIDE_RIGHT);
      canSend(frame);
      return true;
    }
    case 1021: {
      auto index = readMuxID(frame);
      switch (index) {
      case 0: {
        int rawOff = (uint8_t)((frame.data[3] >> 1) & 0x3F) - 30;
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

// ---- HW4Handler ----

struct HW4Handler : public CarManagerBase {
  bool handelMessage(can_frame& frame) override {
    if (handleCommonUiChassisControl(frame)) return true;

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
      setBit(frame, 13, true);
      setBit(frame, 48, true);
      setBit(frame, 49, true);
      setDriverSide(frame, UI_DRIVER_SIDE_RIGHT);
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

// ============================================================
// Test harness
// ============================================================

static int testsPassed = 0;
static int testsFailed = 0;

#define RUN(fn) do { \
  sentFrames.clear(); \
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
  if (_va != _vb) { \
    printf("FAIL\n    line %d: %s == %d, expected %s == %d\n", \
           __LINE__, #a, (int)_va, #b, (int)_vb); \
    testsFailed++; return; \
  } \
} while (0)

// ---- helpers ----

static bool getBit(const can_frame& f, int bit) {
  return (f.data[bit / 8] >> (bit % 8)) & 0x01;
}

static uint8_t getField(const can_frame& f, int startBit, int width) {
  return (f.data[startBit / 8] >> (startBit % 8)) & ((1 << width) - 1);
}

static can_frame makeFrame(uint32_t id,
    uint8_t d0=0, uint8_t d1=0, uint8_t d2=0, uint8_t d3=0,
    uint8_t d4=0, uint8_t d5=0, uint8_t d6=0, uint8_t d7=0) {
  can_frame f{};
  f.can_id = id;  f.can_dlc = 8;
  f.data[0]=d0; f.data[1]=d1; f.data[2]=d2; f.data[3]=d3;
  f.data[4]=d4; f.data[5]=d5; f.data[6]=d6; f.data[7]=d7;
  return f;
}

// ============================================================
// Utility tests
// ============================================================

void test_setBit_sets_and_preserves() {
  can_frame f = makeFrame(0);
  setBit(f, 5, true);
  ASSERT_TRUE(getBit(f, 5));
  ASSERT_FALSE(getBit(f, 4));
  ASSERT_FALSE(getBit(f, 6));
}

void test_setBit_clears_and_preserves() {
  can_frame f = makeFrame(0);
  f.data[0] = 0xFF;
  setBit(f, 3, false);
  ASSERT_FALSE(getBit(f, 3));
  ASSERT_TRUE(getBit(f, 2));
  ASSERT_TRUE(getBit(f, 4));
}

void test_setBit_cross_byte_boundaries() {
  can_frame f = makeFrame(0);
  setBit(f, 14, true);   // byte 1 bit 6
  ASSERT_EQ(f.data[1], 0x40);
  setBit(f, 38, true);   // byte 4 bit 6
  ASSERT_EQ(f.data[4], 0x40);
  setBit(f, 60, true);   // byte 7 bit 4
  ASSERT_EQ(f.data[7], 0x10);
}

void test_readMuxID_extracts_lower_3_bits() {
  can_frame f = makeFrame(0, 0x05);
  ASSERT_EQ(readMuxID(f), 5);
  f.data[0] = 0xFE;   // 0xFE & 0x07 = 6
  ASSERT_EQ(readMuxID(f), 6);
  f.data[0] = 0x08;   // 0x08 & 0x07 = 0
  ASSERT_EQ(readMuxID(f), 0);
}

void test_isFSDSelectedInUI_reads_bit38() {
  can_frame f = makeFrame(0, 0,0,0,0, 0x40);  // byte4=0x40 -> bit6=1
  ASSERT_TRUE(isFSDSelectedInUI(f));
  f.data[4] = 0xBF;   // everything except bit 6
  ASSERT_FALSE(isFSDSelectedInUI(f));
}

void test_setSpeedProfileV12V13_writes_bits_1_2() {
  can_frame f = makeFrame(0);
  f.data[6] = 0xFF;
  setSpeedProfileV12V13(f, 2);
  ASSERT_EQ(f.data[6] & 0x06, 0x04);
  ASSERT_EQ(f.data[6] & 0xF9, 0xF9);  // other bits untouched
  setSpeedProfileV12V13(f, 0);
  ASSERT_EQ(f.data[6] & 0x06, 0x00);
}

void test_readDriverSide_extracts_bits_40_41() {
  can_frame f = makeFrame(1016, 0, 0, 0, 0, 0, 0x02);
  ASSERT_EQ(readDriverSide(f), 2);
  f.data[5] = 0x01;
  ASSERT_EQ(readDriverSide(f), 1);
  f.data[5] = 0x00;
  ASSERT_EQ(readDriverSide(f), 0);
}

void test_readApmv3Branch_extracts_bits_40_42() {
  can_frame f = makeFrame(1021, 0x01, 0, 0, 0, 0, 0x05);
  ASSERT_EQ(readApmv3Branch(f), 5);
  f.data[5] = 0x03;
  ASSERT_EQ(readApmv3Branch(f), 3);
  f.data[5] = 0x00;
  ASSERT_EQ(readApmv3Branch(f), 0);
}

// ============================================================
// LegacyHandler tests
// ============================================================

void test_legacy_unhandled_id_returns_false() {
  LegacyHandler h;
  can_frame f = makeFrame(999);
  ASSERT_FALSE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 0);
}

void test_legacy_1006_mux0_sets_bit46_and_speed_profile() {
  LegacyHandler h;
  // off = ((data[3]>>1) & 0x3F) - 30;  want off=1 -> val=31, data[3]=62
  can_frame f = makeFrame(1006, 0x00, 0,0, 62);
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_TRUE(getBit(sentFrames[0], 46));
  ASSERT_EQ(h.speedProfile, 1);
  ASSERT_EQ(getField(sentFrames[0], 49, 2), 1);  // V12V13 profile in byte6 bits 1-2
}

void test_legacy_1006_mux1_clears_eceR79() {
  LegacyHandler h;
  can_frame f = makeFrame(1006, 0x01);
  f.data[2] = 0xFF;   // bit 19 = byte2 bit3, pre-set
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_FALSE(getBit(sentFrames[0], 19));
}

void test_legacy_2047_sets_autopilot_to_3() {
  LegacyHandler h;
  can_frame f = makeFrame(2047, 2, 0,0,0,0, 0xFF);
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_EQ(getField(sentFrames[0], 42, 3), 3);
}

void test_legacy_659_disables_auto_lane_change() {
  LegacyHandler h;
  can_frame f = makeFrame(659);
  f.data[3] = 0x03;
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_EQ(readAutoLaneChangeEnable(sentFrames[0]), UI_AUTO_LANE_CHANGE_ENABLE_OFF);
}

// ============================================================
// HW3Handler tests
// ============================================================

void test_hw3_unhandled_id_returns_false() {
  HW3Handler h;
  can_frame f = makeFrame(999);
  ASSERT_FALSE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 0);
}

void test_hw3_1016_enables_nav_on_maps_bits() {
  HW3Handler h;
  can_frame f = makeFrame(1016, 0,0,0,0,0, 2u << 5);  // followDist=2
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_TRUE(getBit(sentFrames[0], 5));    // UI_dasDeveloper
  ASSERT_TRUE(getBit(sentFrames[0], 14));   // UI_handsOnRequirementDisable
  ASSERT_TRUE(getBit(sentFrames[0], 13));   // UI_driveOnMapsEnable
  ASSERT_TRUE(getBit(sentFrames[0], 48));   // UI_hasDriveOnNav
  ASSERT_TRUE(getBit(sentFrames[0], 49));   // UI_followNavRouteEnable
  ASSERT_EQ(readDriverSide(sentFrames[0]), UI_DRIVER_SIDE_RIGHT);
  ASSERT_EQ(h.speedProfile, 1);            // followDist 2 -> profile 1
}

void test_hw3_1016_follow_distance_1() {
  HW3Handler h;
  can_frame f = makeFrame(1016, 0,0,0,0,0, 1u << 5);
  h.handelMessage(f);
  ASSERT_EQ(h.speedProfile, 2);
}

void test_hw3_1016_follow_distance_3() {
  HW3Handler h;
  can_frame f = makeFrame(1016, 0,0,0,0,0, 3u << 5);
  h.handelMessage(f);
  ASSERT_EQ(h.speedProfile, 0);
}

void test_hw3_1021_mux0_sets_fsdStops_and_bit46() {
  HW3Handler h;
  // data[3]=60 -> ((60>>1)&0x3F)-30 = 0 -> speedProfile unchanged (default 1)
  can_frame f = makeFrame(1021, 0x00, 0,0, 60);
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_TRUE(getBit(sentFrames[0], 38));   // UI_fsdStopsControlEnabled
  ASSERT_TRUE(getBit(sentFrames[0], 46));
}

void test_hw3_1021_mux0_preserves_speedProfile_from_1016() {
  HW3Handler h;
  can_frame f1016 = makeFrame(1016, 0,0,0,0,0, 1u << 5);  // followDist=1 -> profile=2
  h.handelMessage(f1016);
  ASSERT_EQ(h.speedProfile, 2);
  sentFrames.clear();
  // data[3]=60 -> off=0 which USED to overwrite speedProfile to 0
  can_frame f1021 = makeFrame(1021, 0x00, 0,0, 60);
  h.handelMessage(f1021);
  ASSERT_EQ(h.speedProfile, 2);   // must NOT be clobbered
}

void test_hw3_1021_mux0_runs_without_fsd_gate() {
  HW3Handler h;
  // bit 38 NOT set in input (fsdStops=0), handler should still process
  can_frame f = makeFrame(1021, 0x00, 0,0, 60, 0x00);
  ASSERT_FALSE(isFSDSelectedInUI(f));
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_TRUE(getBit(sentFrames[0], 38));   // forced to 1
}

void test_hw3_1021_mux1_clears_eceR79() {
  HW3Handler h;
  can_frame f = makeFrame(1021, 0x01);
  f.data[2] = 0xFF;
  f.data[5] = 0x08;
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_FALSE(getBit(sentFrames[0], 19));
  ASSERT_FALSE(getBit(sentFrames[0], 43));
  ASSERT_EQ(readApmv3Branch(sentFrames[0]), UI_APMV3_BRANCH_DEV);
}

void test_hw3_1021_mux2_writes_speed_offset() {
  HW3Handler h;
  h.speedOffset = 25;
  can_frame f = makeFrame(1021, 0x02, 0xFF);
  h.handelMessage(f);
  ASSERT_EQ((int)sentFrames.size(), 1);
  uint8_t lo = (sentFrames[0].data[0] >> 6) & 0x03;
  uint8_t hi = sentFrames[0].data[1] & 0x3F;
  ASSERT_EQ((int)(lo | (hi << 2)), 25);
}

void test_hw3_2047_sets_autopilot_to_3() {
  HW3Handler h;
  can_frame f = makeFrame(2047, 2, 0,0,0,0, 0xFF);
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ(getField(sentFrames[0], 42, 3), 3);
}

void test_hw3_659_disables_auto_lane_change() {
  HW3Handler h;
  can_frame f = makeFrame(659);
  f.data[3] = 0x03;
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_EQ(readAutoLaneChangeEnable(sentFrames[0]), UI_AUTO_LANE_CHANGE_ENABLE_OFF);
}

// ============================================================
// HW4Handler tests
// ============================================================

void test_hw4_unhandled_id_returns_false() {
  HW4Handler h;
  can_frame f = makeFrame(999);
  ASSERT_FALSE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 0);
}

void test_hw4_1016_enables_nav_on_maps_bits() {
  HW4Handler h;
  can_frame f = makeFrame(1016, 0,0,0,0,0, 2u << 5);
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_TRUE(getBit(sentFrames[0], 5));
  ASSERT_TRUE(getBit(sentFrames[0], 14));
  ASSERT_TRUE(getBit(sentFrames[0], 13));
  ASSERT_TRUE(getBit(sentFrames[0], 48));
  ASSERT_TRUE(getBit(sentFrames[0], 49));
  ASSERT_EQ(readDriverSide(sentFrames[0]), UI_DRIVER_SIDE_RIGHT);
  ASSERT_EQ(h.speedProfile, 2);   // HW4: fd=2 -> profile=2
}

void test_hw4_1016_follow_distance_mapping() {
  struct { int fd; int profile; } cases[] = {
    {1,3}, {2,2}, {3,1}, {4,0}, {5,4}
  };
  for (auto& c : cases) {
    sentFrames.clear();
    HW4Handler h;
    can_frame f = makeFrame(1016, 0,0,0,0,0, (uint8_t)(c.fd << 5));
    h.handelMessage(f);
    ASSERT_EQ(h.speedProfile, c.profile);
  }
}

void test_hw4_1021_mux0_sets_bits_38_46_60() {
  HW4Handler h;
  can_frame f = makeFrame(1021, 0x00);
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_TRUE(getBit(sentFrames[0], 38));
  ASSERT_TRUE(getBit(sentFrames[0], 46));
  ASSERT_TRUE(getBit(sentFrames[0], 60));
}

void test_hw4_1021_mux1_clears_bit19_sets_bit47() {
  HW4Handler h;
  can_frame f = makeFrame(1021, 0x01);
  f.data[2] = 0xFF;
  f.data[5] = 0x08;
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_FALSE(getBit(sentFrames[0], 19));
  ASSERT_FALSE(getBit(sentFrames[0], 43));
  ASSERT_TRUE(getBit(sentFrames[0], 47));
  ASSERT_EQ(readApmv3Branch(sentFrames[0]), UI_APMV3_BRANCH_DEV);
}

void test_hw4_1021_mux2_writes_speed_profile() {
  HW4Handler h;
  h.speedProfile = 5;
  can_frame f = makeFrame(1021, 0x02);
  h.handelMessage(f);
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_EQ((sentFrames[0].data[7] >> 4) & 0x07, 5);
}

void test_hw4_2047_sets_autopilot_to_4() {
  HW4Handler h;
  can_frame f = makeFrame(2047, 2, 0,0,0,0, 0xFF);
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ(getField(sentFrames[0], 42, 3), 4);
}

void test_hw4_659_disables_auto_lane_change() {
  HW4Handler h;
  can_frame f = makeFrame(659);
  f.data[3] = 0x03;
  ASSERT_TRUE(h.handelMessage(f));
  ASSERT_EQ((int)sentFrames.size(), 1);
  ASSERT_EQ(readAutoLaneChangeEnable(sentFrames[0]), UI_AUTO_LANE_CHANGE_ENABLE_OFF);
}

// ============================================================
// Main
// ============================================================

int main() {
  printf("=== Utility tests ===\n");
  RUN(test_setBit_sets_and_preserves);
  RUN(test_setBit_clears_and_preserves);
  RUN(test_setBit_cross_byte_boundaries);
  RUN(test_readMuxID_extracts_lower_3_bits);
  RUN(test_isFSDSelectedInUI_reads_bit38);
  RUN(test_setSpeedProfileV12V13_writes_bits_1_2);
  RUN(test_readDriverSide_extracts_bits_40_41);
  RUN(test_readApmv3Branch_extracts_bits_40_42);

  printf("\n=== LegacyHandler tests ===\n");
  RUN(test_legacy_unhandled_id_returns_false);
  RUN(test_legacy_1006_mux0_sets_bit46_and_speed_profile);
  RUN(test_legacy_1006_mux1_clears_eceR79);
  RUN(test_legacy_2047_sets_autopilot_to_3);
  RUN(test_legacy_659_disables_auto_lane_change);

  printf("\n=== HW3Handler tests ===\n");
  RUN(test_hw3_unhandled_id_returns_false);
  RUN(test_hw3_1016_enables_nav_on_maps_bits);
  RUN(test_hw3_1016_follow_distance_1);
  RUN(test_hw3_1016_follow_distance_3);
  RUN(test_hw3_1021_mux0_sets_fsdStops_and_bit46);
  RUN(test_hw3_1021_mux0_preserves_speedProfile_from_1016);
  RUN(test_hw3_1021_mux0_runs_without_fsd_gate);
  RUN(test_hw3_1021_mux1_clears_eceR79);
  RUN(test_hw3_1021_mux2_writes_speed_offset);
  RUN(test_hw3_2047_sets_autopilot_to_3);
  RUN(test_hw3_659_disables_auto_lane_change);

  printf("\n=== HW4Handler tests ===\n");
  RUN(test_hw4_unhandled_id_returns_false);
  RUN(test_hw4_1016_enables_nav_on_maps_bits);
  RUN(test_hw4_1016_follow_distance_mapping);
  RUN(test_hw4_1021_mux0_sets_bits_38_46_60);
  RUN(test_hw4_1021_mux1_clears_bit19_sets_bit47);
  RUN(test_hw4_1021_mux2_writes_speed_profile);
  RUN(test_hw4_2047_sets_autopilot_to_4);
  RUN(test_hw4_659_disables_auto_lane_change);

  printf("\n========================================\n");
  printf("Results: %d passed, %d failed\n", testsPassed, testsFailed);
  return testsFailed > 0 ? 1 : 0;
}
