#pragma once

/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <cstring>
#include <memory>
#include <CANSAME5x.h>
#include <Adafruit_NeoPixel.h>

struct can_frame {
  uint32_t can_id;
  uint8_t  can_dlc;
  uint8_t  data[8];
};

CANSAME5x CAN;
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

const uint32_t COLOR_RED    = Adafruit_NeoPixel::Color(255, 0, 0);
const uint32_t COLOR_GREEN  = Adafruit_NeoPixel::Color(0, 255, 0);
const uint32_t COLOR_BLUE   = Adafruit_NeoPixel::Color(0, 0, 255);
const uint32_t COLOR_YELLOW = Adafruit_NeoPixel::Color(255, 255, 0);

constexpr uint32_t UI_VEHICLE_CONTROL_ID = 627;
constexpr uint32_t UI_CHASSIS_CONTROL_ID = 659;
constexpr uint32_t UI_DRIVER_ASSIST_CONTROL_ID = 1016;
constexpr uint32_t UI_AUTOPILOT_CONTROL_ID = 1021;
constexpr int UI_DOME_LIGHT_SWITCH_BIT = 59;
constexpr int UI_DOME_LIGHT_SWITCH_LEN = 2;
constexpr int UI_DRIVER_SIDE_BIT = 40;
constexpr int UI_DRIVER_SIDE_LEN = 2;
constexpr int UI_APMV3_BRANCH_BIT = 40;
constexpr int UI_APMV3_BRANCH_LEN = 3;
constexpr int UI_AUTO_LANE_CHANGE_ENABLE_BIT = 24;
constexpr int UI_AUTO_LANE_CHANGE_ENABLE_LEN = 2;
constexpr uint8_t UI_APMV3_BRANCH_MUX = 1;
constexpr uint8_t UI_DRIVER_SIDE_LEFT = 0;
constexpr uint8_t UI_DRIVER_SIDE_RIGHT = 1;
constexpr uint8_t UI_DRIVER_SIDE_UNKNOWN = 2;
constexpr uint8_t UI_DRIVER_SIDE_OVERRIDE = UI_DRIVER_SIDE_RIGHT;
constexpr uint8_t UI_APMV3_BRANCH_LIVE = 0;
constexpr uint8_t UI_APMV3_BRANCH_STAGE = 1;
constexpr uint8_t UI_APMV3_BRANCH_DEV = 2;
constexpr uint8_t UI_APMV3_BRANCH_STAGE2 = 3;
constexpr uint8_t UI_APMV3_BRANCH_EAP = 4;
constexpr uint8_t UI_APMV3_BRANCH_DEMO = 5;
constexpr uint8_t UI_APMV3_BRANCH_OVERRIDE = UI_APMV3_BRANCH_DEV;
constexpr uint8_t UI_AUTO_LANE_CHANGE_ENABLE_OFF = 0;
constexpr uint8_t UI_AUTO_LANE_CHANGE_ENABLE_ON = 1;
constexpr uint8_t UI_AUTO_LANE_CHANGE_ENABLE_SNA = 3;
constexpr uint8_t UI_AUTO_LANE_CHANGE_ENABLE_OVERRIDE = UI_AUTO_LANE_CHANGE_ENABLE_OFF;
constexpr uint8_t UI_DOME_LIGHT_SWITCH_OFF = 0;
constexpr uint8_t UI_DOME_LIGHT_SWITCH_ON = 1;
constexpr uint8_t UI_DOME_LIGHT_SWITCH_AUTO = 2;
constexpr uint32_t STARTUP_SIGNAL_PHASE_MS = 1000;
constexpr uint32_t STARTUP_SIGNAL_RESEND_MS = 100;
constexpr uint8_t STARTUP_SIGNAL_PHASE_COUNT = 6; // on/off repeated three times

inline void setNeoColor(uint32_t color) {
  pixel.setPixelColor(0, color);
  pixel.show();
}

#define LEGACY LegacyHandler
#define HW3 HW3Handler
#define HW4 HW4Handler //HW4 since Version 2026.2.3 uses FSDV14, before that compile for HW3, even for HW4 vehicles.

#define HW HW3  //for what car to compile

#define ENABLE_PRINT

#define LED_PIN PIN_LED

constexpr size_t kFloodSlotCount = 8;
constexpr int kMaxRxFramesPerLoop = 16;
constexpr uint32_t kFloodRateMultiplier = 100;
constexpr uint32_t kFloodLogIntervalUs = 1000000;

inline uint8_t sanitizeDlc(int dlc) {
  if (dlc < 0) return 0;
  if (dlc > 8) return 8;
  return static_cast<uint8_t>(dlc);
}

int canRead(can_frame& frame) {
  int packetSize = CAN.parsePacket();
  if (packetSize <= 0 || CAN.packetRtr()) return -1;

  // Use the decoded RX metadata from Adafruit_CAN so we keep the original DLC
  // even if the library returns a shorter payload length for special cases.
  frame.can_id = static_cast<uint32_t>(CAN.packetId());
  frame.can_dlc = sanitizeDlc(CAN.packetDlc() > 0 ? CAN.packetDlc() : packetSize);
  memset(frame.data, 0, sizeof(frame.data));

  const int bytesToRead = std::min(packetSize, static_cast<int>(sizeof(frame.data)));
  for (int i = 0; i < bytesToRead; i++) {
    frame.data[i] = CAN.read();
  }

  while (CAN.available()) {
    CAN.read();
  }

  return 0;
}

bool canSendImmediate(const can_frame& frame) {
  const uint8_t dlc = sanitizeDlc(frame.can_dlc);
  // The SAME5x library buffers a single packet internally. Start, write and end
  // explicitly so we can log where TX fails instead of silently dropping frames.
  const bool began = frame.can_id > 0x7FF ? CAN.beginExtendedPacket(frame.can_id)
                                          : CAN.beginPacket(frame.can_id);
  if (!began) {
#ifdef ENABLE_PRINT
    Serial.printf("CAN TX begin failed for id=0x%03lX\n", static_cast<unsigned long>(frame.can_id));
#endif
    return false;
  }

  if (CAN.write(frame.data, dlc) != dlc) {
#ifdef ENABLE_PRINT
    Serial.printf("CAN TX write failed for id=0x%03lX len=%u\n",
                  static_cast<unsigned long>(frame.can_id), dlc);
#endif
    return false;
  }

  if (!CAN.endPacket()) {
#ifdef ENABLE_PRINT
    Serial.printf("CAN TX end failed for id=0x%03lX\n", static_cast<unsigned long>(frame.can_id));
#endif
    return false;
  }

  return true;
}

inline uint8_t readMuxID(const can_frame& frame) {
  return frame.data[0] & 0x07;
}

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

FloodSlot floodSlots[kFloodSlotCount];

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
  case 659:
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

#ifdef ENABLE_PRINT
inline uint32_t periodToHzX100(uint32_t period_us) {
  return period_us == 0 ? 0 : 100000000UL / period_us;
}

void logFloodLearned(const FloodSlot& slot) {
  const uint32_t source_hz_x100 = periodToHzX100(slot.source_period_us);
  const uint32_t flood_hz_x100 = periodToHzX100(slot.flood_period_us);
  Serial.printf(
      "Flood learned id=0x%03lX sel=%u rx=%lu src=%luus (%lu.%02luHz) flood=%luus (%lu.%02luHz)\n",
      static_cast<unsigned long>(slot.key.can_id),
      static_cast<unsigned>(slot.key.selector),
      static_cast<unsigned long>(slot.rx_count),
      static_cast<unsigned long>(slot.source_period_us),
      static_cast<unsigned long>(source_hz_x100 / 100),
      static_cast<unsigned long>(source_hz_x100 % 100),
      static_cast<unsigned long>(slot.flood_period_us),
      static_cast<unsigned long>(flood_hz_x100 / 100),
      static_cast<unsigned long>(flood_hz_x100 % 100));
}

void maybeLogFloodTx(FloodSlot& slot, uint32_t now_us) {
  if (slot.flood_tx_count != 1 &&
      static_cast<uint32_t>(now_us - slot.last_flood_log_us) < kFloodLogIntervalUs) {
    return;
  }

  slot.last_flood_log_us = now_us;
  Serial.printf("Flood TX id=0x%03lX sel=%u count=%lu period=%luus\n",
                static_cast<unsigned long>(slot.key.can_id),
                static_cast<unsigned>(slot.key.selector),
                static_cast<unsigned long>(slot.flood_tx_count),
                static_cast<unsigned long>(slot.flood_period_us));
}
#endif

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
#ifdef ENABLE_PRINT
    Serial.println("Flood slot overflow");
#endif
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
#ifdef ENABLE_PRINT
  if (slot->rx_count == 2) {
    logFloodLearned(*slot);
  }
#endif
}

bool canSend(const can_frame& frame) {
  const uint32_t now_us = micros();
  if (!canSendImmediate(frame)) {
    return false;
  }
  updateFloodSlot(frame, now_us);
  return true;
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

    if (!canSendImmediate(slot.frame)) {
      continue;
    }
    sentAny = true;
    slot.flood_tx_count += 1;
#ifdef ENABLE_PRINT
    maybeLogFloodTx(slot, now_us);
#endif

    slot.next_tx_us += slot.flood_period_us;
    while (isTimeDue(now_us, slot.next_tx_us)) {
      slot.next_tx_us += slot.flood_period_us;
    }
  }

  return sentAny;
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

inline uint8_t readDomeLightSwitch(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_DOME_LIGHT_SWITCH_BIT, UI_DOME_LIGHT_SWITCH_LEN));
}

inline void setDomeLightSwitch(can_frame& frame, uint8_t value) {
  setField(frame, UI_DOME_LIGHT_SWITCH_BIT, UI_DOME_LIGHT_SWITCH_LEN, value);
}

inline uint8_t readDriverSide(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_DRIVER_SIDE_BIT, UI_DRIVER_SIDE_LEN));
}

inline uint8_t readApmv3Branch(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_APMV3_BRANCH_BIT, UI_APMV3_BRANCH_LEN));
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

const char* driverSideName(uint8_t value) {
  switch (value) {
  case UI_DRIVER_SIDE_LEFT: return "LEFT";
  case UI_DRIVER_SIDE_RIGHT: return "RIGHT";
  case UI_DRIVER_SIDE_UNKNOWN: return "UNKNOWN";
  default: return "INVALID";
  }
}

const char* apmv3BranchName(uint8_t value) {
  switch (value) {
  case UI_APMV3_BRANCH_LIVE: return "LIVE";
  case UI_APMV3_BRANCH_STAGE: return "STAGE";
  case UI_APMV3_BRANCH_DEV: return "DEV";
  case UI_APMV3_BRANCH_STAGE2: return "STAGE2";
  case UI_APMV3_BRANCH_EAP: return "EAP";
  case UI_APMV3_BRANCH_DEMO: return "DEMO";
  default: return "INVALID";
  }
}

const char* autoLaneChangeEnableName(uint8_t value) {
  switch (value) {
  case UI_AUTO_LANE_CHANGE_ENABLE_OFF: return "OFF";
  case UI_AUTO_LANE_CHANGE_ENABLE_ON: return "ON";
  case UI_AUTO_LANE_CHANGE_ENABLE_SNA: return "SNA";
  default: return "INVALID";
  }
}

bool handleCommonUiChassisControl(can_frame& frame) {
  if (frame.can_id != UI_CHASSIS_CONTROL_ID) return false;

  uint8_t rxAutoLaneChangeEnable = readAutoLaneChangeEnable(frame);
  setAutoLaneChangeEnable(frame, UI_AUTO_LANE_CHANGE_ENABLE_OVERRIDE);
  canSend(frame);
#ifdef ENABLE_PRINT
  Serial.printf("ID659: autoLaneChange=%u (%s)->%u (%s)\n",
                rxAutoLaneChangeEnable, autoLaneChangeEnableName(rxAutoLaneChangeEnable),
                UI_AUTO_LANE_CHANGE_ENABLE_OVERRIDE,
                autoLaneChangeEnableName(UI_AUTO_LANE_CHANGE_ENABLE_OVERRIDE));
#endif
  return true;
}

struct StartupSignal {
  bool active = true;
  bool hasTemplate = false;
  bool originalCaptured = false;
  can_frame templateFrame{};
  uint8_t originalDomeSetting = UI_DOME_LIGHT_SWITCH_AUTO;
  uint8_t phase = 0;
  uint32_t phaseStartedAtMs = 0;
  uint32_t lastTxAtMs = 0;

  void observe(const can_frame& frame) {
    if (frame.can_id != UI_VEHICLE_CONTROL_ID) return;

    // Mirror the live vehicle-control frame so the startup blink only changes
    // the dome-light bits and preserves every other UI-controlled signal.
    templateFrame = frame;
    hasTemplate = true;

    if (!originalCaptured) {
      originalDomeSetting = readDomeLightSwitch(frame);
      originalCaptured = true;
      phaseStartedAtMs = millis();
      lastTxAtMs = 0;
#ifdef ENABLE_PRINT
      Serial.printf("Startup signal armed, original dome=%u\n", originalDomeSetting);
#endif
    }
  }

  bool service() {
    if (!active || !hasTemplate || !originalCaptured) return false;

    const uint32_t now = millis();
    while (phase < STARTUP_SIGNAL_PHASE_COUNT &&
           static_cast<uint32_t>(now - phaseStartedAtMs) >= STARTUP_SIGNAL_PHASE_MS) {
      phase++;
      phaseStartedAtMs += STARTUP_SIGNAL_PHASE_MS;
      lastTxAtMs = 0;
    }

    if (phase >= STARTUP_SIGNAL_PHASE_COUNT) {
      can_frame restoreFrame = templateFrame;
      setDomeLightSwitch(restoreFrame, originalDomeSetting);
      canSendImmediate(restoreFrame);
      active = false;
#ifdef ENABLE_PRINT
      Serial.println("Startup signal complete");
#endif
      return true;
    }

    if (lastTxAtMs != 0 &&
        static_cast<uint32_t>(now - lastTxAtMs) < STARTUP_SIGNAL_RESEND_MS) {
      return false;
    }

    can_frame frame = templateFrame;
    // Alternate ON/OFF once per second so the cabin light becomes a visible
    // startup heartbeat without blocking the main message handling loop.
    setDomeLightSwitch(frame, (phase % 2 == 0) ? UI_DOME_LIGHT_SWITCH_ON
                                               : UI_DOME_LIGHT_SWITCH_OFF);
    if (canSendImmediate(frame)) {
      lastTxAtMs = now;
      return true;
    }

    return false;
  }
};

struct CarManagerBase {
  int speedProfile = 1;
  virtual bool handelMessage(can_frame& frame) = 0;
  virtual ~CarManagerBase() = default;
};

struct LegacyHandler : public CarManagerBase {
  bool handelMessage(can_frame& frame) override {
    if (handleCommonUiChassisControl(frame)) return true;

    switch (frame.can_id) {
    case 1006: {
      switch (readMuxID(frame)) {
      case 0: {
        auto off = static_cast<uint8_t>((frame.data[3] >> 1) & 0x3F) - 30;
        switch (off) {
        case 2: speedProfile = 2; break;
        case 1: speedProfile = 1; break;
        case 0: speedProfile = 0; break;
        default: break;
        }
        setBit(frame, 46, true);   // UI_showTrackLabels (m1 bit, enables FSD viz in mux 0)
        setSpeedProfileV12V13(frame, speedProfile);
        canSend(frame);
#ifdef ENABLE_PRINT
        Serial.printf("LegacyHandler: Profile: %d\n", speedProfile);
#endif
        break;
      }
      case 1:
        setBit(frame, 19, false);  // UI_applyEceR79: disable ECE R79 steering limit
        canSend(frame);
        break;
      }
      return true;
    }
    case 2047:
      if (frame.data[0] != 2) return false;
      {
        uint8_t rxAutopilot = (frame.data[5] >> 2) & 0x07;
        frame.data[5] &= ~(0x07 << 2);
        frame.data[5] |= (3 & 0x07) << 2;
        canSend(frame);
#ifdef ENABLE_PRINT
        Serial.printf("ID2047m2: autopilot=%d->3\n", rxAutopilot);
#endif
      }
      return true;
    default:
      return false;
    }
  }
};

struct HW3Handler : public CarManagerBase {
  int speedOffset = 0;

  bool handelMessage(can_frame& frame) override {
    if (handleCommonUiChassisControl(frame)) return true;

    switch (frame.can_id) {
    case 1016: {
      bool rxDasDev = (frame.data[0] >> 5) & 0x01;
      bool rxHandsOff = (frame.data[1] >> 6) & 0x01;
      bool rxDriveOnMaps = (frame.data[1] >> 5) & 0x01;
      bool rxHasDriveOnNav = frame.data[6] & 0x01;
      bool rxFollowNavRoute = (frame.data[6] >> 1) & 0x01;
      uint8_t rxDriverSide = readDriverSide(frame);
      uint8_t followDistance = (frame.data[5] & 0b11100000) >> 5;
      switch (followDistance) {
      case 1: speedProfile = 2; break;
      case 2: speedProfile = 1; break;
      case 3: speedProfile = 0; break;
      default: break;
      }
      setBit(frame, 5, true);   // UI_dasDeveloper: enable developer mode
      setBit(frame, 14, true);  // UI_handsOnRequirementDisable: suppress hands-on nag
      setBit(frame, 13, true);  // UI_driveOnMapsEnable: enable navigation on maps
      setBit(frame, 48, true);  // UI_hasDriveOnNav: advertise nav-on-autopilot availability
      setBit(frame, 49, true);  // UI_followNavRouteEnable: follow active navigation route
      setDriverSide(frame, UI_DRIVER_SIDE_OVERRIDE);
      canSend(frame);
#ifdef ENABLE_PRINT
      Serial.printf("ID1016: driverSide=%u (%s)->%u (%s) dasDev=%d->1 handsOffDisable=%d->1 driveOnMaps=%d->1 hasDriveOnNav=%d->1 followNavRoute=%d->1 followDist=%d\n",
                    rxDriverSide, driverSideName(rxDriverSide),
                    UI_DRIVER_SIDE_OVERRIDE, driverSideName(UI_DRIVER_SIDE_OVERRIDE),
                    rxDasDev, rxHandsOff, rxDriveOnMaps, rxHasDriveOnNav, rxFollowNavRoute, followDistance);
#endif
      return true;
    }
    case 1021: {
      auto index = readMuxID(frame);
      bool rxFsdStops = isFSDSelectedInUI(frame);
      switch (index) {
      case 0: {
        int rawOff = static_cast<uint8_t>((frame.data[3] >> 1) & 0x3F) - 30;
        speedOffset = std::max(std::min(rawOff * 5, 100), 0);
        setBit(frame, 38, true);  // UI_fsdStopsControlEnabled: enable FSD stop control
        setBit(frame, 46, true);  // UI_showTrackLabels (m1 bit, enables FSD viz in mux 0)
        setSpeedProfileV12V13(frame, speedProfile);
        canSend(frame);
#ifdef ENABLE_PRINT
        Serial.printf("HW3Handler: fsdStops=%d->1 Profile: %d, Offset: %d (raw=%d)\n", rxFsdStops, speedProfile, speedOffset, rawOff);
#endif
        break;
      }
      case 1: {
        uint8_t rxApmv3Branch = readApmv3Branch(frame);
        // UI_applyEceR79 (bit 19): disable ECE R79 steering torque limit
        setApmv3Branch(frame, UI_APMV3_BRANCH_OVERRIDE);
        setBit(frame, 19, false);
        setBit(frame, 43, false);  // UI_enableCabinCamera: keep cabin camera disabled
        canSend(frame);
#ifdef ENABLE_PRINT
        Serial.printf("ID1021 m1: apmv3Branch=%u (%s)->%u (%s)\n",
                      rxApmv3Branch, apmv3BranchName(rxApmv3Branch),
                      UI_APMV3_BRANCH_OVERRIDE, apmv3BranchName(UI_APMV3_BRANCH_OVERRIDE));
#endif
        break;
      }
      case 2:
        // AUTOPILOT_CONTROL_2: write speedOffset (0-100%) into undocumented field.
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
      {
        uint8_t rxAutopilot = (frame.data[5] >> 2) & 0x07;
        frame.data[5] &= ~(0x07 << 2);
        frame.data[5] |= (3 & 0x07) << 2;
        canSend(frame);
#ifdef ENABLE_PRINT
        Serial.printf("ID2047m2: autopilot=%d->3\n", rxAutopilot);
#endif
      }
      return true;
    default:
      return false;
    }
  }
};

struct HW4Handler : public CarManagerBase {
  bool handelMessage(can_frame& frame) override {
    if (handleCommonUiChassisControl(frame)) return true;

    switch (frame.can_id) {
    case 1016: {
      bool rxDasDev = (frame.data[0] >> 5) & 0x01;
      bool rxHandsOff = (frame.data[1] >> 6) & 0x01;
      bool rxDriveOnMaps = (frame.data[1] >> 5) & 0x01;
      bool rxHasDriveOnNav = frame.data[6] & 0x01;
      bool rxFollowNavRoute = (frame.data[6] >> 1) & 0x01;
      uint8_t rxDriverSide = readDriverSide(frame);
      auto fd = (frame.data[5] & 0b11100000) >> 5;
      switch (fd) {
      case 1: speedProfile = 3; break;
      case 2: speedProfile = 2; break;
      case 3: speedProfile = 1; break;
      case 4: speedProfile = 0; break;
      case 5: speedProfile = 4; break;
      }
      setBit(frame, 5, true);   // UI_dasDeveloper: enable developer mode
      setBit(frame, 14, true);  // UI_handsOnRequirementDisable: suppress hands-on nag
      setBit(frame, 13, true);  // UI_driveOnMapsEnable: enable navigation on maps
      setBit(frame, 48, true);  // UI_hasDriveOnNav: advertise nav-on-autopilot availability
      setBit(frame, 49, true);  // UI_followNavRouteEnable: follow active navigation route
      setDriverSide(frame, UI_DRIVER_SIDE_OVERRIDE);
      canSend(frame);
#ifdef ENABLE_PRINT
      Serial.printf("ID1016: driverSide=%u (%s)->%u (%s) dasDev=%d->1 handsOffDisable=%d->1 driveOnMaps=%d->1 hasDriveOnNav=%d->1 followNavRoute=%d->1 followDist=%d\n",
                    rxDriverSide, driverSideName(rxDriverSide),
                    UI_DRIVER_SIDE_OVERRIDE, driverSideName(UI_DRIVER_SIDE_OVERRIDE),
                    rxDasDev, rxHandsOff, rxDriveOnMaps, rxHasDriveOnNav, rxFollowNavRoute, fd);
#endif
      return true;
    }
    case 1021: {
      auto index = readMuxID(frame);
      bool rxFsdStops = isFSDSelectedInUI(frame);
      switch (index) {
      case 0:
        setBit(frame, 38, true);  // UI_fsdStopsControlEnabled: enable FSD stop control
        setBit(frame, 46, true);  // UI_showTrackLabels (m1 bit, enables FSD viz in mux 0)
        setBit(frame, 60, true);  // UI_enableVisionOnlyStops
        canSend(frame);
#ifdef ENABLE_PRINT
        Serial.printf("HW4Handler: fsdStops=%d->1 profile: %d\n", rxFsdStops, speedProfile);
#endif
        break;
      case 1: {
        uint8_t rxApmv3Branch = readApmv3Branch(frame);
        setApmv3Branch(frame, UI_APMV3_BRANCH_OVERRIDE);
        setBit(frame, 19, false);  // UI_applyEceR79: disable ECE R79 steering limit
        setBit(frame, 43, false);  // UI_enableCabinCamera: keep cabin camera disabled
        setBit(frame, 47, true);   // UI_hardCoreSummon: enable hardcore summon mode
        canSend(frame);
#ifdef ENABLE_PRINT
        Serial.printf("ID1021 m1: apmv3Branch=%u (%s)->%u (%s)\n",
                      rxApmv3Branch, apmv3BranchName(rxApmv3Branch),
                      UI_APMV3_BRANCH_OVERRIDE, apmv3BranchName(UI_APMV3_BRANCH_OVERRIDE));
#endif
        break;
      }
      case 2:
        // AUTOPILOT_CONTROL_2: write speedProfile into undocumented field at byte 7 bits 4-6.
        frame.data[7] &= ~(0x07 << 4);
        frame.data[7] |= (speedProfile & 0x07) << 4;
        canSend(frame);
        break;
      }
      return true;
    }
    case 2047:
      if (frame.data[0] != 2) return false;
      {
        uint8_t rxAutopilot = (frame.data[5] >> 2) & 0x07;
        frame.data[5] &= ~(0x07 << 2);
        frame.data[5] |= (4 & 0x07) << 2;
        canSend(frame);
#ifdef ENABLE_PRINT
        Serial.printf("ID2047m2: autopilot=%d->4\n", rxAutopilot);
#endif
      }
      return true;
    default:
      return false;
    }
  }
};

std::unique_ptr<CarManagerBase> handler;
StartupSignal startupSignal;

void setup() {
  pinMode(PIN_NEOPIXEL_POWER, OUTPUT);
  digitalWrite(PIN_NEOPIXEL_POWER, HIGH);
  pixel.begin();
  pixel.setBrightness(30);
  setNeoColor(COLOR_BLUE);

  handler.reset(new HW());
  for (auto& slot : floodSlots) {
    resetFloodSlot(slot);
  }
  delay(1500);

#ifdef ENABLE_PRINT
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1000) {}
#endif

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false);
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true);

  if (!CAN.begin(500E3)) {
#ifdef ENABLE_PRINT
    Serial.println("CAN init failed");
#endif
    setNeoColor(COLOR_RED);
    while (true) { delay(1000); }
  }
#ifdef ENABLE_PRINT
  Serial.println("CAN ready @ 500k");
#endif
}

__attribute__((optimize("O3"))) void loop() {
  bool sawFrame = false;
  bool modified = false;
  bool flooded = false;
  bool startupTx = startupSignal.service();

  can_frame frame;
  for (int i = 0; i < kMaxRxFramesPerLoop; i++) {
    if (canRead(frame) != 0) {
      break;
    }
    sawFrame = true;
    startupSignal.observe(frame);
    startupTx = startupSignal.service() || startupTx;
    modified = handler->handelMessage(frame) || modified;
    flooded = serviceFloodSlots(micros()) || flooded;
  }

  flooded = serviceFloodSlots(micros()) || flooded;

  if (modified || flooded || startupTx) {
    setNeoColor(COLOR_GREEN);
  } else if (sawFrame) {
    setNeoColor(COLOR_YELLOW);
  } else {
    setNeoColor(COLOR_BLUE);
  }
}
