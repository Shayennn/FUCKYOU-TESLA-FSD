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
#include <driver/twai.h>

struct can_frame {
  uint32_t can_id;
  uint8_t  can_dlc;
  uint8_t  data[8];
};

#define LED_PIN    2
#define CAN_RX_PIN GPIO_NUM_27
#define CAN_TX_PIN GPIO_NUM_26
#define UI_DRIVER_ASSIST_CONTROL_ID 1016
#define UI_AUTOPILOT_CONTROL_ID 1021
#define UI_DRIVING_SIDE_BIT 40
#define UI_DRIVING_SIDE_LEN 2
#define UI_APMV3_BRANCH_BIT 40
#define UI_APMV3_BRANCH_LEN 3
#define UI_APMV3_BRANCH_MUX 1
#define UI_DRIVING_SIDE_LEFT 0
#define UI_DRIVING_SIDE_RIGHT 1
#define UI_DRIVING_SIDE_UNKNOWN 2
#define UI_DRIVING_SIDE_OVERRIDE UI_DRIVING_SIDE_LEFT
#define UI_APMV3_BRANCH_LIVE 0
#define UI_APMV3_BRANCH_STAGE 1
#define UI_APMV3_BRANCH_DEV 2
#define UI_APMV3_BRANCH_STAGE2 3
#define UI_APMV3_BRANCH_EAP 4
#define UI_APMV3_BRANCH_DEMO 5
#define UI_APMV3_BRANCH_OVERRIDE UI_APMV3_BRANCH_DEV

bool canInit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}

int canRead(can_frame& frame) {
  twai_message_t msg;
  if (twai_receive(&msg, 0) != ESP_OK) return -1;
  frame.can_id  = msg.identifier;
  frame.can_dlc = msg.data_length_code;
  memcpy(frame.data, msg.data, 8);
  return 0;
}

void canSend(can_frame& frame) {
  twai_message_t msg = {};
  msg.identifier       = frame.can_id;
  msg.data_length_code = frame.can_dlc;
  memcpy(msg.data, frame.data, 8);
  twai_transmit(&msg, pdMS_TO_TICKS(100));
}


#define LEGACY LegacyHandler
#define HW3 HW3Handler
#define HW4 HW4Handler //HW4 since Version 2026.2.3 uses FSDV14, before that compile for HW3, even for HW4 vehicles.


#define HW HW3  //for what car to compile

bool enablePrint = true;

struct CarManagerBase {
  int speedProfile = 1;
  bool FSDEnabled = false;
  virtual void handelMessage(can_frame& frame);
};

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

inline uint8_t readDrivingSide(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_DRIVING_SIDE_BIT, UI_DRIVING_SIDE_LEN));
}

inline uint8_t readApmv3Branch(const can_frame& frame) {
  return static_cast<uint8_t>(readField(frame, UI_APMV3_BRANCH_BIT, UI_APMV3_BRANCH_LEN));
}

inline void setField(can_frame& frame, int startBit, int length, uint32_t value) {
  for (int i = 0; i < length; ++i) {
    setBit(frame, startBit + i, (value >> i) & 0x01);
  }
}

inline void setDrivingSide(can_frame& frame, uint8_t value) {
  setField(frame, UI_DRIVING_SIDE_BIT, UI_DRIVING_SIDE_LEN, value);
}

inline void setApmv3Branch(can_frame& frame, uint8_t value) {
  setField(frame, UI_APMV3_BRANCH_BIT, UI_APMV3_BRANCH_LEN, value);
}

const char* drivingSideName(uint8_t value) {
  switch (value) {
    case UI_DRIVING_SIDE_LEFT: return "LEFT";
    case UI_DRIVING_SIDE_RIGHT: return "RIGHT";
    case UI_DRIVING_SIDE_UNKNOWN: return "UNKNOWN";
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

struct LegacyHandler : public CarManagerBase {
  virtual void handelMessage(can_frame& frame) override {
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
        if (enablePrint) {
          Serial.printf("LegacyHandler: Profile: %d\n", speedProfile);
        }
        break;
      }
      case 1:
        setBit(frame, 19, false);
        canSend(frame);
        break;
      }
      return;
    }
    case 2047:
      if (frame.data[0] != 2) return;
      {
        uint8_t rxAutopilot = (frame.data[5] >> 2) & 0x07;
        frame.data[5] &= ~(0x07 << 2);
        frame.data[5] |= (3 & 0x07) << 2;
        canSend(frame);
        if (enablePrint) {
          Serial.printf("ID2047m2: autopilot=%d->3\n", rxAutopilot);
        }
      }
      return;
    }
  }
};

struct HW3Handler : public CarManagerBase {
  int speedOffset = 0;
  virtual void handelMessage(can_frame& frame) override {
    switch (frame.can_id) {
    case 1016: {
      bool rxDasDev = (frame.data[0] >> 5) & 0x01;
      bool rxHandsOff = (frame.data[1] >> 6) & 0x01;
      bool rxDriveOnMaps = (frame.data[1] >> 5) & 0x01;
      bool rxHasDriveOnNav = frame.data[6] & 0x01;
      bool rxFollowNavRoute = (frame.data[6] >> 1) & 0x01;
      uint8_t rxDrivingSide = readDrivingSide(frame);
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
      setDrivingSide(frame, UI_DRIVING_SIDE_OVERRIDE);
      canSend(frame);
      if (enablePrint) {
        Serial.printf("ID1016: drivingSide=%u (%s)->%u (%s) dasDev=%d->1 handsOffDisable=%d->1 driveOnMaps=%d->1 hasDriveOnNav=%d->1 followNavRoute=%d->1 followDist=%d\n",
                      rxDrivingSide, drivingSideName(rxDrivingSide),
                      UI_DRIVING_SIDE_OVERRIDE, drivingSideName(UI_DRIVING_SIDE_OVERRIDE),
                      rxDasDev, rxHandsOff, rxDriveOnMaps, rxHasDriveOnNav, rxFollowNavRoute, followDistance);
      }
      return;
    }
    case 1021: {
      auto index = readMuxID(frame);
      bool rxFsdStops = isFSDSelectedInUI(frame);
      switch (index) {
      case 0: {
        int rawOff = (uint8_t)((frame.data[3] >> 1) & 0x3F) - 30;
        speedOffset = std::max(std::min(rawOff * 5, 100), 0);
        setBit(frame, 38, true);
        setBit(frame, 46, true);
        setSpeedProfileV12V13(frame, speedProfile);
        canSend(frame);
        if (enablePrint) {
          Serial.printf("HW3Handler: fsdStops=%d->1 Profile: %d, Offset: %d (raw=%d)\n", rxFsdStops, speedProfile, speedOffset, rawOff);
        }
        break;
      }
      case 1: {
        uint8_t rxApmv3Branch = readApmv3Branch(frame);
        setApmv3Branch(frame, UI_APMV3_BRANCH_OVERRIDE);
        setBit(frame, 19, false);
        setBit(frame, 45, true);
        canSend(frame);
        if (enablePrint) {
          Serial.printf("ID1021 m1: apmv3Branch=%u (%s)->%u (%s)\n",
                        rxApmv3Branch, apmv3BranchName(rxApmv3Branch),
                        UI_APMV3_BRANCH_OVERRIDE, apmv3BranchName(UI_APMV3_BRANCH_OVERRIDE));
        }
        break;
      }
      case 2:
        frame.data[0] &= ~(0b11000000);
        frame.data[1] &= ~(0b00111111);
        frame.data[0] |= (speedOffset & 0x03) << 6;
        frame.data[1] |= (speedOffset >> 2);
        canSend(frame);
        break;
      }
      return;
    }
    case 2047:
      if (frame.data[0] != 2) return;
      {
        uint8_t rxAutopilot = (frame.data[5] >> 2) & 0x07;
        frame.data[5] &= ~(0x07 << 2);
        frame.data[5] |= (3 & 0x07) << 2;
        canSend(frame);
        if (enablePrint) {
          Serial.printf("ID2047m2: autopilot=%d->3\n", rxAutopilot);
        }
      }
      return;
    }
  }
};

struct HW4Handler : public CarManagerBase {
  virtual void handelMessage(can_frame& frame) override {
    switch (frame.can_id) {
    case 1016: {
      bool rxDasDev = (frame.data[0] >> 5) & 0x01;
      bool rxHandsOff = (frame.data[1] >> 6) & 0x01;
      bool rxDriveOnMaps = (frame.data[1] >> 5) & 0x01;
      bool rxHasDriveOnNav = frame.data[6] & 0x01;
      bool rxFollowNavRoute = (frame.data[6] >> 1) & 0x01;
      uint8_t rxDrivingSide = readDrivingSide(frame);
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
      setDrivingSide(frame, UI_DRIVING_SIDE_OVERRIDE);
      canSend(frame);
      if (enablePrint) {
        Serial.printf("ID1016: drivingSide=%u (%s)->%u (%s) dasDev=%d->1 handsOffDisable=%d->1 driveOnMaps=%d->1 hasDriveOnNav=%d->1 followNavRoute=%d->1 followDist=%d\n",
                      rxDrivingSide, drivingSideName(rxDrivingSide),
                      UI_DRIVING_SIDE_OVERRIDE, drivingSideName(UI_DRIVING_SIDE_OVERRIDE),
                      rxDasDev, rxHandsOff, rxDriveOnMaps, rxHasDriveOnNav, rxFollowNavRoute, fd);
      }
      return;
    }
    case 1021: {
      auto index = readMuxID(frame);
      bool rxFsdStops = isFSDSelectedInUI(frame);
      switch (index) {
      case 0:
        setBit(frame, 38, true);
        setBit(frame, 46, true);
        setBit(frame, 60, true);
        canSend(frame);
        if (enablePrint) {
          Serial.printf("HW4Handler: fsdStops=%d->1 profile: %d\n", rxFsdStops, speedProfile);
        }
        break;
      case 1: {
        uint8_t rxApmv3Branch = readApmv3Branch(frame);
        setApmv3Branch(frame, UI_APMV3_BRANCH_OVERRIDE);
        setBit(frame, 19, false);
        setBit(frame, 45, true);
        setBit(frame, 47, true);
        canSend(frame);
        if (enablePrint) {
          Serial.printf("ID1021 m1: apmv3Branch=%u (%s)->%u (%s)\n",
                        rxApmv3Branch, apmv3BranchName(rxApmv3Branch),
                        UI_APMV3_BRANCH_OVERRIDE, apmv3BranchName(UI_APMV3_BRANCH_OVERRIDE));
        }
        break;
      }
      case 2:
        frame.data[7] &= ~(0x07 << 4);
        frame.data[7] |= (speedProfile & 0x07) << 4;
        canSend(frame);
        break;
      }
      return;
    }
    case 2047:
      if (frame.data[0] != 2) return;
      {
        uint8_t rxAutopilot = (frame.data[5] >> 2) & 0x07;
        frame.data[5] &= ~(0x07 << 2);
        frame.data[5] |= (4 & 0x07) << 2;
        canSend(frame);
        if (enablePrint) {
          Serial.printf("ID2047m2: autopilot=%d->4\n", rxAutopilot);
        }
      }
      return;
    }
  }
};


std::unique_ptr<CarManagerBase> handler;


void setup() {
  handler = std::make_unique<HW>();
  delay(1500);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1000) {}

  if (!canInit()) {
    Serial.println("CAN init failed");
    while (true) { delay(1000); }
  }
  Serial.println("CAN ready @ 500k");
}


__attribute__((optimize("O3"))) void loop() {
  can_frame frame;
  if (canRead(frame) != 0) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  digitalWrite(LED_PIN, LOW);
  handler->handelMessage(frame);
}
