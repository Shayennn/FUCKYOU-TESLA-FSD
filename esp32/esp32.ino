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
      uint8_t followDistance = (frame.data[5] & 0b11100000) >> 5;
      switch (followDistance) {
        case 1: speedProfile = 2; break;
        case 2: speedProfile = 1; break;
        case 3: speedProfile = 0; break;
        default: break;
      }
      setBit(frame, 5, true);
      setBit(frame, 14, true);
      canSend(frame);
      if (enablePrint) {
        Serial.printf("ID1016: dasDev=%d->1 handsOffDisable=%d->1 followDist=%d\n", rxDasDev, rxHandsOff, followDistance);
      }
      return;
    }
    case 1021: {
      auto index = readMuxID(frame);
      bool rxFsdStops = isFSDSelectedInUI(frame);
      switch (index) {
      case 0:
        speedOffset = std::max(std::min(((uint8_t)((frame.data[3] >> 1) & 0x3F) - 30) * 5, 100), 0);
        {
          auto off = (uint8_t)((frame.data[3] >> 1) & 0x3F) - 30;
          switch (off) {
            case 2: speedProfile = 2; break;
            case 1: speedProfile = 1; break;
            case 0: speedProfile = 0; break;
            default: break;
          }
        }
        setBit(frame, 38, true);
        setBit(frame, 46, true);
        setSpeedProfileV12V13(frame, speedProfile);
        canSend(frame);
        if (enablePrint) {
          Serial.printf("HW3Handler: fsdStops=%d->1 Profile: %d, Offset: %d\n", rxFsdStops, speedProfile, speedOffset);
        }
        break;
      case 1:
        setBit(frame, 19, false);
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
      canSend(frame);
      if (enablePrint) {
        Serial.printf("ID1016: dasDev=%d->1 handsOffDisable=%d->1 followDist=%d\n", rxDasDev, rxHandsOff, fd);
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
      case 1:
        setBit(frame, 19, false);
        setBit(frame, 47, true);
        canSend(frame);
        break;
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
