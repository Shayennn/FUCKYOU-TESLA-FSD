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

int canRead(can_frame& frame) {
  int packetSize = CAN.parsePacket();
  if (packetSize <= 0) return -1;
  frame.can_id  = CAN.packetId();
  frame.can_dlc = packetSize;
  memset(frame.data, 0, 8);
  for (int i = 0; i < packetSize; i++) {
    frame.data[i] = CAN.read();
  }
  return 0;
}

void canSend(can_frame& frame) {
  CAN.beginPacket(frame.can_id, frame.can_dlc);
  CAN.write(frame.data, frame.can_dlc);
  CAN.endPacket();
}

struct CarManagerBase {
  int speedProfile = 1;
  virtual bool handelMessage(can_frame& frame) = 0;
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
  // Determine which byte and which bit within that byte
  int byteIndex = bit / 8;
  int bitIndex = bit % 8;
  // Set the desired bit
  uint8_t mask = static_cast<uint8_t>(1U << bitIndex);
  if (value) {
    frame.data[byteIndex] |= mask;
  } else {
    frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
  }
}


struct LegacyHandler : public CarManagerBase {
  virtual bool handelMessage(can_frame& frame) override {
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
#ifdef ENABLE_PRINT
        Serial.printf("LegacyHandler: Profile: %d\n", speedProfile);
#endif
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
  virtual bool handelMessage(can_frame& frame) override {
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
#ifdef ENABLE_PRINT
      Serial.printf("ID1016: dasDev=%d->1 handsOffDisable=%d->1 followDist=%d\n", rxDasDev, rxHandsOff, followDistance);
#endif
      return true;
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
#ifdef ENABLE_PRINT
        Serial.printf("HW3Handler: fsdStops=%d->1 Profile: %d, Offset: %d\n", rxFsdStops, speedProfile, speedOffset);
#endif
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
  virtual bool handelMessage(can_frame& frame) override {
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
#ifdef ENABLE_PRINT
      Serial.printf("ID1016: dasDev=%d->1 handsOffDisable=%d->1 followDist=%d\n", rxDasDev, rxHandsOff, fd);
#endif
      return true;
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
#ifdef ENABLE_PRINT
        Serial.printf("HW4Handler: fsdStops=%d->1 profile: %d\n", rxFsdStops, speedProfile);
#endif
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


void setup() {
  pinMode(PIN_NEOPIXEL_POWER, OUTPUT);
  digitalWrite(PIN_NEOPIXEL_POWER, HIGH);
  pixel.begin();
  pixel.setBrightness(30);
  setNeoColor(COLOR_BLUE);

  handler.reset(new HW());
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
  can_frame frame;
  if (canRead(frame) != 0) {
    setNeoColor(COLOR_BLUE);
    return;
  }
  bool modified = handler->handelMessage(frame);
  setNeoColor(modified ? COLOR_GREEN : COLOR_YELLOW);
}
