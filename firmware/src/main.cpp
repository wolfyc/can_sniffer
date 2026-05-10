#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include "rgb_lcd.h"

static const uint8_t CAN_CS_PIN  = 53;
static const uint8_t CAN_INT_PIN = 2;

static const unsigned long SERIAL_BAUD = 115200;
static const uint16_t LCD_UPDATE_MS = 250;
static const uint16_t HEARTBEAT_MS = 1000;

// Print only 1 frame out of N to avoid serial choking on a busy car bus.
// Set to 1 to print every frame.
static const uint8_t SERIAL_DECIMATION = 1;

MCP2515 mcp2515(CAN_CS_PIN);
rgb_lcd lcd;
struct can_frame rxFrame;

volatile bool canInterruptPending = false;

uint32_t irqCount = 0;
uint32_t rxCount = 0;
uint32_t rxPrintedCount = 0;
uint32_t rxErrorCount = 0;
uint32_t rxOverflowCount = 0;
uint32_t lastCanId = 0;

unsigned long lastFrameMs = 0;
unsigned long lastLcdUpdateMs = 0;
unsigned long lastHeartbeatMs = 0;

// ISR callback: set a flag when a CAN interrupt is asserted.
void onCanInterrupt() {
  canInterruptPending = true;
  irqCount++;
}

// LCD status display during boot.
void lcdSetBoot() {
  lcd.setRGB(0, 0, 255);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CAN Sniffer");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");
}

// LCD status display when CAN is ready.
void lcdSetReady() {
  lcd.setRGB(0, 255, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CAN ListenOnly");
  lcd.setCursor(0, 1);
  lcd.print("Waiting traffic");
}

// LCD status display on error with custom second line.
void lcdSetError(const char* line2) {
  lcd.setRGB(255, 0, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MCP2515 ERROR");
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// Extract the CAN identifier depending on standard or extended frame format.
uint32_t extractCanId(const struct can_frame &f) {
  if (f.can_id & CAN_EFF_FLAG) {
    return (f.can_id & CAN_EFF_MASK);
  }
  return (f.can_id & CAN_SFF_MASK);
}

// Print a byte as two hex digits with leading zero padding as needed.
void printHexByte(uint8_t b) {
  if (b < 0x10) {
    Serial.print('0');
  }
  Serial.print(b, HEX);
}

// Serialize one CAN frame as CSV line.
void printFrameCSV(const struct can_frame &f, unsigned long ts_ms) {
  uint32_t id = extractCanId(f);
  bool isExt = (f.can_id & CAN_EFF_FLAG) != 0;
  bool isRtr = (f.can_id & CAN_RTR_FLAG) != 0;

  Serial.print(ts_ms);
  Serial.print(',');

  Serial.print(id, HEX);
  Serial.print(',');
  Serial.print(isExt ? 1 : 0);
  Serial.print(',');
  Serial.print(isRtr ? 1 : 0);
  Serial.print(',');
  Serial.print(f.can_dlc);
  Serial.print(',');

  for (uint8_t i = 0; i < f.can_dlc; i++) {
    if (i) Serial.print(' ');
    printHexByte(f.data[i]);
  }
  Serial.println();
}

// Handle a received CAN frame and optionally print it based on decimation.
void processFrame(const struct can_frame &f) {
  unsigned long ts = millis();
  rxCount++;
  lastFrameMs = ts;
  lastCanId = extractCanId(f);

  if ((rxCount % SERIAL_DECIMATION) == 0) {
    printFrameCSV(f, ts);
    rxPrintedCount++;
  }
}

// Poll MCP2515 interrupt flags and read pending frames from RX buffers.
void pollAndDrainCan() {
  uint8_t irq = mcp2515.getInterrupts();

  if (irq & MCP2515::CANINTF_RX0IF) {
    if (mcp2515.readMessage(MCP2515::RXB0, &rxFrame) == MCP2515::ERROR_OK) {
      processFrame(rxFrame);
    } else {
      rxErrorCount++;
    }
  }

  if (irq & MCP2515::CANINTF_RX1IF) {
    if (mcp2515.readMessage(MCP2515::RXB1, &rxFrame) == MCP2515::ERROR_OK) {
      processFrame(rxFrame);
    } else {
      rxErrorCount++;
    }
  }

  // Clear overflow flags if they happened.
  mcp2515.clearRXnOVRFlags();
  mcp2515.clearInterrupts();
}

// Update the LCD with traffic counters and status.
void updateLcd() {
  unsigned long now = millis();

  if ((now - lastFrameMs) < 1000) {
    lcd.setRGB(255, 140, 0);
  } else {
    lcd.setRGB(0, 255, 0);
  }

  lcd.setCursor(0, 0);
  lcd.print("RX:");
  lcd.print(rxCount);
  lcd.print(" P:");
  lcd.print(rxPrintedCount);
  lcd.print("   ");

  lcd.setCursor(0, 1);
  lcd.print("ID:");
  lcd.print(lastCanId, HEX);
  lcd.print(" E:");
  lcd.print(rxErrorCount);
  lcd.print("   ");
}

// Print periodic heartbeat diagnostics to serial.
void printHeartbeat() {
  Serial.print("# d,uptime_ms=");
  Serial.print(millis());
  Serial.print(",irq=");
  Serial.print(irqCount);
  Serial.print(",rx=");
  Serial.print(rxCount);
  Serial.print(",printed=");
  Serial.print(rxPrintedCount);
  Serial.print(",err=");
  Serial.println(rxErrorCount);
}

// Initialize the MCP2515 CAN controller in listen-only mode.
bool initCan() {
  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    return false;
  }

  if (mcp2515.setListenOnlyMode() != MCP2515::ERROR_OK) {
    return false;
  }

  mcp2515.clearInterrupts();
  mcp2515.clearRXnOVRFlags();
  return true;
}

// Arduino setup function: initialize peripherals and CAN interface.
void setup() {
  Serial.begin(SERIAL_BAUD);
  SPI.begin();
  Wire.begin();

  lcd.begin(16, 2);
  lcdSetBoot();

  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  Serial.println("# boot");
  Serial.println("ts_ms,id,ext,rtr,dlc,data");

  if (!initCan()) {
    lcdSetError("Init failed");
    Serial.println("# error,mcp2515_init_failed");
    while (true) {
      delay(1000);
    }
  }

  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), onCanInterrupt, FALLING);

  lcdSetReady();
}

// Arduino loop function: process CAN frames, update LCD, and send heartbeat.
void loop() {
  if (canInterruptPending) {
    noInterrupts();
    canInterruptPending = false;
    interrupts();

    pollAndDrainCan();
  }

  unsigned long now = millis();

  if (now - lastLcdUpdateMs >= LCD_UPDATE_MS) {
    lastLcdUpdateMs = now;
    updateLcd();
  }

  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    printHeartbeat();
  }
}
