#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include "rgb_lcd.h"

static const uint8_t CAN_CS_PIN  = 53;
static const uint8_t CAN_INT_PIN = 2;

struct can_frame rxFrame;
MCP2515 mcp2515(CAN_CS_PIN);
rgb_lcd lcd;

volatile bool canIntFlag = false;

unsigned long lastLcdUpdateMs = 0;
unsigned long lastFrameMs = 0;
uint32_t frameCount = 0;
uint32_t lastCanId = 0;
bool canReady = false;

void onCanInterrupt() {
  canIntFlag = true;
}

void setLcdStatusBoot() {
  lcd.setRGB(0, 0, 255);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CAN Sniffer");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");
}

void setLcdStatusReady() {
  lcd.setRGB(0, 255, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CAN 500kbps");
  lcd.setCursor(0, 1);
  lcd.print("Waiting bus...");
}

void setLcdStatusError() {
  lcd.setRGB(255, 0, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MCP2515 ERROR");
  lcd.setCursor(0, 1);
  lcd.print("Init failed");
}

void updateLcdTrafficStatus() {
  unsigned long now = millis();

  if (!canReady) {
    return;
  }

  if ((now - lastFrameMs) < 1000) {
    lcd.setRGB(255, 140, 0);
  } else {
    lcd.setRGB(0, 255, 0);
  }

  lcd.setCursor(0, 0);
  lcd.print("CAN 500k      ");

  lcd.setCursor(0, 1);
  lcd.print("F:");
  lcd.print(frameCount);
  lcd.print(" ID:");
  lcd.print(lastCanId, HEX);
  lcd.print("   ");
}

void printFrameCSV(const struct can_frame &f, unsigned long ts_ms) {
  Serial.print(ts_ms);
  Serial.print(',');

  uint32_t id = f.can_id & CAN_EFF_MASK;
  bool isExt = (f.can_id & CAN_EFF_FLAG) != 0;
  bool isRtr = (f.can_id & CAN_RTR_FLAG) != 0;

  if (!isExt) {
    id = f.can_id & CAN_SFF_MASK;
  }

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
    if (f.data[i] < 0x10) Serial.print('0');
    Serial.print(f.data[i], HEX);
  }
  Serial.println();
}

void drainCanFrames() {
  while (mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
    unsigned long ts = millis();

    uint32_t id = rxFrame.can_id & CAN_EFF_MASK;
    bool isExt = (rxFrame.can_id & CAN_EFF_FLAG) != 0;
    if (!isExt) {
      id = rxFrame.can_id & CAN_SFF_MASK;
    }

    frameCount++;
    lastFrameMs = ts;
    lastCanId = id;

    printFrameCSV(rxFrame, ts);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  lcd.begin(16, 2);
  setLcdStatusBoot();

  SPI.begin();

  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    setLcdStatusError();
    Serial.println("error,mcp2515_bitrate");
    while (1) {}
  }

  if (mcp2515.setListenOnlyMode() != MCP2515::ERROR_OK) {
    setLcdStatusError();
    Serial.println("error,mcp2515_mode");
    while (1) {}
  }

  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), onCanInterrupt, FALLING);

  canReady = true;
  setLcdStatusReady();

  Serial.println("ts_ms,id,ext,rtr,dlc,data");
}

void loop() {
  if (canIntFlag) {
    noInterrupts();
    canIntFlag = false;
    interrupts();

    drainCanFrames();
  }

  if (millis() - lastLcdUpdateMs >= 250) {
    lastLcdUpdateMs = millis();
    updateLcdTrafficStatus();
  }
}