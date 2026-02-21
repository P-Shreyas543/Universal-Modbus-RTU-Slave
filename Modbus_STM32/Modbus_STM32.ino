#include "ModbusSlave.h"

// ==========================================
//        HARDWARE CONFIGURATION
// ==========================================

#define RS485_PORT Serial2
#define RS485_DE PD4
#define MB_BAUD 115200

// LED Pins
#define LED1 PD12
#define LED2 PD13
#define LED3 PD14
#define LED4 PD15
#define BUTTON PA0

// ==========================================
//        MEMORY MODEL CONFIGURATION
// ==========================================
#define NUM_COILS 10
#define NUM_INPUTS 10
#define NUM_HOLDING 10
#define NUM_INPUT_REGS 10

bool coils[NUM_COILS];
bool discreteInputs[NUM_INPUTS];
uint16_t holdingRegs[NUM_HOLDING];
uint16_t inputRegs[NUM_INPUT_REGS];

// Create Slave Instance
ModbusSlave slave(RS485_PORT, RS485_DE, 1);

unsigned long lastBlink = 0;

void handleInterrupt() {
  inputRegs[0] = 0;
}

void setup() {
  // 1. Setup LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // 2. Initialize Data
  holdingRegs[0] = 100;
  holdingRegs[1] = 200;
  discreteInputs[0] = true;

  // 3. Link Arrays to Modbus Library
  slave.configureCoils(coils, NUM_COILS);
  slave.configureDiscreteInputs(discreteInputs, NUM_INPUTS);
  slave.configureHoldingRegisters(holdingRegs, NUM_HOLDING);
  slave.configureInputRegisters(inputRegs, NUM_INPUT_REGS);

  // 4.Switch Intrupts
  attachInterrupt(digitalPinToInterrupt(BUTTON), handleInterrupt, FALLING);

  // 5. Begin Communication
  slave.begin(MB_BAUD);
}

void loop() {
  // 1. Run Modbus Protocol
  slave.poll();

  // 2. Application Logic: Map Modbus Coils to LEDs
  digitalWrite(LED1, coils[0]);
  digitalWrite(LED2, coils[1]);
  digitalWrite(LED3, coils[2]);
  digitalWrite(LED4, coils[3]);

  // 3. Application Logic: Status Blink
  if (millis() - lastBlink > 100) {
    lastBlink = millis();
    inputRegs[0]++;
  }
}