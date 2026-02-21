# Universal Modbus RTU Slave
A lightweight, non-blocking Modbus RTU slave library designed for Arduino-compatible microcontrollers (STM32, ESP32, Arduino Mega, AVR, RP2040, etc.). 

This library provides full control over your memory structures by linking user-defined arrays to Modbus memory banks, and it manages RS485 Transceiver Enable (DE/RE) pins automatically.

## 🚀 Features

* **Non-blocking Execution:** Designed to run seamlessly inside the `loop()` via a simple `.poll()` method.
* **Hardware Agnostic:** Works with any MCU that supports the standard Arduino `HardwareSerial` class.
* **Automatic RS485 Direction Control:** Automatically toggles the DE/RE pin for half-duplex RS485 transceivers (like MAX485).
* **Supported Modbus Functions:**
    * `0x01` - Read Coils
    * `0x02` - Read Discrete Inputs
    * `0x03` - Read Holding Registers
    * `0x04` - Read Input Registers
    * `0x05` - Write Single Coil
    * `0x06` - Write Single Register
    * `0x0F` - Write Multiple Coils
    * `0x10` - Write Multiple Registers

## 🛠 Hardware Requirements
* Any Arduino-compatible microcontroller with at least one Hardware Serial port.
* An RS485 Transceiver module (e.g., MAX485, SN65HVD230).

## 📦 Installation

1. Download this repository as a `.zip` file.
2. In the Arduino IDE, go to **Sketch** -> **Include Library** -> **Add .ZIP Library...**
3. Select the downloaded `.zip` file.
4. Include the library in your sketch: `#include "ModbusSlave.h"`

## ⚙️ Adapting to Different Microcontrollers

This library requires you to pass a `HardwareSerial` object, an RS485 DE (Data Enable) pin, and a Slave ID to the constructor. This is what makes it universally adaptable.

### 1. Arduino Mega (AVR)
The Mega has multiple hardware serial ports (`Serial1`, `Serial2`, `Serial3`). 
```cpp
#define RS485_PORT Serial1 
#define RS485_DE_PIN 2     // Connect to DE/RE on MAX485
#define SLAVE_ID 1

ModbusSlave slave(RS485_PORT, RS485_DE_PIN, SLAVE_ID);
```
### 2. ESP32 / ESP8266
ESP32 allows you to remap UART pins. You can assign the pins before starting the Modbus library, or just use the default `Serial2` pins.
```cpp
#define RS485_PORT Serial2
#define RS485_DE_PIN 4
#define SLAVE_ID 1

ModbusSlave slave(RS485_PORT, RS485_DE_PIN, SLAVE_ID);

void setup() {
  // Optional: For ESP32, you can map custom RX/TX pins before slave.begin()
  // Serial2.setPins(RX_PIN, TX_PIN); 
  slave.begin(115200);
}
```
### 3. STM32 (Nucleo / BluePill)
STM32 boards often use `Serial2` or `Serial3` mapped to specific PA or PB pins depending on the board variant.
```cpp
#define RS485_PORT Serial2
#define RS485_DE_PIN PD4
#define SLAVE_ID 1

ModbusSlave slave(RS485_PORT, RS485_DE_PIN, SLAVE_ID);
```
## 📖 Usage Example
Here is a complete example mapping LEDs to coils and a push-button to an input register.
```cpp
#include "ModbusSlave.h"

// 1. Hardware Configuration
#define RS485_PORT Serial2
#define RS485_DE PD4
#define MB_BAUD 115200
#define SLAVE_ID 1

// 2. Define Memory Sizes
#define NUM_COILS 10
#define NUM_INPUTS 10
#define NUM_HOLDING 10
#define NUM_INPUT_REGS 10

// 3. Create Memory Arrays
bool coils[NUM_COILS];
bool discreteInputs[NUM_INPUTS];
uint16_t holdingRegs[NUM_HOLDING];
uint16_t inputRegs[NUM_INPUT_REGS];

// 4. Instantiate Library
ModbusSlave slave(RS485_PORT, RS485_DE, SLAVE_ID);

void setup() {
  // Link your arrays to the Modbus library
  slave.configureCoils(coils, NUM_COILS);
  slave.configureDiscreteInputs(discreteInputs, NUM_INPUTS);
  slave.configureHoldingRegisters(holdingRegs, NUM_HOLDING);
  slave.configureInputRegisters(inputRegs, NUM_INPUT_REGS);

  // Initialize data
  holdingRegs[0] = 100;
  discreteInputs[0] = true;

  // Start the Modbus slave
  slave.begin(MB_BAUD);
}

void loop() {
  // 1. Process incoming Modbus requests
  slave.poll();

  // 2. Map Modbus memory to physical hardware
  // Example: Turn on an LED based on Coil 0 state
  // digitalWrite(LED_PIN, coils[0]);
  
  // Example: Read a sensor into an Input Register
  // inputRegs[0] = analogRead(SENSOR_PIN);
}
```
## 📚 API Reference
ModbusSlave(HardwareSerial& serial, uint8_t dePin, uint8_t slaveId)

**Constructor**. * `serial`: The hardware serial port to use (e.g., `Serial1`, `Serial2`).
  * `dePin`: The digital pin connected to the DE/RE pins of the RS485 transceiver.
  * `slaveId`: The Modbus node address (1-247).

`void configureCoils(bool* buffer, uint16_t size)`
Links a user-defined `boolean` array to the Coils (0x) memory bank.

`void configureDiscreteInputs(bool* buffer, uint16_t size)`
Links a user-defined `boolean` array to the Discrete Inputs (1x) memory bank.

`void configureHoldingRegisters(uint16_t* buffer, uint16_t size)`
Links a user-defined `uint16_t` array to the Holding Registers (4x) memory bank.

`void configureInputRegisters(uint16_t* buffer, uint16_t size)`
Links a user-defined `uint16_t` array to the Input Registers (3x) memory bank.

`void begin(unsigned long baud)`
Initializes the serial port with the specified baud rate at `8N1` (8 data bits, no parity, 1 stop bit) and configures the dePin as an output.

`void poll()`
Reads the serial buffer, validates incoming frames via CRC, executes the requested Modbus function, and transmits the response. Must be called frequently inside `loop()`.

***
