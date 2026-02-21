#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <Arduino.h>

// Modbus Exception Codes
#define EX_ILLEGAL_FUNCTION 0x01
#define EX_ILLEGAL_ADDRESS 0x02
#define EX_ILLEGAL_VALUE 0x03
#define EX_SLAVE_FAILURE 0x04

class ModbusSlave {
public:
  ModbusSlave(HardwareSerial& serial, uint8_t dePin, uint8_t slaveId);

  void configureCoils(bool* buffer, uint16_t size);
  void configureDiscreteInputs(bool* buffer, uint16_t size);
  void configureHoldingRegisters(uint16_t* buffer, uint16_t size);
  void configureInputRegisters(uint16_t* buffer, uint16_t size);

  void begin(unsigned long baud);
  void poll();

private:
  HardwareSerial& _serial;
  uint8_t _dePin;
  uint8_t _slaveId;
  unsigned long _baudRate;

  uint8_t _rxBuffer[256];
  uint8_t _txBuffer[256];
  uint16_t _rxIndex;
  unsigned long _lastByteMicros;

  bool* _coils;
  bool* _discreteInputs;
  uint16_t* _holdingRegisters;
  uint16_t* _inputRegisters;

  uint16_t _coilsSize;
  uint16_t _discreteInputsSize;
  uint16_t _holdingRegistersSize;
  uint16_t _inputRegistersSize;

  uint16_t calcCRC(uint8_t* buf, int len);
  void sendResponse(uint8_t* frame, uint16_t len);
  void sendException(uint8_t funcCode, uint8_t exceptionCode);

  void handleReadCoils(uint8_t* frame);             // Func 01
  void handleReadDiscreteInputs(uint8_t* frame);    // Func 02
  void handleReadHoldingRegisters(uint8_t* frame);  // Func 03
  void handleReadInputRegisters(uint8_t* frame);    // Func 04
  void handleWriteSingleCoil(uint8_t* frame);       // Func 05
  void handleWriteSingleRegister(uint8_t* frame);   // Func 06
  void handleWriteMultipleCoils(uint8_t* frame);      // Func 15
  void handleWriteMultipleRegisters(uint8_t* frame);  // Func 16
};

#endif