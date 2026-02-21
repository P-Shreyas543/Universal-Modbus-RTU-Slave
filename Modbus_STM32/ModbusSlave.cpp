#include "ModbusSlave.h"

// Silence interval: >19200 baud = fixed 1.75ms
#define T35_US 1750

ModbusSlave::ModbusSlave(HardwareSerial& serial, uint8_t dePin, uint8_t slaveId)
  : _serial(serial), _dePin(dePin), _slaveId(slaveId) {
  _rxIndex = 0;
  _lastByteMicros = 0;
  _coils = nullptr;
  _discreteInputs = nullptr;
  _holdingRegisters = nullptr;
  _inputRegisters = nullptr;
}

void ModbusSlave::begin(unsigned long baud) {
  _baudRate = baud;
  pinMode(_dePin, OUTPUT);
  digitalWrite(_dePin, LOW);
  _serial.begin(_baudRate, SERIAL_8N1);
}

void ModbusSlave::configureCoils(bool* buffer, uint16_t size) {
  _coils = buffer;
  _coilsSize = size;
}
void ModbusSlave::configureDiscreteInputs(bool* buffer, uint16_t size) {
  _discreteInputs = buffer;
  _discreteInputsSize = size;
}
void ModbusSlave::configureHoldingRegisters(uint16_t* buffer, uint16_t size) {
  _holdingRegisters = buffer;
  _holdingRegistersSize = size;
}
void ModbusSlave::configureInputRegisters(uint16_t* buffer, uint16_t size) {
  _inputRegisters = buffer;
  _inputRegistersSize = size;
}

void ModbusSlave::poll() {
  while (_serial.available()) {
    if (_rxIndex < 256) {
      _rxBuffer[_rxIndex++] = _serial.read();
      _lastByteMicros = micros();
    } else {
      _rxIndex = 0;
    }
  }

  if (_rxIndex > 3 && (micros() - _lastByteMicros) > T35_US) {
    if (_rxBuffer[0] != _slaveId) {
      _rxIndex = 0;
      return;
    }

    uint16_t receivedCRC = _rxBuffer[_rxIndex - 2] | (_rxBuffer[_rxIndex - 1] << 8);
    if (receivedCRC != calcCRC(_rxBuffer, _rxIndex - 2)) {
      _rxIndex = 0;
      return;
    }

    uint8_t funcCode = _rxBuffer[1];
    switch (funcCode) {
      case 0x01: handleReadCoils(_rxBuffer); break;
      case 0x02: handleReadDiscreteInputs(_rxBuffer); break;
      case 0x03: handleReadHoldingRegisters(_rxBuffer); break;
      case 0x04: handleReadInputRegisters(_rxBuffer); break;
      case 0x05: handleWriteSingleCoil(_rxBuffer); break;
      case 0x06: handleWriteSingleRegister(_rxBuffer); break;
      case 0x0F: handleWriteMultipleCoils(_rxBuffer); break;
      case 0x10: handleWriteMultipleRegisters(_rxBuffer); break;
      default: sendException(funcCode, EX_ILLEGAL_FUNCTION); break;
    }
    _rxIndex = 0;
  }
}

// Function 01: Read Coils
void ModbusSlave::handleReadCoils(uint8_t* frame) {
  if (!_coils) {
    sendException(0x01, EX_SLAVE_FAILURE);
    return;
  }
  uint16_t startAddr = (frame[2] << 8) | frame[3];
  uint16_t quantity = (frame[4] << 8) | frame[5];
  if ((startAddr + quantity) > _coilsSize) {
    sendException(0x01, EX_ILLEGAL_ADDRESS);
    return;
  }
  uint8_t byteCount = (quantity + 7) / 8;
  _txBuffer[0] = _slaveId;
  _txBuffer[1] = 0x01;
  _txBuffer[2] = byteCount;
  uint8_t idx = 3;
  for (int k = 0; k < byteCount; k++) _txBuffer[idx + k] = 0;
  for (int i = 0; i < quantity; i++) {
    if (_coils[startAddr + i]) _txBuffer[idx + (i / 8)] |= (1 << (i % 8));
  }
  sendResponse(_txBuffer, idx + byteCount);
}

// Function 02: Read Discrete Inputs
void ModbusSlave::handleReadDiscreteInputs(uint8_t* frame) {
  if (!_discreteInputs) {
    sendException(0x02, EX_SLAVE_FAILURE);
    return;
  }
  uint16_t startAddr = (frame[2] << 8) | frame[3];
  uint16_t quantity = (frame[4] << 8) | frame[5];
  if ((startAddr + quantity) > _discreteInputsSize) {
    sendException(0x02, EX_ILLEGAL_ADDRESS);
    return;
  }
  uint8_t byteCount = (quantity + 7) / 8;
  _txBuffer[0] = _slaveId;
  _txBuffer[1] = 0x02;
  _txBuffer[2] = byteCount;
  uint8_t idx = 3;
  for (int k = 0; k < byteCount; k++) _txBuffer[idx + k] = 0;
  for (int i = 0; i < quantity; i++) {
    if (_discreteInputs[startAddr + i]) _txBuffer[idx + (i / 8)] |= (1 << (i % 8));
  }
  sendResponse(_txBuffer, idx + byteCount);
}

// Function 03: Read Holding Registers
void ModbusSlave::handleReadHoldingRegisters(uint8_t* frame) {
  if (!_holdingRegisters) {
    sendException(0x03, EX_SLAVE_FAILURE);
    return;
  }
  uint16_t startAddr = (frame[2] << 8) | frame[3];
  uint16_t quantity = (frame[4] << 8) | frame[5];
  if ((startAddr + quantity) > _holdingRegistersSize) {
    sendException(0x03, EX_ILLEGAL_ADDRESS);
    return;
  }
  _txBuffer[0] = _slaveId;
  _txBuffer[1] = 0x03;
  _txBuffer[2] = quantity * 2;
  uint8_t idx = 3;
  for (int i = 0; i < quantity; i++) {
    uint16_t val = _holdingRegisters[startAddr + i];
    _txBuffer[idx++] = (val >> 8);
    _txBuffer[idx++] = (val & 0xFF);
  }
  sendResponse(_txBuffer, idx);
}

// Function 04: Read Input Registers
void ModbusSlave::handleReadInputRegisters(uint8_t* frame) {
  if (!_inputRegisters) {
    sendException(0x04, EX_SLAVE_FAILURE);
    return;
  }
  uint16_t startAddr = (frame[2] << 8) | frame[3];
  uint16_t quantity = (frame[4] << 8) | frame[5];
  if ((startAddr + quantity) > _inputRegistersSize) {
    sendException(0x04, EX_ILLEGAL_ADDRESS);
    return;
  }
  _txBuffer[0] = _slaveId;
  _txBuffer[1] = 0x04;
  _txBuffer[2] = quantity * 2;
  uint8_t idx = 3;
  for (int i = 0; i < quantity; i++) {
    uint16_t val = _inputRegisters[startAddr + i];
    _txBuffer[idx++] = (val >> 8);
    _txBuffer[idx++] = (val & 0xFF);
  }
  sendResponse(_txBuffer, idx);
}

// Function 05: Write Single Coil
void ModbusSlave::handleWriteSingleCoil(uint8_t* frame) {
  if (!_coils) {
    sendException(0x05, EX_SLAVE_FAILURE);
    return;
  }
  uint16_t addr = (frame[2] << 8) | frame[3];
  uint16_t val = (frame[4] << 8) | frame[5];
  if (addr >= _coilsSize) {
    sendException(0x05, EX_ILLEGAL_ADDRESS);
    return;
  }
  if (val == 0xFF00) _coils[addr] = true;
  else if (val == 0x0000) _coils[addr] = false;
  else {
    sendException(0x05, EX_ILLEGAL_VALUE);
    return;
  }
  memcpy(_txBuffer, frame, 6);
  sendResponse(_txBuffer, 6);
}

// Function 06: Write Single Register
void ModbusSlave::handleWriteSingleRegister(uint8_t* frame) {
  if (!_holdingRegisters) {
    sendException(0x06, EX_SLAVE_FAILURE);
    return;
  }
  uint16_t addr = (frame[2] << 8) | frame[3];
  uint16_t val = (frame[4] << 8) | frame[5];
  if (addr >= _holdingRegistersSize) {
    sendException(0x06, EX_ILLEGAL_ADDRESS);
    return;
  }
  _holdingRegisters[addr] = val;
  memcpy(_txBuffer, frame, 6);
  sendResponse(_txBuffer, 6);
}

// Function 15: Write Multiple Coils
void ModbusSlave::handleWriteMultipleCoils(uint8_t* frame) {
  if (!_coils) {
    sendException(0x0F, EX_SLAVE_FAILURE);
    return;
  }
  uint16_t startAddr = (frame[2] << 8) | frame[3];
  uint16_t quantity = (frame[4] << 8) | frame[5];
  uint8_t byteCount = frame[6];
  // Validate Address
  if ((startAddr + quantity) > _coilsSize) {
    sendException(0x0F, EX_ILLEGAL_ADDRESS);
    return;
  }
  // Parse Data Bytes
  for (int i = 0; i < quantity; i++) {
    uint8_t byteIndex = i / 8;
    uint8_t bitIndex = i % 8;
    // Data starts at frame[7]
    uint8_t dataByte = frame[7 + byteIndex];
    // Extract bit
    bool state = (dataByte >> bitIndex) & 0x01;
    _coils[startAddr + i] = state;
  }
  // Response: Echo ID, Func, Addr, Qty (6 Bytes)
  memcpy(_txBuffer, frame, 6);
  sendResponse(_txBuffer, 6);
}

// Function 16: Write Multiple Registers
void ModbusSlave::handleWriteMultipleRegisters(uint8_t* frame) {
  if (!_holdingRegisters) {
    sendException(0x10, EX_SLAVE_FAILURE);
    return;
  }
  uint16_t startAddr = (frame[2] << 8) | frame[3];
  uint16_t quantity = (frame[4] << 8) | frame[5];
  uint8_t byteCount = frame[6];  // Not strictly needed for logic but part of frame
  // Validate Address
  if ((startAddr + quantity) > _holdingRegistersSize) {
    sendException(0x10, EX_ILLEGAL_ADDRESS);
    return;
  }
  // Parse Data Bytes
  for (int i = 0; i < quantity; i++) {
    // Data starts at frame[7]. Each register is 2 bytes.
    uint8_t hi = frame[7 + (i * 2)];
    uint8_t lo = frame[7 + (i * 2) + 1];

    _holdingRegisters[startAddr + i] = (hi << 8) | lo;
  }
  // Response: Echo ID, Func, Addr, Qty (6 Bytes)
  memcpy(_txBuffer, frame, 6);
  sendResponse(_txBuffer, 6);
}

// -------------------------------------------------------------------------

void ModbusSlave::sendResponse(uint8_t* frame, uint16_t len) {
  uint16_t crc = calcCRC(frame, len);
  frame[len++] = crc & 0xFF;
  frame[len++] = (crc >> 8);

  digitalWrite(_dePin, HIGH);
  delayMicroseconds(10);
  _serial.write(frame, len);
  _serial.flush();
  delayMicroseconds(10);
  digitalWrite(_dePin, LOW);
}

void ModbusSlave::sendException(uint8_t funcCode, uint8_t exceptionCode) {
  _txBuffer[0] = _slaveId;
  _txBuffer[1] = funcCode | 0x80;
  _txBuffer[2] = exceptionCode;
  sendResponse(_txBuffer, 3);
}

uint16_t ModbusSlave::calcCRC(uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}