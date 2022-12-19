/*
  AT28C256 Programmer for MEGA 2560

  PORT MAPPING
  +---------+----------+
  | ARDUINO | AT28C256 |
  +---------+----------+
  | PORTF   |  A0-A7   |
  +---------+----------+
  | PK0-PK6 |  A8-A14  |
  +---------+----------+
  | PORTC   |  IO0-IO7 |
  +---------+----------+
  | PH4   7 |  CEN     |
  | PH5   8 |  OEN     |
  | PH6   9 |  WEN     |
  +---------+----------+
*/

#include <Arduino.h>

#define PORT_ADDR_LO PORTF
#define DDR_ADDR_LO  DDRF
#define PORT_ADDR_HI PORTK
#define DDR_ADDR_HI  DDRK
#define MAX_ADDR     32767U
#define ADDR         ((PORT_ADDR_HI << 8) | PORT_ADDR_LO)

#define PORT_DATA    PORTC
#define PIN_DATA     PINC
#define DDR_DATA     DDRC

#define PIN_CEN      7
#define PIN_OEN      8
#define PIN_WEN      9

static inline void chipEnable() {
    digitalWrite(PIN_CEN, LOW);
}

static inline void chipDisable() {
    digitalWrite(PIN_CEN, HIGH);
}

static inline void outputEnable() {
    digitalWrite(PIN_OEN, LOW);
}

static inline void outputDisable() {
    digitalWrite(PIN_OEN, HIGH);
}

static inline void writeEnable() {
    PORTH &= 0xbf;  // faster than digitalWrite(PIN_WEN, LOW)
}

static inline void writeDisable() {
    PORTH |= 0x40;  // faster than digitalWrite(PIN_WEN, HIGH)
}

void setAddress(uint16_t address) {
    if (address > MAX_ADDR)
        address = MAX_ADDR;

    PORT_ADDR_LO = (byte)address;
    PORT_ADDR_HI &= 0x80;
    PORT_ADDR_HI |= (byte)(address >> 8);
}

static inline void setData(byte data) {
    PORT_DATA = data;
}

static inline byte readData() {
    return PIN_DATA;
}

void setDataBusMode(byte mode) {
    if (mode == INPUT)
        DDR_DATA = 0x00;
    else
        DDR_DATA = 0xff;
}

byte readByte(uint16_t address) {
    setAddress(address);
    setDataBusMode(INPUT);
    chipEnable();
    outputEnable();
    byte data = readData();
    outputDisable();
    chipDisable();
    return data;
}

void writeByte(uint16_t address, byte data) {
    setAddress(address);
    setDataBusMode(OUTPUT);
    setData(data);
    chipEnable();
    writeEnable();
    delay(1);
    writeDisable();
    chipDisable();
    delay(10);
}

void unlockChip() {
    setDataBusMode(OUTPUT);
    chipEnable();

    setAddress(0x5555);
    setData(0xaa);
    writeEnable();
    writeDisable();

    setAddress(0x2aaa);
    setData(0x55);
    writeEnable();
    writeDisable();

    setAddress(0x5555);
    setData(0x80);
    writeEnable();
    writeDisable();

    setAddress(0x5555);
    setData(0xaa);
    writeEnable();
    writeDisable();

    setAddress(0x2aaa);
    setData(0x55);
    writeEnable();
    writeDisable();

    setAddress(0x5555);
    setData(0x20);
    writeEnable();
    writeDisable();

    chipDisable();
    delay(10);
}

void lockChip() {
    setDataBusMode(OUTPUT);
    chipEnable();

    setAddress(0x5555);
    setData(0xaa);
    writeEnable();
    writeDisable();

    setAddress(0x2aaa);
    setData(0x55);
    writeEnable();
    writeDisable();

    setAddress(0x5555);
    setData(0xa0);
    writeEnable();
    writeDisable();

    chipDisable();
    delay(10);
}

void clearChip() {
    for (uint16_t address = 0x00; address <= MAX_ADDR; address++) {
        writeByte(address, 0xff);
    }
}

void setup() {
    Serial.begin(9600);
    chipDisable();
    pinMode(PIN_CEN, OUTPUT);
    writeDisable();
    pinMode(PIN_WEN, OUTPUT);
    outputDisable();
    pinMode(PIN_OEN, OUTPUT);

    DDR_ADDR_HI |= 0x7f;  // set all (bust most significant) address bits bit to output
    DDR_ADDR_LO = 0xff;   // all low address bits to output

    // init registers to known state
    setDataBusMode(OUTPUT);
    setAddress(0);
    setData(0);

    delay(1);

    // Test read
    // byte data[16];
    // char buf[80];
    // Serial.println("");
    // for (uint16_t address = 0; address < 16; address++) {
    //     data[address] = readByte(address);
    // }
    // sprintf(buf, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
    //         data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
    //         data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
    // Serial.println(buf);
}

void loop() {
}

