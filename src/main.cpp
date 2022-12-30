/*
  AT28C256 Programmer for Arduino MEGA 2560

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

bool writeByte(uint16_t address, byte data) {
    setAddress(address);
    setDataBusMode(OUTPUT);
    setData(data);
    chipEnable();
    writeEnable();
    delay(1);
    writeDisable();
    chipDisable();
    // delay(10);
    
    // Read byte to confirm write
    bool confirmed = false;
    uint8_t count = 10;
    setData(0);
    setDataBusMode(INPUT);
    chipEnable();
    while (!confirmed && count > 0) {
        outputEnable();
        confirmed = data == readData();
        count--;
        outputDisable();
        delay(1);
    }
    return confirmed;
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
    Serial.println("<OK=");
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
    Serial.println("<OK=");
}

void clearChip() {
    char buf[10];
    for (uint16_t address = 0x00; address <= MAX_ADDR; address++) {
        sprintf(buf, "<%04x:%02x=", address, 0xff);
        Serial.println(buf);
        if (!writeByte(address, 0xff)) {
            Serial.println("<ERROR=");
            return;
        }
    }
    Serial.println("<OK=");
}

void readChip() {
    char buf[10];
    for (uint16_t address = 0x00; address <= MAX_ADDR; address++) {
        byte data = readByte(address);
        sprintf(buf, "<%04x:%02x=", address, data);
        Serial.println(buf);
    }
    Serial.println("<OK=");
}

void setup() {
    Serial.begin(115200);
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

    delay(100);
}

void OK() {
    Serial.println("<OK=");
}

char cmd;
uint16_t address = 0;
byte data = 0;
uint8_t pos = -1;
bool cmdReceived = false;

void loop() {
    int c = Serial.read();
    if (c != -1) {
        // incoming byte
        if ((char)c == '=') {
            // commant received
            cmdReceived = true;
        } else if ((char)c == '<') {
            // start command
            pos = 0;
        } else {
            switch (pos)
            {
            case 0:
                cmd = (char)c;
                pos++;
                address = 0;
                data = 0;
                break;

            case 1:
                address = ((uint16_t)c << 8);
                pos++;
                break;

            case 2:
                address += (uint16_t)c;
                pos++;
                break;

            case 3:
                data += (uint16_t)c;
                pos++;
                break;
            }
        }
    }

    if (cmdReceived) {
        switch (cmd) {
            case 'H':  // HELLO
                Serial.println("<HELLO=");
                break;

            case 'C': // CLEAR chip
                clearChip();
                break;

            case 'L':  // LOCK chip
                lockChip();
                break;

            case 'U':  // UNLOCK chip
                unlockChip();
                break;

            case 'R':  // READ chip
                readChip();
                break;

            case 'W':  // WRITE chip
            {
                if (writeByte(address, data))
                    Serial.println("<OK=");
                else
                    Serial.println("<ERROR=");
            }
                break;
        }
        cmdReceived = false;
        pos = -1;
    }
}
