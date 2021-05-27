#include <Wire.h>

#define Addr 0x40
#define _1 0x01
#define _2 0x02
#define _3 0x03
#define _4 0x04
#define _5 0x05
#define _6 0x06
#define _7 0x07
#define _8 0x08
#define _9 0x09
#define _0 0x00
#define _DOT 0xFF
const uint8_t dataPin =  10;
const uint8_t sclkPin =  11;
const uint32_t TRHSTEP   = 5000UL;

float temperature;
bool isAddress = true;
byte address;
bool isCommand = false;
byte command;
byte data;

void setWriteModeRS485() {
  PORTD |= 1 << PD2;
  delay(1);
}

ISR(USART_TX_vect) {

  PORTD &= ~(1 << PD2);
}

String  stringOne;
byte message[] = {};
String dataT[] = {};
int writeData(float temp) {


  stringOne = String(temp);

  byte message[stringOne.length() + 2] = {};
  for (int i = 0; i < stringOne.length(); i++) {
    if (stringOne[i] == '0') {

      message[i] = _0;
    }
    if (stringOne[i] == '1') {

      message[i] = _1;
    }
    if (stringOne[i] == '2') {
      message[i] = _2;
    }
    if (stringOne[i] == '3') {
      message[i] = _3;
    }
    if (stringOne[i] == '4') {
      message[i] = _4;
    }
    if (stringOne[i] == '5') {
      message[i] = _5;
    }
    if (stringOne[i] == '6') {
      message[i] = _6;
    }
    if (stringOne[i] == '7') {
      message[i] = _7;
    }
    if (stringOne[i] == '8') {

      message[i] = _8;
    }
    if (stringOne[i] == '9') {
      message[i] = _9;
    }
    if (stringOne[i] == '.') {

      message[i] = _DOT;

    }

  }

  byte reflected[stringOne.length()];
  for (int i = 0; i < stringOne.length(); i++) {
    reflected[i] = ReverseByte(message[i]);
  }
  message[5] = 0x00;
  message[6] = 0x00;
  unsigned short checkSum = Compute_CRC16(reflected);
  byte firstByteOfCheckSum = (checkSum >> 8) & 0xFF;
  byte secondByteOfCheckSum = checkSum & 0xFF;

  message[stringOne.length()] = firstByteOfCheckSum;
  message[stringOne.length() + 1] = secondByteOfCheckSum;

  for (int sendMess = 0; sendMess < stringOne.length() + 2; sendMess++) {
    Serial.write(message[sendMess]);
  }
}


void setup() {

  delay(1000);

  Wire.begin();

  DDRD = 0b00000111;
  PORTD = 0b11111000;

  Serial.begin(9600, SERIAL_8N1);
  UCSR0B |= (1 << UCSZ02) | (1 << TXCIE0);
  UCSR0A |= (1 << MPCM0);

  delay(1);

  address = 0x7B;

}

void loop() {
  if (Serial.available()) {
    byte inByte = Serial.read();
    if (isAddress) {
      if (address == inByte) {
        isAddress = false;
        isCommand = true;
        UCSR0A &= ~(1 << MPCM0);
      }
    } else if (isCommand) {
      command = inByte;
      isCommand = false;
      if (command = 0xB1) {
        if (inc == 3) {
          inc = 0;
        }
        isAddress = true;
        setWriteModeRS485();
        unsigned int data[2];
        // Start I2C transmission
        Wire.beginTransmission(Addr);
        // Send temperature measurement command, NO HOLD master
        Wire.write(0xF5);
        // Stop I2C transmission
        Wire.endTransmission();
        // Request 2 bytes of data
        Wire.requestFrom(Addr, 2);
        // Read 2 bytes of data
        // temperature msb, temperature lsb
        if (Wire.available() == 2)
        {
          data[0] = Wire.read();
          data[1] = Wire.read();
          // Convert the data
          float temperature = (((data[0] * 256.0 + data[1]) * 125.0) / 65536.0) - 6;
        }
      }
    }
  }



}

unsigned short Compute_CRC16(byte* bytes) {
  const unsigned short generator = 0x589;
  unsigned short crc = 0x0000;

  for (int b = 0; b < 5; b++) {
    crc ^= (unsigned short) (bytes[b] << 8);

    for (int i = 0; i < 8; i++) {
      if ((crc & 0x8000) != 0) {
        crc = (unsigned short) ((crc << 1) ^ generator);
      } else {
        crc <<= 1;
      }
    }
  }
  unsigned short myNewResult = Reflect16(crc);
  return myNewResult;
}


unsigned short Reflect16(unsigned short val) {
  unsigned short resVal = 0;
  for (int i = 0; i < 16; i++) {
    if ((val & (1 << i)) != 0) {
      resVal |= (unsigned short)(1 << (15 - i));
    }
  }
  return resVal;
}

byte ReverseByte(byte b) {
  int a = 0;
  for (int i = 0; i < 8; i++) {
    if ((b & (1 << i)) != 0) {
      a |= 1 << (7 - i);
    }
  }
  return (byte) a;
}

int CombineBytes(byte b1, byte b2) {
  int combined = b1 << 8 | b2;
  return combined;
}
