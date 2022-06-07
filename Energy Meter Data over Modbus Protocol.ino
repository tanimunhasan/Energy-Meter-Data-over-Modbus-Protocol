#include <ModbusMaster.h>                //need to include Modbus Master library
#include <SoftwareSerial.h>

SoftwareSerial mySerial(A3, A2);

#define MAX485_DE      A4
#define MAX485_RE_NEG  A5

// instantiate ModbusMaster object
ModbusMaster node;
word CRC16 (const byte *nData, word wLength);

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud
  Serial.begin(9600);
  mySerial.begin(9600);

  // Modbus slave ID 1
  node.begin(2, mySerial);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

bool state = true;

void loop()
{
  uint8_t result;
  uint16_t data[6];

  // // Toggle the coil at address 0x0002 (Manual Load Control)
  // result = node.writeSingleCoil(0x0002, state);
  // state = !state;

  // Read 16 registers starting at 0x3100)
  result = node.readHoldingRegisters(0x0061, 10);
  if (result == node.ku8MBSuccess)
  {
    Serial.print("Phase A Voltage: ");
    Serial.println(node.getResponseBuffer(0x00) / 10.0f);
    Serial.print("Phase B Voltage: ");
    Serial.println(node.getResponseBuffer(0x01) / 10.0f);
    Serial.print("Phase C Voltage: ");
    Serial.println(node.getResponseBuffer(0x02) / 10.0f);

    Serial.print("Phase A Current: ");
    Serial.println(node.getResponseBuffer(0x03) / 100.0f);
    Serial.print("Phase B Current: ");
    Serial.println(node.getResponseBuffer(0x04) / 100.0f);
    Serial.print("Phase C Current: ");
    Serial.println(node.getResponseBuffer(0x05) / 100.0f);

  }

  delay(1000);
}
