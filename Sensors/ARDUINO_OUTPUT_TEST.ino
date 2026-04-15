#include <Wire.h>

#define SLAVE_ADDRESS 0x48
#define BAUDRATE 9600

struct __attribute__((packed)) imu_packet_t {
  uint8_t header;
  uint8_t counter;
  int16_t roll_x100;
  int16_t pitch_x100;
};

volatile bool newPacket = false;
volatile imu_packet_t rxPacket;

void Received(int numBytes)
{
  if (numBytes != sizeof(imu_packet_t)) {
    while (Wire.available()) {
      Wire.read();   // flush bad packet
    }
    return;
  }

  uint8_t *ptr = (uint8_t *)&rxPacket;
  for (int i = 0; i < sizeof(imu_packet_t) && Wire.available(); i++) {
    ptr[i] = Wire.read();
  }

  if (rxPacket.header == 0xAA) {
    newPacket = true;
  }
}


void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(Received);
  Serial.begin(BAUDRATE);
}

void loop() {
  if (newPacket) {
    noInterrupts();
    imu_packet_t localCopy = rxPacket;
    newPacket = false;
    interrupts();

    float roll = localCopy.roll_x100 / 100.0;
    float pitch = localCopy.pitch_x100 / 100.0;

    Serial.print("counter: ");
    Serial.print(localCopy.counter);
    Serial.print("  roll: ");
    Serial.print(roll);
    Serial.print("  pitch: ");
    Serial.println(pitch);
  }
}
