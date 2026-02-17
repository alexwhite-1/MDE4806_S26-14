/*
 *   Pin 13 (SCK)  -> (SCK)
 *   Pin 11 (MOSI) -> (MOSI)
 *   Pin 12 (MISO) -> (MISO)
 *   Pin 10 (SS)   -> (CS)
 *   3.3V          -> (VDD)
 *   GND           -> (GND)
 */

#include <SPI.h>

// --- Pin Definitions ---
#define CS_PIN 10

// --- Register Addresses ---
#define REG_SELF_TEST_X   0x00
#define REG_SELF_TEST_Y   0x01
#define REG_SELF_TEST_Z   0x02
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_LP_MODE_CFG   0x1E
#define REG_FIFO_EN       0x23
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_INT_STATUS    0x3A
#define REG_TEMP_OUT_H    0x41
#define REG_TEMP_OUT_L    0x42
#define REG_GYRO_XOUT_H   0x43
#define REG_GYRO_XOUT_L   0x44
#define REG_GYRO_YOUT_H   0x45
#define REG_GYRO_YOUT_L   0x46
#define REG_GYRO_ZOUT_H   0x47
#define REG_GYRO_ZOUT_L   0x48
#define REG_SIGNAL_PATH_RESET 0x68
#define REG_USER_CTRL     0x6A
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_WHO_AM_I      0x75

// --- Expected WHO_AM_I value ---
#define IAM20380_WHO_AM_I 0xB5

// --- Full-Scale Range Options ---
#define GYRO_FS_250DPS    0x00  // ±250 dps,  sensitivity = 131   LSB/dps
#define GYRO_FS_500DPS    0x01  // ±500 dps,  sensitivity = 65.5  LSB/dps
#define GYRO_FS_1000DPS   0x02  // ±1000 dps, sensitivity = 32.8  LSB/dps
#define GYRO_FS_2000DPS   0x03  // ±2000 dps, sensitivity = 16.4  LSB/dps

// --- Configuration ---
#define GYRO_FS_SEL       GYRO_FS_250DPS
#define GYRO_SENSITIVITY  131.0f  // LSB per dps for ±250 dps

// Sensitivity lookup (index by FS_SEL)
const float sensitivity_lut[] = { 131.0f, 65.5f, 32.8f, 16.4f };

// --- SPI Helper Functions ---

void spiWrite(uint8_t reg, uint8_t data) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F); 
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t spiRead(uint8_t reg) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80); 
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return val;
}

void spiReadBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(CS_PIN, HIGH);
}

bool iam20380_init() {
  // Reset the device
  spiWrite(REG_PWR_MGMT_1, 0x80); 
  delay(100);

  // Check WHO_AM_I
  uint8_t whoami = spiRead(REG_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  if (whoami != IAM20380_WHO_AM_I) {
    Serial.println("ERROR: WHO_AM_I mismatch! Check wiring.");
    return false;
  }

  // Wake up: clear SLEEP bit, set CLKSEL = 1 (auto-select best clock)
  spiWrite(REG_PWR_MGMT_1, 0x01);
  delay(50);

  // Disable I2C interface (SPI only)
  spiWrite(REG_USER_CTRL, 0x10);  // I2C_IF_DIS = 1

  // Configure gyroscope: set full-scale range, FCHOICE_B = 00 (use DLPF)
  spiWrite(REG_GYRO_CONFIG, (GYRO_FS_SEL << 3));

  // Configure DLPF: BW = 92 Hz, sample rate = 1 kHz
  spiWrite(REG_CONFIG, 0x02);  // DLPF_CFG = 2

  // Set sample rate divider: ODR = 1000 / (1 + 9) = 100 Hz
  spiWrite(REG_SMPLRT_DIV, 9);

  // Enable all gyro axes (default, but explicit)
  spiWrite(REG_PWR_MGMT_2, 0x00);

  // Enable data ready interrupt (optional)
  spiWrite(REG_INT_ENABLE, 0x01);  // DATA_RDY_INT_EN = 1

  delay(50);  // Allow gyro to stabilize

  Serial.println("IAM-20380 initialized successfully.");
  return true;
}

// --- Data Reading Functions ---

// Read all 3 gyro axes in a single burst (6 bytes starting at 0x43)
void readGyro(float *gx, float *gy, float *gz) {
  uint8_t buf[6];
  spiReadBurst(REG_GYRO_XOUT_H, buf, 6);

  int16_t rawX = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t rawY = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t rawZ = (int16_t)((buf[4] << 8) | buf[5]);

  float sens = sensitivity_lut[GYRO_FS_SEL];
  *gx = (float)rawX / sens;
  *gy = (float)rawY / sens;
  *gz = (float)rawZ / sens;
}

// Read temperature
float readTemp() {
  uint8_t buf[2];
  spiReadBurst(REG_TEMP_OUT_H, buf, 2);

  int16_t rawTemp = (int16_t)((buf[0] << 8) | buf[1]);

  // From datasheet: TEMP_degC = ((TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity) + 25
  // RoomTemp_Offset = 0, Temp_Sensitivity = 326.8 LSB/°C
  float temp_c = ((float)rawTemp / 326.8f) + 25.0f;
  return temp_c;
}

// --- Main ---

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // SPI Mode 0 or 3 supported; 4 MHz clock 
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  Serial.println("Starting IAM-20380...");

  if (!iam20380_init()) {
    Serial.println("Init failed. Halting.");
    while (1);
  }
}

void loop() {
  float gx, gy, gz;
  readGyro(&gx, &gy, &gz);
  float temp = readTemp();

  Serial.print(gx, 4);
  Serial.print("\t");
  Serial.print(gy, 4);
  Serial.print("\t");
  Serial.print(gz, 4);
  Serial.print("\t");
  Serial.println(temp, 4);

  delay(100);
}
