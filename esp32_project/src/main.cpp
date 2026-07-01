#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ---------------- I2C addresses (HW-290 / GY-87) ----------------
#define MPU6050_ADDR   0x68
#define HMC5883L_ADDR  0x1E
#define QMC5883L_ADDR  0x0D
#define QMC5883P_ADDR  0x2C
#define BMP180_ADDR    0x77

// ---------------- MPU6050 registers ----------------
#define PWR_MGMT_1     0x6B
#define USER_CTRL      0x6A
#define INT_PIN_CFG    0x37
#define ACCEL_CONFIG   0x1C
#define GYRO_CONFIG    0x1B
#define ACCEL_XOUT_H   0x3B

// ---------------- generic I2C helpers ----------------
void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void readBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len, true);
  for (size_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ---------------- magnetometer ----------------
enum MagType { MAG_NONE, MAG_HMC5883L, MAG_QMC5883L, MAG_QMC5883P };
MagType magType = MAG_NONE;
uint8_t magChipId = 0;

void magInit() {
  // QMC5883P (newer HW-290 boards) lives at 0x2C, chip ID reg 0x00 == 0x80.
  if (i2cPresent(QMC5883P_ADDR)) {
    readBytes(QMC5883P_ADDR, 0x00, &magChipId, 1);
    magType = MAG_QMC5883P;
    writeReg(QMC5883P_ADDR, 0x29, 0x06); // axis sign definition
    writeReg(QMC5883P_ADDR, 0x0B, 0x08); // Set/Reset On, range 8 Gauss
    writeReg(QMC5883P_ADDR, 0x0A, 0xCF); // OSR, 200Hz ODR, continuous mode
    return;
  }
  // HMC5883L identifies itself as 'H','4','3' in registers 0x0A..0x0C.
  if (i2cPresent(HMC5883L_ADDR)) {
    uint8_t id[3] = {0};
    readBytes(HMC5883L_ADDR, 0x0A, id, 3);
    if (id[0] == 'H' && id[1] == '4' && id[2] == '3') {
      magType = MAG_HMC5883L;
      writeReg(HMC5883L_ADDR, 0x00, 0x70); // 8-sample avg, 15 Hz
      writeReg(HMC5883L_ADDR, 0x01, 0x20); // gain: 1090 LSB/Gauss (+/-1.3Ga)
      writeReg(HMC5883L_ADDR, 0x02, 0x00); // continuous measurement
      return;
    }
  }
  // Many HW-290 boards actually carry a QMC5883L clone at 0x0D.
  if (i2cPresent(QMC5883L_ADDR)) {
    magType = MAG_QMC5883L;
    writeReg(QMC5883L_ADDR, 0x0B, 0x01); // set/reset period
    writeReg(QMC5883L_ADDR, 0x09, 0x1D); // OSR=512, RNG=8G, ODR=200Hz, continuous
    return;
  }
  magType = MAG_NONE;
}

// Returns magnetic field in microtesla.
bool magRead(float &mx, float &my, float &mz) {
  uint8_t d[6] = {0};
  if (magType == MAG_QMC5883P) {
    readBytes(QMC5883P_ADDR, 0x01, d, 6); // little-endian X, Y, Z
    int16_t x = (int16_t)((d[1] << 8) | d[0]);
    int16_t y = (int16_t)((d[3] << 8) | d[2]);
    int16_t z = (int16_t)((d[5] << 8) | d[4]);
    mx = x / 37.5f; my = y / 37.5f; mz = z / 37.5f; // 3750 LSB/G @8G -> uT
    return true;
  }
  if (magType == MAG_HMC5883L) {
    readBytes(HMC5883L_ADDR, 0x03, d, 6); // output order is X, Z, Y
    int16_t x = (int16_t)((d[0] << 8) | d[1]);
    int16_t z = (int16_t)((d[2] << 8) | d[3]);
    int16_t y = (int16_t)((d[4] << 8) | d[5]);
    mx = x / 10.9f; my = y / 10.9f; mz = z / 10.9f; // 1090 LSB/G -> uT
    return true;
  } else if (magType == MAG_QMC5883L) {
    readBytes(QMC5883L_ADDR, 0x00, d, 6); // little-endian X, Y, Z
    int16_t x = (int16_t)((d[1] << 8) | d[0]);
    int16_t y = (int16_t)((d[3] << 8) | d[2]);
    int16_t z = (int16_t)((d[5] << 8) | d[4]);
    mx = x / 30.0f; my = y / 30.0f; mz = z / 30.0f; // 3000 LSB/G @8G -> uT
    return true;
  }
  mx = my = mz = 0.0f;
  return false;
}

// ---------------- BMP180 barometer ----------------
bool bmpPresent = false;
int16_t  bAC1, bAC2, bAC3, bB1, bB2, bMB, bMC, bMD;
uint16_t bAC4, bAC5, bAC6;
const uint8_t BMP_OSS = 0; // oversampling (0..3)

int16_t bmpS16(uint8_t reg) {
  uint8_t d[2] = {0};
  readBytes(BMP180_ADDR, reg, d, 2);
  return (int16_t)((d[0] << 8) | d[1]);
}
uint16_t bmpU16(uint8_t reg) {
  uint8_t d[2] = {0};
  readBytes(BMP180_ADDR, reg, d, 2);
  return (uint16_t)((d[0] << 8) | d[1]);
}

void bmpInit() {
  if (!i2cPresent(BMP180_ADDR)) { bmpPresent = false; return; }
  bmpPresent = true;
  bAC1 = bmpS16(0xAA); bAC2 = bmpS16(0xAC); bAC3 = bmpS16(0xAE);
  bAC4 = bmpU16(0xB0); bAC5 = bmpU16(0xB2); bAC6 = bmpU16(0xB4);
  bB1 = bmpS16(0xB6);  bB2 = bmpS16(0xB8);
  bMB = bmpS16(0xBA);  bMC = bmpS16(0xBC);  bMD = bmpS16(0xBE);
}

// Reads temperature (C) and pressure (hPa) using the Bosch datasheet algorithm.
void bmpRead(float &tempC, float &pressureHpa) {
  // uncompensated temperature
  writeReg(BMP180_ADDR, 0xF4, 0x2E);
  delay(5);
  long UT = (long)bmpU16(0xF6);

  // uncompensated pressure
  writeReg(BMP180_ADDR, 0xF4, 0x34 + (BMP_OSS << 6));
  delay(5 + (3 << BMP_OSS));
  uint8_t p[3] = {0};
  readBytes(BMP180_ADDR, 0xF6, p, 3);
  long UP = (((long)p[0] << 16) | ((long)p[1] << 8) | p[2]) >> (8 - BMP_OSS);

  long X1 = (UT - (long)bAC6) * (long)bAC5 >> 15;
  long X2 = ((long)bMC << 11) / (X1 + bMD);
  long B5 = X1 + X2;
  tempC = ((B5 + 8) >> 4) / 10.0f;

  long B6 = B5 - 4000;
  X1 = ((long)bB2 * (B6 * B6 >> 12)) >> 11;
  X2 = (long)bAC2 * B6 >> 11;
  long X3 = X1 + X2;
  long B3 = ((((long)bAC1 * 4 + X3) << BMP_OSS) + 2) >> 2;
  X1 = (long)bAC3 * B6 >> 13;
  X2 = ((long)bB1 * (B6 * B6 >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  unsigned long B4 = (unsigned long)bAC4 * (unsigned long)(X3 + 32768) >> 15;
  unsigned long B7 = ((unsigned long)UP - B3) * (50000 >> BMP_OSS);
  long pr = (B7 < 0x80000000UL) ? (long)((B7 * 2) / B4) : (long)((B7 / B4) * 2);
  X1 = (pr >> 8) * (pr >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * pr) >> 16;
  pr = pr + ((X1 + X2 + 3791) >> 4);
  pressureHpa = pr / 100.0f; // Pa -> hPa
}

// ---------------- MPU6050 ----------------
void mpuInit() {
  writeReg(MPU6050_ADDR, PWR_MGMT_1, 0x00);   // wake up
  writeReg(MPU6050_ADDR, ACCEL_CONFIG, 0x00); // +/-2g
  writeReg(MPU6050_ADDR, GYRO_CONFIG, 0x00);  // +/-250 deg/s
  // Expose the aux-bus magnetometer on the main I2C bus:
  writeReg(MPU6050_ADDR, USER_CTRL, 0x00);    // disable I2C master
  writeReg(MPU6050_ADDR, INT_PIN_CFG, 0x02);  // I2C_BYPASS_EN = 1
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(100);

  mpuInit();

  // Diagnostic: scan the I2C bus after bypass is enabled.
  Serial.print("# i2c scan:");
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) { Serial.print(" 0x"); Serial.print(a, HEX); }
  }
  Serial.println();

  magInit();
  bmpInit();

  Serial.print("# mag=");
  Serial.print(magType == MAG_HMC5883L ? "HMC5883L"
             : magType == MAG_QMC5883L ? "QMC5883L"
             : magType == MAG_QMC5883P ? "QMC5883P" : "NONE");
  Serial.print(" chipid=0x"); Serial.println(magChipId, HEX);
  Serial.print("# baro=");
  Serial.println(bmpPresent ? "BMP180" : "NONE");
  // CSV header (single line, no leading '#')
  Serial.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                 "mx_uT,my_uT,mz_uT,temp_C,press_hPa,alt_m");
}

void loop() {
  // ---- MPU6050: burst read accel + gyro (same sample instant) ----
  uint8_t data[14];
  readBytes(MPU6050_ADDR, ACCEL_XOUT_H, data, 14);
  int16_t ax = (int16_t)((data[0]  << 8) | data[1]);
  int16_t ay = (int16_t)((data[2]  << 8) | data[3]);
  int16_t az = (int16_t)((data[4]  << 8) | data[5]);
  int16_t gx = (int16_t)((data[8]  << 8) | data[9]);
  int16_t gy = (int16_t)((data[10] << 8) | data[11]);
  int16_t gz = (int16_t)((data[12] << 8) | data[13]);

  float accelX = ax / 16384.0f, accelY = ay / 16384.0f, accelZ = az / 16384.0f;
  float gyroX  = gx / 131.0f,   gyroY  = gy / 131.0f,   gyroZ  = gz / 131.0f;

  // ---- magnetometer (uT) ----
  float mx, my, mz;
  magRead(mx, my, mz);

  // ---- barometer (C, hPa) + altitude (m) ----
  float tempC = 0.0f, pressHpa = 0.0f, altM = 0.0f;
  if (bmpPresent) {
    bmpRead(tempC, pressHpa);
    altM = 44330.0f * (1.0f - powf(pressHpa / 1013.25f, 0.1903f));
  }

  Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                millis(), accelX, accelY, accelZ, gyroX, gyroY, gyroZ,
                mx, my, mz, tempC, pressHpa, altM);

  delay(200);
}
