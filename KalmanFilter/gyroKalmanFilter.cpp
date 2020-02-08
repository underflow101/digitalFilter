// Arudino Gyroscope + Kalman Filter (MPU6050)
// Use this code for MCU with MPU6050 or Accelerometer/Gyroscope

#include <math.h>
#include <Wire.h>
#include <Arduino.h>
#include "gyroKalmanFilter.h"

static const float R_angle = 0.3;
static const float Q_angle = 0.01;
static const float Q_gyro = 0.04;

// Value limitation of accelerometer (may vary)
const int lowX = -2150;
const int highX = 2210;
const int lowY = -2150;
const int highY = 2210;
const int lowZ = -2150;
const int highZ = 2550;

// Time
unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;

int xInit[5] = { 0, 0, 0, 0, 0 };
int yInit[5] = { 0, 0, 0, 0, 0 };
int zInit[5] = { 0, 0, 0, 0, 0 };
int initIndex = 0;
int initSize = 5;
int xCal = 0, yCal = 0, zCal = 1800;

// For return
int _x = 0, _y = 0, _z = 0;

struct GyroKalman angX;
struct GyroKalman angY;
struct GyroKalman angZ;

void gyroInit() {
    Serial.begin(SERIAL_SPEED);
    Wire.begin();
    
    int error;
    uint8_t c;

    /************************************
     * Default at power up:
     * Gyro at 250 degrees second
     * Acceleration at 2g
     * Clock source at internal 8MHz
     ***********************************/

    initGyroKalman(&angX, Q_angle, Q_gyro, R_angle);
	initGyroKalman(&angY, Q_angle, Q_gyro, R_angle);
	initGyroKalman(&angZ, Q_angle, Q_gyro, R_angle);

    error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);

    // Uses Serial.print(F()) F() function to use flash memory, not SRAM memory
    Serial.print(F("WHO_AM_I: "));
    Serial.print(c, HEX);
    Serial.print(F(", error = "));
    Serial.println(error, DEC);

    // Clear Sleep Bit to start the MPU6050 sensor
    MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0);
}

void gyroExecute() {
    int error;
    double dT;
    uint8_t tmp;
    accel_t_gyro_union accel_t_gyro;

    curSensoredTime = millis();

    // Read raw values (14 bit) of acceleration/gyro/temperature (Not Stable)
    error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t*)&accel_t_gyro, sizeof(accel_t_gyro));
    if(error != 0) {
        Serial.print(F("Read acceleration, temperature, gyro... error = "));
        Serial.println(error, DEC);
    }

    #define SWAP(x, y) tmp = x; x = y; y = tmp;
    SWAP(accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
    SWAP(accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
    SWAP(accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
    SWAP(accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
    SWAP(accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
    SWAP(accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
    SWAP(accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

    if(prevSensoredTime > 0) {
        int gx1 = 0, gy1 = 0, gz1 = 0;
        float gx2 = 0, gy2 = 0, gz2 = 0;

        int loopTime = curSensoredTime - prevSensoredTime;

        gx2 = angleInDegrees(lowX, highX, accel_t_gyro.value.x_gyro);
        gy2 = angleInDegrees(lowY, highY, accel_t_gyro.value.y_gyro);
        gz2 = angleInDegrees(lowZ, highZ, accel_t_gyro.value.z_gyro);

        predict(&angX, gx2, loopTime);
        predict(&angY, gy2, loopTime);
        predict(&angZ, gz2, loopTime);

        gx1 = update(&angX, accel_t_gyro.value.x_accel) / 10;
        gy1 = update(&angY, accel_t_gyro.value.y_accel) / 10;
        gz1 = update(&angZ, accel_t_gyro.value.z_accel) / 10;

        if(initIndex < initSize) {
            xInit[initIndex] = gx1;
            yInit[initIndex] = gy1;
            zInit[initIndex] = gz1;

            if(initIndex == initSize - 1) {
                int sumX = 0, sumY = 0, sumZ = 0;
                for(int k = 1; k <= initSize; k++) {
                    sumX += xInit[k];
                    sumY += yInit[k];
                    sumZ += zInit[k];
                }
                xCal -= sumX / (initSize - 1);
                yCal -= sumY / (initSize - 1);
                zCal = (sumZ / (initSize - 1) - zCal);
            }
            initIndex++;
        } else {
            gx1 += xCal;
            gy1 += yCal;
        }
        Serial.print(F("Angle x, y, z: "));
        Serial.print(gx1, DEC);
        _x = gx1;
        Serial.print(F(", "));
        Serial.print(gy1, DEC);
        _y = gy1;
        Serial.print(F(", "));
        Serial.print(gz1, DEC);
        _z = gz1;
        Serial.println(F(""));
    }
    prevSensoredTime = curSensoredTime;
    newDelay(200);
}

int getXcoord() {
    return _x;
}

int getYcoord() {
    return _y;
}

int getZcoord() {
    return _z;
}

/***********************
 * Sensor Read / Write
 ***********************/
int MPU6050_read(int start, uint8_t *buffer, int size) {
    int i, n, error;
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);

    n = Wire.write(start);
    if(n != 1) {
        return -10;
    }
    n = Wire.endTransmission(false);
    if(n != 0) {
        return n;
    }
    Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
    i = 0;
    while(Wire.available() && i < size) {
        buffer[i++] = Wire.read();
    }
    if(i != size) {
        return -11;
    }
    return 0;
}

int MPU6050_write(int start, const uint8_t *pData, int size) {
    int n, error;
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);

    n = Wire.write(start);
    if(n != 1) {
        return -20;
    }
    n = Wire.write(pData, size);
    if(n != size) {
        return -21;
    }
    error = Wire.endTransmission(true);
    if(error != 0) {
        return error;
    }
    return 0;
}

int MPU6050_write_reg(int reg, uint8_t data) {
    int error;
    error = MPU6050_write(reg, &data, 1);
    return error;
}

/************************
 * Raw data processing
 ************************/
float angleInDegrees(int lo, int hi, int measured) {
    float x = (hi - lo) / 180.0;
    return (float)measured / x;
}

void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle) {
    kalman->Q_angle = Q_angle;
    kalman->Q_gyro = Q_gyro;
    kalman->R_angle = R_angle;

    kalman->P_00 = 0;
    kalman->P_01 = 0;
    kalman->P_10 = 0;
    kalman->P_11 = 0;
}

/********************************************************************************************
 * Kalman Predict Method
 * kalman: the kalman data structure
 * dotAngle: Derivitive Of The (D O T) Angle. This is the change in the angle from the gyro.
 *           This is the value from the Wii Motion Plus, scaled to fast/slow.
 * dt: the change in time, in sec.
 ********************************************************************************************/
void predict(struct GyroKalman *kalman, float dotAngle, float dt) {
    kalman->x_angle += dt * (dotAngle - kalman->x_bias);
    kalman->P_00 += (-1) * dt * (kalman->P_10 + kalman->P_01) + dt * dt * kalman->P_11 + kalman->Q_angle;
    kalman->P_01 += (-1) * dt * kalman->P_11;
    kalman->P_10 += (-1) * dt * kalman->P_11;
    kalman->P_11 += kalman->Q_gyro;
}

float update(struct GyroKalman *kalman, float angle_m) {
    const float y = angle_m - kalman->x_angle;
    const float S = kalman->P_00 + kalman->R_angle;
    const float K_0 = kalman->P_00 / S;
    const float K_1 = kalman->P_10 / S;
    kalman->x_angle += K_0 * y;
    kalman->x_bias += K_1 * y;
    kalman->P_00 -= K_0 * kalman->P_00;
    kalman->P_01 -= K_0 * kalman->P_01;
    kalman->P_10 -= K_1 * kalman->P_00;
    kalman->P_11 -= K_1 * kalman->P_01;

    return kalman->x_angle;
}

/**************
 * Utility
 **************/
void newDelay(int x) {
    // Delay function for x milliseconds which does not interfere with timer0
    for(int i = 0; i < x; i++) delayMicroseconds(1000);
}
