#ifndef _GYROKALMANFILTER_H_
#define _GYROKALMANFILTER_H_

#define SERIAL_SPEED 115200

// MPU-6050 Sensor
#define MPU6050_ACCEL_XOUT_H 0x3B   // R
#define MPU6050_PWR_MGMT_1   0x6B   // R/W
#define MPU6050_PWR_MGMT_2   0x6C   // R/W
#define MPU6050_WHO_AM_I     0x75   // R
#define MPU6050_I2C_ADDRESS  0x68   // I2C Address

// Kalman Filter
struct GyroKalman {
    // Variable to represent state matrix x
    float x_angle, x_bias;

    // Error covariance matrix
    float P_00, P_01, P_10, P_11;

    /******************************************************************
     * Q is 2x2 matriax of covariance
     * Gyro and accelerometer noise is independent of each other
     * x = F(x) + B(u) + w
     * w has a normal distribution with covariance Q
     * covariance = E[(X - E[X]) * (X - E[X])']
     * Covariance R: observation noise from accelerometer (1x1 Matrix)
     ******************************************************************/

    float Q_angle, Q_gyro;
    float R_angle;
};

typedef union accel_t_gyro_union {
    struct {
        uint8_t x_accel_h, x_accel_l;
        uint8_t y_accel_h, y_accel_l;
        uint8_t z_accel_h, z_accel_l;
        uint8_t t_h, t_l;
        uint8_t x_gyro_h, x_gyro_l;
        uint8_t y_gyro_h, y_gyro_l;
        uint8_t z_gyro_h, z_gyro_l;
    } reg;
    struct {
        int x_accel, y_accel, z_accel;
        int temperature;
        int x_gyro, y_gyro, z_gyro;
    } value;
};

// Function Prototype
void gyroInit();
void gyroExecute();
int MPU6050_read(int start, uint8_t *buffer, int size);
int MPU6050_write(int start, const uint8_t *pData, int size);
int MPU6050_write_reg(int reg, uint8_t data);
float angleInDegrees(int lo, int hi, int measured);
void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle);
void predict(struct GyroKalman *kalman, float dotAngle, float dt);
float update(struct GyroKalman *kalman, float angle_m);
int getXcoord();
int getYcoord();
int getZcoord();
void newDelay(int x);


#endif /*_GYROKALMANFILTER_H_*/