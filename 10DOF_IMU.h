#ifndef IMU_10DOF_h
#define IMU_10DOF_h

// #include Arduino

#include<Wire.h>
#include<ADXL345_WE.h>
#include <DFRobot_ITG3200.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define ADXL345_I2CADDR 0x53  // 0x1D if SDO = HIGH
#define ITG_I2C_ADDR 0x68

class Imu
{
  public:
    Imu();
    void init(void);
    xyzFloat read_accel(void);
    xyzFloat read_gyro(void);
    float read_mag(void);
    xyzFloat read_mag_raw(void);
    void calibrateMagnetometer(void);
    void calibrateAccelerometer(void);
    void calibrateGyro(void);
    void reset_imu_calibration(void);

    float heading;
    xyzFloat mag_scalar;
    xyzFloat mag_offset;
    
    
    xyzFloat raw_accel;
    // xyzFloat accel_scalar;
    xyzFloat accel_offset;

    xyzFloat raw_gyro;
    // xyzFloat gyro_scalar;
    xyzFloat gyro_offset;
    float temperature;
  private:

    // int _pin;
    DFRobot_ITG3200 gyro;
    ADXL345_WE accel;
    Adafruit_HMC5883_Unified mag;
};
// the #include statement and code go here...

#endif