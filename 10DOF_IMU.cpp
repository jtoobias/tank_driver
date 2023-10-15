#include "10DOF_IMU.h"
#include "Arduino.h"

Imu::Imu():gyro(&Wire, ITG_I2C_ADDR),accel(ADXL345_I2CADDR),mag(12345){

}
void Imu::init(void)
{
  if(!accel.init()){
    Serial.println("ADXL345 not connected!");
  }

  accel.setDataRate(ADXL345_DATA_RATE_50);
  accel.setRange(ADXL345_RANGE_4G);
  /* Assign a unique ID to this sensor at the same time */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("HMC5883 not connected!");
  }
}

xyzFloat Imu::read_gyro(void){
  xyzFloat raw_gyro;
  gyro.readGyro(&raw_gyro.x,&raw_gyro.y,&raw_gyro.z);
  return raw_gyro;
}

xyzFloat Imu::read_accel(void){
  return accel.getRawValues();
}

// Return heading in radians
float Imu::read_mag(void){
   /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  // Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  // Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  // float headingDegrees = heading * 180/M_PI; 
  return heading;
}
