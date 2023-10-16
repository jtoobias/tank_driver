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

    mag_offset.x = 0.0;
    mag_offset.y = 0.0;
    mag_offset.z = 0.0;
    mag_scalar.x = 0.0;
    mag_scalar.y = 0.0;
    mag_scalar.z = 0.0;

    accel_offset.x = 0.0;
    accel_offset.y = 0.0;
    accel_offset.z = 0.0;
    accel_scalar.x = 0.0;
    accel_scalar.y = 0.0;
    accel_scalar.z = 0.0;

    gyro_offset.x = 0.0;
    gyro_offset.y = 0.0;
    gyro_offset.z = 0.0;
    gyro_scalar.x = 0.0;
    gyro_scalar.y = 0.0;
    gyro_scalar.z = 0.0;
}

xyzFloat Imu::read_gyro(void){
  xyzFloat raw_gyro;
  gyro.readGyro(&raw_gyro.x,&raw_gyro.y,&raw_gyro.z);
  return raw_gyro;
}

xyzFloat Imu::read_accel(void){
  return accel.getRawValues();
}

xyzFloat Imu::read_mag_raw(void){
  xyzFloat raw_mag;
   /* Get a new sensor event */ 
  sensors_event_t event; 
  raw_mag.x = event.magnetic.x;
  raw_mag.y = event.magnetic.y;
  raw_mag.z = event.magnetic.z;
  return raw_mag;
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

void Imu::calibrateMagnetometer(void) {
  Serial.println("Calibrating Magnetometer...");
  float minX = 0.0, maxX = 0.0,minY = 0.0, maxY = 0.0,minZ = 0.0, maxZ = 0.0;
  float start_heading = read_mag();
  float fValue = 0.0;
  bool fin = 0;
  uint32_t start_time = millis();
  // Perform magnetometer calibration routine
  // Record min and max values over a period of time
  // Adjust the following loop duration based on your needs
  while (fin == 0) {
    sensors_event_t event;
    mag.getEvent(&event);
    
    // Check if magnetometer values are outside expected range
    // Update min and max values accordingly
    // You may need to adjust the range based on your use case
    if (event.magnetic.x < minX) minX = event.magnetic.x;
    if (event.magnetic.y < minY) minY = event.magnetic.y;
    if (event.magnetic.z < minZ) minZ = event.magnetic.z;
    
    if (event.magnetic.x > maxX) maxX = event.magnetic.x;
    if (event.magnetic.y > maxY) maxY = event.magnetic.y;
    if (event.magnetic.z > maxZ) maxZ = event.magnetic.z;

    delay(10); // Adjust delay based on your application requirements
    fValue += abs(read_mag() - start_heading);
      // 1.5 turns
    if (fValue > 9.42) {
      fin = 1;
      break;
    }

    if ((millis()-start_time) > float(5.0*1000.0*60.0)){
      Serial.println("Magnetometer Calibration Failed. Due to Timeout");
      return;
    }
  }
  if (fin == 1){
    // Calculate calibration offsets and scale factors
    mag_offset.x = (minX + maxX) / 2.0;
    mag_offset.y = (minY + maxY) / 2.0;
    mag_offset.z = (minZ + maxZ) / 2.0;

    mag_scalar.x = 2.0 / (maxX - minX);
    mag_scalar.y = 2.0 / (maxY - minY);
    mag_scalar.z = 2.0 / (maxZ - minZ);

    Serial.println("Magnetometer Calibration Complete.");
    return;
  } else {
    Serial.println("Magnetometer Calibration Failed.");
    return;
  }
}
