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
  reset_imu_calibration();
}

void Imu::reset_imu_calibration(void){
  mag_offset.x = 0.0;
  mag_offset.y = 0.0;
  mag_offset.z = 0.0;
  mag_scalar.x = 1.0;
  mag_scalar.y = 1.0;
  mag_scalar.z = 1.0;

  accel_offset.x = 0.0;
  accel_offset.y = 0.0;
  accel_offset.z = 0.0;

  gyro_offset.x = 0.0;
  gyro_offset.y = 0.0;
  gyro_offset.z = 0.0;
}

xyzFloat Imu::read_gyro(void){
  xyzFloat raw_gyro;
  gyro.readGyro(&raw_gyro.x,&raw_gyro.y,&raw_gyro.z);
  raw_gyro.x = raw_gyro.x + gyro_offset.x;
  raw_gyro.y = raw_gyro.y + gyro_offset.y;
  raw_gyro.z = raw_gyro.z + gyro_offset.z;
  return raw_gyro;
}

xyzFloat Imu::read_accel(void){
  xyzFloat xyz;
  xyz = accel.getRawValues();
  xyz.x = xyz.x + accel_offset.x;
  xyz.y = xyz.y + accel_offset.y;
  xyz.z = xyz.z + accel_offset.z;
  return xyz;
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
  xyzFloat xyzValue;
   /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  xyzValue.x = (event.magnetic.x-mag_offset.x)*mag_scalar.x;
  xyzValue.y = (event.magnetic.y-mag_offset.y)*mag_scalar.y;
  xyzValue.z = (event.magnetic.z-mag_offset.z)*mag_scalar.z;
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(xyzValue.y, xyzValue.x);
  
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
  const float full_circle = 2*3.141592654, cal_time = 0.75*1000.0*60.0;
  float minX = 0.0, maxX = 0.0,minY = 0.0, maxY = 0.0,minZ = 0.0, maxZ = 0.0;
  float last_heading = read_mag();
  float fValue = 0.0,fTmp = 0.0,fDiff = 0.0;
  bool fin = 0;
  uint32_t start_time = millis(),lValue = 0;
  reset_imu_calibration();
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
    // fTmp = read_mag();
    // fDiff = fTmp-last_heading;
    // if (fDiff > (full_circle-0.1)) fDiff -= full_circle;
    // else if (fDiff < -(full_circle-0.1)) fDiff += full_circle;

    // Serial.println("Summed Heading: " + String(fValue) + " " + String(fTmp) + " "+ String(last_heading) + " diff: " + String(fDiff));
    // fValue += fDiff;
    // last_heading = fTmp;
    //   // 1.5 turns
    // if ((fValue > 9.42) || (fValue < -9.42)){
    //   fin = 1;
    //   break;
    // }
    if ((lValue = millis()-start_time) > cal_time){
      Serial.println("Magnetometer Calibration Completed. Due to Timeout");
      fin = 1;
    }else{
      Serial.println("Time used: "+ String(lValue) +"/" +String(cal_time));
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

void Imu::calibrateAccelerometer() {
  const int numSamples = 100;
  xyzFloat sumXYZ;
  xyzFloat rawXYZ;
  Serial.println("Calibrating Accelerometer...");
  // Collect accelerometer readings while stationary
  for (int i = 0; i < numSamples; ++i) {
    rawXYZ = read_accel();
    sumXYZ.x += rawXYZ.x;
    sumXYZ.y += rawXYZ.y;
    sumXYZ.z += rawXYZ.z;
    delay(10); // Adjust delay based on your application requirements
  }
  // Calculate average offsets
  accel_offset.x += sumXYZ.x / numSamples;
  accel_offset.y += sumXYZ.y / numSamples;
  accel_offset.z += sumXYZ.z / numSamples;
  Serial.println("Accelerometer Calibration Complete.");
}

void Imu::calibrateGyro() {
  const int numSamples = 100;
  xyzFloat sumXYZ;
  xyzFloat rawXYZ;
  Serial.println("Calibrating Gyroscope...");
  // Collect accelerometer readings while stationary
  for (int i = 0; i < numSamples; ++i) {
    rawXYZ = read_gyro();
    sumXYZ.x += rawXYZ.x;
    sumXYZ.y += rawXYZ.y;
    sumXYZ.z += rawXYZ.z;
    delay(10); // Adjust delay based on your application requirements
  }
  // Calculate average offsets
  gyro_offset.x  += sumXYZ.x / numSamples;
  gyro_offset.y  += sumXYZ.y / numSamples;
  gyro_offset.z  += sumXYZ.z / numSamples;
  Serial.println("Gyroscope Calibration Complete.");
}




