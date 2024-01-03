#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

double dt;
unsigned long imu_dt;

/* Get a new normalized sensor event */
sensors_event_t accel;
sensors_event_t gyro;

  
void setup_imu(double dt_){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(100);
    }
  }

  Serial.println("MPU6050 Found!");
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();

  imu_dt = (unsigned long) (dt_+1);
  dt = imu_dt * 0.001;
  Serial.print("DeltaT: ");
  Serial.println(imu_dt);
}


void computeAngles(){
  static float old_t = millis();

  unsigned long m = millis();
  unsigned long t = m - old_t;
  
  if(t < imu_dt) return;
  old_t = m;

  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);

  // angular velocity (rad/s) * time (s) = rotation angle (rad)
  // backwards euler integration using the estimare of the previous loop and the new angular velocity reading
  double giroAngleX = gyro.gyro.x * dt;
  double giroAngleY = gyro.gyro.y * dt;
  double giroAngleZ = gyro.gyro.z * dt;

  angleX += giroAngleX;
  angleY += giroAngleY;
  angleZ += giroAngleZ;

  // Get angles from accelerometer
  // https://mwrona.com/posts/accel-roll-pitch/
  double accRoll = atan2(accel.acceleration.y, accel.acceleration.z);
  double accPitch = asin (accel.acceleration.x / sqrt ( accel.acceleration.x*accel.acceleration.x + accel.acceleration.y*accel.acceleration.y + accel.acceleration.z*accel.acceleration.z));

  // A little complementary filter to avoid gyroscope drift
  angleX = 0.96*angleX + 0.04*accRoll;
  angleY = 0.96*angleY + 0.04*accPitch;
  
#ifdef PRINT_SENSOR_DATA
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
 #endif

#ifdef PRINT_ANGLES
#ifdef PRINT_ACC_ANGLES
  Serial.print("\t\\t Roll: ");
  Serial.print(accRoll);
  Serial.print(" \tPitch: ");
  Serial.print(accPitch);
  Serial.println(" radians ");
#endif
  
  Serial.print("\t\tAngle X: ");
  Serial.print(angleX);
  Serial.print(" \tY: ");
  Serial.print(angleY);
  Serial.print(" \tZ: ");
  Serial.print(angleZ);
  Serial.println(" radians ");
  Serial.println();
#endif
}
