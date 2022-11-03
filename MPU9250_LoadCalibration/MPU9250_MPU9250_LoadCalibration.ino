#include "MPU9250.h"
#include "eeprom_utils.h"

MPU9250 mpu;

float ax, ay, az, gx, gy, gz, mx, my, mz, la_x, la_y, la_z;
float h, qx, qy, qz, qw, ex, ey, ez, yaw, pitch, roll, t;
float declinationAngle = 0.02234021, heading; // Magnetic Declination in Karachi is 0.02234021 radians, 1.28 degrees.
int headingDegrees;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
        printCalibration();
        loadCalibration();
}

void mpu9250(){ 
      if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            //accelerometer();
            //gyrometer();
            //magnetometer();
            //linear_acc();
            //quaternion();
            //euler();
            //roll_pitch_yaw();
            //temperature();
            mag_heading();
            prev_ms = millis();
        }
    }
}

void loop() {
  mpu9250(); 
}

void accelerometer() {
  ax = mpu.getAccX();
  ay = mpu.getAccY();
  az = mpu.getAccZ();
  Serial.println("AccX: " + String(ax) + " AccY: " + String(ay) + " AccZ: " + String(az));
}

void gyrometer() {
  gx = mpu.getGyroX();
  gy = mpu.getGyroX();
  gz = mpu.getGyroX();
  Serial.println("GyroX: " + String(gx) + " GyroY: " + String(gy) + " GyroZ: " + String(gz));
}

void magnetometer() {
  mx = mpu.getMagX();
  my = mpu.getMagY();
  mz = mpu.getMagZ();
  //Serial.println("MagX: " + String(mx) + " MagY: " + String(my) + " MagZ: " + String(mz));
}

void linear_acc() {
  la_x = mpu.getLinearAccX();
  la_y = mpu.getLinearAccY();
  la_z = mpu.getLinearAccZ();
  Serial.println("LinAccX: " + String(la_x) + " LinAccY: " + String(la_y) + " LinAccZ: " + String(la_z));
}

void quaternion() {
  qx = mpu.getQuaternionX();
  qy = mpu.getQuaternionY();
  qz = mpu.getQuaternionZ();
  qw = mpu.getQuaternionW();
  Serial.println("QuatX: " + String(qx) + " QuatY: " + String(qy) + " QuatZ: " + String(qz) + " QuatW: " + String(qw));
}

void euler() {
  ex = mpu.getEulerX();
  ey = mpu.getEulerX();
  ez = mpu.getEulerZ();
  Serial.println("EulerX: " + String(ex) + " EulerY: " + String(ey) + " EulerZ: " + String(ez));
}

void roll_pitch_yaw() {
  yaw = mpu.getYaw()+46;
  pitch = mpu.getPitch();
  roll = mpu.getRoll();
  Serial.println("Yaw: " + String(yaw) + " Pitch: " + String(pitch) + " Roll: " + String(roll));
}

void temperature(){
  t= mpu.getTemperature();
  Serial.println("Temperature: " + String(t));
}

void mag_heading(){
  headingDegrees = atan2(mpu.getMagY(), mpu.getMagX())/0.0174532925;
  if(headingDegrees<=0)
  {headingDegrees+=360;}
  Serial.println("Heading: " + String(headingDegrees));
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
