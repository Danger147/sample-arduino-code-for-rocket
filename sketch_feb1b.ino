#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>


Adafruit_MPU6050 mpu;
Servo servoPitch1;  // Pitch servo 1 (e.g., left elevator)
Servo servoPitch2;  // Pitch servo 2 (e.g., right elevator)
Servo servoRoll1;   // Roll servo 1 (e.g., left aileron)
Servo servoRoll2;   // Roll servo 2 (e.g., right aileron)

// Variables for yaw integration
float yaw = 0.0;
unsigned long lastTime = 0;

// Neutral positions (90° for center; adjust based on your linkage)
const int neutral = 90;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("MPU6050 RPY with 4 MG90S for Projectile Control!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Attach servos
  servoPitch1.attach(9);
  servoPitch2.attach(10);
  servoRoll1.attach(11);
  servoRoll2.attach(6);

  // Set to neutral
  servoPitch1.write(neutral);
  servoPitch2.write(neutral);
  servoRoll1.write(neutral);
  servoRoll2.write(neutral);

  Serial.println("Setup complete. Servos at neutral.");
  lastTime = millis();

}
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate Roll and Pitch from accelerometer (in degrees)
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  // Calculate Yaw from gyro integration (in degrees)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  yaw += g.gyro.z * dt * 180.0 / PI;
  lastTime = currentTime;

  // Control gains (adjust for sensitivity; higher = more response)
  float pitchGain = 2.0;  // Degrees of servo deflection per degree of pitch error
  float rollGain = 2.0;
  float yawGain = 1.0;    // For differential on roll servos

  // Desired RPY (set to 0 for level flight; change for maneuvers)
  float desiredPitch = 0.0;
  float desiredRoll = 0.0;
  float desiredYaw = 0.0;

  // Errors
  float pitchError = desiredPitch - pitch;
  float rollError = desiredRoll - roll;
  float yawError = desiredYaw - yaw;

  // Servo deflections (symmetric for pitch, differential for roll/yaw)
  int pitchDeflect = constrain(pitchError * pitchGain, -30, 30);  // Limit to ±30° deflection
  int rollDeflect = constrain(rollError * rollGain, -30, 30);
  int yawDeflect = constrain(yawError * yawGain, -15, 15);        // Smaller for yaw

  // Set servo positions
  servoPitch1.write(neutral + pitchDeflect);  // Both pitch servos move together
  servoPitch2.write(neutral + pitchDeflect);
  servoRoll1.write(neutral + rollDeflect + yawDeflect);  // Differential: one up, one down for roll; add yaw
  servoRoll2.write(neutral - rollDeflect - yawDeflect);

  // Print debug info
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Roll: ");
  Serial.print(roll);
  Serial.print(" | Yaw: ");
  Serial.print(yaw);
  Serial.print(" | Pitch Deflect: ");
  Serial.print(pitchDeflect);
  Serial.print(" | Roll Deflect: ");
  Serial.println(rollDeflect);

  delay(50);  // Faster updates for control; adjust based on servo speed
}