/*
 * Integrated Arduino Code: Autonomous Steering for 4-Fin Projectile Using MPU6050 Feedback + 1.3" OLED Display
 * 
 * This code enables the projectile to "steer" itself using the MPU6050 as the primary sensor for orientation feedback.
 * - **Steering Mechanism**: Closed-loop control where MPU6050 detects current pitch, roll, and yaw deviations from a target (level flight by default).
 *   - Fins deflect proportionally to correct errors (negative feedback): E.g., if nose pitches up (positive pitch detected), vertical fins deflect down to pitch the nose back down.
 *   - This stabilizes and steers the projectile to maintain a straight, level path during flight.
 *   - Proportional gain (Kp) added for tunable responsiveness (higher Kp = more aggressive steering).
 * - **Autonomous Operation**: No external input needed—relies solely on MPU6050. Fixed in "Stabilize" mode (steer to level).
 * - **Yaw Handling**: Gyro-integrated with calibration; holds heading but may drift. For true heading hold, add magnetometer.
 * - **Application**: Ideal for model rockets, gliders, or fin-stabilized projectiles. Ground-test by tilting: Fins should counter to "steer" back to level.
 * 
 * Limitations:
 * - Short-term yaw hold (drifts ~1-2°/s without fusion). For longer flights, integrate a full AHRS library (e.g., Madgwick).
 * - Assumes forward-facing MPU6050 (X-axis along projectile nose).
 * - Flight: Low-speed tests first; adjust Kp and MAX_DEFLECT for your aerodynamics.
 * 
 * Hardware: 4 servos for fins, MPU6050, optional OLED (no button needed).
 * Libraries: Servo (built-in), Adafruit MPU6050, Adafruit SSD1306 (with GFX).
 * 
 * Operation:
 * - Power on: Calibrates, then steers based on MPU6050 tilts/rotations.
 * - Tilt forward: Fins deflect to steer nose down (correction).
 * - Rotate: Horizontal fins steer to counter yaw.
 */

#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>  // For atan2, sqrt, and M_PI

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C  // Common I2C address; try 0x3D if no display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Servo Configuration (One per Fin/Wing for Steering)
Servo fin1Servo, fin2Servo, fin3Servo, fin4Servo;
int fin1Pin = 9;   // Fin 1: Top (Vertical: Pitch/Roll correction)
int fin2Pin = 10;  // Fin 2: Right (Horizontal: Yaw/Roll correction)
int fin3Pin = 11;  // Fin 3: Bottom (Vertical: Pitch/Roll correction)
int fin4Pin = 3;   // Fin 4: Left (Horizontal: Yaw/Roll correction)

// MPU6050 Configuration
Adafruit_MPU6050 mpu;

// Steering Variables
float pitchError = 0.0;  // Detected pitch deviation
float rollError = 0.0;   // Detected roll deviation
float yawError = 0.0;    // Detected yaw deviation (integrated)
unsigned long previousTime = 0;
float yawOffset = 0.0;   // Gyro bias
const float Kp = 1.0;    // Proportional gain (tune: 0.5-2.0 for steering aggressiveness)
const int MAX_DEFLECT = 45;  // Max fin deflection (±45° from neutral)

// Fin Deflections (degrees from 90° neutral)
int fin1Deflect, fin2Deflect, fin3Deflect, fin4Deflect;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Initializing MPU6050-Based Autonomous Steering System...");

  // Initialize Servos (Fins)
  fin1Servo.attach(fin1Pin);
  fin2Servo.attach(fin2Pin);
  fin3Servo.attach(fin3Pin);
  fin4Servo.attach(fin4Pin);
  setAllFinsNeutral();
  delay(500);
  Serial.println("Fins set to neutral (ready for steering).");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    displayError("MPU6050 not found!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found! (Primary steering sensor)");

  // MPU6050 Settings (balanced for steering feedback)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 configured for steering.");

  // Calibrate gyro bias (keep projectile still and level)
  Serial.println("Calibrating MPU6050 gyro (keep still and level for accurate steering)...");
  float gyroSum = 0.0;
  int calSamples = 200;
  for (int i = 0; i < calSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroSum += g.gyro.z;
    delay(10);
  }
  yawOffset = gyroSum / calSamples;
  Serial.print("Yaw offset calibrated: "); Serial.println(yawOffset);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    displayError("OLED not found!");
    while (1);
  }
  Serial.println("OLED initialized for steering feedback.");

  // OLED Startup Display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.cp437(true);
  display.println(F("MPU6050 Steering Ready"));
  display.println(F("Autonomous control via"));
  display.println(F("sensor feedback."));
  display.println(F("Mode: Stabilize"));
  display.display();
  delay(2000);

  previousTime = millis();
  Serial.println("Autonomous steering loop started. MPU6050 will guide fins.");
}

void loop() {
  // Delta time for integration
  unsigned long currentTime = millis();
  float deltaT = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Read MPU6050 (core sensor for steering)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Compute errors (deviations from target: level = 0°)
  pitchError = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180.0 / M_PI;
  rollError = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / M_PI;
  yawError += (g.gyro.z - yawOffset) * deltaT * 180.0 / M_PI;  // Integrate yaw error
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;

  // Compute steering deflections (proportional to errors, for correction)
  // Pitch steering: Vertical fins (1 top, 3 bottom) deflect to counter pitchError
  float pitchSteer = Kp * -pitchError;  // Negative for correction
  fin1Deflect = 90 + constrain(pitchSteer, -MAX_DEFLECT, MAX_DEFLECT);  // Top fin: + for nose down
  fin3Deflect = 90 - constrain(pitchSteer, -MAX_DEFLECT, MAX_DEFLECT);  // Bottom fin: opposing

  // Yaw steering: Horizontal fins (2 right, 4 left) deflect to counter yawError
  float yawSteer = Kp * -yawError;
  fin2Deflect = 90 + constrain(yawSteer, -MAX_DEFLECT, MAX_DEFLECT);  // Right fin: + for right yaw correction
  fin4Deflect = 90 - constrain(yawSteer, -MAX_DEFLECT, MAX_DEFLECT);  // Left fin: opposing

  // Roll steering: Differential on all fins to counter rollError
  float rollSteer = Kp * -rollError;
  fin1Deflect += constrain(rollSteer, -MAX_DEFLECT / 2, MAX_DEFLECT / 2);  // Top: + for right roll correction
  fin2Deflect += constrain(rollSteer, -MAX_DEFLECT / 2, MAX_DEFLECT / 2);  // Right: +
  fin3Deflect -= constrain(rollSteer, -MAX_DEFLECT / 2, MAX_DEFLECT / 2);  // Bottom: -
  fin4Deflect -= constrain(rollSteer, -MAX_DEFLECT / 2, MAX_DEFLECT / 2);  // Left: -

  // Final constrain (safe range: 45-135°)
  fin1Deflect = constrain(fin1Deflect, 90 - MAX_DEFLECT, 90 + MAX_DEFLECT);
  fin2Deflect = constrain(fin2Deflect, 90 - MAX_DEFLECT, 90 + MAX_DEFLECT);
  fin3Deflect = constrain(fin3Deflect, 90 - MAX_DEFLECT, 90 + MAX_DEFLECT);
  fin4Deflect = constrain(fin4Deflect, 90 - MAX_DEFLECT, 90 + MAX_DEFLECT);

  // Actuate fins for steering
  fin1Servo.write(fin1Deflect);
  fin2Servo.write(fin2Deflect);
  fin3Servo.write(fin3Deflect);
  fin4Servo.write(fin4Deflect);

  // Update OLED (steering status)
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("MPU6050 Steering"));

  // Errors (for monitoring steering performance)
  display.print(F("P:")); display.print(pitchError, 0); display.print(F(" Y:")); display.print(yawError, 0); display.println();
  display.print(F("R:")); display.print(rollError, 0); display.println(F(" deg"));

  // Fin Positions
  display.println(F("Fins (deg):"));
  display.print(F("1T:")); display.print(fin1Deflect); display.print(F(" 2R:")); display.print(fin2Deflect); display.println();
  display.print(F("3B:")); display.print(fin3Deflect); display.print(F(" 4L:")); display.print(fin4Deflect); display.println();

  // Mode and Temp (fixed mode)
  display.print(F("Mode: Stabilize"));
  display.print(F(" T:")); display.print(temp.temperature, 0); display.println(F("C"));

  display.display();

  // Serial output (debug steering)
  Serial.print("Errors - P:"); Serial.print(pitchError,1); Serial.print(" Y:"); Serial.print(yawError,1); Serial.print(" R:"); Serial.print(rollError,1);
  Serial.print(" | Fins - 1T:"); Serial.print(fin1Deflect); Serial.print(" 2R:"); Serial.print(fin2Deflect);
  Serial.print(" 3B:"); Serial.print(fin3Deflect); Serial.print(" 4L:"); Serial.println(fin4Deflect);
  Serial.println("Steering Mode: Stabilize");

  delay(50);  // 20Hz update for smooth steering
}

// Neutral position for all fins
void setAllFinsNeutral() {
  fin1Servo.write(90);
  fin2Servo.write(90);
  fin3Servo.write(90);
  fin4Servo.write(90);
}

// Error handler
void displayError(String message) {
  Serial.println(message);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println(F("STEERING ERROR:"));
    display.println(message);
    display.display();
  }
}
