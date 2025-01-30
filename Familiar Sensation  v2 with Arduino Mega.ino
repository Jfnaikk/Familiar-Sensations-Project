#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Initialize MPU6050
Adafruit_MPU6050 mpu;

// Motor pins
const int motors[8] = {0, 1, 2, 3, 4, 5, 6, 7};

// Variables for touch simulation
float motorIntensity[8] = {0}; // Stores intensity for each motor
const float smoothingFactor = 0.1; // For smoothing transitions
unsigned long previousMillis = 0;
const int waveSpeed = 50; // Speed of wave motion (ms)

void setup() {
  Serial.begin(115200);

  // Initialize motors as output
  for (int i = 0; i < 8; i++) {
    pinMode(motors[i], OUTPUT);
  }

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 sensor!");
    while (1);
  }
  Serial.println("MPU6050 ready!");
}

void loop() {
  // Get accelerometer and gyroscope data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate motion magnitude using accelerometer
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Calculate rotational intensity using gyroscope
  float gyroX = g.gyro.x;
  float gyroY = g.gyro.y;
  float gyroZ = g.gyro.z;

  // Compute overall acceleration magnitude
  float accelMagnitude = sqrt(sq(accelX) + sq(accelY) + sq(accelZ));
  float gyroMagnitude = sqrt(sq(gyroX) + sq(gyroY) + sq(gyroZ));

  // Normalize intensities
  int motionIntensity = map(constrain(accelMagnitude + gyroMagnitude, 0, 20), 0, 20, 50, 255);

  // Calculate directional scaling
  float xFactor = abs(accelX) / accelMagnitude;
  float yFactor = abs(accelY) / accelMagnitude;
  float zFactor = abs(accelZ) / accelMagnitude;

  // Simulate directional vibration
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= waveSpeed) {
    previousMillis = currentMillis;

    // Assign motor intensity based on direction
    for (int i = 0; i < 4; i++) {
      motorIntensity[i] = xFactor * motionIntensity; // Left-Right motors
    }
    for (int i = 4; i < 8; i++) {
      motorIntensity[i] = yFactor * motionIntensity; // Forward-Backward motors
    }

    // Add Z-axis (up-down) to ripple effect
    for (int i = 0; i < 8; i++) {
      float ripple = (sin(currentMillis / 300.0 + i * 0.5) + 1.0) / 2.0; // Sine wave (0 to 1)
      motorIntensity[i] += zFactor * motionIntensity * ripple;
    }
  }

  // Apply smoothing and write to motors
  for (int i = 0; i < 8; i++) {
    motorIntensity[i] = (1 - smoothingFactor) * motorIntensity[i] + smoothingFactor * motionIntensity;
    analogWrite(motors[i], constrain(motorIntensity[i], 0, 255));
  }

  // Debugging output
  Serial.print("Accel Mag: "); Serial.print(accelMagnitude);
  Serial.print(" Gyro Mag: "); Serial.println(gyroMagnitude);
  delay(20); // Short delay for smooth operation
}
