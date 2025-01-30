#include <Adafruit_CircuitPlayground.h>

#define HAPTIC_MOTOR_1 0  // Pin 0 for Motor 1
#define HAPTIC_MOTOR_2 1  // Pin 1 for Motor 2

void setup() {
  // Initialize Circuit Playground
  CircuitPlayground.begin();

  // Set motor pins as outputs
  pinMode(HAPTIC_MOTOR_1, OUTPUT);
  pinMode(HAPTIC_MOTOR_2, OUTPUT);

  // Start Serial Communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read accelerometer data
  float x = CircuitPlayground.motionX();  // Left/Right
  float y = CircuitPlayground.motionY();  // Up/Down

  // Map motion values to intensity (0 to 255)
  int intensityX = map(abs(x), 0, 10, 0, 255);  // Left/Right motion intensity
  int intensityY = map(abs(y), 0, 10, 0, 255);  // Up/Down motion intensity

  // Threshold for activation
  int threshold = 50;

  if (intensityX > threshold || intensityY > threshold) {
    if (abs(y) > abs(x)) {
      // Up/Down motion: Both motors vibrate strongly
      int dynamicIntensity = map(abs(y), 0, 10, 100, 255);  // Adjust range for smoother vibrations
      analogWrite(HAPTIC_MOTOR_1, dynamicIntensity);
      analogWrite(HAPTIC_MOTOR_2, dynamicIntensity);

      // Light NeoPixels from bottom to top
      setNeoPixelsForUpDown(dynamicIntensity);
    } else {
      // Left/Right motion: Vary motor intensities for spatial effect
      int motor1Intensity = map(x, -10, 10, 0, 255);  // Left is weak, Right is strong
      int motor2Intensity = map(x, -10, 10, 255, 0);  // Right is weak, Left is strong
      analogWrite(HAPTIC_MOTOR_1, constrain(motor1Intensity, 0, 255));
      analogWrite(HAPTIC_MOTOR_2, constrain(motor2Intensity, 0, 255));

      // Light NeoPixels left-to-right or right-to-left
      setNeoPixelsForLeftRight(x);
    }
  } else {
    // Stop vibrations if no significant motion is detected
    analogWrite(HAPTIC_MOTOR_1, 0);
    analogWrite(HAPTIC_MOTOR_2, 0);

    // Turn off NeoPixels
    CircuitPlayground.clearPixels();
  }

  // Debugging: Print accelerometer data
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" | Y: ");
  Serial.print(y);
  Serial.println();

  delay(50);  // Small delay for smooth response
}

// Function to light NeoPixels for Up/Down motion
void setNeoPixelsForUpDown(int intensity) {
  int numPixels = map(intensity, 0, 255, 0, 10);  // Map intensity to NeoPixels
  for (int i = 0; i < 10; i++) {
    if (i < numPixels) {
      CircuitPlayground.setPixelColor(i, CircuitPlayground.colorWheel(map(i, 0, 10, 0, 255)));
    } else {
      CircuitPlayground.setPixelColor(i, 0);  // Turn off the rest
    }
  }
}

// Function to light NeoPixels for Left/Right motion
void setNeoPixelsForLeftRight(float x) {
  int direction = x > 0 ? 1 : -1;  // Determine direction
  int intensity = map(abs(x), 0, 10, 0, 255);  // Map intensity
  int numPixels = map(abs(x), 0, 10, 0, 10);   // Determine how many NeoPixels to light

  for (int i = 0; i < 10; i++) {
    if (direction > 0 && i < numPixels) {
      CircuitPlayground.setPixelColor(i, intensity, 0, 0);  // Red for right
    } else if (direction < 0 && i >= (10 - numPixels)) {
      CircuitPlayground.setPixelColor(i, 0, 0, intensity);  // Blue for left
    } else {
      CircuitPlayground.setPixelColor(i, 0);  // Turn off the rest
    }
  }
}
