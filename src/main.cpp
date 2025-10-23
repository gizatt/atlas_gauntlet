#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>

/**
 * 
 * Pinout:
 *   PCA9685 on I2C and OE on D20.
 *   VBAT -> 4.7k -> A7 -> 4.7k -> GND voltage divider
 *   D2 -> Neopixel strip
 */

const int num_servos = 4;
const int PCA9685_OE_PIN = 20;    // Output Enable pin for PCA9685
const int NEOPIXEL_PIN = 2;       // NeoPixel data pin
const int NEOPIXEL_COUNT = 30;    // Number of pixels in strip (adjust as needed)

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Create NeoPixel object
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Servo pulse width constants (in microseconds)
const int SERVO_MIN = 1000;  // Minimum pulse width
const int SERVO_MAX = 2000;  // Maximum pulse width
const int SERVO_MID = 1500;  // Middle position
const float PWM_FREQ = 50.0;
const float PWM_PERIOD_US = 1E6 / PWM_FREQ;

// Rainbow variables
uint16_t rainbow_offset = 0;

void run_servos(){
  // For now just set all servos to 1500 / middle of range
  for(int i = 0; i < num_servos; i++) {
    // Convert microseconds to PWM value
    // PCA9685 uses 12-bit PWM (0-4095) at 50Hz
    // 1500us pulse width = (1500 / 20000) * 4096 = ~307
    float servo_target_us = SERVO_MID + sin(((float)millis()) / 1000.)*500;

    int pulseWidth = 4096 * servo_target_us / PWM_PERIOD_US;
    pwm.setPWM(i, 0, pulseWidth);
  }
}

void run_neopixel(){
  // push a rainbow pattern up the neopixel strip
  for(int i = 0; i < strip.numPixels(); i++) {
    // Calculate hue for this pixel (0-65536 range)
    uint16_t hue = (i * 65536L / strip.numPixels()) + rainbow_offset;
    
    // Convert HSV to RGB and set pixel color
    uint32_t color = strip.gamma32(strip.ColorHSV(hue));
    strip.setPixelColor(i, color);
  }
  
  // Show the colors
  strip.show();
  
  // Move the rainbow pattern
  rainbow_offset += 256; // Adjust speed by changing this value
}

static unsigned long lastBlinkTime = 0;
static bool ledState = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // // Initialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
  pwm.setPWMFreq(PWM_FREQ);  // Servo frequency is typically 50Hz
  
  // Set up Output Enable pin for PCA9685 (active low)
  pinMode(PCA9685_OE_PIN, OUTPUT);
  digitalWrite(PCA9685_OE_PIN, LOW);  // Enable the PCA9685 outputs
  
  // Initialize NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(50); // Set brightness to 50% (adjust as needed)  
}


void loop() {
  // Update servos
  run_servos();
  
  // Update NeoPixel rainbow
  run_neopixel();
  
  // Small delay to control animation speed
  delay(20);

  // Read battery voltage
  int rawValue = analogRead(A7);
  // Convert to voltage (assuming 3.3V reference and 4.7k+4.7k voltage divider)
  float voltage = (rawValue / 1023.0) * 3.3 * 2.0;  // *2 for voltage divider
  Serial.print("Battery voltage: ");
  Serial.print(voltage);
  Serial.println("V");

  // Blink onboard LED at 500ms intervals
  if (millis() - lastBlinkTime >= 500) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlinkTime = millis();
  }
}

