#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>


/**
 *
 * Pinout:
 *   PCA9685 on I2C and OE on D20.
 *   VBAT -> 4.7k -> A7 -> 4.7k -> GND voltage divider
 *   D2 -> Neopixel strip
 */

const int PCA9685_OE_PIN = 20; // Output Enable pin for PCA9685
const int NEOPIXEL_PIN = 2;    // NeoPixel data pin
const int NEOPIXEL_COUNT = 63; // Number of pixels in strip (adjust as needed)

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Create NeoPixel object
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Servo pulse width constants (in microseconds)
const int SERVO_MIN = 1000; // Minimum pulse width
const int SERVO_MAX = 2000; // Maximum pulse width
const int SERVO_MID = 1500; // Middle position
const float PWM_FREQ = 50.0;
const float PWM_PERIOD_US = 1E6 / PWM_FREQ;

// flip this to open/close flaps
const int SERVO_TOGGLE_PIN = D3;

// Debouncing variables for servo toggle pin
static bool last_servo_toggle_state = false;
static bool debounced_servo_toggle_state = false;
static unsigned long last_toggle_debounce_time = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce delay

// define a struct with a servo pin (on the 9865), a "closed" us value, and an
// "open" us value.
struct ServoConfig {
  int pin;       // PCA9685 channel/pin number
  int closed_us; // Pulse width in microseconds for closed position
  int open_us;   // Pulse width in microseconds for open position
  int us_now; // Current state, for blending purposes
};
// Servo configurations array
ServoConfig servo_configs[] = {
    {0, 2000, 1250, 2000},
    {1, 1500, 2250, 1500},
    {2, 1500, 2000, 1500},
    {4, 1900, 1700, 1900}, // right flap
    {5, 1200, 1400, 1200}, // left flap
    {8, 800, 2250, 1500}, // power meter, centered when closed
};

// Rainbow variables
uint16_t rainbow_offset = 0;

void write_servo(int servo, int servo_target_us) {
  // Convert microseconds to PWM value
  // PCA9685 uses 12-bit PWM (0-4095) at 50Hz
  // 1500us pulse width = (1500 / 20000) * 4096 = ~307
  int pulseWidth = 4096 * servo_target_us / PWM_PERIOD_US;
  pwm.setPWM(servo, 0, pulseWidth);
}

bool get_debounced_servo_toggle_pin() {
  // Read the current pin state
  bool current_toggle_state = digitalRead(SERVO_TOGGLE_PIN);
  
  // Check if the reading has changed
  if (current_toggle_state != last_servo_toggle_state) {
    // Reset the debouncing timer
    last_toggle_debounce_time = millis();
  }
  
  // If enough time has passed, accept the new reading
  if ((millis() - last_toggle_debounce_time) > DEBOUNCE_DELAY) {
    if (current_toggle_state != debounced_servo_toggle_state) {
      debounced_servo_toggle_state = current_toggle_state;
    }
  }
  
  // Save the current reading for next iteration
  last_servo_toggle_state = current_toggle_state;
  
  return debounced_servo_toggle_state;
}




void run_servos() {
  bool servo_state = get_debounced_servo_toggle_pin();
  
  for (auto &servo_config : servo_configs) {
    int target_us = servo_state ? servo_config.open_us : servo_config.closed_us;
    // blend us_now to the target us by a fixed step (up to fixed step per tick)
    int step = min(abs(target_us - servo_config.us_now), 100);
    if (target_us > servo_config.us_now) {
      servo_config.us_now += step;
    } else if (target_us < servo_config.us_now) {
      servo_config.us_now -= step;
    }
    write_servo(servo_config.pin, servo_config.us_now);
  }

  // // For servos on range 8 to 12, set the servo to 1500
  // for (int i = 8; i < 12; i++) {
  //   write_servo(i, 1500);
  // }

  // // For servos range 12 to 16, set servo to a sin wave from 1000 to 2000
  // for (int i = 12; i < 16; i++) {
  //   float servo_target_us = SERVO_MID + sin(((float)millis()) / 1000.) * 500;
  //   write_servo(i, servo_target_us);
  // }
}

// Subtle neon-blue breath across the whole strip
// Tweak lfo_hz (speed) and pulse_depth (range) to taste.
void run_neopixel() {
  // --- Tunables ---
  const float   lfo_hz       = 0.6f;  // pulse speed (Hz)
  const uint8_t base_value   = 220;   // base brightness (0–255)
  const uint8_t pulse_depth  = 25;    // how much it swells (0–255)
  const uint16_t hue_blue    = 31800; // ~175° on HSV wheel (cyan-blue)
  const uint8_t saturation   = 100;

  // LFO: 0..1 sine
  float t    = millis() * 0.001f;                 // seconds
  float s    = 0.5f * (1.0f + sinf(TWO_PI * lfo_hz * t));  // 0..1
  int   val  = base_value + (int)(pulse_depth * s);
  val        = constrain(val, 0, 255);

  // One color for all pixels (fast)
  uint32_t c = strip.gamma32(strip.ColorHSV(hue_blue, saturation, (uint8_t)val));
  strip.fill(c, 0, strip.numPixels());
  strip.show();
}

// IMU data structure
struct IMUData {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  bool accel_valid, gyro_valid, mag_valid;
};

// Global IMU data buffer
IMUData imu_data = {0};

void read_imu() {
  // Read accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
    imu_data.accel_valid = true;
  } else {
    imu_data.accel_valid = false;
  }

  // Read gyroscope data
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
    imu_data.gyro_valid = true;
  } else {
    imu_data.gyro_valid = false;
  }

  // Read magnetometer data
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
    imu_data.mag_valid = true;
  } else {
    imu_data.mag_valid = false;
  }
}

static unsigned long lastBlinkTime = 0;
static bool ledState = false;

void setup() {
  Serial.begin(9600); // Add this line to initialize Serial

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO_TOGGLE_PIN, INPUT_PULLUP);

  // Initialize IMU
  bool have_imu = false;
  while (!have_imu) {
    have_imu = IMU.begin();
    if (!have_imu) {
      Serial.println("Failed to initialize IMU!");
      delay(100);
    }
  }

  // Initialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // The int.osc. is closer to 27MHz
  pwm.setPWMFreq(PWM_FREQ);             // Servo frequency is typically 50Hz

  // Set up Output Enable pin for PCA9685 (active low)
  pinMode(PCA9685_OE_PIN, OUTPUT);
  digitalWrite(PCA9685_OE_PIN, LOW); // Enable the PCA9685 outputs

  // Initialize NeoPixel strip
  strip.begin();
  strip.show();            // Initialize all pixels to 'off'
  strip.setBrightness(50); // Set brightness to 50% (adjust as needed)
}

static unsigned long lastLoopTime = 0;
static float loopRate = 0.0;

void loop() {
  // Update servos
  run_servos();

  // Update NeoPixel rainbow
  run_neopixel();

  // Read and print IMU data
  read_imu();

  // Small delay to control animation speed
  delay(20);

  // Read battery voltage
  // Calculate loop rate
  unsigned long currentTime = millis();
  if (lastLoopTime != 0) {
    float deltaTime =
        (currentTime - lastLoopTime) / 1000.0; // Convert to seconds
    loopRate = 1.0 / deltaTime;                // Calculate frequency in Hz
  }
  lastLoopTime = currentTime;

  int rawValue = analogRead(A7);
  // Convert to voltage (assuming 3.3V reference and 4.7k+4.7k voltage divider)
  float voltage = (rawValue / 4095.0) * 3.3 * 2.0; // *2 for voltage divider

  // Print battery voltage, IMU gyro values, and loop rate in one line with
  // fixed precision
  Serial.print("Batt: ");
  Serial.print(voltage, 2);
  Serial.print("V | Gyro X: ");
  Serial.print(imu_data.gyro_x, 2);
  Serial.print(" Y: ");
  Serial.print(imu_data.gyro_y, 2);
  Serial.print(" Z: ");
  Serial.print(imu_data.gyro_z, 2);
  Serial.print(" | Loop: ");
  Serial.print(loopRate, 1);
  Serial.println(" Hz");

  // Blink onboard LED at 500ms intervals
  if (millis() - lastBlinkTime >= 500) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlinkTime = millis();
  }
}
