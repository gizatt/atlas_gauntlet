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
  int us_target; // Current target
  float start_opening_phase; // At what phase value this servo start to transition from closed to open
  float end_opening_phase; // At what phase value this servo finishes transitioning from closed to open
};
// Servo configurations array
ServoConfig servo_configs[] = {
    {0, 2000, 1250, 2000, 2000, 0.1, 0.2}, // Front left flap
    {1, 1500, 2250, 1500, 1500, 0.1, 0.2}, // Front right flap
    {2, 1500, 2000, 1500, 1500, 0.45, 0.55}, // Middle wrist flap
    {4, 1900, 1750, 1900, 1900, 0.8, 0.9}, // Back right flap
    {5, 1200, 1350, 1200, 1200, 0.8, 0.9}, // Back left flap
    {8, 800, 2250, 1500, 1500, 0.1, 0.9}, // Power / gas gauge
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

float flap_opening_phase = 0.0;
const float PHASE_RAMP_UP_TIME_S = 0.75;
const float PHASE_RAMP_DOWN_TIME_S = 0.5;

// Motion control tunable parameters
const float MOTION_TRIGGER_GYRO_THRESH = 250.0;
const float MOTION_TRIGGER_WINDOW_S = 1.0;           // Time window for gyro flip detection (gyro_y: +GYRO_THRESH to -GYRO_THRESH)
const float MOTION_TRIGGER_DELAY_S = 0.0;            // Delay after motion settles before triggering button press
const float MOTION_TRIGGER_DURATION_S = 2.0;         // How long the motion trigger simulates button press

// Motion control state variables
static unsigned long last_gyro_positive_time_ms = 0; // Last time gyro_y was > MOTION_TRIGGER_GYRO_THRESH
static unsigned long motion_flip_detected_time_ms = 0; // When flip was detected (gyro_y < -MOTION_TRIGGER_GYRO_THRESH)
static unsigned long motion_settled_time_ms = 0;     // When motion settled back (gyro_y > -MOTION_TRIGGER_GYRO_THRESH)
static unsigned long motion_trigger_time_ms = 0;     // When motion trigger activates
static float gyro_y_min = 0.0;                       // Minimum gyro_y value seen
static float gyro_y_max = 0.0;                       // Maximum gyro_y value seen

void update_servo_targets(double dt) {
  // Updates servo targets based on the button state or motion trigger.
  // Currently, toggling the button state drives an accumulator which ramps a "phase"
  // value from 0 to 1 over a fixed period. As the phase ramps up, sets of servos
  // open in sequence.
  // Motion trigger: if detected, simulates a 2-second button press.

  bool toggle_state = get_debounced_servo_toggle_pin();
  
  // Check if motion trigger is active (within the duration window)
  unsigned long current_time_ms = millis();
  unsigned long time_since_motion_ms = current_time_ms - motion_trigger_time_ms;
  bool motion_active = (time_since_motion_ms < (MOTION_TRIGGER_DURATION_S * 1000));
  
  // Treat motion trigger as equivalent to button press
  if (toggle_state || motion_active){
    flap_opening_phase += dt / PHASE_RAMP_UP_TIME_S;
    flap_opening_phase = min(1.0, flap_opening_phase);
  } else {
    flap_opening_phase -= dt / PHASE_RAMP_DOWN_TIME_S;
    flap_opening_phase = max(0.0, flap_opening_phase);
  }
  
  for (auto &servo_config : servo_configs) {
    float open_amount = (flap_opening_phase - servo_config.start_opening_phase) / (servo_config.end_opening_phase - servo_config.start_opening_phase);
    open_amount = min(1.0, max(0.0, open_amount));
    servo_config.us_target = open_amount * servo_config.open_us + (1.0 - open_amount) * servo_config.closed_us;
  }
}

const float SERVO_SLEW_RATE_US_PER_S = 8000;

void run_servos(double dt) {
  // Drives servos towards their target at a max speed and commits the updated servo commands.
  const int max_step_us = dt * SERVO_SLEW_RATE_US_PER_S;
  for (auto &servo_config : servo_configs) {
    // blend us_now to the target us by a fixed step (up to fixed step per tick)
    int step = min(abs(servo_config.us_target - servo_config.us_now), max_step_us);
    if (servo_config.us_target > servo_config.us_now) {
      servo_config.us_now += step;
    } else if (servo_config.us_target < servo_config.us_now) {
      servo_config.us_now -= step;
    }
    write_servo(servo_config.pin, servo_config.us_now);
  }

  // Unused old dev utility for installing servos
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
void run_neopixel() {
  // --- Tunables ---
  const float   lfo_hz       = 0.6f;  // pulse speed (Hz)
  const float flicker_hz = 10.0f;
  const uint8_t base_value   = 220 + flap_opening_phase * 20;   // base brightness (0–255)
  const uint8_t pulse_depth  = 25;    // how much it swells (0–255)
  const uint16_t hue_blue    = 31800 + flap_opening_phase * 1000; // ~175° on HSV wheel (cyan-blue)
  const uint8_t saturation   = 100;

  // LFO: 0..1 sine
  float t    = millis() * 0.001f;                 // seconds
  float s1 = sinf(TWO_PI * lfo_hz * t);
  float s2 = sinf(TWO_PI * sqrtf(2) * lfo_hz * t);
  float s_base = s1*0.5 + s2 * 0.5; // kinda aperiod-looking sin
  int val  = base_value + (int)(pulse_depth * s_base);
  
  // As phase ramps up, ramp in a negative signal that'll cause a flicker
  float s_flicker = sinf(TWO_PI * flicker_hz * t);
  float flicker_magnitude = (s_flicker * s_flicker) * s_base * flap_opening_phase;
  val -= flicker_magnitude * 100;
  val = constrain(val, 0, 255);

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

void update_motion_controls() {
  // Check for a quick flipping motion: gyro_y going from >+MOTION_TRIGGER_GYRO_THRESH to <-MOTION_TRIGGER_GYRO_THRESH within 1 second,
  // then settling back, with a delay before triggering.
  if (!imu_data.gyro_valid) {
    return;
  }
  
  unsigned long current_time_ms = millis();
  
  // Track min/max gyro_y values
  if (imu_data.gyro_y < gyro_y_min) {
    gyro_y_min = imu_data.gyro_y;
  }
  if (imu_data.gyro_y > gyro_y_max) {
    gyro_y_max = imu_data.gyro_y;
  }
  
  // Track when gyro_y is positive and above threshold
  if (imu_data.gyro_y > MOTION_TRIGGER_GYRO_THRESH) {
    last_gyro_positive_time_ms = current_time_ms;
    // Reset flip detection if we're back to positive
    motion_flip_detected_time_ms = 0;
    motion_settled_time_ms = 0;
  }
  
  // Check if gyro_y is now negative below threshold (flip detected)
  if (imu_data.gyro_y < -MOTION_TRIGGER_GYRO_THRESH && motion_flip_detected_time_ms == 0) {
    // Check if we were positive recently (within the time window)
    unsigned long time_since_positive_ms = current_time_ms - last_gyro_positive_time_ms;
    if (time_since_positive_ms < (MOTION_TRIGGER_WINDOW_S * 1000)) {
      // Flip detected! Record the time but don't trigger yet
      motion_flip_detected_time_ms = current_time_ms;
      Serial.println("*** Flip detected, waiting for settle... ***");
    }
  }
  
  // After flip, wait for motion to settle (gyro_y comes back above -MOTION_TRIGGER_GYRO_THRESH)
  if (motion_flip_detected_time_ms > 0 && motion_settled_time_ms == 0) {
    if (imu_data.gyro_y > -MOTION_TRIGGER_GYRO_THRESH) {
      // Motion has settled
      motion_settled_time_ms = current_time_ms;
      Serial.println("*** Motion settled, starting delay... ***");
    }
  }
  
  // After settling, wait for the delay period before activating trigger
  if (motion_settled_time_ms > 0) {
    unsigned long time_since_settled_ms = current_time_ms - motion_settled_time_ms;
    if (time_since_settled_ms >= (MOTION_TRIGGER_DELAY_S * 1000)) {
      // Only trigger once
      if (motion_trigger_time_ms == 0 || (current_time_ms - motion_trigger_time_ms) > (MOTION_TRIGGER_DURATION_S * 1000)) {
        motion_trigger_time_ms = current_time_ms;
        Serial.println("*** Motion trigger activated! ***");
      }
      // Reset state for next detection
      motion_flip_detected_time_ms = 0;
      motion_settled_time_ms = 0;
    }
  }
}

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

static unsigned long last_loop_t_ms = 0;

void loop() {
  // Calculate loop rate
  unsigned long t_ms = millis();
  double dt = (t_ms - last_loop_t_ms) / 1000.0; // Convert to seconds
  last_loop_t_ms = t_ms;

  // Read and process IMU data for motion controls
  read_imu();
  update_motion_controls();

  // Update servos
  update_servo_targets(dt);
  run_servos(dt);

  // Update NeoPixel rainbow
  run_neopixel();

  // Small delay to control animation speed
  delay(20);

  // Read battery voltage

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
  Serial.print(" | Gyro Y min/max: ");
  Serial.print(gyro_y_min, 2);
  Serial.print("/");
  Serial.print(gyro_y_max, 2);
  Serial.print(" | Phase: ");
  Serial.print(flap_opening_phase);
  Serial.print(" | dt: ");
  Serial.println(dt);

  // Blink onboard LED at 500ms intervals
  if (millis() - lastBlinkTime >= 500) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlinkTime = millis();
  }
}
