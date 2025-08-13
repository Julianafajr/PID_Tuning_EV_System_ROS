#include <Encoder.h>
#include <Adafruit_MCP4725.h>
#include <math.h>

// === Hardware setup ===
const int encoderPin1A = 2;
const int encoderPin1B = 3;
Encoder motorEncoder(encoderPin1A, encoderPin1B);
Adafruit_MCP4725 dac;
const float faktor_skala = 3.3 / 4095.0;

// === Motion config ===
const float wheel_diameter = 0.36; // meters
const float pulses_per_revolution = 400.0;
const float calibration_factor = 0.258962914; // empirical

// === Control config ===
const float desired_velocity = 3.78;  // m/s
const float max_dac_voltage = 3.3;
const float kp = 0.; // Proportional gain

// === Speed logging ===
float velocity = 0.0;
long oldPosition = 0;
long newPosition = 0;
unsigned long lastTime = 0;
const unsigned long control_interval = 100; // 10 Hz

void setup() {
  Serial.begin(115200);
  dac.begin(0x60);
  oldPosition = motorEncoder.read();
  lastTime = millis();

  Serial.println("Measured_Speed(m/s),Desired_Speed(m/s)");
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= control_interval) {
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    newPosition = motorEncoder.read();
    long deltaTicks = newPosition - oldPosition;
    oldPosition = newPosition;

    float distance = -((float)deltaTicks / pulses_per_revolution) * (PI * wheel_diameter) * calibration_factor;
    velocity = abs(distance / dt); // m/s


    // Send voltage to DAC
    dac.setVoltage(2046, false);

    // === Print only measured and desired speeds ===
    Serial.print(velocity, 2); // measured speed
    Serial.print(",");
    Serial.println(desired_velocity, 2); // constant desired speed
  }
}
