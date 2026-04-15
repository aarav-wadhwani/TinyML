#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <PDM.h>

// Aarav Wadhwani

// -----------------------------
// Thresholds
// Tune these on your board if needed
// -----------------------------
const int MIC_THRESHOLD = 1800;          // microphone activity threshold
const int CLEAR_DARK_THRESHOLD = 80;     // clear channel below this => dark
const float MOTION_THRESHOLD = 0.18;     // accel magnitude change threshold
const int PROX_NEAR_THRESHOLD = 120;     // proximity above this => near

// -----------------------------
// Microphone globals
// -----------------------------
volatile int samplesRead = 0;
short sampleBuffer[256];
int micLevel = 0;

// -----------------------------
// Motion globals
// -----------------------------
float prevAccelMag = 1.0;

// -----------------------------
// PDM callback
// -----------------------------
void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;   // 2 bytes per sample
}

// -----------------------------
// Compute microphone activity
// Simple average absolute amplitude
// -----------------------------
int getMicLevel() {
  if (samplesRead <= 0) {
    return micLevel;
  }

  long sum = 0;
  for (int i = 0; i < samplesRead; i++) {
    sum += abs(sampleBuffer[i]);
  }

  micLevel = sum / samplesRead;
  samplesRead = 0;
  return micLevel;
}

// -----------------------------
// Read clear-channel brightness
// -----------------------------
int getClearValue() {
  int r, g, b, c;

  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b, c);
    return c;
  }

  return -1;
}

// -----------------------------
// Read proximity
// -----------------------------
int getProximityValue() {
  if (APDS.proximityAvailable()) {
    return APDS.readProximity();
  }

  return -1;
}

// -----------------------------
// Compute motion metric from IMU
// Uses change in acceleration magnitude
// -----------------------------
float getMotionMetric() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    float accelMag = sqrt(x * x + y * y + z * z);
    float motionMetric = abs(accelMag - prevAccelMag);
    prevAccelMag = accelMag;

    return motionMetric;
  }

  return 0.0;
}

// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  PDM.onReceive(onPDMdata);

  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to initialize PDM microphone.");
    while (1);
  }

  Serial.println("Smart workspace situation classifier started");
}

// -----------------------------
// Loop
// -----------------------------
void loop() {
  int mic = getMicLevel();
  int clearVal = getClearValue();
  float motion = getMotionMetric();
  int prox = getProximityValue();

  // If a sensor has no fresh reading, keep logic conservative
  if (clearVal < 0) {
    clearVal = 0;
  }

  if (prox < 0) {
    prox = 0;
  }

  // Binary decisions
  int sound = (mic >= MIC_THRESHOLD) ? 1 : 0;
  int dark = (clearVal < CLEAR_DARK_THRESHOLD) ? 1 : 0;
  int moving = (motion >= MOTION_THRESHOLD) ? 1 : 0;
  int near = (prox >= PROX_NEAR_THRESHOLD) ? 1 : 0;

  // Final rule-based classification
  String finalLabel;

  if (sound == 1 && dark == 0 && moving == 1 && near == 1) {
    finalLabel = "NOISY_BRIGHT_MOVING_NEAR";
  }
  else if (sound == 0 && dark == 1 && moving == 0 && near == 1) {
    finalLabel = "QUIET_DARK_STEADY_NEAR";
  }
  else if (sound == 1 && dark == 0 && moving == 0 && near == 0) {
    finalLabel = "NOISY_BRIGHT_STEADY_FAR";
  }
  else {
    finalLabel = "QUIET_BRIGHT_STEADY_FAR";
  }

  // Required Serial Monitor format
  Serial.print("raw,mic=");
  Serial.print(mic);
  Serial.print(",clear=");
  Serial.print(clearVal);
  Serial.print(",motion=");
  Serial.print(motion, 3);
  Serial.print(",prox=");
  Serial.println(prox);

  Serial.print("flags,sound=");
  Serial.print(sound);
  Serial.print(",dark=");
  Serial.print(dark);
  Serial.print(",moving=");
  Serial.print(moving);
  Serial.print(",near=");
  Serial.println(near);

  Serial.print("state,");
  Serial.println(finalLabel);

  delay(250);
}