// UNIVERSAL 16-CHANNEL SERVO TESTER v2 — DIRECT PULSE CONTROL (BETTER!)
// Direct pulse control — "5:510" sends exactly 510 → perfect for testing limits
// Optional safety limits — "nolimit" lets you go full 0–4095 if needed
// "all90" uses fixed 375 pulse (standard 90° for most servos — change if yours is different)
// "min150" / "max620" → change safe limits
// "save" prints clean defines
// Works with any servo type (analog/digital)
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);  // change address if needed

// Optional safe limits (you can ignore with direct pulses)
uint16_t PULSE_MIN = 150;   // typical safe min
uint16_t PULSE_MAX = 600;   // typical safe max
bool useLimits = true;      // set false to allow any pulse 0–4095

void setPulse(uint8_t ch, uint16_t pulse) {
  if (useLimits) {
    pulse = constrain(pulse, PULSE_MIN, PULSE_MAX);
  }
  pca.setPWM(ch, 0, pulse);
  Serial.printf("Ch %d → pulse %d\n", ch, pulse);
}

void allCenter() {
  uint16_t center = 375;  // typical 90° pulse (adjust if needed)
  for (uint8_t ch = 0; ch < 16; ch++) {
    setPulse(ch, center);
  }
  Serial.println("All 16 channels → center pulse (375)");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);  // 50Hz standard
  delay(500);
  allCenter();

  Serial.println(F("\nUNIVERSAL SERVO TESTER v2 — DIRECT PULSE MODE"));
  Serial.println(F("Commands:"));
  Serial.println(F("  5:510     → channel 5 to exact pulse 510"));
  Serial.println(F("  all90     → all to center pulse (375)"));
  Serial.println(F("  min150    → set safe min pulse"));
  Serial.println(F("  max600    → set safe max pulse"));
  Serial.println(F("  nolimit   → disable safety limits"));
  Serial.println(F("  limit     → re-enable safety limits"));
  Serial.println(F("  freq60    → change frequency"));
  Serial.println(F("  save      → print current settings"));
  Serial.printf("Current: Limits %s | MIN=%d MAX=%d\n\n", useLimits ? "ON" : "OFF", PULSE_MIN, PULSE_MAX);
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toLowerCase();

  // Direct channel:pulse — highest priority
  if (cmd.indexOf(':') > 0) {
    uint8_t ch = cmd.substring(0, cmd.indexOf(':')).toInt();
    uint16_t p = cmd.substring(cmd.indexOf(':')+1).toInt();
    if (ch < 16) {
      setPulse(ch, p);
    } else {
      Serial.println("Channel must be 0–15");
    }
    return;
  }

  // Text commands
  if (cmd == "all90" || cmd == "center") {
    allCenter();
  }
  else if (cmd.startsWith("min")) {
    PULSE_MIN = cmd.substring(3).toInt();
    Serial.printf("PULSE_MIN = %d\n", PULSE_MIN);
  }
  else if (cmd.startsWith("max")) {
    PULSE_MAX = cmd.substring(3).toInt();
    Serial.printf("PULSE_MAX = %d\n", PULSE_MAX);
  }
  else if (cmd == "nolimit") {
    useLimits = false;
    Serial.println("Safety limits DISABLED — full 0–4095 range");
  }
  else if (cmd == "limit") {
    useLimits = true;
    Serial.println("Safety limits ENABLED");
  }
  else if (cmd.startsWith("freq")) {
    int f = cmd.substring(4).toInt();
    if (f >= 40 && f <= 1000) {
      pca.setPWMFreq(f);
      Serial.printf("Frequency = %d Hz\n", f);
    } else {
      Serial.println("Frequency 40–1000 Hz");
    }
  }
  else if (cmd == "save") {
    Serial.println(F("\nCOPY TO MAIN CODE:"));
    Serial.printf("// Safe pulse range\n");
    Serial.printf("uint16_t PULSE_MIN = %d;\n", PULSE_MIN);
    Serial.printf("uint16_t PULSE_MAX = %d;\n", PULSE_MAX);
    Serial.println("// Use setPulse(ch, pulse) with optional constrain");
  }
  else {
    Serial.println("Unknown command");
  }

  Serial.printf("Limits %s | MIN=%d MAX=%d\n\n", useLimits ? "ON " : "OFF", PULSE_MIN, PULSE_MAX);
}
