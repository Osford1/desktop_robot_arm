#include <AccelStepper.h>
#include <math.h>    // for lround
#include <stdlib.h>  // for strtol

// ===================== PIN MAP (CNC Shield V3 on Arduino UNO) =====================
// Joint 1 (X axis)
const int X_STEP = 2;
const int X_DIR  = 5;

// Joint 2 mapped to A-axis pins (NOTE: some CNC shields don't support independent A)
// If your A socket doesn't respond, this is a SHIELD/JUMPER limitation, not code.
const int J2_STEP = 12;   // A_STEP
const int J2_DIR  = 13;   // A_DIR

// Joint 3 (Z axis)
const int Z_STEP = 4;
const int Z_DIR  = 7;

// Gripper mapped to Y-axis pins (verified working on most CNC shields)
const int G_STEP = 3;     // Y_STEP
const int G_DIR  = 6;     // Y_DIR

// Enable pin (CNC shield often uses D8 for EN, active LOW)
const int EN_PIN = 8;
const bool EN_ACTIVE_LOW = true;

// ===================== MOTOR CONSTANTS =====================
const long MOTOR_STEPS = 200;   // 1.8 deg/step
const long MICROSTEPS  = 8;     // set to match your MS jumpers (1/8 => 8)
const long STEPS_PER_MOTOR_REV = MOTOR_STEPS * MICROSTEPS;

// Gear ratios (motor revs per output rev)
const float GR1 = 25.0;
const float GR2 = 100.0;
const float GR3 = 10.0;

// Steps per output revolution for each joint
const float STEPS_PER_OUTREV_1 = STEPS_PER_MOTOR_REV * GR1;
const float STEPS_PER_OUTREV_2 = STEPS_PER_MOTOR_REV * GR2;
const float STEPS_PER_OUTREV_3 = STEPS_PER_MOTOR_REV * GR3;

// ===================== MOTION TUNING =====================
const float MAX_SPEED_1 = 8000.0;
const float ACCEL_1     = 1600.0;

const float MAX_SPEED_2 = 8000.0;
const float ACCEL_2     = 1600.0;

const float MAX_SPEED_3 = 8000.0;
const float ACCEL_3     = 1600.0;

// Gripper tuning (gentler)
const float MAX_SPEED_G = 400.0;
const float ACCEL_G     = 200.0;

// ===================== STEPPERS =====================
AccelStepper j1(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper j2(AccelStepper::DRIVER, J2_STEP, J2_DIR);
AccelStepper j3(AccelStepper::DRIVER, Z_STEP, Z_DIR);
AccelStepper g (AccelStepper::DRIVER, G_STEP, G_DIR);

// Track target angles (degrees)
float targetDeg1 = 0, targetDeg2 = 0, targetDeg3 = 0;

// Gripper invert flag (software-level inversion; DOES NOT rely on DIR pin inversion)
bool g_inverted = false;

// ===================== HELPERS =====================
long degToSteps(float deg, float stepsPerOutRev) {
  return lround(deg * (stepsPerOutRev / 360.0));
}

void enableDrivers(bool enable) {
  digitalWrite(EN_PIN, EN_ACTIVE_LOW ? (enable ? LOW : HIGH)
                                     : (enable ? HIGH : LOW));
}

// Robust parse (handles negatives reliably; rejects weird minus chars)
long parseLongStrict(const String& s, bool* ok) {
  char* endp = nullptr;
  long v = strtol(s.c_str(), &endp, 10);
  *ok = (endp != s.c_str());   // parsed at least something
  return v;
}

// Tokenizer: get nth space-separated token
String getToken(const String& s, int idx) {
  int start = 0;
  int end = -1;
  for (int i = 0; i <= idx; i++) {
    start = end + 1;
    while (start < (int)s.length() && s.charAt(start) == ' ') start++;
    end = s.indexOf(' ', start);
    if (end == -1) end = s.length();
  }
  if (start >= (int)s.length()) return "";
  return s.substring(start, end);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  enableDrivers(true);

  // Joint 1
  j1.setMaxSpeed(MAX_SPEED_1);
  j1.setAcceleration(ACCEL_1);
  j1.setMinPulseWidth(3);
  j1.setCurrentPosition(0);

  // Joint 2 (A pins)
  j2.setMaxSpeed(MAX_SPEED_2);
  j2.setAcceleration(ACCEL_2);
  j2.setMinPulseWidth(3);
  j2.setCurrentPosition(0);

  // Joint 3
  j3.setMaxSpeed(MAX_SPEED_3);
  j3.setAcceleration(ACCEL_3);
  j3.setMinPulseWidth(3);
  j3.setCurrentPosition(0);

  // Gripper (Y pins)
  g.setMaxSpeed(MAX_SPEED_G);
  g.setAcceleration(ACCEL_G);
  g.setMinPulseWidth(5);
  g.setCurrentPosition(0);

  Serial.println("Controller is ready!");
  Serial.println("Commands:");
  Serial.println("  J1 <deg>");
  Serial.println("  J2 <deg>   (mapped to A pins 12/13; may not work on clone-only shields)");
  Serial.println("  J3 <deg>");
  Serial.println("  ALL a b c  (example: ALL 0 30 -20)");
  Serial.println("  G <steps>      (relative gripper move; supports negative)");
  Serial.println("  GOTO <steps>   (absolute gripper move; supports negative)");
  Serial.println("  GINV 0/1       (invert gripper direction in software)");
}

// ===================== COMMAND HANDLER =====================
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Echo what we received (helps catch wrong minus characters / line endings)
  Serial.print("RX: ["); Serial.print(line); Serial.println("]");

  line.toUpperCase();

  String t0 = getToken(line, 0);
  if (t0.length() == 0) return;

  // ----- ALL a b c -----
  if (t0 == "ALL") {
    String t1 = getToken(line, 1);
    String t2 = getToken(line, 2);
    String t3 = getToken(line, 3);

    if (t1 == "" || t2 == "" || t3 == "") {
      Serial.println("Bad ALL cmd. Use: ALL a b c");
      return;
    }

    bool ok1, ok2, ok3;
    long a = parseLongStrict(t1, &ok1);
    long b = parseLongStrict(t2, &ok2);
    long c = parseLongStrict(t3, &ok3);
    if (!ok1 || !ok2 || !ok3) {
      Serial.println("Bad ALL numbers. Example: ALL 0 30 -20");
      return;
    }

    targetDeg1 = a; targetDeg2 = b; targetDeg3 = c;

    j1.moveTo(degToSteps(targetDeg1, STEPS_PER_OUTREV_1));
    j2.moveTo(degToSteps(targetDeg2, STEPS_PER_OUTREV_2));
    j3.moveTo(degToSteps(targetDeg3, STEPS_PER_OUTREV_3));

    Serial.print("ALL -> ");
    Serial.print(targetDeg1); Serial.print(", ");
    Serial.print(targetDeg2); Serial.print(", ");
    Serial.println(targetDeg3);
    return;
  }

  // ----- G <steps> (relative gripper) -----
  if (t0 == "G") {
    String t1 = getToken(line, 1);
    if (t1 == "") { Serial.println("Bad G cmd. Use: G <steps>"); return; }

    bool ok;
    long delta = parseLongStrict(t1, &ok);
    if (!ok) { Serial.println("Bad number. Type like: G -200"); return; }

    // Software inversion (reliable even if DIR inversion doesn't)
    if (g_inverted) delta = -delta;

    long newTarget = g.currentPosition() + delta;
    g.moveTo(newTarget);

    Serial.print("G delta="); Serial.print(delta);
    Serial.print(" target="); Serial.println(newTarget);
    return;
  }

  // ----- GOTO <steps> (absolute gripper) -----
  if (t0 == "GOTO") {
    String t1 = getToken(line, 1);
    if (t1 == "") { Serial.println("Bad GOTO cmd. Use: GOTO <steps>"); return; }

    bool ok;
    long pos = parseLongStrict(t1, &ok);
    if (!ok) { Serial.println("Bad number. Type like: GOTO -500"); return; }

    // Apply inversion consistently: inverted means "reverse the coordinate system"
    if (g_inverted) pos = -pos;

    g.moveTo(pos);

    Serial.print("GOTO target="); Serial.println(pos);
    return;
  }

  // ----- GINV 0/1 -----
  if (t0 == "GINV") {
    String t1 = getToken(line, 1);
    if (t1 == "") { Serial.println("Bad GINV cmd. Use: GINV 0 or GINV 1"); return; }

    bool ok;
    long v = parseLongStrict(t1, &ok);
    if (!ok) { Serial.println("Bad GINV value. Use 0 or 1"); return; }

    g_inverted = (v != 0);
    Serial.print("GINV -> "); Serial.println(g_inverted ? "1 (inverted)" : "0 (normal)");
    return;
  }

  // ----- J1/J2/J3 <deg> -----
  if (t0 == "J1" || t0 == "J2" || t0 == "J3") {
    String t1 = getToken(line, 1);
    if (t1 == "") {
      Serial.println("Bad cmd. Use: J1 <deg>, J2 <deg>, J3 <deg>, ALL a b c, G <steps>, GOTO <steps>, GINV 0/1");
      return;
    }

    bool ok;
    long deg = parseLongStrict(t1, &ok);
    if (!ok) {
      Serial.println("Bad number for joint. Example: J2 -15");
      return;
    }

    if (t0 == "J1") {
      targetDeg1 = deg;
      long steps = degToSteps(targetDeg1, STEPS_PER_OUTREV_1);
      j1.moveTo(steps);
      Serial.print("J1 -> "); Serial.print(targetDeg1);
      Serial.print(" deg ("); Serial.print(steps); Serial.println(" steps)");
    } else if (t0 == "J2") {
      targetDeg2 = deg;
      long steps = degToSteps(targetDeg2, STEPS_PER_OUTREV_2);
      j2.moveTo(steps);
      Serial.print("J2 -> "); Serial.print(targetDeg2);
      Serial.print(" deg ("); Serial.print(steps); Serial.println(" steps)");
    } else { // J3
      targetDeg3 = deg;
      long steps = degToSteps(targetDeg3, STEPS_PER_OUTREV_3);
      j3.moveTo(steps);
      Serial.print("J3 -> "); Serial.print(targetDeg3);
      Serial.print(" deg ("); Serial.print(steps); Serial.println(" steps)");
    }
    return;
  }

  Serial.println("Unknown cmd. Use J1/J2/J3, ALL, G, GOTO, or GINV.");
}

// ===================== LOOP =====================
void loop() {
  static String line = "";
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(line);
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }

  j1.run();
  j2.run();
  j3.run();
  g.run();
}
