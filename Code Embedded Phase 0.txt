#include <Arduino.h>

// Sensor order: LEFT to RIGHT.
// BLACK = 0, WHITE = 1.

// ================= SENSOR PINS =================

const uint8_t SENSOR[5] = {4, 5, 6, 7, 15};

// ================= MOTOR PINS =================

// RIGHT WHEELS
const int ENA = 20;
const int IN1 = 21;
const int IN2 = 47;

// LEFT WHEELS
const int ENB = 0;
const int IN3 = 48;
const int IN4 = 45;

// ================= SENSOR PATTERNS =================

const uint8_t CENTER = 0b11011;
const uint8_t WHITE = 0b11111;
const uint8_t BLACK = 0b00000;

const uint8_t RIGHT_CORNER = 0b11000;
const uint8_t LEFT_CORNER = 0b00011;

// ================= SPEED SETTINGS =================

// Reduced by 20% from 130.
const int FORWARD_PWM = 104;

const int CORRECTION_FAR_PWM = 100;
const int CORRECTION_MID_PWM = 85;
const int CORRECTION_NEAR_PWM = 75;

// Reduced by 20% from 115 and 90.
const int TURN_PWM = 92;
const int TURN_ALIGN_PWM = 72;

const int PROBE_PWM = 95;
const int SEARCH_PWM = 90;

// ================= TIMING SETTINGS =================

const uint32_t START_DELAY_MS = 2000;

const uint32_t SENSOR_INTERVAL_MS = 5;
const uint32_t PRINT_INTERVAL_MS = 150;

const uint32_t MOTOR_STOP_MS = 35;

const int CENTER_SAMPLES = 3;
const int CORNER_SAMPLES = 3;
const int BLACK_SAMPLES = 3;
const int FINISH_WHITE_SAMPLES = 6;

const int CORRECTION_CONFIRM_SAMPLES = 2;
const int OPPOSITE_CONFIRM_SAMPLES = 4;

const uint32_t DIRECTION_HOLD_MS = 100;
const uint32_t CORRECTION_SWEEP_MS = 850;

const uint32_t MIN_TURN_MS = 90;
const uint32_t TURN_REARM_MS = 250;

const uint32_t PROBE_MIN_MS = 50;
const uint32_t PROBE_MAX_MS = 550;

// Calibration assumption:
// Short black-then-white event = possible no-forward junction.
// Longer black-then-white event = possible finish marker.
//
// Tune using the printed PROBE DURATION MS.
// This is a timing heuristic, not a guaranteed classification.
const uint32_t FINISH_MARKER_MIN_MS = 250;

const uint32_t LOST_LINE_MS = 180;
const uint32_t SEARCH_SWEEP_MS = 950;
const uint32_t SEARCH_BRAKE_MS = 40;

// ================= ROBOT STATES =================

enum RobotState {
  STARTUP,
  TRACK,
  BRAKE_CORRECTION,
  CORRECTING,
  TURNING,
  PROBE_BLACK,
  FINISH_VERIFY,
  LOST_SEARCH,
  SEARCH_BRAKE,
  FINISHED
};

RobotState state = STARTUP;

// ================= GLOBAL VARIABLES =================

uint8_t pattern = WHITE;

// +1 = RIGHT, -1 = LEFT.
int lastDirection = 1;
int correctionDirection = 1;
int turnDirection = 1;
int searchDirection = 1;

int rightPWM = 0;
int leftPWM = 0;

int centeredCount = 0;
int blackCount = 0;
int cornerCount = 0;
int pendingCorner = 0;
int whiteCount = 0;

// Alternates globally across qualified junction events.
// Resets to RIGHT after power-on or reset.
bool nextNoForwardRight = true;
bool noForwardJunctionCandidate = false;
uint32_t probeDuration = 0;

int correctionCandidateCount = 0;
int correctionCandidateDirection = 0;
int oppositeCount = 0;

bool turnLeftOldLine = false;
bool turnNearCenter = false;

uint32_t startupTime = 0;
uint32_t stateStart = 0;

uint32_t lastScanTime = 0;
uint32_t lastPrintTime = 0;
uint32_t lastRateTime = 0;

uint32_t turnFinishedAt = 0;
uint32_t correctionStart = 0;
uint32_t directionChangedAt = 0;
uint32_t searchStart = 0;
uint32_t lostStart = 0;

uint32_t scanCount = 0;
uint32_t lastRateCount = 0;
uint32_t scansPerSecond = 0;

uint32_t correctionCount = 0;
uint32_t correctionReversals = 0;
uint32_t failedCorrections = 0;

// ================= MOTOR CONTROL =================

void rightDrive(int speed) {
  speed = constrain(speed, -255, 255);

  analogWrite(ENA, 0);

  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, abs(speed));
}

void leftDrive(int speed) {
  speed = constrain(speed, -255, 255);

  analogWrite(ENB, 0);

  if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENB, abs(speed));
}

void setMotors(int right, int left) {
  right = constrain(right, -255, 255);
  left = constrain(left, -255, 255);

  if (right == rightPWM && left == leftPWM) {
    return;
  }

  rightPWM = right;
  leftPWM = left;

  rightDrive(right);
  leftDrive(left);
}

void stopCar() {
  setMotors(0, 0);
}

void moveForward() {
  setMotors(FORWARD_PWM, FORWARD_PWM);
}

void pivot(int direction, int speed) {
  if (direction > 0) {
    setMotors(-speed, speed);
  } else {
    setMotors(speed, -speed);
  }
}

// ================= SENSOR READING =================

uint8_t readSensors() {
  uint8_t value = 0;

  for (int i = 0; i < 5; i++) {
    value = (value << 1) | digitalRead(SENSOR[i]);
  }

  scanCount++;

  return value;
}

// ================= LINE POSITION =================

int getLineDirection(uint8_t p) {
  const int weights[5] = {-2, -1, 0, 1, 2};

  int sum = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    if (((p >> (4 - i)) & 1) == 0) {
      sum += weights[i];
      count++;
    }
  }

  if (count == 0 || sum == 0) {
    return 0;
  }

  return sum > 0 ? 1 : -1;
}

float getLineError(uint8_t p) {
  const int weights[5] = {-2, -1, 0, 1, 2};

  float sum = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    if (((p >> (4 - i)) & 1) == 0) {
      sum += weights[i];
      count++;
    }
  }

  if (count == 0) {
    return 0;
  }

  return sum / count;
}

int correctionSpeed() {
  float error = fabsf(getLineError(pattern));

  if (error <= 0.6f) {
    return CORRECTION_NEAR_PWM;
  }

  if (error <= 1.3f) {
    return CORRECTION_MID_PWM;
  }

  return CORRECTION_FAR_PWM;
}

// ================= HELPERS =================

bool isNearCenter() {
  return pattern == CENTER ||
         pattern == 0b10111 ||
         pattern == 0b11101;
}

int detectCorner() {
  if (pattern == RIGHT_CORNER) {
    return 1;
  }

  if (pattern == LEFT_CORNER) {
    return -1;
  }

  return 0;
}

void changeState(RobotState next, uint32_t now) {
  state = next;
  stateStart = now;
  centeredCount = 0;
}

bool stableCenter() {
  if (pattern == CENTER) {
    if (centeredCount < CENTER_SAMPLES) {
      centeredCount++;
    }
  } else {
    centeredCount = 0;
  }

  return centeredCount >= CENTER_SAMPLES;
}

// ================= RESUME TRACKING =================

void resumeTracking(uint32_t now) {
  stopCar();

  blackCount = 0;
  cornerCount = 0;
  pendingCorner = 0;

  correctionCandidateCount = 0;
  correctionCandidateDirection = 0;
  oppositeCount = 0;

  lostStart = 0;
  turnFinishedAt = now;

  changeState(TRACK, now);
}

// ================= CORRECTION =================

void startCorrection(int direction, uint32_t now) {
  correctionDirection = direction;
  lastDirection = direction;

  correctionStart = now;
  directionChangedAt = now;
  oppositeCount = 0;

  correctionCount++;

  stopCar();
  changeState(BRAKE_CORRECTION, now);
}

void reverseCorrection(uint32_t now) {
  stopCar();

  correctionDirection = -correctionDirection;
  lastDirection = correctionDirection;

  correctionStart = now;
  directionChangedAt = now;
  oppositeCount = 0;

  correctionReversals++;

  changeState(BRAKE_CORRECTION, now);
}

// ================= TURNING =================

void startTurn(int direction, uint32_t now) {
  turnDirection = direction;
  lastDirection = direction;

  turnLeftOldLine = false;
  turnNearCenter = false;

  stopCar();
  changeState(TURNING, now);

  Serial.println(
    direction > 0 ? "TURN RIGHT" : "TURN LEFT"
  );
}

// Used only after the short black-then-white event
// has passed white confirmation.
void startNoForwardJunctionTurn(uint32_t now) {
  int direction = nextNoForwardRight ? 1 : -1;

  nextNoForwardRight = !nextNoForwardRight;

  Serial.print("NO FORWARD: choosing ");
  Serial.print(direction > 0 ? "RIGHT" : "LEFT");

  Serial.print(" | NEXT NO-FORWARD CHOICE: ");
  Serial.println(nextNoForwardRight ? "RIGHT" : "LEFT");

  startTurn(direction, now);
}

// ================= BLACK MARKER PROBE =================

void startBlackProbe(uint32_t now) {
  stopCar();

  noForwardJunctionCandidate = false;
  probeDuration = 0;
  whiteCount = 0;

  changeState(PROBE_BLACK, now);

  Serial.println("BLACK MARKER");
}

// ================= FINISH =================

void finishRobot(uint32_t now) {
  stopCar();

  changeState(FINISHED, now);
  Serial.println("FINISH CONFIRMED");
}

// ================= MAIN CONTROLLER =================

void controlRobot(uint32_t now) {
  bool center = (pattern == CENTER);
  bool allBlack = (pattern == BLACK);
  bool allWhite = (pattern == WHITE);

  if (allBlack) {
    if (blackCount < BLACK_SAMPLES) {
      blackCount++;
    }
  } else {
    blackCount = 0;
  }

  // ---------- FINISHED ----------

  if (state == FINISHED) {
    stopCar();
    return;
  }

  // ---------- ALL-BLACK PRIORITY ----------

  if (allBlack &&
      (state == TRACK ||
       state == BRAKE_CORRECTION ||
       state == CORRECTING ||
       state == LOST_SEARCH ||
       state == SEARCH_BRAKE)) {

    stopCar();

    if (blackCount >= BLACK_SAMPLES) {
      startBlackProbe(now);
    }

    return;
  }

  // ---------- BLACK MARKER INSPECTION ----------

  if (state == PROBE_BLACK) {
    uint32_t elapsed = now - stateStart;

    if (elapsed >= PROBE_MAX_MS) {
      stopCar();

      // Existing timeout behavior: right turn.
      // Does not change the alternating flag.
      startTurn(1, now);
      return;
    }

    setMotors(PROBE_PWM, PROBE_PWM);

    if (allBlack) {
      return;
    }

    if (elapsed < PROBE_MIN_MS) {
      return;
    }

    if (allWhite) {
      stopCar();

      probeDuration = elapsed;

      noForwardJunctionCandidate =
        probeDuration < FINISH_MARKER_MIN_MS;

      Serial.print("PROBE DURATION MS: ");
      Serial.println(probeDuration);

      Serial.println(
        noForwardJunctionCandidate
          ? "VERIFYING: possible no-forward junction"
          : "VERIFYING: possible finish marker"
      );

      whiteCount = 1;

      changeState(FINISH_VERIFY, now);
      return;
    }

    // Mixed pattern: existing right-turn behavior.
    // Does not consume an alternating choice.
    startTurn(1, now);
    return;
  }

  // ---------- WHITE CONFIRMATION ----------

  if (state == FINISH_VERIFY) {
    stopCar();

    if (allWhite) {
      whiteCount++;

      if (whiteCount >= FINISH_WHITE_SAMPLES) {
        if (noForwardJunctionCandidate) {
          startNoForwardJunctionTurn(now);
        } else {
          finishRobot(now);
        }
      }

      return;
    }

    // White did not persist.
    // Do not consume an alternating choice.
    noForwardJunctionCandidate = false;
    whiteCount = 0;

    startTurn(1, now);
    return;
  }

  // ---------- CORNER / INTERSECTION TURN ----------

  if (state == TURNING) {
    uint32_t elapsed = now - stateStart;

    if (!center) {
      turnLeftOldLine = true;
    }

    if (turnLeftOldLine && isNearCenter()) {
      turnNearCenter = true;
    }

    if (turnLeftOldLine &&
        elapsed >= MIN_TURN_MS &&
        stableCenter()) {

      resumeTracking(now);
      return;
    }

    int speed =
      turnNearCenter ? TURN_ALIGN_PWM : TURN_PWM;

    pivot(turnDirection, speed);
    return;
  }

  // ---------- BRAKE BEFORE CORRECTION ----------

  if (state == BRAKE_CORRECTION) {
    stopCar();

    if (stableCenter()) {
      resumeTracking(now);
      return;
    }

    if (now - stateStart >= MOTOR_STOP_MS) {
      changeState(CORRECTING, now);
    }

    return;
  }

  // ---------- ORIGINAL AUTOCORRECTION ----------

  if (state == CORRECTING) {
    if (center) {
      stopCar();

      if (stableCenter()) {
        resumeTracking(now);
      }

      return;
    }

    centeredCount = 0;

    int observedDirection = getLineDirection(pattern);

    if (observedDirection != 0 &&
        observedDirection != correctionDirection) {
      oppositeCount++;
    } else {
      oppositeCount = 0;
    }

    if (oppositeCount >= OPPOSITE_CONFIRM_SAMPLES &&
        now - directionChangedAt >= DIRECTION_HOLD_MS) {

      reverseCorrection(now);
      return;
    }

    if (now - correctionStart >= CORRECTION_SWEEP_MS) {
      failedCorrections++;

      reverseCorrection(now);
      return;
    }

    pivot(correctionDirection, correctionSpeed());
    return;
  }

  // ---------- LOST-LINE SEARCH ----------

  if (state == LOST_SEARCH) {
    if (center) {
      stopCar();

      if (stableCenter()) {
        resumeTracking(now);
      }

      return;
    }

    centeredCount = 0;

    if (!allWhite && !allBlack) {
      int direction = getLineDirection(pattern);

      if (direction == 0) {
        direction = searchDirection;
      }

      startCorrection(direction, now);
      return;
    }

    if (now - searchStart >= SEARCH_SWEEP_MS) {
      stopCar();

      searchDirection = -searchDirection;
      lastDirection = searchDirection;

      changeState(SEARCH_BRAKE, now);
      return;
    }

    pivot(searchDirection, SEARCH_PWM);
    return;
  }

  // ---------- SEARCH BRAKE ----------

  if (state == SEARCH_BRAKE) {
    stopCar();

    if (center) {
      if (stableCenter()) {
        resumeTracking(now);
      }

      return;
    }

    centeredCount = 0;

    if (!allWhite && !allBlack) {
      int direction = getLineDirection(pattern);

      if (direction == 0) {
        direction = searchDirection;
      }

      startCorrection(direction, now);
      return;
    }

    if (now - stateStart >= SEARCH_BRAKE_MS) {
      searchStart = now;

      changeState(LOST_SEARCH, now);
    }

    return;
  }

  // ---------- NORMAL TRACKING ----------

  if (state == TRACK) {
    if (center) {
      cornerCount = 0;
      pendingCorner = 0;

      correctionCandidateCount = 0;
      correctionCandidateDirection = 0;

      lostStart = 0;

      if (stableCenter()) {
        moveForward();
      } else {
        stopCar();
      }

      return;
    }

    stopCar();
    centeredCount = 0;

    // Forced corners retain their original behavior.
    // They do not change the alternating flag.
    int corner = detectCorner();

    if (corner != 0 &&
        now - turnFinishedAt >= TURN_REARM_MS) {

      if (pendingCorner == corner) {
        cornerCount++;
      } else {
        pendingCorner = corner;
        cornerCount = 1;
      }

      if (cornerCount >= CORNER_SAMPLES) {
        startTurn(corner, now);
        return;
      }

      return;
    } else {
      pendingCorner = 0;
      cornerCount = 0;
    }

    if (allWhite) {
      correctionCandidateCount = 0;

      if (lostStart == 0) {
        lostStart = now;
      }

      if (now - lostStart >= LOST_LINE_MS) {
        searchDirection = lastDirection;
        searchStart = now;

        changeState(LOST_SEARCH, now);
      }

      return;
    }

    lostStart = 0;

    int direction = getLineDirection(pattern);

    if (direction == 0) {
      direction = lastDirection;
    }

    if (direction == correctionCandidateDirection) {
      correctionCandidateCount++;
    } else {
      correctionCandidateDirection = direction;
      correctionCandidateCount = 1;
    }

    if (correctionCandidateCount >=
        CORRECTION_CONFIRM_SAMPLES) {

      startCorrection(direction, now);
      return;
    }

    return;
  }
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(SENSOR[i], INPUT);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  rightPWM = 0;
  leftPWM = 0;

  startupTime = millis();
  lastRateTime = startupTime;

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32-S3 SMART MAZE ROBOT");
  Serial.println("PRECISION CORRECTION ENABLED");
  Serial.println("AUTOMATIC START");

  Serial.println(
    "NO-FORWARD JUNCTIONS: RIGHT, LEFT, RIGHT, LEFT"
  );

  Serial.println("================================");
}

// ================= MAIN LOOP =================

void loop() {
  uint32_t now = millis();

  if (state == STARTUP) {
    stopCar();

    if (now - startupTime < START_DELAY_MS) {
      return;
    }

    changeState(TRACK, now);
    Serial.println("ROBOT STARTED");
  }

  if (now - lastScanTime < SENSOR_INTERVAL_MS) {
    return;
  }

  lastScanTime = now;

  pattern = readSensors();

  controlRobot(now);

  if (now - lastRateTime >= 1000) {
    uint32_t elapsed = now - lastRateTime;
    uint32_t difference = scanCount - lastRateCount;

    scansPerSecond =
      difference * 1000UL / elapsed;

    lastRateCount = scanCount;
    lastRateTime = now;
  }

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    for (int i = 4; i >= 0; i--) {
      Serial.print((pattern >> i) & 1);
    }

    Serial.print(" | STATE: ");
    Serial.print((int)state);

    Serial.print(" | R: ");
    Serial.print(rightPWM);

    Serial.print(" | L: ");
    Serial.print(leftPWM);

    Serial.print(" | SCANS/S: ");
    Serial.print(scansPerSecond);

    Serial.print(" | CORRECTIONS: ");
    Serial.print(correctionCount);

    Serial.print(" | REVERSALS: ");
    Serial.print(correctionReversals);

    Serial.print(" | FAILED: ");
    Serial.println(failedCorrections);
  }
}