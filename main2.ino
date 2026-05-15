#include <Servo.h>
#include <math.h>

// ── PINS ──────────────────────────────────────
#define RH_PIN  PA1
#define RK_PIN  PA2
#define RA_PIN  PA3

#define LH_PIN  PA4
#define LK_PIN  PA5
#define LA_PIN  PA6

// ── SERVOS ────────────────────────────────────
Servo RH, RK, RA;
Servo LH, LK, LA;

// ── LEG DIMENSIONS (cm) ───────────────────────
float l1 = 10.0;
float l2 = 10.0;

// ── IK SAFE POSITION FUNCTION ─────────────────
void pos(float x, float z, char leg) {

  // safer than atan(x/z)
  float hipRad2 = atan2(x, z);

  float z2 = z / cos(hipRad2);

  // avoid impossible positions
  z2 = constrain(z2, 1.0, l1 + l2 - 0.5);

  // IK calculations
  float hipRad1 =
    acos((sq(l1) + sq(z2) - sq(l2)) /
    (2 * l1 * z2));

  float kneeRad =
    PI - acos((sq(l1) + sq(l2) - sq(z2)) /
    (2 * l1 * l2));

  float ankleRad =
    PI / 2 + hipRad2 -
    acos((sq(l2) + sq(z2) - sq(l1)) /
    (2 * l2 * z2));

  // convert to degrees
  float hipDeg =
    (hipRad1 + hipRad2) * (180 / PI);

  float kneeDeg =
    kneeRad * (180 / PI);

  float ankleDeg =
    ankleRad * (180 / PI);

  // =================================================
  // LEFT LEG CALIBRATION
  // =================================================

  if (leg == 'L') {

    // yellow hip
    int LH_angle = constrain(hipDeg + 10, 50, 130);

    // gray knee
    int LK_angle = constrain(kneeDeg - 15, 45, 130);

    // gray ankle
    int LA_angle = constrain(ankleDeg + 20, 45, 140);

    LH.write(LH_angle);
    LK.write(LK_angle);
    LA.write(LA_angle);
  }

  // =================================================
  // RIGHT LEG CALIBRATION
  // =================================================

  else if (leg == 'R') {

    // gray hip
    int RH_angle =
      constrain(180 - hipDeg - 10, 45, 130);

    // gray knee
    int RK_angle =
      constrain(180 - kneeDeg + 15, 45, 130);

    // yellow ankle
    int RA_angle =
      constrain(180 - ankleDeg - 20, 45, 140);

    RH.write(RH_angle);
    RK.write(RK_angle);
    RA.write(RA_angle);
  }
}

// ── NEUTRAL ───────────────────────────────────
void standNeutral() {

  pos(0, 18, 'L');
  pos(0, 18, 'R');

  delay(300);
}

// ── WALK ──────────────────────────────────────
void walkForward() {

  // right step
  pos(-3, 16, 'R');

  delay(180);

  standNeutral();

  // left step
  pos(3, 16, 'L');

  delay(180);

  standNeutral();
}

// ── TURN LEFT ─────────────────────────────────
void turnLeft() {

  pos(-2, 17, 'R');

  delay(150);

  standNeutral();
}

// ── TURN RIGHT ────────────────────────────────
void turnRight() {

  pos(2, 17, 'L');

  delay(150);

  standNeutral();
}

// ── KICK ──────────────────────────────────────
void kickBall() {

  Serial.println("KICK");

  // strong left support
  LA.write(135);

  delay(180);

  // backward preload
  RH.write(115);
  RK.write(115);

  delay(180);

  // fast kick
  RH.write(55);

  delay(40);

  RK.write(55);

  delay(180);

  standNeutral();
}

// ── SETUP ─────────────────────────────────────
void setup() {

  Serial.begin(115200);

  RH.attach(RH_PIN);
  RK.attach(RK_PIN);
  RA.attach(RA_PIN);

  LH.attach(LH_PIN);
  LK.attach(LK_PIN);
  LA.attach(LA_PIN);

  standNeutral();

  Serial.println("IK Football Robot Ready");
}

// ── LOOP ──────────────────────────────────────
void loop() {

  if (Serial.available()) {

    char c = Serial.read();

    if (c == 'F') {

      walkForward();
    }

    else if (c == 'L') {

      turnLeft();
    }

    else if (c == 'R') {

      turnRight();
    }

    else if (c == 'K') {

      kickBall();
    }
  }
}