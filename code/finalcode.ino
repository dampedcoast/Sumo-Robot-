#include <SharpIR.h>

#define IR_PIN A1
#define MODEL SharpIR::GP2Y0A41SK0F   // <-- must match YOUR sensor's model

SharpIR sensor(MODEL, IR_PIN);   // model first, pin second


enum Direction { DIR_RIGHT, DIR_FRONT, DIR_LEFT, DIR_NONE };
enum Strategy  { Normal, Flags, Flank, Searching, SitandSearch, SitandWait };
Direction last_seen = DIR_NONE;
Strategy strategy = Normal;
bool startRight = true;
bool turned = false;
bool searching = false;
#define IR_right   12
#define IR_front   14
#define IR_left    1

#define QTR_right A4
#define QTR_left  A5
#define WHITE_THRESHOLD 150

/* ============ MOTOR WIRING (confirmed via bench test) ============ */
#define left_motor_DIR   5
#define left_motor_PWM   9
#define right_motor_DIR 13
#define right_motor_PWM 10

#define DS1 6
#define DS2 7
#define DS3 8

/* ============ SEARCH TUNING ============ */
const int PIVOT_R_PWM = 110;
const int PIVOT_L_PWM = 110;
const unsigned long SWEEP_RIGHT_MS = 270;
const unsigned long SWEEP_LEFT_MS  = 300;
const unsigned long ADVANCE_MS     = 350;

// speedDiff = measured right/left mismatch (right needs more PWM to match left).
// Re-measure with the motor test sketch if you change gearing, wheels, or battery.
const float speedDiff  = 1.133;
const int SEARCH_ADV_L = 150;
const int SEARCH_ADV_R = SEARCH_ADV_L * speedDiff;   // derived, stays in sync with speedDiff

const int PUSH_DISTANCE = 6;   // cm — body pressed at shield (Flags)
const int SIT_DISTANCE  = 8;   // cm — opponent close enough to wake up (SitandWait)

// Flank arc time limit — if no opponent found by this point, re-enable edge
// detection so the robot doesn't drive blind off the dohyo. Tune to match
// how long your actual flank arc takes.
const unsigned long FLANK_TIMEOUT_MS = 900;
/* ======================================= */

enum SearchStep { SWEEP_OUT, SWEEP_BACK, ADVANCE };
SearchStep searchStep = SWEEP_OUT;
unsigned long stepStart = 0;

bool FlankDone = true;
unsigned long flankStart = 0;

void setup() {
  Serial.begin(9600);
  pinMode(IR_left,  INPUT);
  pinMode(IR_front, INPUT);
  pinMode(IR_right, INPUT);
  pinMode(QTR_right, INPUT);
  pinMode(QTR_left,  INPUT);
  pinMode(left_motor_DIR,  OUTPUT);
  pinMode(left_motor_PWM,  OUTPUT);
  pinMode(right_motor_DIR, OUTPUT);
  pinMode(right_motor_PWM, OUTPUT);
  pinMode(DS1, INPUT_PULLUP);
  pinMode(DS2, INPUT_PULLUP);
  pinMode(DS3, INPUT_PULLUP);

  delay(5000);

  bool s1 = digitalRead(DS1);
  bool s2 = digitalRead(DS2);
  bool s3 = digitalRead(DS3);
  if (s1 && s2 && s3)   { strategy = Normal;       } //000
  if (s1 && s2 && !s3)  { strategy = Searching;    } //001
  if (s1 && !s2 && s3)  { strategy = Flags;        } //010
  if (s1 && !s2 && !s3) {                              //011
    strategy = Flank;
    FlankDone = false;
    flankStart = millis();
  }
  if (!s1 && s2 && s3)  { strategy = SitandSearch; } //100
  if (!s1 && s2 && !s3) { strategy = SitandWait;   } //101

  stepStart = millis();

  Serial.print(!s1);
  Serial.print(!s2);
  Serial.println(!s3);
}

void loop() {
  bool leftDetected  = !digitalRead(IR_left);
  bool frontDetected = !digitalRead(IR_front);
  bool rightDetected = !digitalRead(IR_right);
  int Qtr1 = analogRead(QTR_right);
  int Qtr2 = analogRead(QTR_left);

  Serial.println("LEFT    | FRONT    | RIGHT   ");
  Serial.print(leftDetected ? "DETECTED |" : " CLEAR  |");
  Serial.print(frontDetected ? "DETECTED |" : "CLEAR |");
  Serial.println(rightDetected ? " DETECTED  |" : "  CLEAR |");

  bool onWhiteright = Qtr1 < WHITE_THRESHOLD;
  bool onWhiteleft  = Qtr2 < WHITE_THRESHOLD;
  bool whiteLine    = onWhiteright || onWhiteleft;
  bool onWhite = whiteLine;
  Serial.println(Qtr1);
  Serial.println(Qtr2);

  // Flank: ignore the edge sensors only until the arc times out, then
  // force it back on so the robot can never drive blind indefinitely.
  if (!FlankDone) {
    if (millis() - flankStart < FLANK_TIMEOUT_MS) {
      whiteLine = false;
    } else {
      FlankDone = true;
    }
  }

  if (whiteLine) {                       // line always wins
    Onwhite(onWhiteright, onWhiteleft);
    RestartSearch(DIR_NONE);
  }
  else if (strategy == Normal) {
    BasicReact(rightDetected, leftDetected, frontDetected);
  }
  else if (strategy == Searching) {
    ReactOrSearch(rightDetected, leftDetected, frontDetected);
  }
  else if (strategy == Flank) {
    if (rightDetected || leftDetected || frontDetected) {
      ReactOrSearch(rightDetected, leftDetected, frontDetected);
      FlankDone = true;
      strategy = Searching;
    } else {
      if (onWhite) {
        DriveMotors(150, 50);   // veer toward black side
      } else {
        DriveMotors(50, 150);   // veer back toward white side
      }
    }
  }
  else if (strategy == Flags) {
    int distance = sensor.getDistance();   // only read the analog sensor here — Flags needs it
    Serial.println(distance);
    int speed = 210;
    bool pushing = (distance > 0 && distance <= PUSH_DISTANCE);   // body pressed near the shield

    if (pushing && turned) {
      DriveMotors(speed, speed * speedDiff);
    }
    else if (turned && frontDetected) {
      DriveMotors(speed, speed * speedDiff);
    }
    else if (!turned) {
      if (leftDetected && frontDetected && rightDetected) {
        DriveMotors(speed, speed * speedDiff);
        // note: opponent may be a stand-still bot — a flank/dodge could be better here
      }
      else if (rightDetected && frontDetected) {
        DriveMotors(speed, -speed);
        delay(225);
        DriveMotors(speed, speed * speedDiff);
        delay(200);
        turned = true;
      }
      else if (leftDetected && frontDetected) {
        DriveMotors(-speed, speed);
        delay(225);
        DriveMotors(speed, speed * speedDiff);
        delay(200);
        turned = true;
      }
      else {
        ReactOrSearch(rightDetected, leftDetected, frontDetected);
      }
    }
    else {                     // turned == true but front lost — reset
      turned = false;
    }
  }
  else if (strategy == SitandSearch) {
     
    if (rightDetected) {
     
      DriveMotors(220,-220);
    } 
    
    else if (leftDetected) {
  
      DriveMotors(-220,220);
    } 
    
    else if (frontDetected) {
     
       DriveMotors(250,255);
    
    }
    else{
   if (!searching) {
     searching = true;
     RunSearch(300);
     searching = false;
   }

      
    }
  


}
else if(strategy == SitandWait){
   if (rightDetected) {
     
      DriveMotors(220,-220);
    } 
    
    else if (leftDetected) {
  
      DriveMotors(-220,220);
    } 
    
    else if (frontDetected) {
     
       DriveMotors(250,255);
    
    }
}
}

unsigned long lastSeenTime = 0;

// Front wins when it overlaps with a side sensor — an opponent dead ahead
// almost always trips a side cone too, and the old right-then-left-then-front
// order made the robot pivot away from a target it was already facing.
void ReactOrSearch(bool R, bool L, bool F) {
  if (F && R && !L)      { DriveMotors(220, 248 * 0.6); RestartSearch(DIR_FRONT); }
  else if (F && L && !R) { DriveMotors(220 * 0.6, 248); RestartSearch(DIR_FRONT); }
  else if (F)            { DriveMotors(220, 248);       RestartSearch(DIR_FRONT); }
  else if (R)            { DriveMotors(220, -220);      RestartSearch(DIR_RIGHT);  }
  else if (L)            { DriveMotors(-220, 220);      RestartSearch(DIR_LEFT);   }
  else                   { UpdateSearch(); }
}

void BasicReact(bool rightDetected, bool leftDetected, bool frontDetected) {
  if (rightDetected) {
    last_seen = DIR_RIGHT;  lastSeenTime = millis();  DriveMotors(150, -150);
  } else if (leftDetected) {
    last_seen = DIR_LEFT;   lastSeenTime = millis();  DriveMotors(-150, 150);
  } else if (frontDetected) {
    last_seen = DIR_FRONT;  lastSeenTime = millis();  DriveMotors(200, 227);
  } else {
    // nothing seen right now
    if (last_seen != DIR_NONE && millis() - lastSeenTime < 300) {
      // recently lost it — keep turning toward where it was
      if (last_seen == DIR_RIGHT)      DriveMotors(150, -170);
      else if (last_seen == DIR_LEFT)  DriveMotors(-150, 170);
      else                             DriveMotors(100, 113); // DIR_FRONT
    } else {
      // gone too long (or never seen) — full sweep
      last_seen = DIR_NONE;
      DriveMotors(200, 220);
    }
  }
}

/* ---------------- non-blocking search ---------------- */
void RestartSearch(Direction bias) {
  if (bias == DIR_RIGHT)      startRight = true;
  else if (bias == DIR_LEFT)  startRight = false;
  searchStep = SWEEP_OUT;
  stepStart = millis();
}

void UpdateSearch() {
  unsigned long now = millis();
  unsigned long elapsed = now - stepStart;

  if (searchStep == SWEEP_OUT) {
    unsigned long dur = startRight ? SWEEP_RIGHT_MS : SWEEP_LEFT_MS;
    if (startRight) pivotRight(); else pivotLeft();
    if (elapsed >= dur) { searchStep = SWEEP_BACK; stepStart = now; }
  }
  else if (searchStep == SWEEP_BACK) {
    unsigned long dur = startRight ? SWEEP_LEFT_MS : SWEEP_RIGHT_MS;
    if (startRight) pivotLeft(); else pivotRight();
    if (elapsed >= dur) { searchStep = ADVANCE; stepStart = now; }
  }
  else { // ADVANCE
    DriveMotors(SEARCH_ADV_L, SEARCH_ADV_R);
    if (elapsed >= ADVANCE_MS) {
      startRight = !startRight;
      searchStep = SWEEP_OUT;
      stepStart = now;
    }
  }
}

void pivotRight() { DriveMotors(PIVOT_R_PWM, -PIVOT_R_PWM); }
void pivotLeft()  { DriveMotors(-PIVOT_L_PWM, PIVOT_L_PWM); }

void DriveMotors(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  -255, 255);   // clamp first
  rightPWM = constrain(rightPWM, -255, 255);
  if (rightPWM >= 0) { digitalWrite(right_motor_DIR, HIGH); analogWrite(right_motor_PWM, rightPWM); }
  else               { digitalWrite(right_motor_DIR, LOW);  analogWrite(right_motor_PWM, -rightPWM); }
  if (leftPWM >= 0)  { digitalWrite(left_motor_DIR, HIGH);  analogWrite(left_motor_PWM, leftPWM); }
  else               { digitalWrite(left_motor_DIR, LOW);   analogWrite(left_motor_PWM, -leftPWM); }
}

// Escape direction fixed: a hit on the RIGHT edge sensor means the border is
// ahead-right, so after backing off you want to pivot LEFT — away from it —
// not back toward the same edge.
void Onwhite(bool whiteright, bool whiteleft) {
  if (whiteright && !whiteleft) {
    DriveMotors(-220, -220); delay(300);
    DriveMotors(-200, 200);  delay(200);
  } else if (!whiteright && whiteleft) {
    DriveMotors(-220, -220); delay(300);
    DriveMotors(200, -200);  delay(200);
  } else {
    DriveMotors(-220, -220); delay(300);
    DriveMotors(-200, 200);  delay(250);
  }
}
void RunSearch(int del) {
  unsigned long myTime = millis();

  if (startRight) {
    pivotRight();
  } else {
    pivotLeft();
  }
  while(millis() - myTime < del){
    if(!digitalRead(IR_left) || !digitalRead(IR_front) || !digitalRead(IR_right)) {
      startRight = !startRight;
      return;
    }
  }

  if (startRight) {
    pivotLeft();
  } else {
    pivotRight();
  }
  while(millis() - myTime < del * 2){
    if(!digitalRead(IR_left) || !digitalRead(IR_front) || !digitalRead(IR_right)) {
      startRight = !startRight;
      return;
    }
  }

  startRight = !startRight;
}