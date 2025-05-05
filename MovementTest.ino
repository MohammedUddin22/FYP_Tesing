// === Encoder Pins ===
const byte RencoderPinA = 3;
const byte RencoderPinB = 18;
const byte LencoderPinA = 2;
const byte LencoderPinB = 19;

// === Motor Control Pins ===
const byte dirPinR = 4;
const byte speedPinR = 5;
const byte dirPinL = 7;
const byte speedPinL = 6;

// === Constants ===
const float PULSES_PER_REV = 960.0;
const unsigned long SAMPLE_MS = 100;
float setpointRPM = 80.0;  //40
float turnRPM = 40; //10

// Robot dimensions
const float wheelBaseCM = 10.9;
const float wheelDiameterCM = 6.0;
const float wheelCircumference = PI * wheelDiameterCM;
const float pulsesPerCM = PULSES_PER_REV / wheelCircumference;

// === Encoder Variables ===
volatile long totalPulsesR = 0;
volatile long totalPulsesL = 0;

// === Timing ===
unsigned long lastSampleTime;

// === Coordinates and Grid ===
struct Point { int x, y; };
const float GRID_SIZE_CM = 10.0;

// === Direction ===
enum Direction { NORTH=0, EAST=1, SOUTH=2, WEST=3 };
Direction currentDir = NORTH;

// === PID Controller Class ===
class PIDController {
public:
  float kp, ki, kd;
  float integral = 0.0;
  float lastError = 0.0;
  float output = 0.0;
  float maxOutput = 255.0;
  float maxIntegral = 200.0;
  float deadband = 0.0;

  PIDController(float _kp, float _ki, float _kd, float _deadband)
    : kp(_kp), ki(_ki), kd(_kd), deadband(_deadband) {}

  int compute(float setpoint, float measured, float dt) {
  float error = setpoint - measured;
  integral += error * dt;
  integral = constrain(integral, -maxIntegral, maxIntegral);
  float derivative = (error - lastError) / dt;
  output = kp * error + ki * integral + kd * derivative;
  lastError = error;

  int pwm = constrain((int)abs(output), 0, (int)maxOutput);
  
  //  Apply deadband inside the class
  if (pwm > 0 && pwm < deadband) {
    pwm = deadband;
  }

  return pwm;
}


  bool direction() {
    return output >= 0;
  }

  void reset() {
    integral = 0;
    lastError = 0;
  }
};

PIDController pidR(1, 5.6, 0.01, 30);
PIDController pidL(1, 5.8, 0.01, 30);

void setup() {
  Serial.begin(9600);

  pinMode(dirPinR, OUTPUT); pinMode(speedPinR, OUTPUT);
  pinMode(dirPinL, OUTPUT); pinMode(speedPinL, OUTPUT);

  pinMode(RencoderPinA, INPUT_PULLUP); pinMode(RencoderPinB, INPUT_PULLUP);
  pinMode(LencoderPinA, INPUT_PULLUP); pinMode(LencoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RencoderPinA), wheelISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(LencoderPinA), wheelISR_L, RISING);

  lastSampleTime = millis();

  Point path[] = { {1,1},{1,2},{2,2},{2,3},{3,3},{4,3},{4,2} };
  int pathLen = sizeof(path) / sizeof(path[0]);
  for (int i = 1; i < pathLen; i++) {
    pathMovement(path[i-1], path[i]);
  }

}

void loop() {
  //moveForward(1);
  //delay(500);

  //turnLeft();
  //delay(500);

  //moveBackwards(1);
  //delay(500);

  //turnRight();
  //delay(500);
}


// === Encoder ISRs ===
void wheelISR_R() {
  totalPulsesR += digitalRead(RencoderPinB) == HIGH ? 1 : -1;
}
void wheelISR_L() {
  totalPulsesL += digitalRead(LencoderPinB) == HIGH ? -1 : +1;
}

void pathMovement(Point start, Point goal) {
  Point cur = start;

  // assume goal is adjacent (check abs(dx)+abs(dy)==1 if you like)
  int dx = goal.x - cur.x;
  int dy = goal.y - cur.y;

  // decide which cardinal we need
  // if x increase 1, we go right. if y increase up 1 we go forward
  // turnTo used to make make sure facing correct direction
  if      (dx ==  1) turnTo(EAST);
  else if (dx == -1) turnTo(WEST);
  else if (dy ==  1) turnTo(NORTH);
  else if (dy == -1) turnTo(SOUTH);
  else {
    Serial.println("Error: non-adjacent!");
    return;
  }

  // now we’re facing exactly the right way—go one cell forward
  moveForward(1);

  // update our software position
  cur = goal;
}

void turnTo(Direction target) {
  // compute difference in [0..3] e.g modulo 4
  // cardinal direction
  int diff = ((int)target - (int)currentDir + 4) % 4;

  if      (diff == 1)        turnRight();           //  90° CW
  else if (diff == 2) {      turnRight(); turnRight(); } // 180°
  else if (diff == 3)        turnLeft();            //  90° CCW
  // if diff==0, we’re already pointing at target

  // after calling turnLeft()/turnRight(), currentDir is auto-updated
  // by those functions, so now currentDir == target
}

// === Forward Function with Debug ===
void moveForward(int gridunit) {
  float targetPulses = gridunit * GRID_SIZE_CM * pulsesPerCM; 
  noInterrupts(); totalPulsesR = 0; totalPulsesL = 0; interrupts();
  pidR.reset(); pidL.reset();
  lastSampleTime = millis();
  long lastPulsesR = 0, lastPulsesL = 0;
  unsigned long startTime = millis();

  while (true) {
    unsigned long now = millis();
    if (now - startTime > 7000) break;

    noInterrupts();
    long currentPulsesR = totalPulsesR;
    long currentPulsesL = totalPulsesL;
    interrupts();

    if ((targetPulses > 0 && currentPulsesR >= targetPulses && currentPulsesL >= targetPulses) ||
        (targetPulses < 0 && currentPulsesR <= targetPulses && currentPulsesL <= targetPulses)) {
      break;
    }

    if (now - lastSampleTime >= SAMPLE_MS) {
      float dt = (now - lastSampleTime) / 1000.0;
      float rpmR = ((currentPulsesR - lastPulsesR) / PULSES_PER_REV) * (60.0 / dt);
      float rpmL = ((currentPulsesL - lastPulsesL) / PULSES_PER_REV) * (60.0 / dt);

      int pwmR = pidR.compute(setpointRPM, rpmR, dt);
      int pwmL = pidL.compute(setpointRPM, rpmL, dt);

      digitalWrite(dirPinR, pidR.direction() ? LOW : HIGH);
      digitalWrite(dirPinL, pidL.direction() ? LOW : HIGH);
      analogWrite(speedPinR, pwmR);
      analogWrite(speedPinL, pwmL);

      //Serial.print("R Δ: "); Serial.print(currentPulsesR);
      //Serial.print(" RPM: "); Serial.print(rpmR, 1);
      //Serial.print(" PWM: "); Serial.print(pwmR);
      //Serial.print(" Dir: "); Serial.print(pidR.direction() ? "FWD" : "REV");

      //Serial.print(" | L Δ: "); Serial.print(currentPulsesL);
      //Serial.print(" RPM: "); Serial.print(rpmL, 1);
      //Serial.print(" PWM: "); Serial.print(pwmL);
      //Serial.print(" Dir: "); Serial.println(pidL.direction() ? "FWD" : "REV");

      lastSampleTime = now;
      lastPulsesR = currentPulsesR;
      lastPulsesL = currentPulsesL;
    }
  }

  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);

  Serial.print("Final Pulses R: ");
  Serial.print(totalPulsesR);
  Serial.print(" | L: ");
  Serial.println(totalPulsesL);

  delay(300);
}

// === Reverse Function with Debug
void moveBackwards(int gridunit) {
  float targetPulses = gridunit * GRID_SIZE_CM * pulsesPerCM *(-1); 
  noInterrupts(); totalPulsesR = 0; totalPulsesL = 0; interrupts();
  pidR.reset(); pidL.reset();
  lastSampleTime = millis();
  long lastPulsesR = 0, lastPulsesL = 0;
  unsigned long startTime = millis();

  while (true) {
    unsigned long now = millis();
    if (now - startTime > 7000) break;

    noInterrupts();
    long currentPulsesR = totalPulsesR;
    long currentPulsesL = totalPulsesL;
    interrupts();

    if ((targetPulses > 0 && currentPulsesR >= targetPulses && currentPulsesL >= targetPulses) ||
        (targetPulses < 0 && currentPulsesR <= targetPulses && currentPulsesL <= targetPulses)) {
      break;
    }

    if (now - lastSampleTime >= SAMPLE_MS) {
      float dt = (now - lastSampleTime) / 1000.0;
      float rpmR = ((currentPulsesR - lastPulsesR) / PULSES_PER_REV) * (60.0 / dt);
      float rpmL = ((currentPulsesL - lastPulsesL) / PULSES_PER_REV) * (60.0 / dt);

      int pwmR = pidR.compute(-setpointRPM, rpmR, dt);
      int pwmL = pidL.compute(-setpointRPM, rpmL, dt);

      digitalWrite(dirPinR, pidR.direction() ? LOW : HIGH);
      digitalWrite(dirPinL, pidL.direction() ? LOW : HIGH);
      analogWrite(speedPinR, pwmR);
      analogWrite(speedPinL, pwmL);

      //Serial.print("R Δ: "); Serial.print(currentPulsesR);
      //Serial.print(" RPM: "); Serial.print(rpmR, 1);
      //Serial.print(" PWM: "); Serial.print(pwmR);
      //Serial.print(" Dir: "); Serial.print(pidR.direction() ? "FWD" : "REV");

      //Serial.print(" | L Δ: "); Serial.print(currentPulsesL);
      //Serial.print(" RPM: "); Serial.print(rpmL, 1);
      //Serial.print(" PWM: "); Serial.print(pwmL);
      //Serial.print(" Dir: "); Serial.println(pidL.direction() ? "FWD" : "REV");

      lastSampleTime = now;
      lastPulsesR = currentPulsesR;
      lastPulsesL = currentPulsesL;
    }
  }

  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);

  Serial.print("Final Pulses R: ");
  Serial.print(totalPulsesR);
  Serial.print(" | L: ");
  Serial.println(totalPulsesL);

  delay(300);
}

// === Turn Left with Debug ===
void turnLeft() {
  float arcLength = (PI * wheelBaseCM) / 4.0;
  long targetPulses = arcLength * pulsesPerCM;
  currentDir = Direction((currentDir + 3) % 4);  

  noInterrupts();
  totalPulsesR = 0;
  totalPulsesL = 0;
  interrupts();

  pidR.reset();
  pidL.reset();
  lastSampleTime = millis();

  long lastPulsesR = 0, lastPulsesL = 0;
  unsigned long startTime = millis();

  float setpointR = turnRPM;   // right motor forward
  float setpointL = -turnRPM;  // left motor reverse

  while (true) {
    unsigned long now = millis();
    if (now - startTime > 7000) break;

    noInterrupts();
    long currentR = totalPulsesR;
    long currentL = totalPulsesL;
    interrupts();

    if (abs(currentR) >= targetPulses && abs(currentL) >= targetPulses) break;

    if (now - lastSampleTime >= SAMPLE_MS) {
      float dt = (now - lastSampleTime) / 1000.0;
      float rpmR = ((currentR - lastPulsesR) / PULSES_PER_REV) * (60.0 / dt);
      float rpmL = ((currentL - lastPulsesL) / PULSES_PER_REV) * (60.0 / dt);

      int pwmR = pidR.compute(setpointR, rpmR, dt);
      int pwmL = pidL.compute(setpointL, rpmL, dt);
      if (pwmR > 0 && pwmR < 30) pwmR = 30;
      if (pwmL > 0 && pwmL < 30) pwmL = 30;

      digitalWrite(dirPinR, pidR.direction() ? LOW : HIGH);
      digitalWrite(dirPinL, pidL.direction() ? LOW : HIGH);

      analogWrite(speedPinR, pwmR);
      analogWrite(speedPinL, pwmL);

      lastSampleTime = now;
      lastPulsesR = currentR;
      lastPulsesL = currentL;
    }
  }

  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);

  Serial.print("Final Pulses R: ");
  Serial.print(totalPulsesR);
  Serial.print(" | L: ");
  Serial.println(totalPulsesL);

  delay(300);
}

// === Turn Right with Debug ===
// R motor reverses and L moves forward
void turnRight() {
  float arcLength = (PI * wheelBaseCM) / 4.0;
  long targetPulses = arcLength * pulsesPerCM;
  currentDir = Direction((currentDir + 1) % 4);

  noInterrupts(); totalPulsesR = 0; totalPulsesL = 0; interrupts();
  pidR.reset(); pidL.reset();
  lastSampleTime = millis();
  long lastPulsesR = 0, lastPulsesL = 0;
  unsigned long startTime = millis();

  float setpointR = -turnRPM;
  float setpointL = turnRPM;

  while (true) {
    unsigned long now = millis();
    if (now - startTime > 7000) break;

    noInterrupts();
    long currentR = totalPulsesR;
    long currentL = totalPulsesL;
    interrupts();

    if (abs(currentR) >= targetPulses && abs(currentL) >= targetPulses) break;

    if (now - lastSampleTime >= SAMPLE_MS) {
      float dt = (now - lastSampleTime) / 1000.0;
      float rpmR = ((currentR - lastPulsesR) / PULSES_PER_REV) * (60.0 / dt);
      float rpmL = ((currentL - lastPulsesL) / PULSES_PER_REV) * (60.0 / dt);

      int pwmR = pidR.compute(setpointR, rpmR, dt);
      int pwmL = pidL.compute(setpointL, rpmL, dt);
      if (pwmR > 0 && pwmR < 30) pwmR = 30;
      if (pwmL > 0 && pwmL < 30) pwmL = 30;

      digitalWrite(dirPinR, pidR.direction() ? LOW : HIGH);
      digitalWrite(dirPinL, pidL.direction() ? LOW : HIGH);
      analogWrite(speedPinR, pwmR);
      analogWrite(speedPinL, pwmL);

      lastSampleTime = now;
      lastPulsesR = currentR;
      lastPulsesL = currentL;
    }
  }

  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);

  Serial.print("Final Pulses R: ");
  Serial.print(totalPulsesR);
  Serial.print(" | L: ");
  Serial.println(totalPulsesL);

  delay(300);
}
