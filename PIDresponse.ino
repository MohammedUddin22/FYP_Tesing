// Arduino sketch: quadrature decoding + RPM calc + PID + serial plotting (5 s capture)

// Encoder pins
const byte encoderPinA   = 3;  //Right3 Left2
const byte encoderPinB   = 18;  //R18 L19

// Motor control pins
const byte dirPinR       = 4;  //R4 L7
const byte speedPinR     = 5;  //R5 L6
// Encoder / timing
const float PULSES_PER_REV  = 960.0;
const unsigned long SAMPLE_MS = 50; //100

// PID gains
float kp =  34.78;  //3.32 - MATLAB  //34.78 Handworked
float ki = 311.67;  //22.3 - MATLAB  //311.67 Handworked
float kd = 1.69; //0.01 - MATLAB //1.69 Handworked

// Desired speed
float setpointRPM = 100;

// PID state
float integral    = 0.0;
float lastError   = 0.0;

// Encoder counts
volatile long pulses = 0;

// Timing
unsigned long lastSampleTime;
unsigned long startTime;
const unsigned long CAPTURE_MS = 2500; //5000
bool capturing = true;

void setup() {
  Serial.begin(57600);

  pinMode(dirPinR, OUTPUT);
  pinMode(speedPinR, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), wheelISR, RISING);

  // initialize timers
  lastSampleTime = millis();
  startTime      = lastSampleTime;

  // header for CSV
  Serial.println("time_ms,RPM");
}

void loop() {
  if (!capturing) return;        // after 5 s, stop printing

  unsigned long now = millis();
  unsigned long elapsed = now - startTime;

  if (elapsed > CAPTURE_MS) {
    capturing = false;
    //Optionally, you could stop the motor here:
    analogWrite(speedPinR, 0);
    return;
  }

  if (now - lastSampleTime >= SAMPLE_MS) {
    // grab-and-clear pulse count
    noInterrupts();
      long count = pulses;
      pulses = 0;
    interrupts();

    float dt = (now - lastSampleTime) / 1000.0;
    lastSampleTime = now;

    // compute RPM
    float rpm = (count / PULSES_PER_REV) * (60.0 / dt);

    // PID
    float error = setpointRPM - rpm;
    integral += error * dt;
    float alpha = 0;  // 0 = no filter, 1 = full filter (very slow)
    float derivative = (error - lastError) / dt; //dUnfiltered easlier
    //float derivative = alpha * derivative + (1 - alpha) * dUnfiltered;

    //float maxI = 255.0;    // tune to something slightly less than 255*Ki
    //integral = constrain(integral, -maxI, +maxI); //anti-windup
    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;
    
    // determine direction & PWM
    bool forward = (output >= 0);
    int pwm = constrain((int)abs(output), 0, 255);
    /*const int pwmDeadband = 30;    // your measured break‑away
    if (abs(pwm) < pwmDeadband) {
      pwm = pwm + pwmDeadband;                      // treat anything below as “off”
    }*/

    digitalWrite(dirPinR, forward ? LOW : HIGH);
    analogWrite(speedPinR, pwm);

    // print CSV: elapsed time, RPM, error, output, PWM
    Serial.print(elapsed, 1); Serial.print(',');
    Serial.println(rpm, 1);    

  }
}

// ISR: quadrature decode
void wheelISR() {
  if (digitalRead(encoderPinB) == HIGH) {
    pulses++; //R++ //L--
  } else {
    pulses--;  //R-- //L++
  }
}
