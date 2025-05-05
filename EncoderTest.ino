#include <Arduino.h>

// ─── CONFIG ────────────────────────────────────────────
constexpr uint8_t N_ENCODERS       = 2;
constexpr float   PULSES_PER_REV   = 960.0;   // Encoder spec
constexpr uint32_t SAMPLE_MS       = 100;     // Print every 100 ms

// A‑ and B‑channel pins 
constexpr uint8_t encA[N_ENCODERS] = { 3,  2 };   // INT1, INT0
constexpr uint8_t encB[N_ENCODERS] = { 18, 19 };   // any digital

// +1 for “normal” count, –1 to invert direction  
constexpr int8_t  mult[N_ENCODERS] = { +1,  -1 };

// ─── SHARED STATE ──────────────────────────────────────
volatile long   pulses[N_ENCODERS]   = { 0 };
volatile bool   dirFwd[N_ENCODERS]   = { true };

float            rpm[N_ENCODERS]     = { 0.0 };

// ─── TEMPLATE ISR ──────────────────────────────────────
template <uint8_t IDX>

void encoderISR() {
  bool b = digitalRead(encB[IDX]);
  dirFwd[IDX]    = b;                         // HIGH = forward
  pulses[IDX]   += b ?  mult[IDX]
                     : -mult[IDX];
}

void setup() {
  Serial.begin(57600);

  // configure all encoder pins
  for (uint8_t i = 0; i < N_ENCODERS; i++) {
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }

  // attachInterrupt 
  attachInterrupt(digitalPinToInterrupt(encA[0]), encoderISR<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(encA[1]), encoderISR<1>, RISING);
}

void loop() {
  static uint32_t lastSample = 0;
  uint32_t now = millis();

  if (now - lastSample >= SAMPLE_MS) {
    long   counts[N_ENCODERS];
    bool   dirs[N_ENCODERS];
    noInterrupts();
      for (uint8_t i = 0; i < N_ENCODERS; i++) {
        counts[i] = pulses[i];
        dirs[i]   = dirFwd[i];
        pulses[i] = 0;
      }
    interrupts();

    //  RPM for each encoder
    float factor = (60.0 * 1000.0) / (PULSES_PER_REV * SAMPLE_MS);
    for (uint8_t i = 0; i < N_ENCODERS; i++) {
      // counts[i] might be negative if backwards, but dirFwd tells you direction
      rpm[i] = counts[i] * factor;
    }

    Serial.print("Left Encoder RPM (Motor B): ");
    Serial.print(rpm[1]);
    Serial.print("\tRight Encoder RPM (Motor A): ");
    Serial.println(rpm[0]);

    lastSample = now;
  }
}
