#define US_VCC A2
#define US_GND A3

int trigPin = 14;    // Trigger pin (usually ORANGE wire)
int echoPin = 15;    // Echo pin (usually YELLOW wire)

long duration;
float cm, inches;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(US_VCC, OUTPUT);
  digitalWrite(US_VCC, HIGH);  // Turn on sensor power
  
  pinMode(US_GND, OUTPUT);
  digitalWrite(US_GND, LOW);   // Connect ground
}

void loop() {
  // Ensure a clean HIGH pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo
  duration = pulseIn(echoPin, HIGH);

  if (duration == 0) {
    Serial.println("No echo received");
  } else {
    // Convert time to distance
    cm = (duration / 2.0) / 29.1;     // Speed of sound: 343 m/s
    inches = (duration / 2.0) / 74.0; // Inches conversion


    Serial.print(cm);
    Serial.println(" cm");
  }

  delay(250);
}
