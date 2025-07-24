#define comparatorAOut 22
#define comparatorBOut 23
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19

//timing the comparator pair window
volatile int aIndex = 0;
volatile int bIndex = 0;
const int Nsamples = 100;
volatile unsigned long lastRiseTimeA[Nsamples] = {0};
volatile unsigned long lastRiseTimeB[Nsamples] = {0};
const float dspacing = 0.177;                 // meters between microphones
const float csound = 343.0;                 // m/s
const unsigned long MAX_EVENT_WINDOW = 520;  // microseconds
bool isListening = true;

//Stepper Motor logic
float targetAngle = 0.0;
bool motorActive = false;
int stepIndex = 0;
long stepsRemaining = 0;
int stepDirection = 0;
long currentPosition = 0;    // In steps (absolute)
long targetPosition = 0;     // In steps
const int stepsPerRevolution = 4096;  // 360° rotation
const int stepSequence[8][4] = {
  {HIGH, LOW,  LOW,  LOW},
  {HIGH, HIGH, LOW,  LOW},
  {LOW,  HIGH, LOW,  LOW},
  {LOW,  HIGH, HIGH, LOW},
  {LOW,  LOW,  HIGH, LOW},
  {LOW,  LOW,  HIGH, HIGH},
  {LOW,  LOW,  LOW,  HIGH},
  {HIGH, LOW,  LOW,  HIGH}
};


void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(comparatorAOut, INPUT);
  pinMode(comparatorBOut, INPUT);
  attachInterrupt(digitalPinToInterrupt(comparatorAOut), comparatorA, RISING);
  attachInterrupt(digitalPinToInterrupt(comparatorBOut), comparatorB, RISING);
}

//ISR interrupts
void comparatorA() { 
  if(aIndex==100) { aIndex=0;}
  lastRiseTimeA[aIndex] = micros();
  aIndex++;
}
void comparatorB() {
  if(bIndex==100) {bIndex=0;}
  lastRiseTimeB[bIndex] = micros();
  bIndex++;
}


//takes two arrays, finds the leading array, and matches the closest initial pairing
void findTimeFromArrays(unsigned long* A, unsigned long* B, int aLen, int bLen) {
  int bestA = 0;
  int bestB = 0;
  unsigned long minDelta = 999999;
  
  for (int i = 0; i < aLen; i++) {
    for (int j = 0; j < bLen; j++) {
      unsigned long delta = abs((long)(A[i] - B[j]));
      if (delta < minDelta && delta <= MAX_EVENT_WINDOW) {
        minDelta = delta;
        bestA = i;
        bestB = j;
      }
    }
  }
  
  A[0] = A[bestA];
  B[0] = B[bestB];
}

//finds the angle from the time delay of two arrays set by ISR interrupts from comparators
//assumes sound originates from the front
float findAngle(unsigned long* A, unsigned long* B, int aLen, int bLen) {
  Serial.print("A index: "); Serial.print(aLen);
  Serial.print(",B index: "); Serial.println(bLen);
  findTimeFromArrays(A, B, aLen, bLen);
  
  Serial.print("A time: "); Serial.print(A[0]);
  Serial.print(",B time: "); Serial.println(B[0]);
  long deltaT = long(A[0]) - long(B[0]);  // Time difference in microseconds
  float deltaT_sec = float(deltaT) / 1000000.0;  // Convert to seconds
  if (abs(deltaT_sec)>(dspacing/csound))
  {
    Serial.print("ERROR: DELTA T TOO LONG"); return 0;
  }

  float ratio = (csound * deltaT_sec) / dspacing;
  ratio = constrain(ratio, -1.0, 1.0);  // Prevent domain error in asin()

  float theta_radians = asin(ratio);
  float theta_degrees = theta_radians * 180.0 / PI;
  if(abs(theta_degrees)<=15) return 0; //keep aim within 30 degree window

  Serial.print("DeltaT (us): ");
  Serial.print(deltaT);
  Serial.print(", Angle (degrees): ");
  Serial.println(theta_degrees);
  
  return theta_degrees;
}

void checkAngleValidity() 
{
  return; //TODO
}

//moves the stepper motor at an angle relative to current position
//0 degrees is straight, -90 is ccw, 90 is cw
void startMove(float angle) {
  long stepDifference = (long)((angle / 360.0) * stepsPerRevolution);
  stepsRemaining = abs(stepDifference);

  if (stepDifference > 0) {
    stepDirection = 1;  // Global variable
  } else {
    stepDirection = -1;
  }
}
bool continueMove() {
  if (stepsRemaining > 0) {
    if (stepDirection > 0) {
      stepIndex = (stepIndex + 1) % 8;
      currentPosition++;
    } else {
      stepIndex = (stepIndex + 7) % 8;
      currentPosition--;
    }

    if (currentPosition >= stepsPerRevolution) currentPosition -= stepsPerRevolution;
    if (currentPosition < 0) currentPosition += stepsPerRevolution;

    digitalWrite(IN1, stepSequence[stepIndex][0]);
    digitalWrite(IN2, stepSequence[stepIndex][1]);
    digitalWrite(IN3, stepSequence[stepIndex][2]);
    digitalWrite(IN4, stepSequence[stepIndex][3]);

    delayMicroseconds(1200); //speed

    stepsRemaining--;   

    return false;  // Still moving
  }

  return true;  // Move complete
}



void loop() {
  if (isListening && !motorActive && aIndex > 0 && bIndex > 0) {
    // Copy ISR data safely
    delayMicroseconds(5);
    noInterrupts();
    unsigned long localA[100];
    unsigned long localB[100];
    int localAIndex = aIndex;
    int localBIndex = bIndex;
    
    memcpy(localA, (unsigned long*)lastRiseTimeA, sizeof(lastRiseTimeA));
    memcpy(localB, (unsigned long*)lastRiseTimeB, sizeof(lastRiseTimeB));
    memset((unsigned long*)lastRiseTimeA,0, sizeof(lastRiseTimeA));
    memset((unsigned long*)lastRiseTimeB,0, sizeof(lastRiseTimeB));
    
    aIndex = 0;
    bIndex = 0;
    interrupts();

    // Process data outside critical section
    targetAngle = findAngle(localA, localB, localAIndex, localBIndex);

    if (targetAngle != 0) {
      startMove(targetAngle);
      motorActive = true;
      isListening = false;  // Pause while motor turns
    }

    checkAngleValidity();  // Optional validation
  }

  // Drive stepper motor
  while (motorActive) {
    if (continueMove()) {
      motorActive = false;
      isListening = true;
    }
  }
}
