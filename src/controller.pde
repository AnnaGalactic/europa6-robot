/*

  joystick.pde, v1.0

  Robot teaching pendant, using old style IBM PC game joystick.
  For Arduino Diecimila or later
  Use Arduino IDE v0017 or later

  From Robot Builder's Bonanza, Fourth Edition
  www.robotoid.com

  License:
    This example code is distributed under Creative Commons
    Attribution-ShareAlike 3.0 Unported license
    (creativecommons.org/licenses/by-sa/3.0)/

*/

// Change to false to reduce traffic to COM port
boolean debugging = true;

// Set up I/O lines
const byte recordLED = 13;
const byte bttnRec = 9;   // Digital D9
const byte bttnPlay = 10; // Digital D10
const byte x_axis = 0;    // Analog A0
const byte y_axis = 1;    // Analog A1

// Configurable settinsgs
const int buttonDelay = 200;  // 200 ms button1/button2 delay
const int stepDelay = 500;    // Time (ms) between recorded steps
const int maxSteps = 60;      // Maximum number of recorded steps

// Bit positions for motorControl byte variable
const byte MotRC = 0;        // Right motor control (1=on)
const byte MotRD = 1;        // Right motor direction (1=forward)
const byte MotLC = 2;        // Left motor control
const byte MotLD = 3;        // Left motor direction

// Member variables
byte motors[maxSteps+1];    // Array holds motor control value for each recorded step
byte motorControl = 0;
int currentStep = 0;
int recordedSteps = 0;

byte minVal = 0;
byte midVal = 50;
byte maxVal = 100;

boolean recordFlag = false;
boolean playFlag = false;

long x, y;
int x_raw, y_raw;

void setup() {
  Serial.begin(9600);        // Set uyp Serial Monitor for debugging
  pinMode(bttnRec, INPUT);
  pinMode(bttnPlay, INPUT);
  pinMode(recordLED, OUTPUT);
  Serial.println("");        // Print a blank line
 }

void loop () {

  // Read X and Y axis of joystick
  x_raw = analogRead(x_axis);
  if (x_raw)
    x = 102300/x_raw - 100;    // Scale value to ~0-100
  else
    x = -1;                   // No or bad joystick

  y_raw = analogRead(y_axis);
  if (y_raw)
    y = 102300/y_raw - 100;
  else
    y = -1;

  readY();                    // Define control output for Y axis
  readX();                    // Define control output for X axis
  readButtons();              // Read buttons (record and play modes)

  // If recording
  if(recordFlag) {
    if(currentStep<=maxSteps) {
      motors[currentStep] = motorControl;  // Add current control output to array
      runMotors(motors[currentStep]);
      if(debugging) {                      // Display control output for debugging
        Serial.print(currentStep, DEC);
        Serial.print(" - ");
        binaryPad(motors[currentStep]);    // Show binary value of control output as bits
      }
      currentStep++;
    } else {                               // Exceeded max number of steps
      recordBttnHit();                     // Turn off recording
    }
  }

  // If playing back
  if(playFlag) {
    if(currentStep<recordedSteps) {
      runMotors(motors[currentStep]);  // Play previously stores control output value
      if(debugging) {
        Serial.print(currentStep, DEC);
        Serial.print(" - ");
        binaryPad(motors[currentStep]);
      }
      currentStep++;
    } else {                                // Exceeded number of recorded steps
      playBttnHit();                        // Turn off playback
    }
  }

  // Operate in free-run mode (neither recording nor playing back)
  if(!playFlag && !recordFlag) {
    runMotors(motorControl);
    if(debugging)
      binaryPad(motorControl);
  }
  showLED();                                // D13 LED lit if recording=on
  delay(stepDelay);

}

// Read Y axis for forward/reverse direction
// Robot is stopped if joystick is +/- 10 units within center
void readY() {
  if(y<(midVal-10))
    motorControl = B00001111;
  if(y>=(midVal-10) && y<=(midVal+10))
    motorControl = B00000000;
  if(y>(midVal+10))
    motorControl = B00000101;
}

// Read X axis for turns
void readX() {
  if(x<(midVal-10))
    motorControl = B00000111;
  //if(x>=(midVal-10) && x<=(midVal+10))
    // Let bits alone
  if(x>(midVal+10))
    motorControl = B00001101;
}

// Read joystick buttons for record and play functions
void readButtons() {
  if(digitalRead(bttnRec) == LOW)
    recordBttnHit();
  if (digitalRead(bttnPlay) == LOW)
    playBttnHit();
}

// If record button hit
void recordBttnHit() {
  if(playFlag)                   // Turn off play mode if on
    playBttnHit();
  recordFlag = !recordFlag;      // Toggle record mode
  if(recordFlag) {               // New recording; set counters to 0
    recordedSteps = 0;
    currentStep = 0;
    Serial.println("Recording On...");
  } else {
    recordedSteps = currentStep; // Recording finished
    Serial.println("Recording Off");
  }
  delay(buttonDelay);
}

// If play button hit
void playBttnHit() {
  if(recordFlag)                 // Turn off record mode if on
    recordBttnHit();
  if(recordedSteps == 0) {
    Serial.println("Nothing recorded");
    return;                      // Exit function if nothing to play back
  }
  playFlag = !playFlag;
  if(playFlag) {                 // Play steps previously stored
    recordedSteps = currentStep;
    Serial.print("Recorded steps: ");
    Serial.println(recordedSteps, DEC);
    Serial.println("Play On...");
    currentStep = 0;
  } else {
    Serial.println("Play Off");
  }
  delay(buttonDelay);
}

void setupMotors() {
  pinMode(RightMotorDirectionPin, OUTPUT);
  pinMode(RightMotorEnablePin, OUTPUT);
  pinMode(LeftMotorDirectionPin, OUTPUT);
  pinMode(LeftMotorEnablePin, OUTPUT);
}

void runMotors(byte motorValue) {
  /*
  byte 0: Right motor control (1=on)
  byte 1: Right motor direction (1=forward)
  byte 2: Left motor control
  byte 3: Left motor direction

  Ctrl: 0=off, 1=on
  Dir:  0=reverse, 1=forward
  */

  int rightMotorOn = motorValue & 0xFF;
  int rightMotorDirection = (motorValue >> 8) & 0xFF;
  int leftMotorOn = (motorValue >> 16) & 0xFF;
  int leftMotorDirection = (motorValue >> 24) & 0xFF;

  if (rightMotorOn) {
    digitalWrite(RightMotorDirectionPin, rightMotorDirection);
    digitalWrite(RightMotorEnablePin, HIGH);
  }
  else {
    digitalWrite(RightMotorEnablePin, LOW);
  }

  if (leftMotorOn) {
    digitalWrite(LeftMotorDirectionPin, leftMotorDirection);
    digitalWrite(LeftMotorEnablePin, HIGH);
  }
  else {
    digitalWrite(LeftMotorEnablePin, LOW);
  }

}

// Pad binary numbers with zeros, print result in Serial Monitor window
void binaryPad( int number ) {
  int pad = 1;
  int bits = 4;      // Display four-bit values
  for (byte i=0; i<bits; i++) {
    if (number < pad)
      Serial.print("0");
   pad *= 2;
  }
  if (number == 0)
    Serial.println("");
  else
    Serial.println(number, BIN);
}

// Update state of D13 LED (recording enabled)
void showLED() {
  digitalWrite(recordLED, recordFlag);
}
