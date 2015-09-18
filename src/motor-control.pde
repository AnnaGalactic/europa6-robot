void setupMotors() {
  pinMode(RightMotorDirectionPin, OUTPUT);
  pinMode(RightMotorEnablePin, OUTPUT);
  pinMode(LeftMotorDirectionPin, OUTPUT);
  pinMode(LeftMotorEnablePin, OUTPUT);
}


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
