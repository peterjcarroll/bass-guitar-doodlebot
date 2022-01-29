//L293D

const int leftMotor = 0;
const int rightMotor = 1;

const int moveStop = 0;
const int moveForward = 1;
const int moveBackward = 2;

//Motor A
const int motorPin1 = 5; // Pin 14 of L293
const int motorPin2 = 6; // Pin 10 of L293
//Motor B
const int motorPin3 = 10; // Pin  7 of L293
const int motorPin4 = 9;  // Pin  2 of L293

//This will run only one time.
void setup()
{

  //Set pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  // //Motor Control - Motor A: motorPin1,motorpin2 & Motor B: motorpin3,motorpin4

  // //This code  will turn Motor A clockwise for 2 sec.
  // digitalWrite(motorPin1, HIGH);
  // digitalWrite(motorPin2, LOW);
  // digitalWrite(motorPin3, LOW);
  // digitalWrite(motorPin4, LOW);
  // delay(2000);
  // //This code will turn Motor A counter-clockwise for 2 sec.
  // digitalWrite(motorPin1, LOW);
  // digitalWrite(motorPin2, HIGH);
  // digitalWrite(motorPin3, LOW);
  // digitalWrite(motorPin4, LOW);
  // delay(2000);

  // //This code will turn Motor B clockwise for 2 sec.
  // digitalWrite(motorPin1, LOW);
  // digitalWrite(motorPin2, LOW);
  // digitalWrite(motorPin3, HIGH);
  // digitalWrite(motorPin4, LOW);
  // delay(2000);
  // //This code will turn Motor B counter-clockwise for 2 sec.
  // digitalWrite(motorPin1, LOW);
  // digitalWrite(motorPin2, LOW);
  // digitalWrite(motorPin3, LOW);
  // digitalWrite(motorPin4, HIGH);
  // delay(2000);

  // //Run both motors clockwise for 2 sec
  // digitalWrite(motorPin1, HIGH);
  // digitalWrite(motorPin2, LOW);
  // digitalWrite(motorPin3, HIGH);
  // digitalWrite(motorPin4, LOW);
  // delay(2000);

  // //And this code will stop motors
  // digitalWrite(motorPin1, LOW);
  // digitalWrite(motorPin2, LOW);
  // digitalWrite(motorPin3, LOW);
  // digitalWrite(motorPin4, LOW);

  //Starting the serial comm
  Serial.begin(115200);
}

void loop()
{
  int action = displayMenu();
  switch (action)
  {
  case 1:
    driveForward();
    break;
  case 2:
    driveBackward();
    break;
  case 3:
    driveLeft();
    break;
  case 4:
    driveRight();
    break;
  case 5:
    driveStop();
    break;

  default:    
    break;
  }
  delay(1000);
}

int displayMenu()
{
  int selection;

  Serial.println("Select an option:");
  Serial.println("-----------------");
  Serial.println("1) Move forward");
  Serial.println("2) Move backward");
  Serial.println("3) Turn left");
  Serial.println("4) Turn right");
  Serial.println("5) Stop");
  Serial.flush();

  while (!Serial.available())
  {
  }                              //Waits for an input on the serial device
  selection = Serial.parseInt(); //Takes the Serial input and looks for an integer
  Serial.flush();

  return selection;
}

void driveForward()
{
  Serial.println("^^^ FORWARD ^^^");
  Serial.flush();
  driveMotor(leftMotor, moveForward);
  driveMotor(rightMotor, moveForward);
}

void driveBackward()
{
  Serial.println("vvv BACKWARD vvv");
  Serial.flush();
  driveMotor(leftMotor, moveBackward);
  driveMotor(rightMotor, moveBackward);
}

void driveLeft()
{
  Serial.println("<<< LEFT <<<");
  Serial.flush();
  driveMotor(leftMotor, moveForward);
  driveMotor(rightMotor, moveStop);
}

void driveRight()
{
  Serial.println(">>> RIGHT >>>");
  Serial.flush();
  driveMotor(leftMotor, moveStop);
  driveMotor(rightMotor, moveForward);
}

void driveStop()
{
  Serial.println("*** STOP ***");
  Serial.flush();
  driveMotor(leftMotor, moveStop);
  driveMotor(rightMotor, moveStop);
}

void driveMotor(int motor, int moveDirection)
{
  int pin1;
  int pin2;
  int speed1;
  int speed2;
  const int moveSpeed = HIGH; // full speed or nothing for now, lower speeds don't seem to work consistently

  if (motor == leftMotor)
  {
    pin1 = motorPin1;
    pin2 = motorPin2;
  }
  else
  {
    pin1 = motorPin3;
    pin2 = motorPin4;
  }

  if (moveDirection == moveForward)
  {
    speed1 = moveSpeed;
    speed2 = LOW;
  }
  else if (moveDirection == moveBackward)
  {
    speed1 = LOW;
    speed2 = moveSpeed;
  }
  else
  {
    speed1 = LOW;
    speed2 = LOW;
  }
  digitalWrite(pin1, speed1);
  digitalWrite(pin2, speed2);
}
