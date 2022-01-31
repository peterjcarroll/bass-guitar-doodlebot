#include <Servo.h>
#include "arduinoFFT.h"
 
#define SAMPLES 128             //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 2048 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
 
arduinoFFT FFT = arduinoFFT();
 
unsigned int samplingPeriod;
unsigned long microSeconds;
 
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values

//Driving constants
const int leftMotor = 0;
const int rightMotor = 1;

const int moveStop = 0;
const int moveForward = 1;
const int moveBackward = 2;

//L293D
//Motor A
const int motorPin1 = 5; // Pin 14 of L293
const int motorPin2 = 6; // Pin 10 of L293
//Motor B
const int motorPin3 = 10; // Pin  7 of L293
const int motorPin4 = 9;  // Pin  2 of L293

const int allowedRange = 3;
double lastNote = 0;

Servo penServo;
const int penUpPos = 70;
const int penDownPos = 85;
const int penDelay = 100;

//This will run only one time.
void setup()
{

  //Set pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  penServo.attach(11); //servo is on pin 11

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

  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds 

  //Starting the serial comm
  Serial.begin(115200);
}

void loop()
{
  const int turnDelay = 250;
  const int driveDelay = 500;

  double note = waitForNote();
  Serial.print("***** Found note: ");
  Serial.println(note);

  if(lastNote == 0) { // No previous notes, start the motors
    penServo.write(penDownPos); // put the pen down
    delay(penDelay);
    driveForward();
    delay(driveDelay);
    lastNote = note;
  }
  else if(lastNote > (note + allowedRange)) // lower note, turn left
  {
    driveLeft();
    delay(turnDelay);
    driveForward();
    delay(driveDelay);
    lastNote = 0;
  }
  else if(lastNote < (note - allowedRange)) // higher note, turn right
  {
    driveRight();
    delay(turnDelay);
    driveForward();
    delay(driveDelay);
    lastNote = 0;
  }
  else { // same note as last time, go back
    penServo.write(penUpPos); // bring the pen up
    delay(penDelay);
    driveBackward();
    delay(driveDelay);
    lastNote = 0;
  }
  driveStop();


  // int action = displayMenu();
  // switch (action)
  // {
  // case 1:
  //   driveForward();
  //   delay(1000);
  //   break;
  // case 2:
  //   driveBackward();
  //   delay(1000);
  //   break;
  // case 3:
  //   driveLeft();
  //   delay(1000);
  //   break;
  // case 4:
  //   driveRight();
  //   delay(1000);
  //   break;
  // case 5:
  //   driveStop();
  //   delay(1000);
  //   break;
  // case 6:
  //   int servoPos = promptServoPosition();
  //   penServo.write(servoPos);
  //   break;

  // default:    
  //   break;
  // }
  
}

double waitForNote()
{
  double lastFreq = 0;
  int consecutive = 0;
  
  while (consecutive < 3)
  {
    double freq = getAudioSampleFreq();
    // Serial.println(freq);
    if(freq <= lastFreq + allowedRange && freq >= lastFreq - allowedRange)
    {
      consecutive++;
    }
    else {
      consecutive = 0;
    }
    lastFreq = freq;
  }
  return lastFreq;
}

double getAudioSampleFreq()
{
  /*Sample SAMPLES times*/
  for(int i=0; i<SAMPLES; i++)
  {
      microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script. 
    
      vReal[i] = analogRead(0); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
      vImag[i] = 0; //Makes imaginary term 0 always

      /*remaining wait time between samples if necessary*/
      while(micros() < (microSeconds + samplingPeriod))
      {
        //do nothing
      }
  }

  /*Perform FFT on samples*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  /*Find peak frequency and print peak*/
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  //Serial.println(peak);     //Print out the most dominant frequency.
  return peak;
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
  Serial.println("6) Move servo");
  Serial.println("7) Sample audio");
  Serial.flush();

  while (!Serial.available()) {} //Waits for an input on the serial device
  selection = Serial.parseInt(); //Takes the Serial input and looks for an integer
  Serial.flush();

  return selection;
}

int promptServoPosition()
{
  int position;

  Serial.println("Enter servo position (0-180):");
  Serial.flush();
  delay(5000);

  while (!Serial.available()) {} //Waits for an input on the serial device
  position = Serial.parseInt(); //Takes the Serial input and looks for an integer
  Serial.flush();

  return position;
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
