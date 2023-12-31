#include "Ticker.h"
#include <EEPROM.h>

//type for storing millisecond values
typedef unsigned long Millis_t;

//instances of the ticker library created for the various tasks
Ticker lineTracking;
Ticker keyMode;
Ticker irReceive;
Ticker voltageMeasure;

//Address for storing line tracking threshold
int addrLineTrackingThreshold = 0;

//Variable to store the key value
int keyValue = 0;

//Variables that store line tracking information for the sensors
bool farLeft = false; //additional decision-making sensor 
bool isLeftLineTracking = false;
bool isRightLineTracking = false;
bool farRight = false; //additional sensor
bool isIrReceive = false;
bool middle = false;

//Defining constants for the command queue
#define MAX_CMD_SIZE 96
#define BUFSIZE 4
uint8_t commandsInQueue = 0;
uint8_t cmdQueueIndexR = 0;
uint8_t cmdQueueIndexW = 0;
char commandQueue[BUFSIZE][MAX_CMD_SIZE];
static int serialCount;
#define parameterNumMax 6
String parameter[parameterNumMax];
float getTime = 0;
static Millis_t getTimeDelay;
int lineTrackingThreshold = 300;
double voltage;

String path; //Variable to store the robot's path

//Class for controlling the L239 motor driver
class L293
{
public:
//pin assignments for the motor driver
  int left1Pin = 7;
  int left2Pin = 8;
  int right1Pin = 12;
  int right2Pin = 4;
  int enableLeftPin = 11;
  int enableRightPin = 10;
  int carSpeed = 0;
  int turnSpeed = 0;
  uint16_t leftSpeed = 0;
  uint16_t rightSpeed = 0;

//enum for different run states of the robot
  enum RunStatus
  {
    STOP,
    BACK,
    FORWARD,
    LEFT,
    RIGHT
  } runStatus = STOP;

//Methods to control the left motor
  void leftFront(int leftSpeed)
  {
    analogWrite(enableLeftPin, leftSpeed);
    digitalWrite(left1Pin, HIGH);
    digitalWrite(left2Pin, LOW);
  }

  void leftBack(int leftSpeed)
  {
    analogWrite(enableLeftPin, leftSpeed);
    digitalWrite(left1Pin, LOW);
    digitalWrite(left2Pin, HIGH);
  }

  void leftStop()
  {
    analogWrite(enableLeftPin, 0);
    digitalWrite(left1Pin, LOW);
    digitalWrite(left2Pin, LOW);
  }

//Methods to control the right motor
  void rightFront(int rightSpeed)
  {
    analogWrite(enableRightPin, rightSpeed);
    digitalWrite(right1Pin, LOW);
    digitalWrite(right2Pin, HIGH);
  }

  void rightBack(int rightSpeed)
  {
    analogWrite(enableRightPin, rightSpeed);
    digitalWrite(right1Pin, HIGH);
    digitalWrite(right2Pin, LOW);
  }

  void rightStop()
  {
    analogWrite(enableRightPin, 0);
    digitalWrite(right1Pin, LOW);
    digitalWrite(right2Pin, LOW);
  }

//Methods to control the robot's movement
  void forward(int speed)
  {
    leftFront(speed);
    rightFront(speed);
  }

  void back(int speed)
  {
    runStatus = BACK;
    leftSpeed = speed;
    rightSpeed = speed;
    leftBack(speed);
    rightBack(speed);
  }

  void left(int speed)
  {
    runStatus = LEFT;
    leftSpeed = speed;
    rightSpeed = speed;
    leftBack(speed);
    rightFront(speed);
  }

  void right(int speed)
  {
    runStatus = RIGHT;
    leftSpeed = speed;
    rightSpeed = speed;
    leftFront(speed);
    rightBack(speed);
  }

//Stop function keeps the speed at 0
  void stop()
  {
    runStatus = STOP;
    leftSpeed = 0;
    rightSpeed = 0;
    carSpeed = 0;
    turnSpeed = 0;
    leftStop();
    rightStop();
  }

//Optimization algorithm, method to shorten the recorded path
  String shortPath()
  {
    int n = path.length();

    if (n <= 3)
    {
      //where L-Left, B- Back, R-Right, S-Straight
      path.replace("LBL", "S");
      path.replace("LBR", "B");
      path.replace("LBS", "R");
      path.replace("RBL", "B");
      path.replace("SBL", "R");
      path.replace("SBS", "B");
      return shortPath();
    }
    return path;
  }

private:
} l293;

//Function to get line tracking data
bool getLineTrackingData()
{
  static int lineTrackingLeftData;
  static int lineTrackingRightData;
  static int irReceiveData;

  //Initializes and reads the middle sensor
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  delay(1);

  pinMode(6, INPUT);
  delay(1);

  byte midIr = digitalRead(6);

  //Reads line tracking and Infared (ir) receive sensors -(manufacturer code)
  lineTrackingLeftData = analogRead(A1);
  lineTrackingRightData = analogRead(A0);
  irReceiveData = analogRead(A2);
  isLeftLineTracking = lineTrackingLeftData >= 160 ? true : false;
  isRightLineTracking = lineTrackingRightData >= 200 ? true : false;
  isIrReceive = irReceiveData >= 989 ? true : false;
  return (midIr);
}

//Function to get data from the left sensor
bool getLeft()
{
  pinMode(A5, OUTPUT);
  digitalWrite(A5, HIGH);
  delay(1);

  pinMode(A5, INPUT);
  delay(1);

  byte leftIr = digitalRead(A5);

  return (leftIr);
}

//Function to get data from the right sensor
bool getRight()
{
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  delay(1);

  pinMode(9, INPUT);
  delay(1);

  byte rightIr = digitalRead(9);

  return (rightIr);
}

//Function to get data from the middle back sensor
bool getBack()
{
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH);
  delay(1);

  pinMode(A4, INPUT);
  delay(1);

  byte backIr = digitalRead(A4);

  return (backIr);
}

//Initialization function for line tracking
void lineTrackingInit()
{
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
  int t = 0;
  EEPROM.get(addrLineTrackingThreshold, t); //Reads line tracking threshold from EEPROM value declared at the start of code
  if (t != -1)
  {
    lineTrackingThreshold = t;
  }
  lineTracking.start(getLineTrackingData, 20);
}

//Function to control the robot based on line tracking and infrared data
void lineTrackingMode(bool front, bool left, bool right, bool back)
{
  //Prints debugging information (manufacturer code)
  Serial.print(left);
  Serial.print("\t");
  Serial.print(isLeftLineTracking);
  Serial.print("\t");
  Serial.print(front);
  Serial.print("\t");
  Serial.print(isIrReceive);
  Serial.print("\t");
  Serial.print(back);
  Serial.print("\t");
  Serial.print(isRightLineTracking);
  Serial.print("\t");
  Serial.print(right);
  Serial.print("\n");

  //maps voltage to the car speed
  l293.carSpeed = map(voltage * 10, 3.6 * 10, 4.2 * 10, 200, 150);
  //constants for movement calculations
  int tSpeed = 170;
  int x = 590, y = 275;
  int fo = 350;
  int lF = 200;

  //Line follower for track alignment
 if(left==1 && isLeftLineTracking==1 && front==0 && back==0 && isRightLineTracking==0 && right==0)
 {
   l293.stop();
    delay(1000);
 }
  //if the robot comes off the track completely, then it will move until line is found
  if (front ==0&& isLeftLineTracking==0 && isRightLineTracking==0&&back==0)
  {
    l293.stop();
    l293.leftBack(180);
    l293.rightFront(170);
    delay(200);
    l293.leftFront(180);
    l293.rightBack(170);
    delay(200);
  }
  if (isLeftLineTracking)
  {
    l293.leftBack(150);
    l293.rightFront(130);
  }
  if (isRightLineTracking)
  {
    l293.leftFront(150);
    l293.rightBack(130);
  }

//alignment for when the robot oversteers
  if ((left==1||isLeftLineTracking)&&front==0&&back==0)
  {
    l293.leftBack(150);
    l293.rightFront(140);
  }
if ((right==1||isRightLineTracking)&&front==0&&back==0)
  {
    l293.leftFront(150);
    l293.rightBack(140);
  }
  
  //Forward condition
  if(isLeftLineTracking==0&&back==1&&front==1&&isRightLineTracking==0)
  {
    l293.leftFront(140);
    l293.rightFront(130);
    delay(100);
  }
  
  //Deadend condition
  if (!isIrReceive) 
  {
    l293.stop();
    delay(1000);
    l293.leftBack(220);
    l293.rightBack(170);
    delay(500);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(x);
  }

  //Left-T condition
  if(left==1&&front==1&&back==1&&right==0)
  {
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  //Cross condition
  if(left==1&&front==1&&back==1&&right==1)
  {
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  //T condition
  if(left==1&&front==0&&back==1&&right==1)
  {
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  //Right-T condition
  if(left==0&&front==1&&back==1&&right==1&&back==1)
  {
    l293.leftFront(170);
    l293.rightFront(170);
    delay(80);
  }
  
  //Right condition
  if(left==0&&front==0&&back==1&&right==1)
  {
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftFront(230);
    l293.rightBack(170);
    delay(460);
  }

  //Left condition
  if(left==1&&front==0&&back==1&&right==0)
  {
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }
  
  

  // Stop
  if (left == 0 && isLeftLineTracking == 0 && front == 0 && back == 0 && isRightLineTracking == 0 && right == 0)
  {
    l293.stop();
    delay(1000);
    //optimized path conditions, requires further testing
    /*
    path = l293.shortPath();
    int s = 200;

    //Excecutes the recorded path
    for (int i = 0; i < path.length(); i++)
    {
      if (path[i] == "L")
      {
        l293.left(s);
      }
      else if (path[i] == "R")
      {
        l293.right(s);
      }
      else if (path[i] == "S")
      {
        l293.forward(s);
      }*/
    }
  }
}

//Initializes the robot
void setup()
{
  //Initializes serial communication and prints a message
  Serial.begin(9600);
  Serial.println("Testing OTI Sensor");
  //Generates a tone as a signal
  tone(4, 4000, 500);
  delay(500);
  //Initializes line tracking parameters
  lineTrackingInit();
}

//loops initialize repeatedly in arduino unlike C++
void loop()
{
  //Variables to store sensor data
  bool f, r, l, b;

  //Updates Ticker tasks
  lineTracking.update();
  keyMode.update();
  irReceive.update();

  //Gets sensor data
  f = getLineTrackingData();
  l = getLeft();
  r = getRight();
  b = getBack();
//Performs line tracking mode logic
  lineTrackingMode(f, l, r, b);
}
