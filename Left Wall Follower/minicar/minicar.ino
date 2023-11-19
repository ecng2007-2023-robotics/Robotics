#include "Ticker.h"
#include <EEPROM.h>

typedef unsigned long Millis_t;

Ticker lineTracking;
Ticker keyMode;
Ticker irReceive;
Ticker voltageMeasure;

int addrLineTrackingThreshold = 0;

int keyValue = 0;

bool farLeft = false;
bool isLeftLineTracking = false;
bool isRightLineTracking = false;
bool farRight = false;
bool isIrReceive = false;
bool middle = false;

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

String path;

class L293
{
public:
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

  enum RunStatus
  {
    STOP,
    BACK,
    FORWARD,
    LEFT,
    RIGHT
  } runStatus = STOP;

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

  String shortPath()
  {
    int n = path.length();

    if (n <= 3)
    {
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

bool getLineTrackingData()
{
  static int lineTrackingLeftData;
  static int lineTrackingRightData;
  static int irReceiveData;

  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  delay(1);

  pinMode(6, INPUT);
  delay(1);

  byte midIr = digitalRead(6);

  lineTrackingLeftData = analogRead(A1);
  lineTrackingRightData = analogRead(A0);
  irReceiveData = analogRead(A2);
  isLeftLineTracking = lineTrackingLeftData >= 160 ? true : false;
  isRightLineTracking = lineTrackingRightData >= 200 ? true : false;
  isIrReceive = irReceiveData >= 989 ? true : false;
  return (midIr);
}

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

void lineTrackingInit()
{
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
  int t = 0;
  EEPROM.get(addrLineTrackingThreshold, t);
  if (t != -1)
  {
    lineTrackingThreshold = t;
  }
  lineTracking.start(getLineTrackingData, 20);
}

void lineTrackingMode(bool front, bool left, bool right, bool back)
{
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

  l293.carSpeed = map(voltage * 10, 3.6 * 10, 4.2 * 10, 200, 150);
  int tSpeed = 170;
  int x = 590, y = 275;
  int fo = 350;
  int lF = 200;

  if (front == 0 && isLeftLineTracking)
  {
    l293.leftBack(180);
    l293.rightFront(170);
  }
  if (front == 0 && isRightLineTracking)
  {
    l293.leftFront(180);
    l293.rightBack(170);
  }
  // Forward
  if (isLeftLineTracking == 0 && back == 1 && isRightLineTracking == 0)
  {
    l293.leftFront(180);
    l293.rightFront(170);
    delay(80);
  }

  // U
  if (!isIrReceive)
  {
    path += "B";
    l293.stop();
    delay(1000);
    l293.leftBack(220);
    l293.rightBack(170);
    delay(500);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(x);
  }

  // Left-T
  if (left == 1 && front == 1 && back == 1 && right == 0)
  {
    path += "L";
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  // Cross
  if (left == 1 && front == 1 && back == 1 && right == 1)
  {
    path += "L";
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  // T
  if (left == 1 && front == 0 && back == 1 && right == 1)
  {
    path += "L";
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  // Right-T
  if (left == 0 && front == 1 && back == 1 && right == 1)
  {
    path += "S";
    l293.leftFront(170);
    l293.rightFront(170);
    delay(80);
  }

  // Right
  if (left == 0 && front == 0 && back == 1 && right == 1)
  {
    path += "R";
    l293.leftFront(170);
    l293.rightFront(170);
    delay(410);
    l293.leftFront(230);
    l293.rightBack(170);
    delay(500);
  }

  // Left
  if (left == 1 && front == 0 && back == 1 && right == 0)
  {
    path += "L";
    l293.leftFront(170);
    l293.rightFront(170);
    delay(410);
    l293.rightFront(170);
    l293.leftBack(230);
    delay(500);
  }

  // Stop
  if (left == 0 && isLeftLineTracking == 0 && front == 0 && back == 0 && isRightLineTracking == 0 && right == 0)
  {
    l293.stop();
    delay(1000);
    path = l293.shortPath();
    int s = 200;
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
      }
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Testing OTI Sensor");
  tone(4, 4000, 500);
  delay(500);
  lineTrackingInit();
}

void loop()
{
  bool f, r, l, b;
  lineTracking.update();
  keyMode.update();
  irReceive.update();
  f = getLineTrackingData();
  l = getLeft();
  r = getRight();
  b = getBack();
  lineTrackingMode(f, l, r, b);
}
