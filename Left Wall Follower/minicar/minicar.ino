#include "Ticker.h"
#include <EEPROM.h>

typedef unsigned long millis_t;

Ticker line_tracking;
Ticker key_mode;
Ticker ir_recevie;
Ticker voltage_measure;

int addr_line_tracking_threshold = 0;

int key_value = 0;

bool far_left = false;
bool is_left_line_tracking = false;
bool is_right_line_tracking = false;
bool far_right = false;
bool is_ir_recevie = false;
bool middle = false;

#define MAX_CMD_SIZE 96
#define BUFSIZE 4
uint8_t commands_in_queue = 0;
uint8_t cmd_queue_index_r = 0;
uint8_t cmd_queue_index_w = 0;
char command_queue[BUFSIZE][MAX_CMD_SIZE];
static int serial_count;
#define parameter_num_max 6          
String parameter[parameter_num_max]; 
float get_time = 0;
static millis_t get_time_delay;
int line_tracking_threshold = 300;
double voltage;

String path;

class L293
{
public:
  int left_1_pin=7;
  int left_2_pin=8;
  int right_1_pin=12;
  int right_2_pin=4;
  int enable_left_pin=11;
  int enable_right_pin=10;
  int car_speed = 0;
  int turn_speed = 0;
  uint16_t left_speed = 0;
  uint16_t right_speed = 0;

  enum RUN_STATUE
  {
    STOP,
    BACK,
    FORWARD,
    LEFT,
    RIGHT
  } run_statue = STOP;

  void leftFront(int leftspeed)
  {
    analogWrite(enable_left_pin, leftspeed);
    digitalWrite(left_1_pin, HIGH);
    digitalWrite(left_2_pin, LOW);
  }

  void leftBack(int leftspeed)
  {
    analogWrite(enable_left_pin, leftspeed);
    digitalWrite(left_1_pin, LOW);
    digitalWrite(left_2_pin, HIGH);
  }

  void leftStop()
  {
    analogWrite(enable_left_pin, 0);
    digitalWrite(left_1_pin, LOW);
    digitalWrite(left_2_pin, LOW);
  }

  void rightFront(int rightspeed)
  {
    analogWrite(enable_right_pin, rightspeed);
    digitalWrite(right_1_pin, LOW);
    digitalWrite(right_2_pin, HIGH);
  }

  void rightBack(int rightspeed)
  {
    analogWrite(enable_right_pin, rightspeed);
    digitalWrite(right_1_pin, HIGH);
    digitalWrite(right_2_pin, LOW);
  }

  void rightStop()
  {
    analogWrite(enable_right_pin, 0);
    digitalWrite(right_1_pin, LOW);
    digitalWrite(right_2_pin, LOW);
  }

  void forward(int speed)
  {
    leftFront(speed);
    rightFront(speed);
  }

  void back(int speed)
  {
    run_statue = BACK;
    left_speed = speed;
    right_speed = speed;
    leftBack(speed);
    rightBack(speed);
  }

  void left(int speed)
  {
    run_statue = LEFT;
    left_speed = speed;
    right_speed = speed;
    leftBack(speed);
    rightFront(speed);
  }

  void right(int speed)
  {
    run_statue = RIGHT;
    left_speed = speed;
    right_speed = speed;
    leftFront(speed);
    rightBack(speed);
  }

  void stop()
  {
    run_statue = STOP;
    left_speed = 0;
    right_speed = 0;
    car_speed = 0;
    turn_speed = 0;
    leftStop();
    rightStop();
  }

  String shortPath()
  {
    int n=path.length();

    if(n<=3)
    {
      path.replace("LBL","S");
      path.replace("LBR","B");
      path.replace("LBS","R");
      path.replace("RBL","B");
      path.replace("SBL","R");
      path.replace("SBS","B");
      return shortPath();
    }
    return path;
  }

private:
} l293;

bool getLineTrackingData()
{
  static int line_tracking_left_data;
  static int line_tracking_right_data;
  static int ir_recevie_data;

  pinMode(6,OUTPUT);
  digitalWrite(6, HIGH);
  delay (1);

  pinMode(6, INPUT);
  delay(1);

  byte mid_ir = digitalRead(6);

  line_tracking_left_data = analogRead(A1);
  line_tracking_right_data = analogRead(A0);
  ir_recevie_data=analogRead(A2);
  is_left_line_tracking = line_tracking_left_data >= 160 ? true : false;
  is_right_line_tracking = line_tracking_right_data >= 200 ? true : false;
  is_ir_recevie = ir_recevie_data >= 989 ? true : false;
  return(mid_ir);
}

bool getLeft()
{
  pinMode(A5,OUTPUT);
  digitalWrite(A5, HIGH);
  delay (1);

  pinMode(A5, INPUT);
  delay(1);

  
  byte left_ir = digitalRead(A5);

  return (left_ir);
}

bool getRight()
{
  pinMode(9,OUTPUT);
  digitalWrite(9, HIGH);
  delay (1);

  pinMode(9, INPUT);
  delay(1);

  
  byte right_ir = digitalRead(9);

  return (right_ir);
}

bool getBack()
{
  pinMode(A4,OUTPUT);
  digitalWrite(A4, HIGH);
  delay (1);

  pinMode(A4, INPUT);
  delay(1);

  
  byte back_ir = digitalRead(A4);

  return (back_ir);
}

void lineTrackingInit()
{
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
  int t = 0;
  EEPROM.get(addr_line_tracking_threshold, t);
  if (t != -1)
  {
    line_tracking_threshold = t;
  }
  line_tracking.start(getLineTrackingData, 20);
}

void lineTrackingMode(bool front, bool left, bool right, bool back)
{
  Serial.print(left);
  Serial.print("\t");
  Serial.print(is_left_line_tracking);
  Serial.print("\t");
  Serial.print(front);
  Serial.print("\t");
  Serial.print(is_ir_recevie);
  Serial.print("\t");
  Serial.print(back);
  Serial.print("\t");
  Serial.print(is_right_line_tracking);
  Serial.print("\t");
  Serial.print(right);
  Serial.print("\n");

  l293.car_speed = map(voltage * 10, 3.6 * 10, 4.2 * 10, 200, 150);
  int tSpeed=170;
  int x=590, y=275;
  int fo=350;
  int lF=200;

  if (front==0&&is_left_line_tracking)
  {
    l293.leftBack(180);
    l293.rightFront(170);
  }
  if (front==0&&is_right_line_tracking)
  {
    l293.leftFront(180);
    l293.rightBack(170);
  }
  //Forward
  if(is_left_line_tracking==0&&back==1&&is_right_line_tracking==0)
  {
    l293.leftFront(180);
    l293.rightFront(170);
    delay(80);
  }

  //U
  if (!is_ir_recevie) 
  {
    path =+ "B";
    l293.stop();
    delay(1000);
    l293.leftBack(220);
    l293.rightBack(170);
    delay(500);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(x);
  }

  //Left-T
  if(left==1&&front==1&&back==1&&right==0)
  {
    path =+ "L";
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  //Cross
  if(left==1&&front==1&&back==1&&right==1)
  {
    path =+ "L";
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  //T
  if(left==1&&front==0&&back==1&&right==1)
  {
    path =+ "L";
    l293.leftFront(lF);
    l293.rightFront(180);
    delay(fo);
    l293.leftBack(230);
    l293.rightFront(170);
    delay(y);
  }

  //Right-T
  if(left==0&&front==1&&back==1&&right==1)
  {
    path =+ "S";
    l293.leftFront(170);
    l293.rightFront(170);
    delay(80);
  }
  
  //Right
  if(left==0&&front==0&&back==1&&right==1)
  {
    path =+ "R";
    l293.leftFront(170);
    l293.rightFront(170);
    delay(410);
    l293.leftFront(230);
    l293.rightBack(170);
    delay(500);
  }

  //Left
  if(left==1&&front==0&&back==1&&right==0)
  {
    path =+ "L";
    l293.leftFront(170);
    l293.rightFront(170);
    delay(410);
    l293.rightFront(170);
    l293.leftBack(230);
    delay(500);
  }

  //Stop 
  if(left==0&&is_left_line_tracking==0&&front==0&&back==0&&is_right_line_tracking==0&&right==0) 
  { 
    l293.stop(); 
    delay(1000); 
    path=l293.shortPath(); 
    int s=200; 
    for(int i=0;i<path.length();i++) 
    { 
      if(path[i]=="L") 
      { 
        l293.left(s); 
      } 
      else if(path[i]=="R") 
      {   
        l293.right(s); 
      } 
      else if(path[i]=="S")
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
  bool f,r,l,b; 
  line_tracking.update();
  key_mode.update();
  ir_recevie.update();
  f=getLineTrackingData();
  l=getLeft();
  r=getRight();
  b=getBack();
  lineTrackingMode(f,l,r,b);
}