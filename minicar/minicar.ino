#include <StackArray.h>
#include "Ticker.h"
#include <EEPROM.h>

typedef unsigned long millis_t;

Ticker line_tracking;
Ticker key_mode;
Ticker ir_recevie;
Ticker voltage_measure;

int addr_line_tracking_threshold = 0;

int key_value = 0;

bool is_left_line_tracking = false;
bool is_right_line_tracking = false;
bool is_ir_recevie = false;

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
int line_tracking_threshold = 150;
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
    delay(100);
    leftStop();
    rightStop();
    delay(1);
  }

  void back(int speed)
  {
    leftBack(speed);
    rightBack(speed);
    delay(100);
    leftStop();
    rightStop();
  }

  void left(int speed)
  {
    leftBack(speed);
    rightFront(speed);
    delay(1000);
    leftStop();
    rightStop();
  }

  void right(int speed)
  {
    leftFront(speed);
    rightBack(speed);
    delay(100);
    leftStop();
    rightStop();
  }

  void stop()
  {
    leftStop();
    rightStop();
    delay(10);
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

void getLineTrackingData()
{
  static int line_tracking_left_data;
  static int line_tracking_right_data;
  static int ir_recevie_data;
  line_tracking_left_data = analogRead(A1);
  line_tracking_right_data = analogRead(A0);
  ir_recevie_data = analogRead(A2);
  is_left_line_tracking = line_tracking_left_data >= line_tracking_threshold ? true : false;
  is_right_line_tracking = line_tracking_right_data >= line_tracking_threshold ? true : false;
  is_ir_recevie = ir_recevie_data >= 900 ? true : false;
}

void lineTrackingInit()
{
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
  pinMode(A2,INPUT);
  int t = 0;
  EEPROM.get(addr_line_tracking_threshold, t);
  if (t != -1)
  {
    line_tracking_threshold = t;
  }
  line_tracking.start(getLineTrackingData, 20);
}

void lineTrackingMode()
{
  l293.car_speed = map(voltage * 10, 3.6 * 10, 4.2 * 10, 150, 100);//200 150
  int tSpeed=170;

  //Forward
  if (is_ir_recevie)
  {
    l293.forward(l293.car_speed-30);
    delay(80);
  }

  //U-turn
  if (!is_ir_recevie) 
   {
    path+='B';
     l293.leftStop();
     l293.rightStop();
     delay(500);
     l293.leftBack(tSpeed);
     l293.rightBack(tSpeed);
     delay(400);
     l293.leftStop();
     l293.rightStop();
     delay(500);
     l293.leftBack(tSpeed);
     l293.rightFront(tSpeed);
     delay(700);
     l293.leftStop();
     l293.rightStop();
     delay(500);
     l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(130);
   }

  //T turn
    if (is_ir_recevie && is_left_line_tracking && is_right_line_tracking) 
  {
    path+='L';
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftBack(tSpeed);
    l293.rightFront(tSpeed);
    delay(400);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //Cross
  if (is_ir_recevie && !is_left_line_tracking && !is_right_line_tracking) 
  {
    path+='L';
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftBack(tSpeed);
    l293.rightFront(tSpeed);
    delay(400);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //Right turn
  if (!is_ir_recevie && is_left_line_tracking && !is_right_line_tracking) 
  {
    path+='R';
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftBack(tSpeed);
    l293.rightFront(tSpeed);
    delay(400);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //Right T
  if (is_ir_recevie && is_left_line_tracking && !is_right_line_tracking) 
  {
    path+='S';
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //Left T && left turn
  if (is_ir_recevie && !is_left_line_tracking && is_right_line_tracking) 
  {
    path+='L';
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftBack(tSpeed);
    l293.rightFront(tSpeed);
    delay(400);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //Stop
  if (!is_ir_recevie && !is_left_line_tracking && is_right_line_tracking) 
  {
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.shortPath();
    int s=200;

    for(int i=0;i<path.length();i++)
    {
      if(path[i]=='L')
      {
        l293.left(s);
      }
      else if(path[i]=='R')
      {
        l293.right(s);
      }
      else
      {
        l293.forward(s);
      }
    }
  }

}

void tremaux()
{
   l293.car_speed = map(voltage * 10, 3.6 * 10, 4.2 * 10, 150, 100);//200 150
  int tSpeed=170;
  StackArray<char> tPath, mPath;

  //Forward
  if (is_ir_recevie)
  {
    l293.forward(l293.car_speed-30);
    delay(80);
  }

  //Left T
  if (is_ir_recevie && !is_left_line_tracking && is_right_line_tracking) 
  {
    tPath.push("L");
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftBack(tSpeed);
    l293.rightFront(tSpeed);
    delay(400);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

//Right T
  if (is_ir_recevie && is_left_line_tracking && !is_right_line_tracking) 
  {
    tPath.push("R");
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //Right turn
  if (!is_ir_recevie && is_left_line_tracking && !is_right_line_tracking) 
  {
    tPath.push("A");
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftBack(tSpeed);
    l293.rightFront(tSpeed);
    delay(400);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //Cross
  if (is_ir_recevie && !is_left_line_tracking && !is_right_line_tracking) 
  {
    tPath.push("C");
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftBack(tSpeed);
    l293.rightFront(tSpeed);
    delay(400);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //T turn
    if (is_ir_recevie && is_left_line_tracking && is_right_line_tracking) 
  {
    tPath.push("T");
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(200);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftBack(tSpeed);
    l293.rightFront(tSpeed);
    delay(400);
    l293.leftStop();
    l293.rightStop();
    delay(500);
    l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(100);
  }

  //U-turn
  if (!is_ir_recevie) 
   {
    int s=200;
     l293.leftStop();
     l293.rightStop();
     delay(500);
     l293.leftBack(tSpeed);
     l293.rightBack(tSpeed);
     delay(400);
     l293.leftStop();
     l293.rightStop();
     delay(500);
     l293.leftBack(tSpeed);
     l293.rightFront(tSpeed);
     delay(700);
     l293.leftStop();
     l293.rightStop();
     delay(500);
     l293.leftFront(tSpeed);
    l293.rightFront(tSpeed);
    delay(130);

    
    while(!tPath.isEmpty()&& (is_ir_recevie || is_left_line_tracking || is_right_line_tracking))
    {
      int pop=tPath.pop();
      if(pop=="R")
      {
        if(is_left_line_tracking&&is_right_line_tracking)
        {
          l293.right(s);
          mPath.push("S");
        }
        else
        {
          l293.left(s);
          mPath.push("A");
        }
      }
      else if(pop=="L")
      {
        if(is_left_line_tracking&&is_right_line_tracking)
        {
          l293.left(s);
          mPath.push("S");
        }
        else
        {
          l293.right(s);
          mPath.push("B");
        }
      }
      else if (pop=="S")
      {
        if(is_left_line_tracking)
        {
          l293.right(s);
          mPath.push("L");
        }
        else if (is_right_line_tracking)
        {
          l293.left(s);
          mPath.push("R");
        }
      }
  }
  }

}

void setup()
{
  Serial.begin(9600);
  lineTrackingInit();
}

void loop()
{
  line_tracking.update();
  key_mode.update();
  ir_recevie.update();
  lineTrackingMode();
}
