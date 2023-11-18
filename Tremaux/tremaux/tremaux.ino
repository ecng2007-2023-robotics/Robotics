#include <EEPROM.h>
#define maze_length 7
#define maze_width 7
#define THRESHOLD 60

typedef unsigned long millis_t;

unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

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

private:
} l293;

bool check_sensors()
{
  static int line_tracking_left_data;
  static int line_tracking_right_data;
  static int ir_recevie_data;

  pinMode(6,OUTPUT);
  digitalWrite(6, HIGH);
  delay(1);

  pinMode(6, INPUT);
  delay(1);

  byte mid_ir = digitalRead(6);

  line_tracking_left_data = analogRead(A1);
  line_tracking_right_data = analogRead(A0);
  ir_recevie_data=analogRead(A2);
  Left_white_line = line_tracking_left_data >= 160 ? true : false;
  Right_white_line = line_tracking_right_data >= 160 ? true : false;
  Center_white_line = mid_ir;

  return(mid_ir);
}

//Tremaux code
int is_at_end;
int path[maze_length][maze_width][maze_length][maze_width];
int parent_of[maze_length][maze_width];
int on_optimal_path[maze_length][maze_width];

// Initialization of variables
void in() {
	int i, j, k, l;
	//init_devices();

	for (i = 0;i < maze_length;i++) {

		for (j = 0;j < maze_width;j++) {

			for (k = 0;k < maze_length;k++) {

				for (l = 0;l < maze_width;l++) {
					path[i][j][k][l] = 0;
				}
			}
		}
	}

	for (i = 0;i < maze_length;i++) {

		for (j = 0;j < maze_width;j++) {
			parent_of[i][j] = -1;
			on_optimal_path[i][j] = 0;
		}

	}
}

// Moves to given destination from given source point
void moveto(int arg1, int arg2, int arg3, int arg4) { // Should edit moveto function

	if (arg2 == arg4) {

		if (arg3 > arg1) {
			l293.forward(170);
			_delay_ms(1000);
			check_sensors();
			while (!(Left_white_line || Right_white_line) && Center_white_line) 
      {
				l293.forward(170);
				check_sensors();
			}
		}

		else {
			l293.back(170);
			_delay_ms(1000);
			check_sensors();
			while (!(Left_white_line|| Right_white_line) && Center_white_line) {
				l293.back(170);
				check_sensors();
			}
		}
	}

	else if (arg1 == arg3) {

		if (arg4 > arg2) {
			l293.left(170);
			l293.forward(170);
			_delay_ms(1000);
			check_sensors();
			while (!(Left_white_line|| Right_white_line) && Center_white_line) {
				l293.forward(170);
				check_sensors();
			}
			l293.right(170);
		}

		else {
			l293.right(170);
			l293.forward(170);
			_delay_ms(1000);
			check_sensors();
			while (!(Left_white_line < THRESHOLD || Right_white_line < THRESHOLD) && Center_white_line < THRESHOLD) {
			l293.forward(170);
				check_sensors();
			}
			l293.left(170);
		}
	}

}

// Checks whether the node has paths to its children or not
int no_out_path_from(int arg1, int arg2) {
	int child_x, child_y;
	int flag = 0;
	int turned_back = 0;

	child_x = arg1 + 1;
	child_y = arg2;

	if (child_x < maze_length && parent_of[arg1][arg2] != 10 * child_x + child_y) {
			l293.left(170);
			l293.forward(170);
		_delay_ms(1000);
		check_sensors();

		while (!(Left_white_line < THRESHOLD || Right_white_line < THRESHOLD) && Center_white_line < THRESHOLD) {
			l293.forward(170);
			check_sensors();
		}

		if (Center_white_line > THRESHOLD) {
			turned_back++;

			while (!(Left_white_line < THRESHOLD || Right_white_line < THRESHOLD)) {
				l293.back(170);
				check_sensors();
			}
			l293.right(170);

		}

		else {
			//back();
			_delay_ms(1000);

			while (!(Left_white_line < THRESHOLD || Right_white_line < THRESHOLD)) {
				l293.back(170);
				check_sensors();
			}
			l293.right(170);

		}

		if (!turned_back) flag++;
	}

	child_x = arg1 - 1;
	child_y = arg2;

	if (child_x > 0 && parent_of[arg1][arg2] != 10 * child_x + child_y) {
			l293.right(170);
			l293.forward(170);
		_delay_ms(1000);
		check_sensors();

		while (!(Left_white_line|| Right_white_line) && Center_white_line) {
			l293.forward(170);
			check_sensors();
		}

		if (Center_white_line > THRESHOLD) {
			turned_back++;

			while (!(Left_white_line|| Right_white_line)) {
				l293.back(170);
				check_sensors();
			}
			l293.left(170);

		}

		else {
				l293.back(170);
			_delay_ms(1000);

			while (!(Left_white_line|| Right_white_line)) {
				l293.back(170);
				check_sensors();
			}
			l293.left(170);

		}

		if (!turned_back) flag++;
	}

	child_x = arg1;
	child_y = arg2 + 1;

	if (child_y < maze_width && parent_of[arg1][arg2] != 10 * child_x + child_y) {
			l293.forward(170);
		_delay_ms(1000);
		check_sensors();

		while (!(Left_white_line|| Right_white_line) && Center_white_line) {
			l293.forward(170);
			check_sensors();
		}

		if (Center_white_line) {
			turned_back++;

			while (!(Left_white_line|| Right_white_line)) {
				l293.back(170);
				check_sensors();
			}

		}

		else {
				l293.back(170);
			_delay_ms(1000);

			while (!(Left_white_line|| Right_white_line)) {
				l293.back(170);
				check_sensors();
			}

		}

		if (!turned_back) flag++;
	}

	child_x = arg1;
	child_y = arg2 - 1;

	if (child_y > 0 && parent_of[arg1][arg2] != 10 * child_x + child_y) {
				l293.back(170);
		_delay_ms(1000);
		check_sensors();

		while (!(Left_white_line|| Right_white_line) && Center_white_line) {
				l293.back(170);
			check_sensors();
		}

		if (Center_white_line) {
			turned_back++;

			while (!(Left_white_line|| Right_white_line)) {
			l293.forward(170);
				check_sensors();
			}

		}

		else {
			l293.forward(170);
			_delay_ms(1000);

			while (!(Left_white_line|| Right_white_line)) {
			l293.forward(170);
				check_sensors();
			}

		}

		if (!turned_back) flag++;
	}

	if (flag == 0) return 1;
	else return 0;
}

// Checks whether the children have outward paths or not
int no_child_with_outpath(int arg1, int arg2) {
	int child_x, child_y;
	int flag = 0;

	child_x = arg1 + 1;
	child_y = arg2;

	if (child_x < maze_length && parent_of[arg1][arg2] != 10 * child_x + child_y) {
		l293.left(170);
    l293.forward(170);
		if (path[arg1][arg2][child_x][child_y] != 2) flag++;
	}

	child_x = arg1 - 1;
	child_y = arg2;

	if (child_x > 0 && parent_of[arg1][arg2] != 10 * child_x + child_y) {
		l293.right(170);
    l293.forward(170);
		if (path[arg1][arg2][child_x][child_y] != 2) flag++;
	}

	child_x = arg1;
	child_y = arg2 + 1;

	if (child_y < maze_width && parent_of[arg1][arg2] != 10 * child_x + child_y) {
    l293.forward(170);
		if (path[arg1][arg2][child_x][child_y] != 2) flag++;
	}

	child_x = arg1;
	child_y = arg2 - 1;

	if (child_y > 0 && parent_of[arg1][arg2] != 10 * child_x + child_y) {
    l293.back(170);
    l293.forward(170);
		if (path[arg1][arg2][child_x][child_y] != 2) flag++;
	}

	if (flag > 0) return 1;
	else return 0;
}

// Goto the parent of the given node
void goto_parent_of(int arg1, int arg2) {
	int current_x, current_y;
	int parent_x, parent_y, parent;

	current_x = arg1;
	current_y = arg2;
	parent = parent_of[current_x][current_y];
	parent_x = parent / 10;
	parent_y = parent % 10;
	on_optimal_path[current_x][current_y] = 0;

	moveto(current_x, current_y, parent_x, parent_y);
}

// Go to one of the children at random
int goto_child_of(int arg1, int arg2) {// Return an integer in this function to depict which child is the bot going to
	int current_x = arg1, current_y = arg2;
	int child_x, child_y, no_of_paths;
	int flag = 0, flagl = 0, flagr = 0, flags = 0, flagb = 0;
	int random_path_number;

	child_x = arg1 + 1;
	child_y = arg2;

	if (child_x < maze_length && parent_of[arg1][arg2] != 10 * child_x + child_y && path[arg1][arg2][child_x][child_y] != 2 && path[arg1][arg2][child_x][child_y]>0) { 
		l293.left(170);
    l293.forward(170);
		flag++;
		flagl++;
	}

	child_x = arg1 - 1;
	child_y = arg2;

	if (child_x > 0 && parent_of[arg1][arg2] != 10 * child_x + child_y && path[arg1][arg2][child_x][child_y] != 2 && path[arg1][arg2][child_x][child_y] > 0) {
		l293.right(170);
    l293.forward(170);
		flag++;
		flagr++;
	}

	child_x = arg1;
	child_y = arg2 + 1;

	if (child_y < maze_width && parent_of[arg1][arg2] != 10 * child_x + child_y && path[arg1][arg2][child_x][child_y] != 2 && path[arg1][arg2][child_x][child_y]>0) {
    l293.forward(170);
		flag++;
		flags++;
	}

	child_x = arg1;
	child_y = arg2 - 1;

	if (child_y > 0 && parent_of[arg1][arg2] != 10 * child_x + child_y && path[arg1][arg2][child_x][child_y] != 2 && path[arg1][arg2][child_x][child_y] > 0) {
		l293.back(170);
    l293.forward(170);
		flag++;
		flagb++;
	}

	no_of_paths = flag;
	random_path_number = 0;

	if (random_path_number == 0) {
		if (flagl > 0) {
			moveto(arg1, arg2, arg1 + 1, arg2);// turn left
			// move forward
			return 0;
		}

		else if (flags > 0) {
			moveto(arg1, arg2, arg1 - 1, arg2);// go forward
			return 1;
		}

		else if (flagr > 0) {
			moveto(arg1, arg2, arg1, arg2 + 1);// turn right
			return 2;
		}

		else if (flagb > 0) {
			moveto(arg1, arg2, arg1, arg2 - 1);// go backward
			return 3;
		}
	}

	if (random_path_number == 1) {
		if (flags > 0) {
			moveto(arg1, arg2, arg1 - 1, arg2);// turn left
			// move forward
			return 1;
		}

		else if (flagr > 0) {
			moveto(arg1, arg2, arg1, arg2 + 1);// go forward
			return 2;
		}

		else if (flagb > 0) {
			moveto(arg1, arg2, arg1, arg2 - 1);// turn right
			return 3;
		}

		else if (flagl > 0) {
			moveto(arg1, arg2, arg1 + 1, arg2);// go backward
			return 0;
		}
	}

	if (random_path_number == 2) {
		if (flagr > 0) {
			moveto(arg1, arg2, arg1, arg2 + 1);// turn left
			// move forward
			return 2;
		}

		else if (flagb > 0) {
			moveto(arg1, arg2, arg1, arg2 - 1);// go forward
			return 3;
		}

		else if (flagl > 0) {
			moveto(arg1, arg2, arg1 + 1, arg2);// turn right
			return 0;
		}

		else if (flags > 0) {
			moveto(arg1, arg2, arg1 - 1, arg2);// go backward
			return 1;
		}
	}
}

// Get the coordinates of the child chosen by the random algorithm
int successor_value(int arg1, int arg2, int arg3) {
	int rand = arg3;
	int retval;

	if (rand == 0) retval = 10 * (arg1 + 1) + arg2;
	else if (rand == 1) retval = 10 * (arg1 - 1) + arg2;
	else if (rand == 2) retval = 10 * arg1 + arg2 + 1;
	else if (rand == 3) retval = 10 * arg1 + arg2 + 1;

	return retval;
}

// Get the coordinates of the successor in the solution of the maze
int correct_successor_value(int arg1, int arg2) {
	int child_x, child_y;

	child_x = arg1 + 1;
	child_y = arg2;

	if (child_x < maze_length && path[arg1][arg2][child_x][child_y] == 1) return 10 * child_x + child_y;

	child_x = arg1 - 1;
	child_y = arg2;

	if (child_x < maze_length && path[arg1][arg2][child_x][child_y] == 1) return 10 * child_x + child_y;

	child_x = arg1;
	child_y = arg2 + 1;

	if (child_x < maze_length && path[arg1][arg2][child_x][child_y] == 1) return 10 * child_x + child_y;

	child_x = arg1;
	child_y = arg2 - 1;

	if (child_x < maze_length && path[arg1][arg2][child_x][child_y] == 1) return 10 * child_x + child_y;
}

// Goes to the successor of the present node in the solution of the maze
void goto_successor_of(int arg1, int arg2) {
	int child_x, child_y;

	child_x = arg1 + 1;
	child_y = arg2;

	if (child_x < maze_length && path[arg1][arg2][child_x][child_y] == 1) moveto(arg1, arg2, child_x, child_y);

	child_x = arg1 - 1;
	child_y = arg2;

	if (child_x < maze_length && path[arg1][arg2][child_x][child_y] == 1) moveto(arg1, arg2, child_x, child_y);

	child_x = arg1;
	child_y = arg2 + 1;

	if (child_x < maze_length && path[arg1][arg2][child_x][child_y] == 1) moveto(arg1, arg2, child_x, child_y);

	child_x = arg1;
	child_y = arg2 - 1;

	if (child_x < maze_length && path[arg1][arg2][child_x][child_y] == 1) moveto(arg1, arg2, child_x, child_y);
}

// Exploring the maze
void tremaux_exploring(int arg1, int arg2) {
	int current_x = arg1, current_y = arg2;
	int successor_x, successor_y, parent_x, parent_y;
	int child_id;
	is_at_end = 0;
	in();

	if (!is_at_end) {

		if (no_out_path_from(current_x, current_y) || no_child_with_outpath(current_x, current_y)) {
			goto_parent_of(current_x, current_y);
			parent_x = parent_of[current_x][current_y] / 10;
			parent_y = parent_of[current_x][current_y] % 10;
			path[parent_x][parent_y][current_x][current_y]++;
			tremaux_exploring(parent_x, parent_y);
		}

		else {
			child_id = goto_child_of(current_x, current_y);
			successor_x = successor_value(current_x, current_y, child_id) / 10;
			successor_y = successor_value(current_x, current_y, child_id) % 10;
			path[current_x][current_y][successor_x][successor_y]++;
			tremaux_exploring(successor_x, successor_y);
		}

	}
}

// Back-tracing maze
void tremaux_solving(int arg1, int arg2) {
	int current_x = arg1, current_y = arg2;
	int successor_x, successor_y;

	if (!(arg1 == 0 && arg2 == 0)) {
		goto_successor_of(arg1, arg2);
		successor_x = correct_successor_value(current_x, current_y) / 10;
		successor_y = correct_successor_value(current_x, current_y) % 10;
		tremaux_solving(successor_x, successor_y);
	}
}

// Main function
int main() {
	int i = 0, j = 0;
	tremaux_exploring(i, j);
	//_delay_ms(5000)
	tremaux_solving(maze_length - 1, maze_width - 1);
	return 0;
}
