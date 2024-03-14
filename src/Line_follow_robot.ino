// This code is for Line following robot with obstacle sensing
// Author : Sai Bharadhwaj Matha, for queries contact bharadhwajsaimatha@gmail.com

//IR sensors for line following
#define line_sensor_1             11
#define line_sensor_2             12
#define line_sensor_3             13
#define line_sensor_4             14
#define line_sensor_5             15

//IR sensor for obstacle avoidance
#define obstacle_sensor_1         8
#define obstacle_sensor_2         9
#define obstacle_sensor_3         10

// Motor pins
// Left motor
#define left_motor_pwm            01        //ENB
#define left_motor_dir_1          02        //IN3
#define left_motor_dir_2          03        //IN4
//right motor
#define right_motor_pwm           04        //ENA
#define right_motor_dir_1         05        //IN1
#define right_motor_dir_2         06        //IN2

//LED pins
#define warning_led               20

//Safety switch
#define switch_button             21


//defined variables
//uint8_t line_arr[5] = {0,0,0,0,0};
//uint8_t obstacle_arr[3] = {0,0,0};
//int line_case =0;
//int obs_case = 0;
uint8_t switch_val;
int steer_action;
float robo_speed = 120;

//state and drive enums
enum state
{
  DRIVE_FWD,
  GOAL_STOP,
  NO_LINE,
  OBSTACLE,
  FAIL_SAFE
};
enum state robo_state;
enum drive
{
  SHARP_LEFT = -4,
  HEFTY_LEFT,
  LEFT,
  LOW_LEFT,
  STRAIGHT,
  LOW_RIGHT,
  RIGHT,
  HEFTY_RIGHT,
  SHARP_RIGHT,
};
enum drive drive_state;
enum obstacle
{
  NO_OBSTACLE,
  EXTREME_LEFT,
  MID_LEFT,
  MIDDLE,
  MID_RIGHT,
  EXTREME_RIGHT,
  SOMEWHERE
};
enum obstacle obstacle_state;

//control variables
float Kp = 10;
float Ki = 5;
float Kd = 0.05;
float prev_error = 0;
float error = 0;
float error_integral = 0;
float error_derivative = 0;
float control_signal = 0;

//defined functions

float PID_control(float error_val)
{
  error_integral = prev_error + error_val;
  error_derivative = error_val - prev_error;
  control_signal = (Kp*error_val) + (Ki*error_integral) + (Kd*error_derivative);
  prev_error = error_val;
  return control_signal;
}
void set_pwm(float ctrl_sig)
{
  digitalWrite(left_motor_dir_1, LOW);
  digitalWrite(left_motor_dir_2, HIGH);
  digitalWrite(right_motor_dir_1, HIGH);
  digitalWrite(right_motor_dir_2, LOW);
  analogWrite(left_motor_pwm, robo_speed+ctrl_sig);
  analogWrite(right_motor_pwm, robo_speed - ctrl_sig);
//  Serial.println(robo_speed+ctrl_sig);
}

void rotate_robot()
{
  digitalWrite(left_motor_dir_1, HIGH);
  digitalWrite(left_motor_dir_2, LOW);
  digitalWrite(right_motor_dir_1, HIGH);
  digitalWrite(right_motor_dir_2, LOW);
  analogWrite(left_motor_pwm, robo_speed - 20);
  analogWrite(right_motor_pwm, robo_speed + 20); 
}

void steering_action(int steer_arr[5])
{
//  Serial.println("started in steering action\n");
  if((steer_arr[0] == 0)&&(steer_arr[1] == 0)&&(steer_arr[2] == 0)&&(steer_arr[3] == 0)&&(steer_arr[4] == 0))
  {
    robo_state = NO_LINE;
  }
  else if((steer_arr[0] == 0)&&(steer_arr[1] == 0)&&(steer_arr[2] == 1)&&(steer_arr[3] == 0)&&(steer_arr[4] == 0))
  {
    robo_state = DRIVE_FWD;
    drive_state = STRAIGHT;
  }
  else if((steer_arr[0] == 0)&&(steer_arr[1] == 1)&&(steer_arr[2] == 1)&&(steer_arr[3] == 0)&&(steer_arr[4] == 0))
  {
    robo_state = DRIVE_FWD;
    drive_state = LOW_LEFT;
  }
  else if((steer_arr[0] == 0)&&(steer_arr[1] == 1)&&(steer_arr[2] == 0)&&(steer_arr[3] == 0)&&(steer_arr[4] == 0))
  {
    robo_state = DRIVE_FWD;
    drive_state = LEFT;
  }
  else if((steer_arr[0] == 1)&&(steer_arr[1] == 1)&&(steer_arr[2] == 0)&&(steer_arr[3] == 0)&&(steer_arr[4] == 0))
  {
    robo_state = DRIVE_FWD;
    drive_state = HEFTY_LEFT;
  }
  else if((steer_arr[0] == 1)&&(steer_arr[1] == 0)&&(steer_arr[2] == 0)&&(steer_arr[3] == 0)&&(steer_arr[4] == 0))
  {
    robo_state = DRIVE_FWD;
    drive_state = SHARP_LEFT;
  }
  else if((steer_arr[0] == 0)&&(steer_arr[1] == 0)&&(steer_arr[2] == 1)&&(steer_arr[3] == 1)&&(steer_arr[4] == 0))
  {
    robo_state = DRIVE_FWD;
    drive_state = LOW_RIGHT;
  }
  else if((steer_arr[0] == 0)&&(steer_arr[1] == 0)&&(steer_arr[2] == 0)&&(steer_arr[3] == 1)&&(steer_arr[4] == 0))
  {
    robo_state = DRIVE_FWD;
    drive_state = RIGHT;
  }
  else if((steer_arr[0] == 0)&&(steer_arr[1] == 0)&&(steer_arr[2] == 0)&&(steer_arr[3] == 1)&&(steer_arr[4] == 1))
  {
    robo_state = DRIVE_FWD;
    drive_state = HEFTY_RIGHT;
  }
  else if((steer_arr[0] == 0)&&(steer_arr[1] == 0)&&(steer_arr[2] == 0)&&(steer_arr[3] == 0)&&(steer_arr[4] == 1))
  {

    robo_state = DRIVE_FWD;
    drive_state = SHARP_RIGHT;
  }
  else if((steer_arr[0] == 1)&&(steer_arr[1] == 1)&&(steer_arr[2] == 1)&&(steer_arr[3] == 1)&&(steer_arr[4] == 1))
  {
    robo_state = GOAL_STOP;
  }
  else
  {
    robo_state = DRIVE_FWD;
    drive_state = STRAIGHT;
//    robo_state = FAIL_SAFE;
  }
// return drive_state;
}

void read_line_sensors(int* line_arr)
{
  line_arr[0] = digitalRead(line_sensor_1);
  line_arr[1] = digitalRead(line_sensor_2);
  line_arr[2] = digitalRead(line_sensor_3);
  line_arr[3] = digitalRead(line_sensor_4);
  line_arr[4] = digitalRead(line_sensor_5);
}
void obstacle_action(int obst_arr[3])
{
  if((obst_arr[0] == 0)&&(obst_arr[1] == 0)&&(obst_arr[2] == 0))
  {
    obstacle_state = NO_OBSTACLE;
//    Serial.println("this is updated\n");
//    Serial.println(obstacle_state);
  }
  else if((obst_arr[0] == 1)&&(obst_arr[1] == 0)&&(obst_arr[2] == 0))
  {
    obstacle_state = EXTREME_LEFT;
  }
  else if((obst_arr[0] == 1)&&(obst_arr[1] == 1)&&(obst_arr[2] == 0))
  {
    obstacle_state = MID_LEFT;
  }
  else if((obst_arr[0] == 0)&&(obst_arr[1] == 1)&&(obst_arr[2] == 0))
  {
    obstacle_state = MIDDLE;
  }
  else if((obst_arr[0] == 0)&&(obst_arr[1] == 1)&&(obst_arr[2] == 1))
  {
    obstacle_state = MID_RIGHT;
  }
  else if((obst_arr[0] == 0)&&(obst_arr[1] == 0)&&(obst_arr[2] == 1))
  {
    obstacle_state = EXTREME_RIGHT;
  }
  else
  {
    obstacle_state = SOMEWHERE;
  }
// return obstacle_state;
}

//void obstacle_action(int obst_arr[3])
//{
//  if((obst_arr[0] == 1)&&(obst_arr[1] == 1)&&(obst_arr[2] == 1))
//  {
//    obstacle_state = NO_OBSTACLE;
////    Serial.println("this is updated\n");
////    Serial.println(obstacle_state);
//  }
//  else if((obst_arr[0] == 0)&&(obst_arr[1] == 1)&&(obst_arr[2] == 1))
//  {
//    obstacle_state = EXTREME_LEFT;
//  }
//  else if((obst_arr[0] == 0)&&(obst_arr[1] == 0)&&(obst_arr[2] == 1))
//  {
//    obstacle_state = MID_LEFT;
//  }
//  else if((obst_arr[0] == 1)&&(obst_arr[1] == 0)&&(obst_arr[2] == 1))
//  {
//    obstacle_state = MIDDLE;
//  }
//  else if((obst_arr[0] == 1)&&(obst_arr[1] == 0)&&(obst_arr[2] == 0))
//  {
//    obstacle_state = MID_RIGHT;
//  }
//  else if((obst_arr[0] == 1)&&(obst_arr[1] == 1)&&(obst_arr[2] == 0))
//  {
//    obstacle_state = EXTREME_RIGHT;
//  }
//  else
//  {
//    obstacle_state = SOMEWHERE;
//  }
//// return obstacle_state;
//}


void read_obstacle_sensors(int* obstacle_arr)
{
  obstacle_arr[0] = digitalRead(obstacle_sensor_1);
  obstacle_arr[1] = digitalRead(obstacle_sensor_2);
  obstacle_arr[2] = digitalRead(obstacle_sensor_3);
}

void halt_robot()
{
  analogWrite(left_motor_pwm, 0);
  analogWrite(right_motor_pwm, 0);
  digitalWrite(left_motor_dir_1, LOW);
  digitalWrite(left_motor_dir_2, LOW);
  digitalWrite(right_motor_dir_1, LOW);
  digitalWrite(right_motor_dir_2, LOW);
//  Serial.println("Robot halted!! System Reboot required!");
  while(true)
  {
    digitalWrite(warning_led, HIGH);
    delay(500);
    digitalWrite(warning_led, LOW);
    delay(500);
  }
}

void temporary_halt()
{
//  Serial.println("Robot is temporarily haulted\n");
  analogWrite(left_motor_pwm, 0);
  analogWrite(right_motor_pwm, 0);
  digitalWrite(left_motor_dir_1, LOW);
  digitalWrite(left_motor_dir_2, LOW);
  digitalWrite(right_motor_dir_1, LOW);
  digitalWrite(right_motor_dir_2, LOW);
}

void setup() 
{
  Serial.begin(115200);

  //Line Follow sensors
  pinMode(line_sensor_1, INPUT);
  pinMode(line_sensor_2, INPUT);
  pinMode(line_sensor_3, INPUT);
  pinMode(line_sensor_4, INPUT);
  pinMode(line_sensor_5, INPUT);

  //Obstacle IR sensors
  pinMode(obstacle_sensor_1, INPUT);
  pinMode(obstacle_sensor_2, INPUT);
  pinMode(obstacle_sensor_3, INPUT);

  //Motor pins
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_dir_1, OUTPUT);
  pinMode(left_motor_dir_2, OUTPUT);

  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_dir_1, OUTPUT);
  pinMode(right_motor_dir_2, OUTPUT);

  //Safety_switch and LED
  pinMode(switch_button, INPUT_PULLUP);
  pinMode(warning_led, OUTPUT);

  //Initialisation
//  Serial.println("\tLine Follower Robot ||\t Topic 04 GROUP B ||\t STARTING.....");
  digitalWrite(left_motor_dir_1, LOW);
  digitalWrite(left_motor_dir_2, LOW);
  digitalWrite(right_motor_dir_1, LOW);
  digitalWrite(right_motor_dir_2, LOW);
  analogWrite(left_motor_pwm, 0);
  analogWrite(right_motor_pwm, 0);
}

void loop()
{

  //Ready safety switch state
  if(digitalRead(switch_button) == LOW)
  {
//    Serial.println("Safety switch is pressed!! Halting the robot\n");
    robo_state = FAIL_SAFE;
  }
//  Serial.println("started in loop\n");
  int line_arr[5];
  int obstacle_arr[3];
  read_line_sensors(line_arr);
  read_obstacle_sensors(obstacle_arr);
//  Serial.println("ead sensor values\n");
  obstacle_action(obstacle_arr);
  steering_action(line_arr);

//  for(int i=0;i<3;i++)
//  {
//    Serial.print("Obstacle sensor reading \t");
//    Serial.print(i);
//    Serial.print("\t");
//    Serial.print(obstacle_arr[i]);
//    Serial.print("\n");
//  }
//  Serial.println("Updated states\n");
//  obstacle_state = NO_OBSTACLE;
  if(obstacle_state != NO_OBSTACLE)
  {
//    Serial.print("Obstacle loop \n");
    robo_state = OBSTACLE;
//    Serial.println("In obstacle if clause\n");
  }
//  Serial.println(robo_state);
//  Serial.println(obstacle_state);
//  Serial.println(drive_state);
  switch(robo_state)
  {
    case OBSTACLE:
//      Serial.println("Obstacle status code :\t");
//      Serial.print(obstacle_state);
//      Serial.print("\n");
      temporary_halt();
      break;
      
    case GOAL_STOP:
//      Serial.println("Goal has been reached. Checkpoint!!!\n");
      temporary_halt();
      break;

    case FAIL_SAFE:
//      Serial.println("FAIL SAFE Mode!!!\n");
      halt_robot();
      break;

    case NO_LINE:
//      Serial.println("No Line ahead!! Doing a rotation\n");
      rotate_robot();
      break;
      
    case DRIVE_FWD:
      Serial.println("Moving forward\n");
      set_pwm(PID_control(drive_state));
      break;
  }
 delay(10);
}
