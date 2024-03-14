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

#define left_motor_pwm            01        //ENA
#define left_motor_dir_1          02        //IN1
#define left_motor_dir_2          03        //IN2
//right motor
#define right_motor_pwm           04        //ENA
#define right_motor_dir_1         05        //IN3
#define right_motor_dir_2         06        //IN4

float robo_speed = 127;

void set_pwm(float ctrl_sig)
{
  digitalWrite(left_motor_dir_1, HIGH);
  digitalWrite(left_motor_dir_2, LOW);
  digitalWrite(right_motor_dir_1, LOW);
  digitalWrite(right_motor_dir_2, HIGH);
  analogWrite(left_motor_pwm, robo_speed);
  analogWrite(right_motor_pwm, robo_speed);
}

void rotate_robot()
{
  digitalWrite(left_motor_dir_1, LOW);
  digitalWrite(left_motor_dir_2, HIGH);
  digitalWrite(right_motor_dir_1, LOW);
  digitalWrite(right_motor_dir_2, HIGH);
  analogWrite(left_motor_pwm, robo_speed - 100);
  analogWrite(right_motor_pwm, robo_speed + 100); 
}

void read_line_sensors(int* line_arr)
{
  line_arr[0] = digitalRead(line_sensor_1);
  line_arr[1] = digitalRead(line_sensor_2);
  line_arr[2] = digitalRead(line_sensor_3);
  line_arr[3] = digitalRead(line_sensor_4);
  line_arr[4] = digitalRead(line_sensor_5);
}

void read_obstacle_sensors(int* obstacle_arr)
{
  obstacle_arr[0] = digitalRead(obstacle_sensor_1);
  obstacle_arr[1] = digitalRead(obstacle_sensor_2);
  obstacle_arr[2] = digitalRead(obstacle_sensor_3);
}

void temporary_halt()
{
  Serial.println("Robot is temporarily haulted\n");
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

  //Motor pins
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_dir_1, OUTPUT);
  pinMode(left_motor_dir_2, OUTPUT);

  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_dir_1, OUTPUT);
  pinMode(right_motor_dir_2, OUTPUT);


  //Initialisation
  Serial.println("\tLine Follower Robot ||\t Topic 04 GROUP B ||\t STARTING.....");
  digitalWrite(left_motor_dir_1, LOW);
  digitalWrite(left_motor_dir_2, LOW);
  digitalWrite(right_motor_dir_1, LOW);
  digitalWrite(right_motor_dir_2, LOW);
  analogWrite(left_motor_pwm, 0);
  analogWrite(right_motor_pwm, 0);
}
void loop()
{
  
  int line_arr[5];
  int obstacle_arr[3];
  read_line_sensors(line_arr);
  read_obstacle_sensors(obstacle_arr);
  for(int i=0;i<5;i++)
  {
    Serial.print("Line sensor reading \t");
    Serial.print(i);
    Serial.print("\t");
    Serial.print(line_arr[i]);
    Serial.print("\n");
  }
   for(int i=0;i<3;i++)
  {
    Serial.print("Obstacle sensor reading \t");
    Serial.print(i);
    Serial.print("\t");
    Serial.print(obstacle_arr[i]);
    Serial.print("\n");
  }
//  Serial.println("Rotating wheels");

//    

  delay(10);
}
