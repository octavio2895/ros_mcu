#include <Arduino.h>
#include <Encoder.h> // TODO Use teensy's built-in quad decoder
#include <PID_v1.h>

// ROS Message Types
#include <ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


#define PWMA 3
#define PWMB 2
#define DIR1_A 9
#define DIR2_A 10
#define DIR1_B 11
#define DIR2_B 12
#define MIN_PWM -255
#define CHAN_3 15
#define CHAN_4 16
#define PID_SAMPLE_TIME 5
#define CPR_CONS 1100 
#define MAX_RPM 2
#define MAX_SPEED 1
#define WHEEL_DIAMETER 0.1
#define FR_WHEELS_DISTANCE 0.30
#define LR_WHEELS_DISTANCE 0.235
#define CMD_TIMEOUT 1000

// Globals
// TODO find optimal constants.
// TODO reduce number of globals.

struct Motor /* Structs that encapsultes motor control */
{
  double kp = 100;
  double ki = 7500;
  double kd = 0.2;
  double target = 0;
  double speed = 0;
  double output = 0;
  long last_point = 0;
  uint32_t last_millis = 0; // TODO Proctect against rollover
  uint8_t pwm_pin;
  bool dir;
  uint8_t dir_pins[2];
}motors[2], temp_motor;

struct SpeedCmd
{
  float lin_vel = 0;
  float ang_vel = 0;
  unsigned long cmd_time = 0;
}new_speed, last_speed, actual_speed;

struct Pose
{
  double x = 0;
  double y = 0;
  double theta = 0;
}pose;

enum direction{FORWARD, BACKWARD, BRAKE, BRAKE2};

bool vel_cmd_flag = 0;
bool timeout_flag = 0;
uint8_t mode=0;

volatile int32_t chan3_pwm_len = 0;
volatile uint32_t chan3_rising_micros = 0;
volatile int32_t chan4_pwm_len = 0;
volatile uint32_t chan4_rising_micros = 0;

nav_msgs::Odometry odom;

// Objects
PID motorA(&motors[0].speed, &motors[0].output, &motors[0].target, motors[0].kp, motors[0].ki, motors[0].kd, DIRECT);
PID motorB(&motors[1].speed, &motors[1].output, &motors[1].target, motors[1].kp, motors[1].ki, motors[1].kd, DIRECT);

ros::NodeHandle nh;

Encoder encoderA(5, 6);
Encoder encoderB(7, 8);

// Prototypes
void calculate_speed(long, Motor*);
int8_t get_serial_parameters(uint8_t*, uint8_t*, double*, double*, double*, double*);
void vel_callback(const geometry_msgs::Twist& cmd_msg);
void pid_update_callback(const geometry_msgs::Vector3& cmd_msg);
void check_cmds();
void write_motor(Motor m1);
void set_dir(Motor m2, direction dir);
void chan3_isr(void);
void chan4_isr(void);
void cmd_to_revs_diff(const SpeedCmd*, Motor*, Motor*);
double compute_input(uint32_t, bool);
void compute_pose(int32_t, int32_t);
void compute_twist(SpeedCmd*, double, double);
void compute_odom(nav_msgs::Odometry*, Pose, SpeedCmd);

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", vel_callback);
ros::Subscriber<geometry_msgs::Vector3> pid_sub("pid_vals", pid_update_callback);

ros::Publisher pub_odometry("odom", &odom);

void setup() 
{
  attachInterrupt(digitalPinToInterrupt(CHAN_3), chan3_isr, CHANGE); // TODO Only attach when in recovery mode;
  attachInterrupt(digitalPinToInterrupt(CHAN_4), chan4_isr, CHANGE);
  analogWriteFrequency(PWMA, 400);
  analogWriteFrequency(PWMB, 400);
  motors[0].pwm_pin = PWMA;
  motors[1].pwm_pin = PWMB;
  motors[0].dir_pins[0] = DIR1_A;
  motors[0].dir_pins[1] = DIR2_A;
  motors[1].dir_pins[0] = DIR1_B;
  motors[1].dir_pins[1] = DIR2_B;
  Serial1.begin(9600);
  motorA.SetSampleTime(PID_SAMPLE_TIME);
  motorB.SetSampleTime(PID_SAMPLE_TIME); // TODO Find optimal sampling time.
  pinMode(motors[0].pwm_pin, OUTPUT);
  pinMode(motors[1].pwm_pin, OUTPUT);
  pinMode(CHAN_3, INPUT);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  motorA.SetMode(AUTOMATIC);
  motorA.SetOutputLimits(MIN_PWM, 255);
  motorB.SetMode(AUTOMATIC);
  motorB.SetOutputLimits(MIN_PWM, 255);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.subscribe(pid_sub);
  nh.advertise(pub_odometry);
}

void loop() 
{
  static uint32_t last_update=0;
  if(millis()-last_update > 1) // TODO make this a timer.
    {
      Serial1.printf("Time diff: %d \r\n", micros()-chan4_rising_micros);
      if(mode /* && micros()-chan4_rising_micros < 2000*/) // FIXME When RC is disonnected, motors spin up.
        {
          Serial1.printf("Chan3 length: %d, Chan4 length: %d \r\n", chan3_pwm_len, chan4_pwm_len);
          SpeedCmd s = {compute_input(chan3_pwm_len, 1), compute_input(chan4_pwm_len, 0)}; // TODO Make deadzone; Make dz automatically adjustable; 
          cmd_to_revs_diff(&s, &motors[0], &motors[1]);
          Serial1.println(motors[0].target);
        }
      Serial1.print(motors[0].speed);
      Serial1.print(",");
      Serial1.print(motors[0].target);
      Serial1.print(",");
      Serial1.print(motors[0].output);
      Serial1.print(",");
      Serial1.print(motors[1].speed);
      Serial1.print(",");
      Serial1.print(motors[1].target);
      Serial1.print(",");
      Serial1.println(motors[1].output);
      calculate_speed(encoderA.read(), &motors[0]); // TODO Protect against rollover
      calculate_speed(encoderB.read(), &motors[1]);
      compute_pose(encoderA.read(), encoderB.read());
      compute_twist(&actual_speed, motors[0].speed, motors[1].speed);
      compute_odom(&odom, pose, actual_speed);
      last_update = millis();
    }
  // TODO rewrite this, values are not updating correctly.
  /*
  double static temp_speed = 0;
  double temp_p, temp_i, temp_d;
  uint8_t new_mode, motor_id = 0;
  temp_p=motors[motor_id].kp, temp_i=motors[motor_id].ki, temp_d=motors[motor_id].kd, new_mode = mode;
  if(get_serial_parameters(&motor_id, &new_mode, &temp_speed, &temp_p, &temp_i, &temp_d))
    {
      char sendbuffer[100];
      sprintf(sendbuffer, "Previous values for motor %u: Mode = %d; Target Speed = %lf; KP = %lf; KI = %lf; KD = %lf", motor_id, mode, motors[motor_id].target, motors[motor_id].kp, motors[motor_id].ki, motors[motor_id].kd);
      Serial1.println(sendbuffer);
      sprintf(sendbuffer, "Current values for motor %u: Mode = %d; Target Speed = %lf; KP = %lf; KI = %lf; KD = %lf", motor_id, new_mode, temp_speed, temp_p, temp_i, temp_d);
      Serial1.println(sendbuffer);
      mode = new_mode;
      if(mode==0) motors[motor_id].target = temp_speed;
      if(motors[motor_id].kp != temp_p || motors[motor_id].ki != temp_i || motors[motor_id].kd != temp_d) motorA.SetTunings(motors[0].kp, motors[0].ki, motors[0].kd);
      Serial1.print("KP: ");
      Serial1.println(motors[motor_id].kp);
      Serial1.print("KI: ");
      Serial1.println(motors[motor_id].ki);
      Serial1.print("KD: ");
      Serial.println(motors[motor_id].kd);
    }
  */
  check_cmds();
  pub_odometry.publish(&odom);
  motorA.Compute();
  motorB.Compute();
  write_motor(motors[0]);
  write_motor(motors[1]);
  nh.spinOnce();
}


void calculate_speed(long new_point, Motor *motor) // Calculates rev/s from encoders.
// TODO change timing into mircoseconds; protect against rollover.
{
  double time_elapsed;
  time_elapsed = ((double)(millis()-motor->last_millis))/1000;
  motor->speed = (double)(new_point-motor->last_point)/(CPR_CONS*time_elapsed);   
  motor->last_point = new_point;
  motor->last_millis = millis();
}

int8_t get_serial_parameters(uint8_t* id, uint8_t* mode, double* speed, double* p, double* i, double* d) // Grab paramaters and cmds from Serial.
// TODO try strtol; protect against overflow; pass Streaming object.
{
  static char serial_buf[100];
  static uint8_t k = 0;
  static unsigned int timeout_time = 0;
  static bool timer_flag = 0;
  if (!timer_flag && Serial1.available()) 
    {
      memset(serial_buf, 0, sizeof(serial_buf));
      timeout_time = millis() + 1000;
      timer_flag = 1;
      k = 0;
    }

  if(millis() < timeout_time && Serial1.available())
    {
      while(Serial1.available() && k < sizeof(serial_buf))
        {
          serial_buf[k] = Serial1.read();
          k++;
        }
      return 0;
    }
  if (millis() > timeout_time && timer_flag)
    {
      k = 0;
      timer_flag = 0; 
      Serial1.print("Message recived:");
      Serial1.println(serial_buf);
      char param_buffer[20];
      char *p1 = serial_buf, *p2;
      for(int j=0; j<6; j++)
        {
          Serial1.println(j);
          p2 = strchr(p1, ';');
          if(p2 == NULL)break;
          if((p2-p1) > sizeof(param_buffer))
            {
              p1 = p2+1;
              continue;
            }
          if((p2-p1) == 0)
            {
              p1 = p2+1;
              continue;
            }
          memset(param_buffer, 0, sizeof(param_buffer));
          memcpy(param_buffer, p1, p2-p1);
          p1 = p2+1;
          if(param_buffer[0] == NULL)
            {
              continue;
            }
          Serial1.println(j);
          if(j==0) sscanf(param_buffer, "%u", id);
          else if(j==1) sscanf(param_buffer, "%u", mode);
          else if(j==2) sscanf(param_buffer, "%lf", speed);
          else if(j==3) sscanf(param_buffer, "%lf", p);
          else if(j==4) sscanf(param_buffer, "%lf", i);
          else if(j==5) sscanf(param_buffer, "%lf", d);
        }
      return 1;
    }
  return 0;
}

void vel_callback(const geometry_msgs::Twist& cmd_msg) // Processes Twist cmd from ROS.
{
  last_speed = new_speed;
  new_speed.lin_vel = cmd_msg.linear.y;
  new_speed.ang_vel = cmd_msg.angular.z;
  new_speed.cmd_time = millis();
  vel_cmd_flag = 1;
  timeout_flag = 0;
}

void check_cmds() // Polls cmds flags.
// TODO change flags var to array and use memcmp with prev state; Abstract cmd exec into separate functions; Use less globals;
{
  if(!timeout_flag && millis() > new_speed.cmd_time + CMD_TIMEOUT)
    {
      motors[0].target = 0;
      motors[1].target = 0;
      Serial1.println("Command timer timed out... Shutting down motors.");
      timeout_flag = 1;
      mode = 1;
      return;
    }
  if(vel_cmd_flag)
    {
      cmd_to_revs_diff(&new_speed, &motors[0], &motors[1]);
      // motors[0].target = (new_speed.lin_vel - LR_WHEELS_DISTANCE*new_speed.ang_vel)/((WHEEL_DIAMETER/2)*(2*PI)); // TODO abstract into function;
      // motors[1].target = (new_speed.lin_vel + LR_WHEELS_DISTANCE*new_speed.ang_vel)/((WHEEL_DIAMETER/2)*(2*PI));
      Serial1.print("Setting motor speed 1 to: ");
      Serial1.println(motors[0].target);
      Serial1.print("Setting motor speed 2 to: ");
      Serial1.println(motors[1].target);
      mode = 0;
      vel_cmd_flag = 0;
    }
}

void write_motor(Motor mwrite)
{
  if(mwrite.output >= 0) 
    {
      set_dir(mwrite, FORWARD);
      analogWrite(mwrite.pwm_pin, mwrite.output);
    }
  else 
    {
      set_dir(mwrite, BACKWARD);     
      analogWrite(mwrite.pwm_pin, -mwrite.output);
    }
}

void set_dir(Motor mdir, direction dir)
{
  switch (dir)
    {
      case FORWARD:
        digitalWrite(mdir.dir_pins[0], HIGH);
        digitalWrite(mdir.dir_pins[1], LOW);
        break;
      case BACKWARD:
        digitalWrite(mdir.dir_pins[0], LOW);
        digitalWrite(mdir.dir_pins[1], HIGH);
        break;
      case BRAKE:
        digitalWrite(mdir.dir_pins[0], LOW);
        digitalWrite(mdir.dir_pins[1], LOW);
        break;
      case BRAKE2:
        digitalWrite(mdir.dir_pins[0], HIGH);
        digitalWrite(mdir.dir_pins[1], HIGH);
        break;
    }

}

void pid_update_callback(const geometry_msgs::Vector3& pid_vals) // Update PID constants from ROS Vector3 msg
// TODO Change to custom msg. Update motors parameters independently.
{
  motorA.SetTunings((double)pid_vals.x, (double)pid_vals.y, (double)pid_vals.z);
  motorB.SetTunings((double)pid_vals.x, (double)pid_vals.y, (double)pid_vals.z);
  Serial1.print("PID Vals - P: ");
  Serial1.print(motorA.GetKp());
  Serial1.print(" I: ");
  Serial1.print(motorA.GetKi());
  Serial1.print(" D: ");
  Serial1.println(motorA.GetKd());
}

void chan3_isr() // TODO Make it more reusable; Change to better name;
{
  if(digitalRead(CHAN_3))
    {
      chan3_rising_micros = micros();
    }
  else
    {
      chan3_pwm_len = micros() - chan3_rising_micros;
      if(chan3_pwm_len <= 0) chan3_pwm_len =  chan3_pwm_len + micros(); // Protect from rollover
      if(chan3_pwm_len > 2000) chan3_pwm_len = 0; // Recover from disconnection
    }
}

void chan4_isr() // TODO Make it more reusable; Change to better name;
{
  if(digitalRead(CHAN_4))
    {
      chan4_rising_micros = micros();
    }
  else
    {
      chan4_pwm_len = micros() - chan4_rising_micros;
      if(chan4_pwm_len <= 0) chan4_pwm_len =  chan4_pwm_len + micros(); // Protect from rollover
      if(chan4_pwm_len > 2000) chan4_pwm_len = 0; // Recover from disconnection
    }
}

void cmd_to_revs_diff(const SpeedCmd* sp, Motor* ma, Motor* mb) // Transform linear and angular velocities into rev/s for differential drive.
// TODO Accept robot parameters as struct
{
  ma->target = (sp->lin_vel - LR_WHEELS_DISTANCE*sp->ang_vel)/((WHEEL_DIAMETER/2)*(2*PI));
  mb->target = (sp->lin_vel + LR_WHEELS_DISTANCE*sp->ang_vel)/((WHEEL_DIAMETER/2)*(2*PI));
}

double compute_input(uint32_t len, bool mode) // Calulates speed from RC pulse length. Mode 1: 0-max, Mode 0: -max - max;
{
  if(mode)
    {
      if (len <= 1110) len = 1000;
      return (((double)len - 1000)*MAX_RPM)/1000;
    }

  else
    {
      if (len <= 1110) len = 1000;
      return (((double)len - 1500)*MAX_RPM)/1000;
    }
}

void compute_pose(int32_t a, int32_t b) // Appoximates odometry from deadreckoning. 
// TODO Accept robot parameters; Resetable coordinate frame; Potect against overflow.
{
  static int32_t last_a = 0, last_b = 0;
  double delta_a_rad = (((double)(a - last_a))/CPR_CONS)*2*PI;
  double delta_b_rad = (((double)(b - last_b))/CPR_CONS)*2*PI;
  double s = (delta_a_rad + delta_b_rad)/2;
  pose.theta = pose.theta + (delta_a_rad-delta_b_rad)/(2*FR_WHEELS_DISTANCE);
  pose.x = pose.x + s*cos(pose.theta);
  pose.y = pose.y + s*sin(pose.theta);
}

void compute_twist(SpeedCmd* s, int32_t m1_speed, int32_t m2_speed) // Appoximates the current speed of robot.
{
  s->lin_vel = ((double)(m1_speed+m2_speed)*WHEEL_DIAMETER)*PI;
  s->ang_vel = ((double)(m1_speed-m2_speed)*WHEEL_DIAMETER*PI)/(FR_WHEELS_DISTANCE);
  s->cmd_time = 0;
}

void compute_odom(nav_msgs::Odometry* odom, Pose p, SpeedCmd s) // Combines current pose and speed into odom to send to ROS
{
  odom->twist.twist.linear.y = (double)s.lin_vel;
  odom->twist.twist.angular.z = (double)s.ang_vel;
  odom->pose.pose.position.x = p.x;
  odom->pose.pose.position.y = p.y;
  odom->pose.pose.position.z = 0;
  odom->pose.pose.orientation.w = cos(p.theta/2);
  odom->pose.pose.orientation.z = sin(p.theta/2);
  odom->pose.pose.orientation.x = 0;
  odom->pose.pose.orientation.y = 0;
}