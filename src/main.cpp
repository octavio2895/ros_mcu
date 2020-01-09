#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>
#include "Kinematics.h"

// ROS Message Types
#include <ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>


#define PWMA 3
#define PWMB 2
#define MIN_PWM 0
#define CHAN_3 15
#define PID_SAMPLE_TIME 5
#define CPR_CONS 29250 
#define MAX_RPM 7
#define WHEEL_DIAMETER 0.1
#define FR_WHEELS_DISTANCE 0.30
#define LR_WHEELS_DISTANCE 0.235

// Globals
// TODO find optimal constants.
// TODO reduce number of globals.

struct Motor /* Structs that encapsultes motor control */
{
  double kp = 700;
  double ki = 120;
  double kd = 15;
  double target = 0;
  double speed = 0;
  double output = 0;
  long last_point = 0;
  uint32_t last_millis = 0; // TODO Proctect against rollover
  uint8_t pwm_pin;
  bool dir;
}motors[2], temp_motor;

struct SpeedCmd
{
  float x_vel = 0;
  float y_vel = 0;
  float a_vel = 0;
  unsigned long cmd_time = 0;
}new_speed, last_speed;


uint8_t mode=0;

// Objects
PID motorA(&motors[0].speed, &motors[0].output, &motors[0].target, motors[0].kp, motors[0].ki, motors[0].kd, DIRECT);
PID motorB(&motors[1].speed, &motors[1].output, &motors[1].target, motors[1].kp, motors[1].ki, motors[1].kd, DIRECT);

ros::NodeHandle nh;

Encoder encoderA(5, 6);
Encoder encoderB(7, 8);

Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

// Prototypes
void calculate_speed(long, Motor*);
int8_t get_serial_parameters(uint8_t*, uint8_t*, double*, double*, double*, double*);
void commandCallback(const geometry_msgs::Twist& cmd_msg);

void messageCb( const std_msgs::Empty& toggle_msg)
{
  motors[0].target = 0;
  motors[1].target = 0; 
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

void setup() 
{
  motors[0].pwm_pin = PWMA;
  motors[1].pwm_pin = PWMB;
  Serial1.begin(9600);
  motorA.SetSampleTime(PID_SAMPLE_TIME);
  motorB.SetSampleTime(PID_SAMPLE_TIME); // TODO Find optimal sampling time.
  pinMode(motors[0].pwm_pin, OUTPUT);
  pinMode(motors[1].pwm_pin, OUTPUT);
  pinMode(CHAN_3, INPUT);
  motorA.SetMode(AUTOMATIC);
  motorA.SetOutputLimits(MIN_PWM, 255); // TODO Find optimal MIN_PWM.
  motorB.SetMode(AUTOMATIC);
  motorB.SetOutputLimits(MIN_PWM, 255);
  nh.initNode();
  nh.subscribe(cmd_sub);
}

void loop() 
{
  static uint32_t last_update=0;
  if(millis()-last_update > 10) // TODO make this a timer.
    {
      if(mode)
        {
          motors[0].target = ((double)pulseIn(CHAN_3, HIGH, 21000))/250; // FIXME pulseIn is breaking and takes too long.
          Serial1.println(motors[0].target);
        }
      Serial1.print(motors[0].speed);
      Serial1.print(",");
      Serial1.print(motors[0].output);
      Serial1.print(",");
      Serial1.print(motors[1].speed);
      Serial1.print(",");
      Serial1.println(motors[1].output);
      calculate_speed(encoderA.read(), &motors[0]);
      calculate_speed(encoderB.read(), &motors[1]);
      last_update = millis();
    }
  if (Serial.available())
    {
      double temp_speed = 0, temp_p, temp_i, temp_d;
      uint8_t new_mode, motor_id = 0;
      temp_p=motors[motor_id].kp, temp_i=motors[motor_id].ki, temp_d=motors[motor_id].kd, new_mode = mode;
      get_serial_parameters(&motor_id, &new_mode, &temp_speed, &temp_p, &temp_i, &temp_d);
      // if (motor_id != 0 || motor_id != 1) motor_id = 1;
      char sendbuffer[100];
      sprintf(sendbuffer, "Previous values for motor %u: Mode = %d; Target Speed = %lf; KP = %lf; KI = %lf; KD = %lf", motor_id, mode, motors[motor_id].target, motors[motor_id].kp, motors[motor_id].ki, motors[motor_id].kd);
      Serial1.println(sendbuffer);
      sprintf(sendbuffer, "Current values for motor %u: Mode = %d; Target Speed = %lf; KP = %lf; KI = %lf; KD = %lf", motor_id, new_mode, temp_speed, temp_p, temp_i, temp_d);
      Serial1.println(sendbuffer);
      mode = new_mode;
      if(mode==0) motors[motor_id].target = temp_speed;
      if(motors[motor_id].kp != temp_p || motors[motor_id].ki != temp_i || motors[motor_id].kd != temp_d) motorA.SetTunings(motors[0].kp, motors[0].ki, motors[0].kd);
    }

  motorA.Compute();
  motorB.Compute();
  analogWrite(PWMA, motors[0].output);
  analogWrite(PWMB, motors[1].output);
  nh.spinOnce();
}


void calculate_speed(long new_point, Motor *motor) // TODO change timing into mircoseconds, protect against rollover.
{
  double time_elapsed;
  time_elapsed = ((double)(millis()-motor->last_millis))/1000;
  motor->speed = (double)(new_point-motor->last_point)/(CPR_CONS*time_elapsed);   
  motor->last_point = new_point;
  motor->last_millis = millis();
}

int8_t get_serial_parameters(uint8_t* id, uint8_t* mode, double* speed, double* p, double* i, double* d) // TODO try strtol, protect against overflow.
{
  static int j = 0;
  while (Serial1.available()) 
  {
    char param_buffer[100];
    int param_len = 0;
    param_len = Serial1.readBytesUntil(';', param_buffer, sizeof(param_buffer)); // FIXME readBytesUntil is timing out
    if(param_len == 0) 
      {
        j++;
        continue;
      }
    if(j==0) sscanf(param_buffer, "%u", id);
    else if(j==1) sscanf(param_buffer, "%u", mode);
    else if(j==2) sscanf(param_buffer, "%lf", speed);
    else if(j==3) sscanf(param_buffer, "%lf", p);
    else if(j==4) sscanf(param_buffer, "%lf", i);
    else if(j==5) sscanf(param_buffer, "%lf", d);
    j++;
    if(j>5) break;
  }
  j=0;
  while (Serial1.available()) Serial1.read(); // Clears up buffer.
  return 0;
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
  last_speed = new_speed;
  new_speed.x_vel = cmd_msg.linear.x;
  new_speed.y_vel = cmd_msg.linear.y;
  new_speed.a_vel = cmd_msg.angular.z;
  new_speed.cmd_time = millis();
}