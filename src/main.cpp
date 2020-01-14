#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

// ROS Message Types
#include <ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>


#define PWMA 3
#define PWMB 2
#define MIN_PWM 0
#define CHAN_3 15
#define PID_SAMPLE_TIME 5
#define CPR_CONS 1100 
#define MAX_RPM 7
#define WHEEL_DIAMETER 0.1
#define FR_WHEELS_DISTANCE 0.30
#define LR_WHEELS_DISTANCE 0.235
#define CMD_TIMEOUT 100

// Globals
// TODO find optimal constants.
// TODO reduce number of globals.

struct Motor /* Structs that encapsultes motor control */
{
  double kp = 100;
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
  float lin_vel = 0;
  float ang_vel = 0;
  unsigned long cmd_time = 0;
}new_speed, last_speed;

bool vel_cmd_flag = 0;
uint8_t mode=0;

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
void check_cmds();

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", vel_callback);

void setup() 
{
  motors[0].pwm_pin = PWMA;
  motors[1].pwm_pin = PWMB;
  Serial1.begin(9600);
  Serial.begin(9600);
  motorA.SetSampleTime(PID_SAMPLE_TIME);
  motorB.SetSampleTime(PID_SAMPLE_TIME); // TODO Find optimal sampling time.
  pinMode(motors[0].pwm_pin, OUTPUT);
  pinMode(motors[1].pwm_pin, OUTPUT);
  pinMode(CHAN_3, INPUT);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
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
  double static temp_speed = 0;
  double temp_p, temp_i, temp_d;
  uint8_t new_mode, motor_id = 0;
  temp_p=motors[motor_id].kp, temp_i=motors[motor_id].ki, temp_d=motors[motor_id].kd, new_mode = mode;
  if(get_serial_parameters(&motor_id, &new_mode, &temp_speed, &temp_p, &temp_i, &temp_d)) // TODO rewrite this.
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
  check_cmds();
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
  static char serial_buf[100]; // TODO must protect against overflow
  static uint8_t k = 0;
  static unsigned int timeout_time = 0;
  static bool timer_flag = 0;
  if (!timer_flag && Serial1.available()) 
    {
      Serial.println("Setting timeout flag");
      memset(serial_buf, 0, sizeof(serial_buf));
      timeout_time = millis() + 1000;
      timer_flag = 1;
      k = 0;
    }

  if(millis() < timeout_time && Serial1.available())
    {
      Serial.println("Reading data");
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
          Serial1.print("Top print: ");
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
          Serial1.print("Bottom print: ");
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

void vel_callback(const geometry_msgs::Twist& cmd_msg)
{
  last_speed = new_speed;
  new_speed.lin_vel = cmd_msg.linear.x;
  new_speed.ang_vel = cmd_msg.angular.x;
  new_speed.cmd_time = millis();
  vel_cmd_flag = 1;
}

void check_cmds() // TODO change flags var to array and use memcmp with prev state; Abstract cmd exec into separate functions
{
  if(vel_cmd_flag)
    {
      if(millis() > new_speed.cmd_time + CMD_TIMEOUT)
        {
          motors[0].speed = (new_speed.lin_vel - LR_WHEELS_DISTANCE*new_speed.ang_vel)/(2*PI);
          motors[1].speed = (new_speed.lin_vel + LR_WHEELS_DISTANCE*new_speed.ang_vel)/(2*PI);
        }
      else
        {
          motors[0].speed = 0;
          motors[1].speed = 0;
        }
    }
}