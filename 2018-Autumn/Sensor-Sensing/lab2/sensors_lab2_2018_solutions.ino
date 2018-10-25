/**
 * Stub for Lab1 code, Sensors and Sensing at Orebro University
 * 2018
 * 
 * The purpose of this lab is to implement a velocity PID controller
 * for a DC motor with an onboard encoder, connected to the MakeBlock 
 * MegaPi board. Please note: the MegaPi libraries already implement 
 * many of the functionalities needed for this lab. You are only 
 * allowed to use the following library calls:
 *    1. uint8_t MeEncoderOnBoard::getPortB(void);      //get second channel port number
 *    2. uint8_t MeEncoderOnBoard::getIntNum(void);     //get interrupt port number
 *    3. void MeEncoderOnBoard::setMotorPwm(int pwm);   //set a desired control to the motor
 * 
 * 
 * This sketch is based on the example under 
 * Me_On_Board_encoder>Me_MegaPi_encoder_pwm.
 * 
 * This code is licensed under the BSD license.
 * Author list:
 *  - Todor Stoyanov
 */

#include <MeMegaPi.h>

#include <ros.h>
#include <sensors_lab2_2018/MotorStates.h>
#include <sensors_lab2_2018/SetDDParams.h>
#include <sensors_lab2_2018/SetPID.h>
#include <sensors_lab2_2018/SetVel.h>
#include <sensors_lab2_2018/ResetOdom.h>

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

//////////////// struct definitions ///////////////////
/** structure to hold PID parameters */
struct PIDParameters {
  float Kp_;
  float Kd_;
  float Ki_;
  float u_max_; //Maximum controller output (<= max PWM)
  float u_min_; //Minimum controller output [>= -(max PWM)]
  float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]

  PIDParameters(float Kp, float Ki, float Kd, float u_max, float u_min, float I) : Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

/** structure to hold values associated with a controlled motor */
struct MotorState {
  float r_;  //setpoint value at current iteration (in radians/sec)
  float e_;  //error at current iteration
  float de_; //error derivative at current iteration
  float u_;  //computed control

  MotorState(float r, float e, float de, float u): r_(r),e_(e),de_(de),u_(u) {};
};

/** structure to hold values for an encoder */
struct EncoderState {
  long ticks_;  //raw number of ticks
  float ratio_; //number of ticks per radian
  float pos_;   //position in radians
  float vel_;   //velocity in radians/sec
  EncoderState(long ticks, float ratio, float pos, float vel): ticks_(ticks),ratio_(ratio),pos_(pos),vel_(vel) {};
};

////////////// global variables ////////////////////
//the maximum value we can command the motor
const short MAX_PWM_MEGAPI = 255;
//values bellow this do not make the motor move, so we can clamp them
const short MIN_ACTUATION_PWM = 30;
//ticks per radian
const float ENC_TICKS = 184/3.1415;

//TODO: here setup encoders to the appropriate ports
MeEncoderOnBoard Encoder_1(SLOT2);
MeEncoderOnBoard Encoder_2(SLOT4);

PIDParameters* pid_left_vc = new PIDParameters(200.0, 0.5, 150.0, MAX_PWM_MEGAPI, -MAX_PWM_MEGAPI, 0.0); //Velocity controller PID parameters for Motor 0
PIDParameters* pid_right_vc = new PIDParameters(200.0, 0.5, 150.0, MAX_PWM_MEGAPI, -MAX_PWM_MEGAPI, 0.0); //Velocity controller PID parameters for Motor 1
MotorState* left_motor_state = new MotorState(0.0,0,0,0); //accumulator for left motor
MotorState* right_motor_state = new MotorState(0,0,0,0);//accumulator for right motor
EncoderState* left_encoder = new EncoderState(0,ENC_TICKS,0,0);
EncoderState* right_encoder = new EncoderState(0,ENC_TICKS,0,0);

//ids of the two motors
const short LEFT_MOTOR=0;
const short RIGHT_MOTOR=1;

//timers
int t_new, t_old, t_old_comm, t_old_serial;
int t_last_twist_milis;
int dT = 10000; //sampling time in microseconds
int dT_serial = 50000; //sampling time for output in microseconds
int seq_number = 0;

//velocity filtering parameter
float alpha = 0.09;
float beta = 0.01;

//tracked state
double x=0,y=0,theta=0;
long ticks_r_prev=0, ticks_l_prev=0;
double wheel_radius=0.03, base_length=0.18;

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////ROS Callbacks//////////////////////////
//callback function for setting new PID parameters
void setPIDCallback(const sensors_lab2_2018::SetPID::Request &req, sensors_lab2_2018::SetPID::Response &res);
void setVelCallback(const sensors_lab2_2018::SetVel::Request &req, sensors_lab2_2018::SetVel::Response &res);
void setDDCallback(const sensors_lab2_2018::SetDDParams::Request &req, sensors_lab2_2018::SetDDParams::Response &res);
void resetOdomCallback(const sensors_lab2_2018::ResetOdom::Request &req, sensors_lab2_2018::ResetOdom::Response &res);
//note: this is for vehicle velocity
void setVelocityCallback(const geometry_msgs::Twist &msg);
void computeCurrentPose();
void checkFreshTwist();
///////////////////////////////////////////////////////////////////////////////////

///////////////////ROS global variables/////////////////////////
ros::NodeHandle nh;
sensors_lab2_2018::MotorStates state;
ros::Publisher state_publisher("/motor_states", &state);
geometry_msgs::Pose odom;
ros::Publisher odom_publisher("/odom", &odom);

ros::ServiceServer<sensors_lab2_2018::SetPID::Request, sensors_lab2_2018::SetPID::Response> pid_server("set_pid", &setPIDCallback);
ros::ServiceServer<sensors_lab2_2018::SetVel::Request, sensors_lab2_2018::SetVel::Response> vel_server("set_vel", &setVelCallback);
ros::ServiceServer<sensors_lab2_2018::SetDDParams::Request, sensors_lab2_2018::SetDDParams::Response> ddrive_param_server("set_ddrive_params", &setDDCallback);
ros::ServiceServer<sensors_lab2_2018::ResetOdom::Request, sensors_lab2_2018::ResetOdom::Response> reset_server("reset", &resetOdomCallback);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub_("/cmd_vel", &setVelocityCallback);

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//////////////////////////// function declarations /////////////////////////////
////////////////////////////////////////////////////////////////////////////////


//setup interupts and serial communication
void setup()
{
 
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  //setup timers
  t_old = micros();
  t_old_serial = micros();

  ////// ROS initializations////
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(state_publisher);
  nh.advertise(odom_publisher);
  nh.advertiseService(pid_server);
  nh.advertiseService(vel_server);
  nh.advertiseService(ddrive_param_server);
  nh.advertiseService(reset_server);
  nh.subscribe(cmd_vel_sub_);
  
  //TODO: advertise additional services and publishers, subscribe to command topic
  ///////////////////////////////
}


/////////////////////////////////////////////////////////////////
/////////////////// Main function ///////////////////////////////
/////////////////////////////////////////////////////////////////

void loop()
{
  
  //spin and check if we should publish
  t_new = micros();
  nh.spinOnce();
  if (abs(t_new - t_old_serial) > dT_serial) {
    //TODO: here update the state varriable with the most recent measurements
    state.header.seq = seq_number;
    state.header.stamp = nh.now();
    state.header.frame_id="/makeblock";
    state.left.ticks = left_encoder->ticks_;
    state.left.pos = left_motor_state->r_; //left_encoder->pos_;
    state.left.vel = left_encoder->vel_;
    state.right.ticks = right_encoder->ticks_;
    state.right.pos = right_motor_state->r_;//right_encoder->pos_;
    state.right.vel = right_encoder->vel_;
    
    state_publisher.publish(&state);

    computeCurrentPose();
    odom_publisher.publish(&odom);

    checkFreshTwist();
    
    t_old_serial = t_new;
    seq_number++;
  }
  
  //Do nothing if the sampling period didn't pass yet
  if (abs(t_new - t_old) < dT)
    return;
  t_old = t_new;

  ////// the logic bellow can be copied from the previous lab ////
  //calculate new encoder velocity and position
  float enc_pos, enc_vel;
  enc_pos = left_encoder->ticks_/left_encoder->ratio_;
  enc_vel = (float)(enc_pos - left_encoder->pos_)*1e6 / (float)dT;
  left_encoder->pos_ = enc_pos;
  //apply low-pass filter
  left_encoder->vel_ = (1-alpha)*left_encoder->vel_ + alpha*enc_vel;
 
  //calculate error and derivative
  float err, de_raw;
  err = left_motor_state->r_-left_encoder->vel_; 
  de_raw = (left_motor_state->e_ - err)*1e6 / (float)dT;
  left_motor_state->de_ = (1-beta)*left_motor_state->de_ + beta*de_raw;
  left_motor_state->e_ = err;
  left_motor_state->u_ = pid(left_motor_state->e_, left_motor_state->de_, pid_left_vc);

  //if bellow minmum command, clamp
  if(left_motor_state->u_ < MIN_ACTUATION_PWM && left_motor_state->u_ > -MIN_ACTUATION_PWM) {
    left_motor_state->u_=0;    
  }
  
  Encoder_1.setMotorPwm(-left_motor_state->u_);
  
  //TODO: same for right motor 
  enc_pos = right_encoder->ticks_/right_encoder->ratio_;
  enc_vel = (float)(enc_pos - right_encoder->pos_)*1e6 / (float)dT;
  right_encoder->pos_ = enc_pos;
  //apply low-pass filter
  right_encoder->vel_ = (1-alpha)*right_encoder->vel_ + alpha*enc_vel;
 
  //calculate error and derivative
  err = right_motor_state->r_-right_encoder->vel_; 
  de_raw = (right_motor_state->e_ - err)*1e6 / (float)dT;
  right_motor_state->de_ = (1-beta)*right_motor_state->de_ + beta*de_raw;
  right_motor_state->e_ = err;
  right_motor_state->u_ = pid(right_motor_state->e_, right_motor_state->de_, pid_right_vc);

  //if bellow minmum command, clamp
  if(right_motor_state->u_ < MIN_ACTUATION_PWM && right_motor_state->u_ > -MIN_ACTUATION_PWM) {
    right_motor_state->u_=0;    
  }
  
  Encoder_2.setMotorPwm(right_motor_state->u_);

}

void setPIDCallback(const sensors_lab2_2018::SetPID::Request &req, sensors_lab2_2018::SetPID::Response &res) {
  res.success = true;
  if (req.motor_id == LEFT_MOTOR) {            
    pid_left_vc->Kp_ = req.kp;
    pid_left_vc->Ki_ = req.ki;
    pid_left_vc->Kd_ = req.kd;
  } else if (req.motor_id == RIGHT_MOTOR) {     
    pid_right_vc->Kp_ = req.kp;
    pid_right_vc->Ki_ = req.ki;
    pid_right_vc->Kd_ = req.kd;
  } else {
    res.success=false;
  }
}

void setVelCallback(const sensors_lab2_2018::SetVel::Request &req, sensors_lab2_2018::SetVel::Response &res) {
  res.success = true;
  if (req.motor_id == LEFT_MOTOR) {            
    left_motor_state->r_ = req.vel;
    left_motor_state->e_ = 0;
    left_motor_state->de_ = 0;
    pid_left_vc->I_ = 0;
  } else if (req.motor_id == RIGHT_MOTOR) {     
    right_motor_state->r_ = req.vel;
    right_motor_state->e_ = 0;
    right_motor_state->de_ = 0;
    pid_right_vc->I_ = 0;
  } else {
    res.success=false;
  }
}

void setDDCallback(const sensors_lab2_2018::SetDDParams::Request &req, sensors_lab2_2018::SetDDParams::Response &res) {
  wheel_radius = req.r;
  base_length = req.L;  
}

void resetOdomCallback(const sensors_lab2_2018::ResetOdom::Request &req, sensors_lab2_2018::ResetOdom::Response &res) {
  x = 0.0;
  y = 0.0;
  theta = 0.0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///                             Odometry functions here                                       ///
/////////////////////////////////////////////////////////////////////////////////////////////////
void computeCurrentPose() {
  float dl, dr, dx, dy, dtheta;
  dr = (right_encoder->ticks_ - ticks_r_prev) / right_encoder->ratio_;
  dl = (left_encoder->ticks_ - ticks_l_prev) / left_encoder->ratio_;
  ticks_r_prev = right_encoder->ticks_;
  ticks_l_prev = left_encoder->ticks_;

  dtheta = wheel_radius*(dr-dl)/base_length;
  dx = wheel_radius*(dr+dl)*cos(dtheta)/2;
  dy = wheel_radius*(dr+dl)*sin(dtheta)/2;

  x = x + cos(theta)*dx - sin(theta)*dy;
  y = y + cos(theta)*dy + sin(theta)*dx;
  theta = theta + dtheta;
  
  odom.position.x = x;
  odom.position.y = y;
  odom.position.z = 0;
  odom.orientation.x = 0;
  odom.orientation.y = 0;
  odom.orientation.z = sin(theta/2);
  odom.orientation.w = cos(theta/2);
}

void setVelocityCallback(const geometry_msgs::Twist &msg) {

   double lin_vel = sqrt(msg.linear.x*msg.linear.x + msg.linear.y*msg.linear.y);
   double ang_vel = msg.angular.z;
   int sign = msg.linear.x > 0 ? 1 : -1;

   left_motor_state->r_ = sign*(lin_vel-base_length*ang_vel/2.0)/wheel_radius;
   left_motor_state->e_ = 0;
   left_motor_state->de_ = 0;
   pid_left_vc->I_ = 0;

   right_motor_state->r_ = sign*(lin_vel+base_length*ang_vel/2.0)/wheel_radius;
   right_motor_state->e_ = 0;
   right_motor_state->de_ = 0;
   pid_right_vc->I_ = 0;

   t_last_twist_milis = millis();
}

void checkFreshTwist() {
   int t_now_milis = millis();
   if (t_now_milis - t_last_twist_milis > 500) {
     //we stop
     left_motor_state->r_ = 0;
     left_motor_state->e_ = 0;
     left_motor_state->de_ = 0;
     pid_left_vc->I_ = 0;
  
     right_motor_state->r_ = 0;
     right_motor_state->e_ = 0;
     right_motor_state->de_ = 0;
     pid_right_vc->I_ = 0;
   }  
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// NOTE: REMAINING FUNCTIONS CAN BE COPPIED FROM THE PREVIOUS LAB ////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//TODO check integer limits
void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    left_encoder->ticks_++;
  }
  else
  {
    left_encoder->ticks_--;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    right_encoder->ticks_--;
  }
  else
  {
    right_encoder->ticks_++;
  }
}

//--------------------------------------------------------------------------//
//   Here compute the PID control for a given error, de, and PID params     //
//--------------------------------------------------------------------------//
float pid(float e, float de, PIDParameters* p)
{  
  p->I_ += p->Ki_ * e; //update the integral term
  float u = p->Kp_ * e + p->Kd_ * de + p->I_; //compute the control value

  //clamp the control value and back-calculate the integral term (the latter to avoid windup)
  if (u > p->u_max_)
  {
    p->I_ -= p->Ki_ * e; //u - p->u_max_;
    u = p->u_max_;
  }
  else if (u < p->u_min_)
  {
    p->I_ -= p->Ki_ * e; //p->u_min_ - u;
    u = p->u_min_;
  }

  return u;
}


