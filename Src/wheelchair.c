/** @file   wheelchair.c
 *  @brief  Source file of control algorithm in wheelchair mode.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#include "wheelchair.h"

JoystickHandle hJoystick;
extern TIM_HandleTypeDef htim3;

static const int JoystickCenterX = 16200;
static const int JoystickCenterY = 16000;
static const int JoystickMagnitudeMax = 11000;
static const int JoystickMagnitudeMin = 2500;
static const int JoyPosBufferSize = 5;
static int joyPosBuffer[2][5] = {0};
static int joy_pos_buffer_cnt = 0;
static int wheelchr_stable_cnt = 0;
static const float JoyForwardAngle = 1.57;
static const float JoyForwardAngleDeadzone = 0.1;

static const float LinearSpeedLevel[3] = {200.0f, 300.0f, 400.0f};
static const float AngularSpeedLevel[3] = {100.0f, 150.0f, 200.0f};

int speed_level = 1; //change the speed level if need higher speed
bool start_from_stationary = true;



void WHEELCHR_Init(void)
{
  wheelchr_stable_cnt = 0;
  wheelchr_wheelspeed_cur.l = 0.0f;
  wheelchr_wheelspeed_cur.r = 0.0f;
  wheelchr_wheelspeed_pre.l = 0.0f;
  wheelchr_wheelspeed_pre.r = 0.0f;
  memset(joyPosBuffer, 0, sizeof(joyPosBuffer));
}

void WHEELCHR_JoystickControl(void)
{
  if (wheelchr_stable_cnt < 200)
  {
    wheelchr_stable_cnt++;
    return;
  }

  WHEELCHR_JoystickCalculatePos();
  WHEELCHR_JoystickCalculateSpeed();

  if (wheelchr_wheelspeed_pre.l == 0 && wheelchr_wheelspeed_pre.r == 0)
    start_from_stationary = true;

  if (hJoystick.magnitude > JoystickMagnitudeMin)
  {
    float left_speed_step = 0.5;
    float right_speed_step = 0.5;

    if (start_from_stationary)
    {
      float accel_loop = 600.0f;
      left_speed_step = fabs(wheelchr_wheelspeed_cur.l) / accel_loop;
      right_speed_step = fabs(wheelchr_wheelspeed_cur.r) / accel_loop;

      if (fabs(wheelchr_wheelspeed_pre.l) > 0.5f * AngularSpeedLevel[speed_level] &&
          fabs(wheelchr_wheelspeed_pre.r) > 0.5f * AngularSpeedLevel[speed_level])
      {
        start_from_stationary = false;
      }
    }

    if ((wheelchr_wheelspeed_cur.l - wheelchr_wheelspeed_pre.l) > left_speed_step)
      wheelchr_wheelspeed_cur.l = wheelchr_wheelspeed_pre.l + left_speed_step;
    else if ((wheelchr_wheelspeed_cur.l - wheelchr_wheelspeed_pre.l) < -left_speed_step)
      wheelchr_wheelspeed_cur.l = wheelchr_wheelspeed_pre.l - left_speed_step;

    if ((wheelchr_wheelspeed_cur.r - wheelchr_wheelspeed_pre.r) > right_speed_step)
      wheelchr_wheelspeed_cur.r = wheelchr_wheelspeed_pre.r + right_speed_step;
    else if ((wheelchr_wheelspeed_cur.r - wheelchr_wheelspeed_pre.r) < -right_speed_step)
      wheelchr_wheelspeed_cur.r = wheelchr_wheelspeed_pre.r - right_speed_step;
  }
  else
  {
    float decel_loop = 150.0f;

    float zero_speed = LinearSpeedLevel[speed_level] / decel_loop;
    if (fabs(wheelchr_wheelspeed_cur.l) < zero_speed)
      wheelchr_wheelspeed_cur.l = 0;
    if (fabs(wheelchr_wheelspeed_cur.r) < zero_speed)
      wheelchr_wheelspeed_cur.r = 0;

    float left_speed_step = fabs(wheelchr_wheelspeed_cur.l) / decel_loop;
    float right_speed_step = fabs(wheelchr_wheelspeed_cur.r) / decel_loop;
    
    if (wheelchr_wheelspeed_cur.l > left_speed_step)
      wheelchr_wheelspeed_cur.l = wheelchr_wheelspeed_pre.r - left_speed_step;
    else if (wheelchr_wheelspeed_cur.l < -left_speed_step)
      wheelchr_wheelspeed_cur.l = wheelchr_wheelspeed_pre.r + left_speed_step;
    else
      wheelchr_wheelspeed_cur.l = 0;
    
    if (wheelchr_wheelspeed_cur.r > right_speed_step)
      wheelchr_wheelspeed_cur.r = wheelchr_wheelspeed_pre.r - right_speed_step;
    else if (wheelchr_wheelspeed_cur.r < -right_speed_step)
      wheelchr_wheelspeed_cur.r = wheelchr_wheelspeed_pre.r + right_speed_step;
    else
      wheelchr_wheelspeed_cur.r = 0;
  }

  if (wheelchr_wheelspeed_cur.l > LinearSpeedLevel[speed_level])
    wheelchr_wheelspeed_cur.l = LinearSpeedLevel[speed_level];
  if (wheelchr_wheelspeed_cur.r > LinearSpeedLevel[speed_level])
    wheelchr_wheelspeed_cur.r = LinearSpeedLevel[speed_level];

  if (wheelchr_wheelspeed_cur.l < -LinearSpeedLevel[speed_level])
    wheelchr_wheelspeed_cur.l = -LinearSpeedLevel[speed_level];
  if (wheelchr_wheelspeed_cur.r < -LinearSpeedLevel[speed_level])
    wheelchr_wheelspeed_cur.r = -LinearSpeedLevel[speed_level];

  MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL = (int)wheelchr_wheelspeed_cur.r  + 1500;
  MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL = (int)wheelchr_wheelspeed_cur.l + 1500;

  wheelchr_wheelspeed_pre.l = wheelchr_wheelspeed_cur.l;
  wheelchr_wheelspeed_pre.r = wheelchr_wheelspeed_cur.r;
  
}

void WHEELCHR_JoystickCalculatePos(void)
{
  // update joystick reading into buffer array
  if (joy_pos_buffer_cnt == JoyPosBufferSize)
    joy_pos_buffer_cnt = 0;
  
  joyPosBuffer[0][joy_pos_buffer_cnt] = tempJoyRawDataX - JoystickCenterX;
  joyPosBuffer[1][joy_pos_buffer_cnt] = tempJoyRawDataY - JoystickCenterY;

  // calculate joystick position average from the buffer
  int sum_x = 0;
  int sum_y = 0;
  for (int i = 0; i < JoyPosBufferSize; i++)
  {
    sum_x += joyPosBuffer[0][i];
    sum_y += joyPosBuffer[1][i];
  }

  hJoystick.x = sum_x / JoyPosBufferSize;
  hJoystick.y = sum_y / JoyPosBufferSize;

  // calculate magnitude and angle
  hJoystick.magnitude = sqrt(pow(hJoystick.x, 2) + pow(hJoystick.y,2));
  hJoystick.angle = atan2(hJoystick.y, hJoystick.x);

  // limit magnitude
  if (hJoystick.magnitude > JoystickMagnitudeMax)
    hJoystick.magnitude = JoystickMagnitudeMax;

  // filter joystick forward deadzone
  if (hJoystick.angle > JoyForwardAngle - JoyForwardAngleDeadzone &&
      hJoystick.angle < JoyForwardAngle + JoyForwardAngleDeadzone)
    hJoystick.angle = JoyForwardAngle;

  // filter joystick backward deadzone
  if (hJoystick.angle > -(JoyForwardAngle + JoyForwardAngleDeadzone) &&
      hJoystick.angle < -(JoyForwardAngle - JoyForwardAngleDeadzone))
    hJoystick.angle = -JoyForwardAngle;

  joy_pos_buffer_cnt++;
}

void WHEELCHR_JoystickCalculateSpeed(void)
{
  float linearSpeed = LinearSpeedLevel[speed_level] * hJoystick.magnitude/JoystickMagnitudeMax * sin(hJoystick.angle);
  float angularSpeed = AngularSpeedLevel[speed_level] *  hJoystick.magnitude/JoystickMagnitudeMax * cos(hJoystick.angle);

  wheelchr_wheelspeed_cur.l = linearSpeed + angularSpeed;
  wheelchr_wheelspeed_cur.r = linearSpeed - angularSpeed;

  // direct step to 0 if speed is small enough
  if(fabs(wheelchr_wheelspeed_cur.l) < 50)
	  wheelchr_wheelspeed_cur.l = 0;
  if(fabs(wheelchr_wheelspeed_cur.r) < 50)
	  wheelchr_wheelspeed_cur.r = 0;
}


