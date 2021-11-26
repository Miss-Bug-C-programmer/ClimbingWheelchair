/** @file   wheelchair.c
 *  @brief  Source file of control algorithm in wheelchair mode.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#include "wheelchair.h"
JoystickHandle hJoystick;
extern TIM_HandleTypeDef htim3;

static const int JoystickCenterX = 32000;
static const int JoystickCenterY = 32000;
static const int JoystickMagnitudeMax = 18000;
static const int JoystickMagnitudeMin = 3000;
static const int JoyPosBufferSize = 5;
static int joyPosBuffer[2][5] = {0};
static int joy_pos_buffer_cnt = 0;
static const float JoyForwardAngle = 1.57;
static const float JoyForwardAngleDeadzone = 0.3;
static const float JoyTurnAngleDeadzone = 0.2;

static const float JoyLeftTurnAngle = 3.142;
static const float JoyRightTurnAngle = 0;

void joystick_Init(void){
  memset(joyPosBuffer, 0, sizeof(joyPosBuffer));
}

void joystickCalculatePos(void)
{
  // update joystick reading into buffer array
  if (joy_pos_buffer_cnt == JoyPosBufferSize)
    joy_pos_buffer_cnt = 0;

  tempJoyRawDataX = (tempJoyRawDataX < 0)? tempJoyRawDataX + JoystickCenterX : tempJoyRawDataX - JoystickCenterX;
  tempJoyRawDataY = (tempJoyRawDataY > 0)? JoystickCenterY - tempJoyRawDataY:  -tempJoyRawDataY - JoystickCenterX;

  joyPosBuffer[0][joy_pos_buffer_cnt] = tempJoyRawDataX;
  joyPosBuffer[1][joy_pos_buffer_cnt] = tempJoyRawDataY;

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

  // filter joystick right turn deadzone
	if (hJoystick.angle > JoyRightTurnAngle - JoyTurnAngleDeadzone &&
		hJoystick.angle < JoyRightTurnAngle + JoyTurnAngleDeadzone)
	  hJoystick.angle = JoyRightTurnAngle;
  // filter joystick left turn deadzone
    if (hJoystick.angle > JoyLeftTurnAngle - JoyTurnAngleDeadzone ||
        hJoystick.angle < -JoyLeftTurnAngle + JoyTurnAngleDeadzone)
      hJoystick.angle = JoyLeftTurnAngle;

  // normalize joystick reading
  hJoystick.linear = hJoystick.magnitude/JoystickMagnitudeMax * sin(hJoystick.angle);
  hJoystick.angular = hJoystick.magnitude/JoystickMagnitudeMax * cos(hJoystick.angle);

  if (fabs(hJoystick.linear) < 0.05)
    hJoystick.linear = 0;
  if (fabs(hJoystick.angular) < 0.05)
    hJoystick.angular = 0;

  joy_pos_buffer_cnt++;
}

void wheelSpeedControl_Init(WheelSpeed* wheel, float max_lin_speed, float max_ang_speed)
{
  wheel->stable_cnt = 0;
  wheel->cur_r = 0.0f;
  wheel->cur_l = 0.0f;
  wheel->pre_l= 0.0f;
  wheel->pre_r = 0.0f;
  wheel->max_angular_speed = max_ang_speed;
  wheel->max_linear_speed = max_lin_speed;
  wheel->start_from_stationary = false;
}

void wheel_Control(WheelSpeed* wheel)
{
  if (wheel->stable_cnt < 25)
  {
      wheel->stable_cnt++;
    return;
  }

  joystickCalculatePos();
  wheelCalculateSpeed(wheel);

  if (wheel->pre_l == 0 && wheel->pre_r == 0)
    wheel->start_from_stationary = true;

  if (hJoystick.magnitude > JoystickMagnitudeMin)
  {
    float left_speed_step = wheel->left_speed_step;
    float right_speed_step = wheel->right_speed_step;

    if (wheel->start_from_stationary)
    {
    	//deadzone 25, speed up initial speed
      left_speed_step = 25 + fabs( wheel->cur_l) / wheel->accel_loop;
      right_speed_step = 25 + fabs( wheel->cur_l) / wheel->accel_loop;

      if (fabs(wheel->pre_l) > 0.5f * wheel->max_angular_speed &&
          fabs(wheel->pre_r) > 0.5f * wheel->max_angular_speed)
      {
	  wheel->start_from_stationary = false;
      }
    }

    if (wheel->cur_l/wheel->pre_l < 0)
    	left_speed_step = 50;
    if (wheel->cur_r/wheel->pre_r < 0)
        	right_speed_step = 50;

    if (( wheel->cur_l - wheel->pre_l) > left_speed_step)
      wheel->cur_l = wheel->pre_l + left_speed_step;
    else if ((wheel->cur_l - wheel->pre_l) < -left_speed_step)
      wheel->cur_l = wheel->pre_l - left_speed_step;

    if ((wheel->cur_r - wheel->pre_r) > right_speed_step)
      wheel->cur_r = wheel->pre_r + right_speed_step;
    else if ((wheel->cur_r - wheel->pre_r) < -right_speed_step)
      wheel->cur_r = wheel->pre_r - right_speed_step;
  }
  else
  {

//    float zero_speed = wheel->max_linear_speed / wheel->decel_loop;
//    if (fabs(wheel->cur_l) < zero_speed)
//      wheel->cur_l = 0;
//    if (fabs(wheel->cur_r) < zero_speed)
//      wheel->cur_r = 0;

//    float left_speed_step = fabs(wheel->cur_l) / wheel->decel_loop;
//    float right_speed_step = fabs(wheel->cur_r) / wheel->decel_loop;
//
    float left_speed_step = 30;
    float right_speed_step = 30;



//    if (wheel->cur_l >= left_speed_step)
//      wheel->cur_l = wheel->pre_l - left_speed_step;
//    else if (wheel->cur_l < -left_speed_step)
//      wheel->cur_l = wheel->pre_l + left_speed_step;
//    else
//      wheel->cur_l = 0;
//
//    if (wheel->cur_r > right_speed_step)
//      wheel->cur_r = wheel->pre_r - right_speed_step;
//    else if (wheel->cur_r < -right_speed_step)
//      wheel->cur_r = wheel->pre_r + right_speed_step;
//    else
//      wheel->cur_r = 0;
    if (wheel->cur_l/wheel->pre_l < 0)
    	left_speed_step = 50;
    if (wheel->cur_r/wheel->pre_r < 0)
    	right_speed_step = 50;

    //Latency when sign change
    if (wheel->pre_l > left_speed_step)
      wheel->cur_l = wheel->pre_l - left_speed_step;
    else if (wheel->pre_l < -left_speed_step)
      wheel->cur_l = wheel->pre_l + left_speed_step;
    else
      wheel->cur_l = 0;
    
    if (wheel->pre_r > right_speed_step)
      wheel->cur_r = wheel->pre_r - right_speed_step;
    else if (wheel->pre_r < -right_speed_step)
      wheel->cur_r = wheel->pre_r + right_speed_step;
    else
      wheel->cur_r = 0;




  }

  if (wheel->cur_l > wheel->max_linear_speed)
    wheel->cur_l = wheel->max_linear_speed;
  if (wheel->cur_r > wheel->max_linear_speed)
    wheel->cur_r = wheel->max_linear_speed;

  if (wheel->cur_l < -wheel->max_linear_speed)
    wheel->cur_l = -wheel->max_linear_speed;
  if (wheel->cur_r < -wheel->max_linear_speed)
    wheel->cur_r = -wheel->max_linear_speed;

  wheel->pre_l = wheel->cur_l;
  wheel->pre_r = wheel->cur_r;
  
}



void wheelCalculateSpeed(WheelSpeed* wheel)
{
  float linearSpeed = wheel->max_linear_speed * hJoystick.linear;
  float angularSpeed = wheel->max_angular_speed *  hJoystick.angular;

  wheel->cur_l = linearSpeed + angularSpeed;
  wheel->cur_r = linearSpeed - angularSpeed;

  // direct step to 0 if speed is small enough50
  // direct step to 0 if speed is small enough50
  if(fabs(wheel->cur_l) < 25)
    wheel->cur_l = 0;
  if(fabs(  wheel->cur_r) < 25)
    wheel->cur_r = 0;
}


