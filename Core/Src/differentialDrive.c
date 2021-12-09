/*
 * differentialDrive.c
 *
 *
 *  Created on: 26 Nov 2021
 *      Author: ray
 */

#include "differentialDrive.h"


#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#define MIN(x,y) (((x) < (y)) ? (x) : (y))



//Local function prototype declaration
static void limit_velocity(float* v);
static void limit_acceleration(float* v, float prev_v, float dt);
static float clamp(float x, float min, float max);
void speedLimiter(motorState* motor_state, uint32_t frequency, float velocity);

/*********************************************************************
 * @fn      		  - differentialDriveInit
 *
 * @brief             - Used to initialize the differential drive handler member to 0
 *
 * @param[differentialDrive_Handler*] - pointer to handler. User must initialize the handler in the main file
 *
 * @return            - None
 *
 * @Note              -
 */
void differentialDriveInit(differentialDrive_Handler* dd_handler, uint32_t frequency){
	dd_handler->m_leftMotor = 0;
	dd_handler->m_rightMotor = 0;
	dd_handler->m_fPivYLimit = 0.15 * COMPUTERANGE;
	dd_handler->frequency = frequency;
	memset(&dd_handler->left_motor_state, 0, sizeof(dd_handler->left_motor_state));
	memset(&dd_handler->right_motor_state, 0, sizeof(dd_handler->right_motor_state));
}

/*********************************************************************
 * @fn      		  - speedConfiguration
 *
 * @brief             - Used to initialize the speed configuration member
 *
 * @param[differentialDrive_Handler*] - pointer to handler. User must initialize the handler in the main file
 *
 * @return            - None
 *
 * @Note              -
 */
//void speedConfiguration(speedConfig* speed_config){
//	speed_config->min_vel = 0;
//	speed_config->max_vel = 0;
//	speed_config->min_acc = 0;
//	speed_config->max_acc = 0;
//	speed_config->min_jerk = 0;
//	speed_config->max_jerk = 0;
//}

/*********************************************************************
 * @fn      		  - computeSpeed
 *
 * @brief             - Compute differential steering from (x,y) values.
 * 						Compute speed of the output motor through mapping of
 * 						joystick position to relative speed
 *
 * @param[XValue]     - X value in [-MAX_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE] range.
 * @param[YValue]     - YValue: Y value in [-MAX_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE] range.
 *
 * @return            - None
 *
 * @Note              -
 */
void computeSpeed(differentialDrive_Handler* dd_handler, int XValue, int YValue, Gear_Level gear_level) {
    float   nMotPremixL = 0;    // Motor (left)  premixed output        (-MAX_JOYSTICK_VALUE..+MAX_JOYSTICK_VALUE)
    float   nMotPremixR = 0;    // Motor (right) premixed output        (-MAX_JOYSTICK_VALUE..+MAX_JOYSTICK_VALUE)
    int     nPivSpeed = 0;      // Pivot Speed                          (-MAX_JOYSTICK_VALUE..+MAX_JOYSTICK_VALUE)
    float   fPivScale = 0;      // Balance scale b/w drive and pivot    (   0..1   )

    // Calculate Drive Turn output due to Joystick X input
    if (YValue >= 0) {
        // Forward
        nMotPremixL = (XValue >= 0) ? COMPUTERANGE : (COMPUTERANGE + XValue);
        nMotPremixR = (XValue >= 0) ? (COMPUTERANGE - XValue) : COMPUTERANGE;
    } else {
        // Reverse
        nMotPremixL = (XValue >= 0) ? (COMPUTERANGE - XValue) : COMPUTERANGE;
        nMotPremixR = (XValue >= 0) ? COMPUTERANGE : (COMPUTERANGE + XValue);
    }

    // Scale Drive output due to Joystick Y input (throttle)
    nMotPremixL = nMotPremixL * YValue / COMPUTERANGE;
    nMotPremixR = nMotPremixR * YValue / COMPUTERANGE;

    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
    nPivSpeed = (abs(YValue) > dd_handler->m_fPivYLimit) ? XValue : XValue * 0.50;
    fPivScale = (abs(YValue) > dd_handler->m_fPivYLimit) ? 0.0 : (1.0 - (float)(abs(YValue)) / (float)dd_handler->m_fPivYLimit);

    // Calculate final mix of Drive and Pivot
    nMotPremixL  = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
    nMotPremixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

    //Normalize the output from -1 to 1
	dd_handler->m_leftMotor = (nMotPremixL / (float)COMPUTERANGE) * ((float)gear_level/100.0);
	dd_handler->m_rightMotor = (nMotPremixR / (float)COMPUTERANGE) * ((float)gear_level/100.0);

	//Force motor when speed approximately zero to avoid current consumption
	dd_handler->m_leftMotor = 	(dd_handler->m_leftMotor < 0.008 && dd_handler->m_leftMotor > -0.005) ? 0 : dd_handler->m_leftMotor;
	dd_handler->m_rightMotor = 	(dd_handler->m_rightMotor < 0.005 && dd_handler->m_rightMotor > -0.005)? 0 : dd_handler->m_rightMotor;
}

void speedLimiter(motorState* motor_state, uint32_t frequency, float velocity){
	float dt;
	//If first read or stationary for too long
	if (motor_state->prev_t == 0){
		motor_state->prev_t = HAL_GetTick();
		motor_state->prev_vel = velocity;
		return;
	}
	motor_state->vel = velocity;
	dt = (float)(HAL_GetTick() - motor_state->prev_t) / (float)frequency;

	limit_acceleration(&motor_state->vel, motor_state->prev_vel, dt);
	limit_velocity(&motor_state->vel);
	motor_state->prev_vel = motor_state->vel;
	motor_state->prev_t = HAL_GetTick();

}

/*********************************************************************
 * @fn      		- limit_velocity
 *
 * @brief          	- Clamp velocity between speed define in speed_config
 *
 * @param[*v]     	- latest velocity calculated
 * @param[*speed_config] - user defined min/max desired speed configuration
 *
 * @return            - None
 *
 * @Note              -
 */
static void limit_velocity(float* v)
{
    *v = clamp(*v, speed_config.min_vel, speed_config.max_vel);
}

/*********************************************************************
 * @fn      		- limit_acceleration
 *
 * @brief          	- Clamp acceleration between acceleration define in speed_config
 *
 * @param[*v]     		- latest velocity calculated
 * @param[prev_v]     	- previous velocity
 * @param[dt]     		- time elapsed
 * @param[*speed_config]- user defined min/max desired speed configuration
 *
 * @return            - None
 *
 * @Note              -
 */
static void limit_acceleration(float *v, float prev_v, float dt)
{
    const float dv_min = speed_config.min_acc * dt;
    const float dv_max = speed_config.max_acc * dt;

    const float dv = clamp(*v - prev_v, dv_min, dv_max);

    *v = prev_v + dv;
}

/*********************************************************************
 * @fn      		- clamp
 *
 * @brief          	- Clamp x between min and max
 *
 * @param[x]     		- input value
 * @param[min]     		- min value allowed
 * @param[max]     		- max value allowed
 *
 * @return            - None
 *
 * @Note              -
 */
static float clamp(float x, float min, float max)
{
  return MIN(MAX(min, x), max);
}



