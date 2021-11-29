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

static const speedConfig speed_config =
{
	.min_vel = -1.0,
	.max_vel = 1.0,
	.min_acc = 1.0,
	.max_acc = 1.0,
	.min_jerk = 1.0,
	.max_jerk = 1.0
};

//Local function prototype declaration
static void limit_velocity(float* v, speedConfig* speed_config);
static void limit_acceleration(float* v, float prev_v, float dt, speedConfig* speed_config);
static void limit_jerk(float* v, float v0, float v1, float dt, speedConfig* speed_config);
static float clamp(float x, float min, float max);

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
void differentialDriveInit(differentialDrive_Handler* dd_handler, uint32_t frequency ){
	dd_handler->m_leftMotor = 0;
	dd_handler->m_rightMotor = 0;
	dd_handler->m_fPivYLimit = 0.2 * COMPUTERANGE;
	dd_handler->frequency = frequency;
	memset(&dd_handler->left_motor_state, 0, sizeof(dd_handler->left_motor_state));
	memset(&dd_handler->right_motor_state, 0, sizeof(dd_handler->right_motor_state));
}

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
void computeSpeed(differentialDrive_Handler* dd_handler, int XValue, int YValue) {
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
    nPivSpeed = XValue;
    fPivScale = (abs(YValue) > dd_handler->m_fPivYLimit) ? 0.0 : (1.0 - abs(YValue) / dd_handler->m_fPivYLimit);

    // Calculate final mix of Drive and Pivot
    nMotPremixL  = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
    nMotPremixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

    //Normalize the output from -1 to 1
	dd_handler->m_leftMotor = nMotPremixL / COMPUTERANGE;
	dd_handler->m_rightMotor = nMotPremixR / COMPUTERANGE;
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
static void limit_velocity(float* v, speedConfig* speed_config)
{
    *v = clamp(*v, speed_config->min_vel, speed_config->max_vel);
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
static void limit_acceleration(float* v, float prev_v, float dt, speedConfig* speed_config)
{
    const float dv_min = speed_config->min_acc * dt;
    const float dv_max = speed_config->max_acc * dt;

    const float dv = clamp(*v - prev_v, dv_min, dv_max);

    *v = prev_v + dv;
}

/*********************************************************************
 * @fn      		- limit_jerk
 *
 * @brief          	- Clamp jerk between acceleration define in speed_config
 *
 * @param[*v]     		- latest velocity calculated
 * @param[v0]     		- last previous velocity
 * @param[v1]     		- second previous velocity
 * @param[dt]     		- time elapsed
 * @param[*speed_config]- user defined min/max desired speed configuration
 *
 * @return            - None
 *
 * @Note              -
 */
static void limit_jerk(float* v, float v0, float v1, float dt, speedConfig* speed_config)
{
    const float dv  = *v  - v0;
    const float dv0 = v0 - v1;

    const float dt2 = 2. * dt * dt;

    const float da_min = speed_config->min_jerk * dt2;
    const float da_max = speed_config->max_jerk * dt2;

    const float da = clamp(dv - dv0, da_min, da_max);

    *v = v0 + dv0 + da;
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



