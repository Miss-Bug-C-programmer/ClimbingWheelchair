/*
 * differentialDrive.c
 *
 *  Created on: 26 Nov 2021
 *      Author: ray
 */

#include "differentialDrive.h"
#include <math.h>
#include <stdlib.h>

#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#define MIN(x,y) (((x) < (y)) ? (x) : (y))

/**
 *  Constructor class.
 */
void differentialDriveInit(differentialDrive_Handler* dd_handler){
	dd_handler->m_leftMotor = 0;
	dd_handler->m_rightMotor = 0;
	dd_handler->m_fPivYLimit = 0.2 * COMPUTERANGE;
}

/**
 * Compute differential steering from (x,y) values.
 *
 * @param XValue: X value in [-MAX_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE] range.
 * @param YValue: Y value in [-MAX_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE] range.
 */
void computeMotors(int XValue, int YValue) {
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
    dd_handler->m_leftMotor  = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
    dd_handler->m_rightMotor = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);
}

