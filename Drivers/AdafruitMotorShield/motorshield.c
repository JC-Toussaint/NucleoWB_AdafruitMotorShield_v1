/*
 * motorshield.c
 *
 *  Created on: 11 August 2014
 *      Author: ktown
 */

#include <string.h>
#include <math.h>

#include "motorshield.h"

/* The maximum number of motors supported by v2 of the motor shield */
#define MS_MOTORCOUNT_DC       (4)                   /* Maximum number of DC motors */
#define MS_MOTORCOUNT_STEPPER  (2)                   /* Maximum number of stepper motors */
#define MS_MOTORCOUNT_SERVO    (2)                   /* Maximum number of servo motors */

/* These dividers set the min and max duty cycle (width) for the servo, and may */
/* need to be adjusted depending on the servo that you are using.  Assuming a   */
/* 50Hz base clock (20000us), the current min and max values are set to 544us   */
/* and 2400uS. */
#define MS_SERVOS_MINTICKS_DIV (36765)               /* Min ticks divider = 20000us/544us (50Hz base clock) = 36.765 */
#define MS_SERVOS_MAXTICKS_DIV (8333)                /* Max ticks divider = 20000us/2400us (50Hz base clock) = 8.333 */

static bool          m_ms_initialized = false;       /* Whether msInit has been called or not */
static msDCMotor_t   m_ms_dcmotor[MS_MOTORCOUNT_DC]={0}; /* DC motor array */

/**************************************************************************/
/*!
    @brief  Initialises the motor shield, setting pins up as appropriate

    @param  addr
            The 7-bit I2C address for the PCA9685 PWM driver

    @return true if the motor shield was properly initialised, otherwise
            false (the most likely cause is an incorrect I2C address or
            bad solder joints)
*/
/**************************************************************************/
bool motorShieldInit(pca9685_handle_t *handle)
{
  memset(&m_ms_dcmotor, 0, sizeof(msDCMotor_t) * MS_MOTORCOUNT_DC);

  /* Initialises the PCA9685 (uses I2C1) */
  if (!pca9685_init(handle)) return false;

  m_ms_initialized = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Initialises the specified DC motor (1..4 for DC motors)

    @param  motorNum
            The DC motor to initialise (1..4)
*/
/**************************************************************************/
void motorShieldDCInit(uint8_t motorNum)
{
  uint8_t pwm, in1, in2;
  pwm = in1 = in2 = 0;

  if ((motorNum > MS_MOTORCOUNT_DC) || (motorNum == 0) || (!m_ms_initialized))
  {
    return;
  }

  /* Set the appropriate PWM channels for the selected motor */
  /* These values assume the pinout for the REV 2.0 Motor Shield */
  /* from Adafruit Industries: */
  /* https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/resources */
  switch (motorNum)
  {
  case 1:
    pwm = 8;
    in2 = 9;
    in1 = 10;
    break;
  case 2:
    pwm = 13;
    in2 = 12;
    in1 = 11;
    break;
  case 3:
    pwm = 2;
    in2 = 3;
    in1 = 4;
    break;
  case 4:
    pwm = 7;
    in2 = 6;
    in1 = 5;
    break;
  }

  m_ms_dcmotor[motorNum-1].pwm = pwm;
  m_ms_dcmotor[motorNum-1].in1 = in1;
  m_ms_dcmotor[motorNum-1].in2 = in2;
  m_ms_dcmotor[motorNum-1].dir = MS_DIR_RELEASE;
  m_ms_dcmotor[motorNum-1].initialized = true;
}

/**************************************************************************/
/*!
    @brief  Updates the speed and direction of the specified DC motor

    @param  motorNum
            The DC motor to update (1..4)
    @param  direction
            The direction that the motor should move (see msDirection_t)
    @param  speed
            The speed that the motor should rotate, which is an 8-bit
            value (0..255)
*/
/**************************************************************************/
//void motorShieldDCRun(pca9685_handle_t *handle, uint8_t motorNum, msDirection_t direction, uint8_t speed)
//{
//  if ((motorNum > MS_MOTORCOUNT_DC) || (motorNum == 0) || (!m_ms_initialized))
//  {
//    return;
//  }
//
//  /* Simulate GPIO using PWM outputs */
//  switch (direction)
//  {
//  case MS_DIR_FORWARD:
//    /* Set the LOW pin first to avoid 'break' */
//	  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in2, 0, 0);
//	  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in1, 4096, 0);
//    break;
//  case MS_DIR_BACKWARD:
//    /* Set the LOW pin first to avoid 'break' */
//	  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in1, 0, 0);
//	  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in2, 4096, 0);
//    break;
//  case MS_DIR_RELEASE:
//	  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in1, 0, 0);
//	  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in2, 0, 0);
//    break;
//  }
//
//  /* Set the PWM output (speed) */
//  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].pwm,
//                speed == 255 ? 4096 : 0,
//                speed == 255 ? 0    : speed*16);
//}

/**************************************************************************/
/*!
    @brief  Control the DC Motor direction and action
    @param  cmd The action to perform, can be FORWARD, BACKWARD or RELEASE
*/
/**************************************************************************/
void motorShieldDCRun(pca9685_handle_t *handle, uint8_t motorNum, msDirection_t direction) {
	  if ((motorNum > MS_MOTORCOUNT_DC) || (motorNum == 0) || (!m_ms_initialized))
	  {
	    return;
	  }

	  /* Simulate GPIO using PWM outputs */
	  switch (direction)
	  {
	  case MS_DIR_FORWARD:
	    /* Set the LOW pin first to avoid 'break' */
		  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in2, 0, 0);
		  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in1, 4096, 0);
	    break;
	  case MS_DIR_BACKWARD:
	    /* Set the LOW pin first to avoid 'break' */
		  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in1, 0, 0);
		  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in2, 4096, 0);
	    break;
	  case MS_DIR_RELEASE:
		  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in1, 0, 0);
		  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].in2, 0, 0);
	    break;
	  }
}

/**************************************************************************/
/*!
    @brief  Control the DC Motor speed/throttle
    @param  speed The 8-bit PWM value, 0 is off, 255 is on
*/
/**************************************************************************/
void motorShieldDCSetSpeed(pca9685_handle_t *handle, uint8_t motorNum, uint8_t speed){
	  if ((motorNum > MS_MOTORCOUNT_DC) || (motorNum == 0) || (!m_ms_initialized))
	  {
	    return;
	  }
	  /* Set the PWM output (speed) */
	  pca9685_set_channel_pwm_times(handle, m_ms_dcmotor[motorNum-1].pwm,
	                speed == 255 ? 4096 : 0,
	                speed == 255 ? 0    : speed*16);
}

