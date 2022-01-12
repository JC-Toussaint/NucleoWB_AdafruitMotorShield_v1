/*
 * File:   main.c Test program
 * Author: DevXP
 *
 * GNU GENERAL PUBLIC LICENSE!
 * Created on April 2, 2012, 8:26 AM
 */

#include <main.h>
#include <pca9685.h>

void main()
{
    pca9685_init(LEDDRV1);
    pca9685_brightness(LEDDRV1,22,0);
    long pwm = 0;
    while(true)
    {
        output_toggle(PIN_D5);
        delay_ms(5);
        if(pwm < 4095)
        {
            pwm+=5;
        }
        else
        {
            pwm = 0;
        }
        pca9685_send(LEDDRV1,pwm,1);
        //PCA9685AllLedOff(LEDDRV1);
    }
}

