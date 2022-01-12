/*
 * File:   pca9685.h
 * Author: DevXP
 *
 * GNU GENERAL PUBLIC LICENSE!
 * Created on April 2, 2012, 8:28 AM
 */

#use I2C(master, slow, i2c1, force_hw)
#include <pca9685_reg.h>

void pca9685_init(int address)
{
/********************************************/
/* Init code for the PCA9685                */
/* Input:[address of the pc9685 to init]    */
/* Output:[void]                            */
/********************************************/
    i2c_start();                // Start
    i2c_write(address);         // Slave address
    i2c_write(MODE1);           // Mode 1 address
    i2c_write(0b00110001);      // Setting mode to sleep so we can change the default PWM frequency
    i2c_stop();                 // Stop
    delay_ms(1);                // Required 50 us delay
    i2c_start();                // Start
    i2c_write(address);         // Slave address
    i2c_write(0xfe);            // PWM frequency PRE_SCALE address
    i2c_write(0x04);            // osc_clk/(4096*update_rate) // 25000000/(4096*1500)= 4.069 ~4
    i2c_stop();                 // Stop
    delay_ms(1);                // delay at least 500 us
    i2c_start();                // Start
    i2c_write(address);         // Slave address
    i2c_write(MODE1);           // Mode 1 register address
    i2c_write(0xa1);            // Set to our preferred mode[ Reset, INT_CLK, Auto-Increment, Normal Mode]
    i2c_stop();                 // Stop
    delay_ms(1);                // delay at least 500 us
    i2c_start();                // Start
    i2c_write(address);         // Slave Address
    i2c_write(MODE2);           // Mode2 register address
    i2c_write(0b00000100);      // Set to our preferred mode[Output logic state not inverted, Outputs change on STOP,
    i2c_stop();                 // totem pole structure, When OE = 1 (output drivers not enabled), LEDn = 0]
}

void pca9685_send_all(int address)
{
/********************************************/
/* Update all PWM register of PCA9685       */
/* Input:[address of the pc9685 to update]  */
/* Output:[void]                            */
/********************************************/
    int i = 0;                  // temp register for LEDCOUNT
    int pwm;                    // temp register for PWM
    for(i=0; i<=LEDCOUNT; i++)  // cycle thru all 16 LED
    {
        i2c_start();            // Start
        i2c_write(address);     // write to selected pca9685
        i2c_write(LED0 + 4 * i);// start from LED0 address, each pwm constructed from
        i2c_write(0x00);        // 4 12bit register, LED_ON_L
        i2c_write(0x00);        // LED_ON_H
        pwm = PWMData[i];       // update selected LED data in the array
        i2c_write(pwm);         // LED_OFF_L
        pwm = PWMData[i]>>8;    // update selected LED data in the array
        i2c_write(pwm);         // LED_OFF_H
        i2c_stop();             // Stop
    }
}

void pca9685_send(int address, long value, int led)
{
/********************************************/
/* pca9685_send(long value, int led)        */
/* Send the 12 bit PWM data to the register */
/* Input[ 0to4095 pwm, 0to15LED channel]    */
/* Output[void]                             */
/********************************************/
    int pwm;                    // temp variable for PWM
    if(value > 4095)            // if larger than 4095 than full on
            value = 4095;       // cant be larger than 4095
    if(led > 15)                // if LED larger than 15 than on other chip
            led = 15;           //***** need to implement to select next pcs9685
    i2c_start();                // Start
    i2c_write(address);         // Address of selected pca9685
    i2c_write(LED0 + 4 * led);  // select selected LED address
    i2c_write(0x00);            // LED_ON_L
    i2c_write(0x00);            // LED_ON_H
    pwm = value;                // PWM value lo byte
    i2c_write(pwm);             // LED_OFF_L
    pwm = value>>8;             // pwm 16 bit long, now shift upper 8 to lower 8
    i2c_write(pwm);             // LED_OFF_H
    i2c_stop();                 // Stop
}


void pca9685_brightness(int address, int percent, int led)
{
/********************************************/
/* Calculate the register values for a      */
/* given percentage and update pca9685     */
/* Input:[address of the chip where LED is_ */
/* percent of PWM on period 0%to100%      _ */
/* LED to set brightness 0to15]             */
/* Output:[void]                            */
/********************************************/
    long x;                             // temp variable
    float off;                          // temp variable
    const float onePercent = 40.96;
    if (percent < 1) {                  // if % less than 1 than LED OFF
    PWMData[led] = PCA9685_LED_OFF>>8;  // update data in array in case we use update all LED next
    pca9685_send(address,0,led);        // update selected LED
    return;                             // return from function
    }
    if (percent >= 100) {               // if % greater than 100 than LED ON
    PWMData[led] = PCA9685_LED_ON>>8;   // update data in array
    pca9685_send(address,4095,led);     // update selected LED
    return;                             // return from function
    }
    off = onePercent * percent;         // different approach with float need to check if code faster than int32 way ?
//    off = (int32)4096 * percent;        // calculate percent (max*percent)/100
//    off = off / 100;                    // ex (4096*50%)=204800/100=2048
//    x = make16(off>>8,off);             // make 16 of 32 ?! why.. dont care at this time
    PWMData[led] = off;                 // update data array in case we update all LED next
    pca9685_send(address,off,led);      // send it to pca9685
}
void PCA9685AllLedOff(int address)
{
    i2c_start();                        // Start
    i2c_write(address);                 // select pca9685
    i2c_write(0xfc);         // AllLED Off register
    i2c_write(0b00000000);              // data
    i2c_write(0b00010000);              // data
    i2c_stop();                         // Stop
} 
