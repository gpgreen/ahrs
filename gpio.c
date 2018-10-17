#include <inttypes.h>
#include <avr/io.h>

#include "gpio.h"

void gpio_setup(void)
{
	// setup the led pins, output, level low
	DDR_LED1 |= _BV(P_LED1);
	DDR_LED2 |= _BV(P_LED2);

	// gyro hardware
	// -------------
	
	// DRDY input
	DDR_GYRODRDY &= ~(_BV(P_GYRODRDY));

#if 0
	// INT input
	DDR_GYROINT &= ~(_BV(P_GYROINT));
#endif
	// we are using INT7 as a can s/w interrupt as the gyro
	// isn't working on this board, so we set it as an output
	// to high level
	// set as output
	DDR_GYROINT |= _BV(P_GYROINT);
	// level hi
	PORT_GYROINT |= _BV(P_GYROINT);
	
	// BMP085 hardware
	// ---------------
	
	// XLCRn, output, level hi
	DDR_XCLR1 |= _BV(P_XCLR1);
	PORT_XCLR1 |= _BV(P_XCLR1);
	
	DDR_XCLR2 |= _BV(P_XCLR2);
	PORT_XCLR2 |= _BV(P_XCLR2);
	
	// EOCn, input
	DDR_EOC1 &= ~(_BV(P_EOC1));
	DDR_EOC2 &= ~(_BV(P_EOC2));

}
