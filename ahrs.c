/*
 * note that the device specific header files are located
 * at: /usr/lib/avr/include/avr/
 *
 * Fuse bits for At90can32
 * External Crystal Oscillator 258CK, start up + 65ms
 * clock prescaler divide by 8
 * serial programming enabled
 * brown-out at 4.0V
 * Low=0x5e Hi=0xd9 Ext=0xfb
 * avrdude settings:
 * -U lfuse:w:0x5e:m -U hfuse:w:0xd9:m
 * -U efuse:w:0xfb:m
 * from http://www.engbedded.com/fusecalc/
 *
 * X is facing forward, or the dsub connector
 * Y is to right wing, or right of dsub connector
 * Z is down
 */

#include "defs.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include "uart.h"
#include "i2cmaster.h"
#include "gpio.h"
#include "timer.h"
#include "spi.h"
#include "bmp085.h"
#include "adxl345.h"
#include "l3g4200d.h"
#include "canaero.h"
#include "canaeromsg.h"
#include "canaero_filters.h"
#include "at90can.h"
#include "globals.h"
#include "watchdog.h"

/*-----------------------------------------------------------------------*/

// Turn on/off some device code for testing, define to 0 if not using...
// ----------------
//#define USE_ACCEL 1
//#define USE_GYRO 1
// ----------------

// set serial port to stdio
static FILE uart_ostr = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE uart_istr = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

// timer flags

// 20 hz timer flag
volatile uint8_t g_timer20_set;
static uint8_t g_timer20_count;
const uint8_t k_timer20_compare_count = 4;

// 80 hz timer flag
volatile uint8_t g_timer80_set;

// CAN controller flags
volatile uint8_t g_can_int_set;	/* set when CAN controller interrupt signaled */

// global error code
volatile uint8_t errcode;

// the can stack initialization struct
canaero_init_t CAN_config;

// the state
enum ahrs_state g_state;

// equipment enabled flags
int g_accelerometer_enabled;
int g_gyros_enabled;
int g_static_air_enabled;
int g_dynamic_air_enabled;

// the cycle time (approx 80hz) in tenth milliseconds
uint32_t g_cycle_time;

// adxl345 device
struct adxl345_device g_adxl345_dev = {
	.axis_map = {0,1,2},
	.sign_map = {-1,1,1},
};

// bmp085 device structures
struct bmp085_dev_t g_bmp085_data[2];

// l3g4200d device structure
l3g4200d_dev_t g_gyro_dev = {
	.sensor_sign = {1, 1, 1},
};

/*-----------------------------------------------------------------------*/

// control the leds

void led1_on(void)
{
	PORT_LED1 |= _BV(P_LED1);
}
void led1_off(void)
{
	PORT_LED1 &= ~_BV(P_LED1);
}

void led2_on(void)
{
	PORT_LED2 |= _BV(P_LED2);
}
void led2_off(void)
{
	PORT_LED2 &= ~_BV(P_LED2);
}

/*-----------------------------------------------------------------------*/

// select the bmp085 device for reset
inline static void xclr_low(int device)
{
    if (device == 0)
        PORT_XCLR1 &= ~(_BV(P_XCLR1));
    else
        PORT_XCLR2 &= ~(_BV(P_XCLR2));
}

/*-----------------------------------------------------------------------*/

// select the bmp085 device for I2C
inline static void xclr_hi(int device)
{
    if (device == 0)
        PORT_XCLR1 |= _BV(P_XCLR1);
    else
        PORT_XCLR2 |= _BV(P_XCLR2);
}

/*-----------------------------------------------------------------------*/

// this function is called if CAN doesn't work, otherwise
// use the failed fn below
// blinks continuously at 20 hz to show offline
void
offline(void)
{
	uint8_t on = 1;
	led1_on();
	while (1) {
		// 10 hz timer
		if (g_timer20_set)
		{
			g_timer20_set = 0;
			if (on)
				led1_off();
			else
				led1_on();
			on ^= 1;
		}
	}
}

/*-----------------------------------------------------------------------*/

// main entry for showing board failure
// blinks the errorcode then a longer pause
void
failed(uint8_t err)
{
	errcode = err;
	uint8_t count = 0;
	uint8_t pause = 0;
	led1_off();
	led2_on();
	while (1) {
		// 20 hz timer
		if (g_timer20_set)
		{
			g_timer20_set = 0;
			if (pause) {
				--pause;
			} else {
				if (bit_is_set(count, 0))
					led2_off();
				else
					led2_on();
				if (++count == errcode * 2) {
					pause = 8;
					count = 0;
				}
			}
		}
	}
}

/*-----------------------------------------------------------------------*/

static void can_error(struct can_device* dev, const can_error_t* err)
{
	printf_P(PSTR("*** can error:%d %d %d\n"), err->error_code, err->dev_buffer,
			 err->dev_code);
	if (err->error_code == CAN_BUS_OFF)
		g_state = AHRSLISTEN;
	else if (err->error_code == CAN_BUS_PASSIVE)
		g_state = AHRSLISTEN;
}

/*-----------------------------------------------------------------------*/

static void canaero_emergency_event(const struct emerg_event* ee)
{
	if (ee->error_code == DISPLAY_BUFFER_OVERFLOW) {
		g_state = AHRSLISTEN;
		can_clear_tx_buffers(&at90can_dev);
		puts_P(PSTR("EE:switching to listen mode"));
	}
	printf_P(PSTR("EE(%u):%d %d %d\n"), ee->node, ee->error_code,
			 ee->operation_id,
			 ee->location_id);
}

/*-----------------------------------------------------------------------*/

void
system_start(void)
{
    // first set the clock prescaler change enable
	CLKPR = _BV(CLKPCE);
	// now set the clock prescaler to clk / 2
	CLKPR = _BV(CLKPS0);

	watchdog_init(WDTO_1S);
}

/*-----------------------------------------------------------------------*/

void
ioinit(void)
{
	gpio_setup();

	// light up led while ioinit executes
	led1_on();
	
	watchdog_reset_count_update();
	
	timer_init();
	
    // setup the 80 hz timer
    // CTC mode, clk at F_CPU/8
	TCCR1A = _BV(WGM12);
	TCCR1B = _BV(CS11);
    // set interrupt so that match = 80hz
	OCR1A = (F_CPU / 80 / 8);
    // set OC interrupt 1A
    TIMSK1 |= _BV(OCIE1A);

	led2_on();
	
	// setup the serial hardware
	uart_init();
	
	puts_P(PSTR("AHRS"));
	printf_P(PSTR("Hardware: %d Software: %d\n-------------------------\n"),
			 HARDWARE_REVISION, SOFTWARE_REVISION);
	led2_off();
	
	// spi needs to be setup first
	if (spi_init(4) == SPI_FAILED)
		offline();
	
	puts_P(PSTR("spi initialized."));
	
	// i2c hardware setup
	i2c_init();
	puts_P(PSTR("i2c initialized."));
	
	// setup the can initialization struct
	CAN_config.can_settings.speed_setting = CAN_250KBPS;
	CAN_config.can_settings.loopback_on = 0;
	CAN_config.can_settings.tx_wait_ms = 20;
	CAN_config.node_id = 2;
	CAN_config.svc_channel = 0;
	CAN_config.nod_msg_templates = nod_msg_templates;
	CAN_config.num_nod_templates = num_nod_templates;
	CAN_config.nsl_dispatcher_fn_array = nsl_dispatcher_fn_array;
	CAN_config.incoming_msg_dispatcher_fn = 0;
	CAN_config.emergency_event_fn = canaero_emergency_event;

	// set the error function
	at90can_dev.handle_error_fn = can_error;
	
	canaero_high_priority_service_filters(&CAN_config);
	
	// now do the CAN stack
	errcode = canaero_init(&CAN_config, &at90can_dev);
	if (errcode == CAN_FAILINIT)
		offline();
	puts_P(PSTR("canaero initialized."));
	
	// self test the CAN stack
	errcode = canaero_self_test(&CAN_config);
	if (errcode != CAN_OK)
		offline();
	puts_P(PSTR("canaero self-test complete."));

	watchdog_reset();
	
    // setup the static, dynamic pressure devices
	for (int i=0; i<2; ++i) {
		switch (i) {
		case 0:
			xclr_hi(0);
			xclr_low(1);
			break;
		case 1:
			xclr_low(0);
			xclr_hi(1);
			break;
		}
		
		struct bmp085_dev_t* p = &g_bmp085_data[i];
		p->num = i;
		
		if (bmp085_init(STANDARD, p) != BMP085_OK)
			failed(1);
		puts_P(PSTR("bmp085 initialized."));
	
		// do the self tests
		if (bmp085_self_test(p) != BMP085_OK)
			failed(1);
		puts_P(PSTR("bmp085 self-test complete."));
	}
	xclr_hi(0);
	
	g_static_air_enabled = 1;
	g_dynamic_air_enabled = 1;

#ifdef USE_ACCEL
    // setup the accelerometer
	adxl345_init(&g_adxl345_dev);
	g_accelerometer_enabled = 1;
	
	puts_P(PSTR("adxl345 initialized."));
	
    // run the self test
    if (adxl345_self_test())
		failed(2);
	adxl345_internal_self_test();
	puts_P(PSTR("adxl345 self-test complete."));

#else
	g_accelerometer_enabled = 0;
#endif

#ifdef USE_GYRO
	// gyros
	l3g4200d_init();
	puts_P(PSTR("gyro initialized."));
	l3g4200d_self_test();
	puts_P(PSTR("gyro self-test complete."));
	g_gyros_enabled = 1;
#else
	g_gyros_enabled = 0;
#endif

	watchdog_print_flags();
	
	// led off when ioinit done
	led1_off();
	
	puts_P(PSTR("ioinit complete."));

}

/*-----------------------------------------------------------------------*/

int
main(void)
{
	int bmp_device = 0;

	g_state = AHRSINIT;

	system_start();

	// stdout is the uart
	stdout = &uart_ostr;
	// stdin is the uart
	stdin = &uart_istr;
	
    ioinit();
	sei();

	g_state = AHRSLISTEN;

	// keep track of the last time
	uint32_t lt = jiffie();
	
    while(1)
    {
		watchdog_reset();
		
		// write the eeprom if necessary
		//canaero_write_eeprom_task();

		// check for can interrupt
		if(canaero_handle_interrupt(&CAN_config) == CAN_INTERRUPT)
			canaero_poll_messages(&CAN_config);
		
        // 80 hz timer
        if (g_timer80_set)
        {
            g_timer80_set = 0;
//			puts_P(PSTR("80hz"));
			// find the current time in tenth ms
			uint32_t ct = jiffie();
#ifdef USE_GYRO
			// read the data
			if (g_gyros_enabled)
			 	l3g4200d_read_data(&g_gyro_dev);
#endif
#ifdef USE_ACCEL
			if (g_accelerometer_enabled)
				adxl345_read_accel();
#endif
			// calc the elapsed time in tenth ms
			g_cycle_time = timer_elapsed(lt, ct);
			// reset the last time
			lt = ct;
//			puts_P(PSTR("end 80hz"));
			if(g_state == AHRSACTIVE) {
				canaero_send_messages(&CAN_config, 0, 1);
#ifdef USE_GYRO
				canaero_send_messages(&CAN_config, 4, 7);
#endif
#ifdef USE_ACCEL
				canaero_send_messages(&CAN_config, 1, 4);
#endif
			}
        }

        // 20 hz timer
        if (g_timer20_set)
        {
            g_timer20_set = 0;
//			puts_P(PSTR("20hz"));

			// read the pressure chips
			// reset the chip we aren't reading, so only one
			// responds on i2c bus
			if(bmp_device == 0)
				xclr_low(1);
			if(bmp_device == 1)
				xclr_low(0);
				
			bmp085_read_data(&g_bmp085_data[bmp_device]);

			// make sure both chips are not in reset, so they work
			xclr_hi(0);
			xclr_hi(1);

			// next cycle, read the other chip
			if(++bmp_device == 2)
				bmp_device = 0;

			if(g_state == AHRSACTIVE)
				canaero_send_messages(&CAN_config, 7, 9);
			
//			puts_P(PSTR("end 20hz"));
        }

    }
    return 0;
}

/*-----------------------------------------------------------------------*/

/*
 * Timer compare output 1A interrupt
 */
ISR(TIMER1_COMPA_vect)
{
	g_timer80_set = 1;
    // every 'k_timer20_compare_count' compare match events is an 20 hz tick
	if (++g_timer20_count == k_timer20_compare_count) {
		g_timer20_count = 0;
		g_timer20_set = 1;
    }
}

/*-----------------------------------------------------------------------*/
