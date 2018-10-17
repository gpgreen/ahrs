#include <stdio.h>
#include <avr/pgmspace.h>
#include "defs.h"
#include "globals.h"
#include "adxl345.h"
#include "l3g4200d.h"
#include "canaeromsg.h"
#include "canaero_nis.h"
#include "canaero_ids.h"
#include "canaero_bss.h"
#include "canaero_filters.h"
#include "watchdog.h"
#include "conversion.h"

/*-----------------------------------------------------------------------*/

// message data functions

static void get_body_long_accel(can_msg_t *msg)
{
	convert_float_to_big_endian(adxl345_accel(0), &(msg->data[4]));
}

static void get_body_lat_accel(can_msg_t *msg)
{
	convert_float_to_big_endian(adxl345_accel(1), &(msg->data[4]));
}

static void get_body_norm_accel(can_msg_t *msg)
{
	convert_float_to_big_endian(adxl345_accel(2), &(msg->data[4]));
}

static void get_body_pitch_rate(can_msg_t *msg)
{
	convert_float_to_big_endian(l3g4200d_raw_data(&g_gyro_dev, 0), &(msg->data[4]));
}

static void get_body_roll_rate(can_msg_t *msg)
{
	convert_float_to_big_endian(l3g4200d_raw_data(&g_gyro_dev, 1), &(msg->data[4]));
}

static void get_body_yaw_rate(can_msg_t *msg)
{
	convert_float_to_big_endian(l3g4200d_raw_data(&g_gyro_dev, 2), &(msg->data[4]));
}

static void get_static_pressure(can_msg_t *msg)
{
	convert_float_to_big_endian(g_bmp085_data[0].press, &(msg->data[4]));
}

static void get_total_pressure(can_msg_t *msg)
{
	convert_float_to_big_endian(g_bmp085_data[1].press, &(msg->data[4]));
}

static void get_cycle_time(can_msg_t *msg)
{
	convert_ushort_to_big_endian(g_cycle_time, &(msg->data[4]));
}

/*-----------------------------------------------------------------------*/

// messages defined for this unit
canaero_msg_tmpl_t nod_msg_templates[] = {
	/* the following are raw data messages for calibration,etc */
	/* ------------------------------------------------------- */
	/* cycle time */
	{NOD, 0x100, 0, USHORT, 0, 0, get_cycle_time},
	/* Body Longitudinal Acceleration */
	{NOD, 0x101, 0, FLOAT, 0, 0, get_body_long_accel},
	/* Body Lateral Acceleration */
	{NOD, 0x102, 0, FLOAT, 0, 0, get_body_lat_accel},
	/* Body Normal Acceleration */
	{NOD, 0x103, 0, FLOAT, 0, 0, get_body_norm_accel},
	/* Body pitch rate */
	{NOD, 0x104, 0, FLOAT, 0, 0, get_body_pitch_rate},
	/* Body roll rate */
	{NOD, 0x105, 0, FLOAT, 0, 0, get_body_roll_rate},
	/* Body yaw rate */
	{NOD, 0x106, 0, FLOAT, 0, 0, get_body_yaw_rate},
	/* Static pressure */
	{NOD, 0x108, 0, FLOAT, 0, 0, get_static_pressure},
	/* Total pressure */
	{NOD, 0x10A, 0, FLOAT, 0, 0, get_total_pressure},
};

int num_nod_templates = sizeof(nod_msg_templates)
	/ sizeof(canaero_msg_tmpl_t);

/*-----------------------------------------------------------------------*/

// service message data functions

static void get_mis0_data(can_msg_t* msg)
{
	// first byte is 1 if in LISTEN state
	msg->data[4] = (g_state == AHRSLISTEN) ? 1 : 0;
	// second byte is 1 if filtering is on
	msg->data[5] = g_ci.can_settings.filters.filtering_on;
}

static void get_mis1_data(can_msg_t* msg)
{
	msg->data[4] = 'A';
	msg->data[5] = 'H';
	msg->data[6] = 'R';
	msg->data[7] = 'S';
}

static void get_mis10_data(can_msg_t* msg)
{
	msg->data[4] = g_accelerometer_enabled;
	msg->data[5] = g_gyros_enabled;
	msg->data[6] = g_static_air_enabled;
	msg->data[7] = g_dynamic_air_enabled;
}

/*-----------------------------------------------------------------------*/

// MIS service reply
// get module configuration
uint8_t reply_mis(service_msg_id_t* svc, can_msg_t* msg)
{
	/* Module Information Service Request code 0 */
	static canaero_svc_msg_tmpl_t t0 = {UCHAR2, 12, 0, get_mis0_data};
	
	/* Module Information Service Request code 1 */
	static canaero_svc_msg_tmpl_t t1 = {UCHAR4, 12, 1, get_mis1_data};
	
	/* Module Information Service Request code 2 */
	static canaero_svc_msg_tmpl_t t2 = {USHORT2, 12, 2, watchdog_mis2_data};
	
	/* Module Information Service Request code 3 */
	static canaero_svc_msg_tmpl_t t3 = {USHORT2, 12, 3, watchdog_mis3_data};
	
	/* Module Information Service Request code 10 */
	static canaero_svc_msg_tmpl_t t10 = {UCHAR4, 12, 10, get_mis10_data};
	
	/* Module Information Service Request invalid code */
	static canaero_svc_msg_tmpl_t invalid = {NODATA, 12, 255, 0};

	// ensure the data format is as we expect
	if (msg->data[1] != NODATA)
		return canaero_send_svc_reply_message(svc, &invalid);
		
	uint8_t snd_stat;

	// the message code gives the type of module configuration requested
	switch (msg->data[3]) {
	case 0:
		snd_stat = canaero_send_svc_reply_message(svc, &t0);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MIS code 0"));
#endif
		break;
	case 1:
		snd_stat = canaero_send_svc_reply_message(svc, &t1);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MIS code 1"));
#endif
		break;
	case 2:
		snd_stat = canaero_send_svc_reply_message(svc, &t2);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MIS code 2"));
#endif
		break;
	case 3:
		snd_stat = canaero_send_svc_reply_message(svc, &t3);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MIS code 3"));
#endif
		break;
	case 10:
		snd_stat = canaero_send_svc_reply_message(svc, &t10);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MIS code 10"));
#endif
		break;
	default:
		// we don't respond to these message codes
		snd_stat = canaero_send_svc_reply_message(svc, &invalid);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MIS code unknown"));
#endif
	}
	
	return snd_stat;
}

/*-----------------------------------------------------------------------*/

// MCS service reply
// set module configuration
uint8_t reply_mcs(service_msg_id_t* svc, can_msg_t* msg)
{
	/* Module Configuration Service Request code 0 */
	static canaero_svc_msg_tmpl_t t0 = {UCHAR2, 13, 0, get_mis0_data};
	
	/* Module Configuration Service Request code 1 */
	static canaero_svc_msg_tmpl_t t1 = {NODATA, 13, 1, 0};
	
	/* Module Configuration Service Request code 10 */
	static canaero_svc_msg_tmpl_t t10 = {UCHAR4, 13, 10, get_mis10_data};
	
	/* Module Configuration Service Request invalid code */
	static canaero_svc_msg_tmpl_t invalid = {NODATA, 13, 255, 0};
		
	uint8_t snd_stat;

	// the message code gives the type of module configuration requested
	switch (msg->data[3]) {
	case 0:
		// ensure the data format is as we expect
		if (msg->data[1] != UCHAR2)
			return canaero_send_svc_reply_message(svc, &invalid);
	
		// set listen/active state
		if (msg->data[4])
			g_state = AHRSLISTEN;
		else
			g_state = AHRSACTIVE;
		// second byte is filtering on/off
		uint8_t reset = 0;
		if (g_ci.can_settings.filters.filtering_on != msg->data[5])
		{
			// setup the filters as setting is changed
			if (msg->data[5])
				canaero_high_priority_service_filters(&g_ci);
			else
				canaero_no_filters(&g_ci);
			reset = 1;
		}

		snd_stat = canaero_send_svc_reply_message(svc, &t0);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MCS code 0"));
#endif
		if (reset) {
			// reinitialize the canaero stack
			errcode = canaero_init(&g_ci, g_can_device);
			if (errcode == CAN_FAILINIT)
				offline();
		}
		break;
	case 1:
		// ensure the data format is as we expect
		if (msg->data[1] != UCHAR2)
			return canaero_send_svc_reply_message(svc, &invalid);

		// reset the message counters, and clear tx buffers, if requested
		if (msg->data[5])
			can_clear_tx_buffers();
		if (msg->data[4])
			canaero_reset_nod_message_sequence();

		snd_stat = canaero_send_svc_reply_message(svc, &t1);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MCS code 1"));
#endif
		break;
	case 10:
		// ensure the data format is as we expect
		if (msg->data[1] != UCHAR4)
			return canaero_send_svc_reply_message(svc, &invalid);
	
		// read and set the configuration variables
		g_accelerometer_enabled = msg->data[4];
		g_gyros_enabled = msg->data[5];
		g_static_air_enabled = msg->data[6];
		g_dynamic_air_enabled = msg->data[7];
		
		snd_stat = canaero_send_svc_reply_message(svc, &t10);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MCS code 10"));
#endif
		break;
	default:
		// we don't respond to these message codes
		snd_stat = canaero_send_svc_reply_message(svc, &invalid);
#ifdef CANAERODEBUG
		puts_P(PSTR("replied to MCS code unknown"));
#endif
	}
	
	return snd_stat;
}

/*-----------------------------------------------------------------------*/

reply_svc_fn* nsl_dispatcher_fn_array[] = {
	// 0 - IDS
	&canaero_reply_ids,
	// 1 - NSS 
	0,
	// 2 - DDS
	0,
	// 3 - DUS
	0,
	// 4 - SCS
	0,
	// 5 - TIS
	0,
	// 6 - FPS
	0,
	// 7 - STS
	0,
	// 8 - FSS
	0,
	// 9 - TCS
	0,
	// 10 - BSS
	&canaero_reply_bss,
	// 11 - NIS
	&canaero_reply_nis,
	// 12 - MIS
	&reply_mis,
	// 13 - MCS
	&reply_mcs,
	// 14 - CSS
	0,
	// 15 - DSS
	0,
};
	
