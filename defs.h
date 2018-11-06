/*
 * Define registers and pins
 * At90can32
 * ahrs board
 */
#ifndef DEFS_H_
#define DEFS_H_

/*-----------------------------------------------------------------------*/
/*
 * Hardware and software revisions
 */
#define HARDWARE_REVISION 4
#define SOFTWARE_REVISION 1

/*-----------------------------------------------------------------------*/
/* functions available */

/* in this hardware, we are using the second serial port */
#define UART1                           1

/* define the size of the fifo buffers for the uart */
#define TX_FIFO_BUFFER_SIZE             128
#define RX_FIFO_BUFFER_SIZE             64

/* define baud rate for serial comm */
#define BAUD                            115200

/*-----------------------------------------------------------------------*/

/* DEBUGGING */
       
#define AT90CANDEBUG (1)
#define CANDEBUG (1)
#define CANAERODEBUG (1)
//#define ADXL345DEBUG (1)
//#define BMP085DEBUG (1)
//#define L3G4200DDEBUG (1)

/* GYRO Hack */
#define L3G4200DSLEEP (1)

/*-----------------------------------------------------------------------*/

/* LEDS */

#define DDR_LED1    DDRA
#define PORT_LED1   PORTA
#define P_LED1      3
       
#define DDR_LED2    DDRA
#define PORT_LED2   PORTA
#define P_LED2      4

/*-----------------------------------------------------------------------*/

/* CAN Controller */
#define AT90CANFILTER (1)
#define AT90CAN_8MHZ (1)

#define AT90CAN_RECVBUFLEN 20
#define AT90CAN_ERRBUFLEN 30

/* I2C pins */
#define PORT_SDA    PORTD
#define P_SDA       1
#define PORT_SCL    PORTD
#define P_SCL       0

/* SPI pins */
#define DDR_SPI     DDRB
#define PORT_SPI    PORTB
#define P_MOSI      2
#define P_MISO      3
#define P_SCK       1

/*-----------------------------------------------------------------------*/

/* BMP pins */
#define DDR_XCLR1     DDRG
#define PORT_XCLR1    PORTG
#define P_XCLR1       1

#define DDR_EOC1      DDRE
#define PORT_EOC1     PORTE
#define PIN_EOC1      PINE
#define P_EOC1        4

#define DDR_XCLR2     DDRG
#define PORT_XCLR2    PORTG
#define P_XCLR2       0

#define DDR_EOC2      DDRE
#define PORT_EOC2     PORTE
#define PIN_EOC2      PINE
#define P_EOC2        5

/* Define software interrupt pin for bmp085 device 1 */
// #define BMP_SW_EOC1_INT  PCINT18
// #define BMP_SW_EOC1_MASK_REG PCMSK2
// #define BMP_SW_EOC1_ENABLE PCIE2
// #define BMP_SW_EOC1_REG PCICR

/* Define software interrupt pin for bmp085 device 2 */
// #define BMP_SW_EOC1_INT  PCINT0
// #define BMP_SW_EOC1_MASK_REG PCMSK0
// #define BMP_SW_EOC1_ENABLE PCIE0
// #define BMP_SW_EOC1_REG PCICR

/*-----------------------------------------------------------------------*/
/* Gyro Pins */

#define DDR_GYRODRDY   DDRE
#define PORT_GYRODRDY  PORTE
#define PIN_GYRODRDY   PINE
#define P_GYRODRDY     6

#define DDR_GYROINT    DDRE
#define PORT_GYROINT   PORTE
#define PIN_GYROINT    PINE
#define P_GYROINT      7

/*-----------------------------------------------------------------------*/
/* I2C addresses */
#define ADXL345_ADDRESS (0x53 << 1)

/*-----------------------------------------------------------------------*/
#endif  // DEFS_H_
