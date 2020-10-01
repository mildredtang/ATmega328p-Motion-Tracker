/*
 * mylib.h
 *
 * u5962182, u5853650
 */ 


#ifndef INCFILE1_H_			// this flag is to avoid multiple inclusion of the same header 
#define INCFILE1_H_


#define F_CPU 20000000UL

#include <avr/io.h>

#include <stdio.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>



///////////////////////////////////////////////////////////////////////
//
// UART Functions
//
///////////////////////////////////////////////////////////////////////
#define BAUD_115200		115200UL
#define UBRR_VALUE		(F_CPU/16/BAUD_115200)	// 10 (the data-sheet formula is inaccurate).

#define BUF_SIZE	128
typedef struct{									// define your own data type here.
	unsigned char buf[BUF_SIZE];
	int head;
	int tail;
	int data_size;
	int overrun;
	unsigned char flag;
} ring_buf_t;

void uart_init();								// function names should be declared before use.
void uart_putchar(unsigned char data);
void uart_putstr(char *str);

///////////////////////////////////////////////////////////////////////
//
// Timer1 ISR
//
///////////////////////////////////////////////////////////////////////
#define OCR1_VALUE_1HZ	19531
#define OCR1_VALUE_10HZ	1953

void timer1_init();

///////////////////////////////////////////////////////////////////////
//
// TWI functions 
//
///////////////////////////////////////////////////////////////////////
void twi_init();
int twi_write(uint8_t addr, uint8_t sub_addr, uint8_t ch);
int twi_read(uint8_t addr, uint8_t sub_addr, uint8_t *data, uint8_t size);

///////////////////////////////////////////////////////////////////////
//
// IMU functions
//
///////////////////////////////////////////////////////////////////////
#define WHO_AM_I_M		0x0F
#define MAG_ADDR		0x1E
#define IMU_ADDR		0x6B
#define BARO_ADDR		0x5D

#define CTRL_REG1_G		0x10
#define CTRL_REG6_XL    0x20
#define CTRL_REG7_XL    0x21
#define STATUS_REG		0x17
#define OUT_X_G			0x18
#define	OUT_X_XL		0x28

#define CTRL_REG1_ODRG0 5
#define CTRL_REG1_ODRG1 6
#define CTRL_REG1_ODRG2 7
#define STATUS_GDA0		0
#define STATUS_XLDA1	1

#define PI				3.141592
#define MAG_RESOL		(0.14*0.001)	// gauss
#define ACCL_RESOL		(0.061*0.001)	// g
#define GYRO_RESOL		(8.75*0.001)	// deg

#define STATUS_REG_M    0x27
#define OUT_X_L_M       0x28

#define PRESS_OUT_XL    0x28
#define TEMP_OUT_L      0x2B

#define CTRL_REG1_M     0x20
#define CTRL_REG3_M     0x22
#define CTRL_REG4_M     0x23

#define CTRL_REG1       0x20

typedef struct {
	float x;
	float y;
	float z;
}vec_t;

typedef struct {
	vec_t accel;
	vec_t gyro;
	vec_t mag;
	vec_t ang_accel;
	vec_t ang_gyro;
	float baro;
	float temperature;
	float dt;
}imu_t;

void mag_init();
int imu_init();
int init_all();

// Kalman filter parameters defination
#define T 			0.1
#define Q_accel		0.1
#define Q_gyro		0.005
#define R 			0.05

// Gyro errors
static float error_gx = 0.0912;
static float error_gy = 0.9562;
static float error_gz = -0.2412;

// Mag errors
static float offset_mx = -0.07487890f;
static float offset_my = 0.1953431f;
static float coef = 0.852;

static float xb = 0.054;
static float yb = -0.1708;

typedef struct KalmanData
{
	float p0;
	float p1;
	float p2;
	float p3;

	float accel_error;
	float accelEst;
	float Q_bias; 	// Gyro drift error
} KalmanData;

void Kalman_init(KalmanData *kalman);

float Kalman_Filter(KalmanData *kalman, float ang_gyro, float ang_accel);

#endif /* INCFILE1_H_ */
