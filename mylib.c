/*
 * mylib.c
 *
 * u5962182, u5853650
 */ 

#include "mylib.h"

int flag;


///////////////////////////////////////////////////////////////////////
//
// global variables here
//
///////////////////////////////////////////////////////////////////////
volatile uint8_t timer_flag_10Hz;
imu_t IMU;

// Parameters used in Kalman Filter

// Calibration parameters

///////////////////////////////////////////////////////////////////////
//
// Timer1 functions
//
///////////////////////////////////////////////////////////////////////
void timer1_init()
{
	TCCR1A = (1<<COM0A0);
	TCCR1B = (1<<WGM12)|(1<<CS12)|(0<<CS11)|(1<<CS10);	// 20Mhz divided by 1024
	OCR1AH = OCR1_VALUE_10HZ>>8;	// Write high byte first
	OCR1AL = OCR1_VALUE_10HZ;
	TIMSK1 = (1<<OCIE0A);
	
	timer_flag_10Hz = 0;
}

ISR(TIMER1_COMPA_vect)
{
	timer_flag_10Hz = 1;
}


///////////////////////////////////////////////////////////////////////
//
// UART Functions
//
///////////////////////////////////////////////////////////////////////
void uart_init()
{
	UBRR0H = (unsigned char)(UBRR_VALUE>>8);	// Baud Rate High
	UBRR0L = (unsigned char)UBRR_VALUE;			// Baud Rate Low
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);// 1 stop bit, 8 data, No parity (default)
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);// Enable receiver and transmitter
}

void uart_putchar(unsigned char data)
{
	while ( !(UCSR0A & (1<<UDRE0)) );// Wait for transmit buffer empty
	UDR0 = data;					// Put data into buffer, sends the data
}

void uart_putstr(char *str)
{
	while(*str)						// the value at the str-address is not null (0x00)
	uart_putchar(*str++);		// send the value and increase the pointer
}

unsigned char get_char(void)
{
	while ( !( UCSR0A & (1<<RXC0)) );

	return UDR0;
}


///////////////////////////////////////////////////////////////////////
//
// I2C (Inter-Integrated Communication, Philip) Functions
//
///////////////////////////////////////////////////////////////////////
void twi_init()
{
	PORTC = (1<<DDC5)|(1<<DDC4);	// writing 1 in an input mode enables the pull-up.
	
	// Set TWI clock to 100 kHz
	//TWBR = 0x5C;	//92 = 0x5C
	TWBR = ((F_CPU / 100000) - 16) / 2;
	TWDR = 0xFF;                        // Default content = SDA released.
	TWCR = (1<<TWEN)|                   // Enable TWI-interface and release TWI pins.
	(0<<TWIE)|(0<<TWINT)|               // Disable Interupt.
	(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|	// No Signal requests.
	(0<<TWWC);
}

int twi_write(uint8_t addr, uint8_t sub_addr, uint8_t ch)
{
	// 1. Start
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_START)
	return -1;
	
	// 2. Send SLA+W (Write Mode)
	TWDR = (addr << 1) | (TW_WRITE);	// SLA+W
	TWCR = (1<<TWINT)|(1<<TWEN);		// Start transmission
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_MT_SLA_ACK)
	return -2;
	
	//	3. Send Data #1 (sub-address)
	TWDR = sub_addr;					// Data (Sub-address)
	TWCR = (1<<TWINT)|(1<<TWEN);		// Start transmission
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_MT_DATA_ACK)
	return -3;

	//	4. Send Data #2 (actual data)
	TWDR = ch;							// Data (at the sub-address register)
	TWCR = (1<<TWINT)|(1<<TWEN);		// Start transmission
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_MT_DATA_ACK)
	return -4;

	// 5. Stop condition
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	_delay_ms(1);						// Allow time for stop to send
	
	return 0;
}

int twi_read(uint8_t addr, uint8_t sub_addr, uint8_t *data, uint8_t size)
{
	// 1. Start
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_START)
	return -1;
	
	// 2. Send SLA+W (Write Mode)
	TWDR = (addr << 1) | TW_WRITE;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_MT_SLA_ACK)
	return -2;
	
	// 3. Send Data #1 (sub-address)
	if (size>1)
	sub_addr |= 0x80;

	TWDR = sub_addr;				// Sub address + Auto Increment
	TWCR = (1<<TWINT)|(1<<TWEN);	// Start transmission
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_MT_DATA_ACK)
	return -3;
	
	// 4. We need to change to Read mode so Re-start (repeated)
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while (!(TWCR & 1<<TWINT));
	if ((TWSR & 0xF8) != TW_REP_START)
	return -4;
	
	// 5. Send SLA+R (Read mode)
	TWDR = (addr << 1) | TW_READ;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_MR_SLA_ACK)
	return -5;

	if (size > 1)	// multiple read
	{
		for (int i=0; i<size-1; i++)
		{
			// 6. Data up to #(N-1) (Read a byte). Need to ACK to the slave
			TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWEA); // ACK enabled
			while (!(TWCR & (1<<TWINT)));
			if ((TWSR & 0xF8) != TW_MR_DATA_ACK)
			return -6;
			
			*data++ = TWDR;
		}
	}
	
	// 6. Data #2 or #N (Read a byte). Last byte needs a NACK
	TWCR = (1<<TWINT)|(1<<TWEN); // No TWEA (send NACK to the slave)
	while (!(TWCR & (1<<TWINT)));
	if ((TWSR & 0xF8) != TW_MR_DATA_NACK)
	return -7;

	*data = TWDR; // last byte
	
	//  7. Stop
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	_delay_ms(1);
	
	return 0;
}


///////////////////////////////////////////////////////////////////////
//
// Kalman filter
//
///////////////////////////////////////////////////////////////////////
void Kalman_init(KalmanData *kalman)
{
		kalman->p0 = 0.5;
		kalman->p1 = 0.5;
		kalman->p2 = 0.5;
		kalman->p3 = 0.5;

		kalman->accel_error = 0;
		kalman->accelEst;
}

float Kalman_Filter(KalmanData *kalman, float ang_gyro, float ang_accel)
{	
    // formula 1
    kalman->accelEst += T * (ang_gyro-kalman->Q_bias);
    
    // formula 2
    kalman->p0 += -T * (kalman->p1+kalman->p2)  + Q_accel;
    kalman->p1 += -T * kalman->p3;
    kalman->p2 += -T * kalman->p3;
    kalman->p3 += Q_gyro;
    
    // formula 3 : Kalman gain
    float K0 = kalman->p0/(kalman->p0+R);
    float K1 = kalman->p2/(kalman->p0+R);
    
    // formula 4
    kalman->accel_error = ang_accel - kalman->accelEst;
    kalman->accelEst += K0 * kalman->accel_error; // Calculate the best angle
    kalman->Q_bias += K1 * kalman->accel_error; //Calculate the best value of Q_bias
    //gyroEst = gyro - Q_bias; // Calculate the best value of gyro "angular speed"
    
    // formula 5
    kalman->p0 -= K0 * kalman->p0;
    kalman->p1 -= K0 * kalman->p1;
    kalman->p2 -= K1 * kalman->p0;
    kalman->p3 -= K1 * kalman->p1;
    
    return kalman->accelEst;
}


///////////////////////////////////////////////////////////////////////
//
// Some IMU Registers and Bit-definitions
//		- IMU/Magnetometer LSM9DS1 datasheet
//		- Barometer LPS25HB datasheet
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


///////////////////////////////////////////////////////////////////////
//
// IMU init() - Confirm the IDs and Setup IMU Output rate
//
///////////////////////////////////////////////////////////////////////
int imu_init()
{
	uint8_t temp;
	char str[100];
	
	IMU.ang_gyro.x = 0.0;
	IMU.ang_gyro.y = 0.0;
	IMU.ang_gyro.z = 0.0;
	IMU.dt = 0.1;			// timer is set up 10Hz
	
	// Read IDs
	
	// Read the IMU "Who_Am_I" register (0x0F) to confirm the ID (0x68)
	int i = twi_read(IMU_ADDR, 0x0F, &temp, 1);
	if (temp == 0x68){
		sprintf(str,"ID IMU: Who Am I = [0x%2X]\r\n",temp);
		uart_putstr(str);
	}
	else{
		sprintf(str,"[ERROR] IMU: Who Am I = [0x%2X]\r\n",temp);
		uart_putstr(str);
		return i;
	}

	// Read the Magnetometer "Who_Am_I" register (0x0F) to confirm the ID (0x3D)
	i = twi_read(MAG_ADDR, 0x0F, &temp, 1);
	if (temp == 0x3D){
		sprintf(str,"ID Magnetometer: Who Am I = [0x%2X]\r\n",temp);
		uart_putstr(str);
	}
	else{
		sprintf(str,"[ERROR] MAG: Who Am I = [0x%2X]\r\n", temp);
		uart_putstr(str);
		return i;
	}

	// Read the Barometer "Who_Am_I" register (0x0F) to confirm the ID (0xBD)
	i = twi_read(BARO_ADDR, 0x0F, &temp, 1);
	if (temp == 0xBD){
			sprintf(str,"ID Barometer: Who Am I = [0x%2X]\r\n",temp);
			uart_putstr(str);
	}
	else{
		sprintf(str,"[ERROR] BARO: Who Am I = [0x%2X]\r\n",temp);
		uart_putstr(str);
		return i;
	}


	// Write the IMU control register to setup the output rate
	// Default is zero/no data
	temp = (1<<CTRL_REG1_ODRG2);	// output rate 14.9Hz
	twi_write(IMU_ADDR, CTRL_REG1_G, temp);


	#define CTRL_REG6_XL_ODRXL0 5
	#define CTRL_REG6_XL_ODRXL1 6
	#define CTRL_REG6_XL_ODRXL2 7
	temp = (1<<CTRL_REG6_XL_ODRXL2);
	twi_write(IMU_ADDR, CTRL_REG6_XL, temp);
	
	#define CTRL_REG7_XL_HR 7
	#define CTRL_REG7_XL_DCF1 6
	#define CTRL_REG7_XL_DCF0 5
	
	temp = (1<<CTRL_REG7_XL_HR) | (1<<CTRL_REG7_XL_DCF0); // Activate accel digital filter p53.
	//temp = (1<<CTRL_REG7_XL_HR);
	twi_write(IMU_ADDR, CTRL_REG7_XL, temp);

	// Set up magnetometer registers here
	#define OM1  6
	#define M_CTRL_REG_1_DO2  4
	#define M_CTRL_REG_1_DO1  3
	#define M_CTRL_REG_1_DO0  2
	#define M_CTRL_REG_1_TEMPCOMP 7
	#define OMZ1  3
    
	// Set mag sampling rate to 80Hz (DO0~DO2 111), p63
    twi_write (MAG_ADDR, CTRL_REG1_M, (1<<OM1) | (1<<M_CTRL_REG_1_TEMPCOMP) | (1<<M_CTRL_REG_1_DO0) | (1<<M_CTRL_REG_1_DO1) | (1<<M_CTRL_REG_1_DO2));            // set high performance mode for x-y, and the default output rate is already 10hz (p63).
    twi_write (MAG_ADDR, CTRL_REG3_M, 0x00);        // Set continuous conversion, default = 11, set to 00 (p64).
    twi_write (MAG_ADDR, CTRL_REG4_M, (1<<OMZ1));            // Set high performance mode for z, default 00 to 10 (p65).
    
   
	// Set up barometer registers here
	#define PD    7
	#define ODR1  5
	#define ODR0  4
    
    twi_write (BARO_ADDR, CTRL_REG1, (1<<PD)|(1<<ODR1)|(1<<ODR0));    // Activate PD, change default rate from 000 to 011 (p35).

	
	return 0;
}

int init_all()
{
	DDRB |= 1<<DDB1; // Set PB1 as output
	
	uart_init();
	uart_putstr("\033c");		// ASCII code clear screen: 'Escape'('\033' in octal)+'c')
	uart_putstr("UART is initialised - 115.2Kbps\r\n");

	twi_init();
	uart_putstr("TWI is initialised - 100Kbps\r\n");

	timer1_init();
	uart_putstr("TIMER1 is initialised - 10Hz event\r\n");

	imu_init();

	uart_putstr("All initialized...\r\n");

}
