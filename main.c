/*
 * ENGN3213 assignment 2
 * Motion tracking demo
 *
 * u5962182, u5853650
 */ 

#include "mylib.h"
#include "uart.h"
#include "global.h"

extern int flag;
extern volatile uint8_t timer_flag_10Hz;
extern imu_t IMU;

int main(void)
{
    char str[200];
    uint8_t data[12], temp;
    int16_t xi, yi, zi, i;
	float x, y, z;
	float p, t;

	float accel_x, accel_y, accel_z;
	float gyro_x, gyro_y, gyro_z;
	float mag_x, mag_y, mag_z;
	float init_pos;
	
	KalmanData kalmanX, kalmanY, kalmanZ;
	
	// Calibration parameters
	//float error_gx, error_gy, error_gz;
	//float offset_mx, offset_my, coef;
	float xb,yb;

	// Angle from accel, gyro and the final fused ones
	float ang_x, ang_y, ang_z;
	float ang_x_g, ang_y_g, ang_z_g;
	float ang_fused_x, ang_fused_y, ang_fused_z;

    int firstSample = 1;	// Used to detect if it is the first sample for Kalman Filter
    
	Kalman_init(&kalmanX);
	Kalman_init(&kalmanY);
	Kalman_init(&kalmanZ);
	
	init_all();
	
	sei();
	
	uart_putstr("Three I2C device IDs are read...\r\n");
	uart_putstr("Press any key to read Accelerometer data...\r\n");
	temp = get_char();
    
	while (1) 
    {
		if(timer_flag_10Hz)					
		{												
			timer_flag_10Hz = 0;

			i = twi_read(IMU_ADDR, STATUS_REG, &temp, 1);
			if ((temp & 0x7) == 0x7)	// data ready
			{
				i = twi_read(IMU_ADDR, OUT_X_XL, data, 6);
				
				xi = ((int16_t)data[1] << 8) | data[0];
				yi = ((int16_t)data[3] << 8) | data[2];
				zi = ((int16_t)data[5] << 8) | data[4];
				
				x = (float) xi*ACCL_RESOL;
				y = (float) yi*ACCL_RESOL;
				z = (float) zi*ACCL_RESOL;

				accel_x = x;
				accel_y = y;
				accel_z = z;
				
				// Read data from Gyros and Magnetometer.

				i = twi_read(IMU_ADDR, OUT_X_G, data, 6);
            
				// Gyroscope zero-drift
	            xi = ((int16_t)data[1] << 8) | data[0];
	            yi = ((int16_t)data[3] << 8) | data[2];
	            zi = ((int16_t)data[5] << 8) | data[4];
	            
	            x = (float) xi*GYRO_RESOL;
	            y = (float) yi*GYRO_RESOL;
	            z = (float) zi*GYRO_RESOL;

	            // Calibration for gyroscope
	            // The error values are from the gyro raw outputs when the board is placed on a horizontal ground
	            gyro_x = x - error_gx;
	            gyro_y = y - error_gy;
				gyro_z = z - error_gz;
				
				//gyro_x = x;
				//gyro_y = y;
				//gyro_z = z;
        	}

        	i = twi_read(MAG_ADDR, STATUS_REG_M, &temp, 1);
        	if ((temp & 0x7) == 0x7)
        	{
	            i = twi_read(MAG_ADDR, OUT_X_L_M, data, 6);
	            
	            xi = ((int16_t)data[1] << 8) | data[0];
	            yi = ((int16_t)data[3] << 8) | data[2];
	            zi = ((int16_t)data[5] << 8) | data[4];

	            x = (float) xi*MAG_RESOL;
	            y = (float) yi*MAG_RESOL;
	            z = (float) zi*MAG_RESOL;
				

	            // Calibration for magnetometer
				
				mag_x = x - offset_mx;
				mag_y = y - offset_my;
				
				//mag_x = x+xb;
				//mag_y = y*coef + yb;
				
				//mag_x = x;
				//mag_y = y;
	            mag_z = z;
				
				//sprintf(str,"%.3f \r\n", mag_z);
				//uart_putstr(str);
            
	        }
				
				// Read data from Baro and Temperature.
	        i = twi_read(BARO_ADDR, STATUS_REG, &temp, 1);
        	if ((temp & 0x3) == 0x3)
        	{
	            i = twi_read(BARO_ADDR, PRESS_OUT_XL, data, 5);
	            
	            int32_t pressure = ((int32_t) data[2] << 16 | (int32_t) data[1] << 8) | data[0];
	            p = pressure/4096.0f;

	            i = twi_read(BARO_ADDR, TEMP_OUT_L, data, 5);
	            
	            int16_t temperature = (int16_t) data[1] << 8 | data[0];
	            t = 42.5f + temperature/480.0f;
	        }

			// Angle calculated from accel and mag
			ang_x = atan2(-accel_y, accel_z)*(180/PI);
			ang_y = atan2(accel_x, accel_z)*(180/PI);
			ang_z = atan2(mag_y, mag_x)*(180/PI);

			// Angle calculated from gyro 
			ang_x_g += gyro_x * IMU.dt;
			ang_y_g += gyro_y * IMU.dt;
			if (firstSample) {
				init_pos = ang_z;
			}
			ang_z_g += gyro_z * IMU.dt;	


            // Kalman Filter
            if (firstSample  == 1)
            {
                ang_fused_x = T * ang_x_g;
                ang_fused_y = T * ang_y_g;
                ang_fused_z = T * ang_z_g;
            } else
            {
                ang_fused_x = Kalman_Filter(&kalmanX, ang_x_g, ang_x) - 1.43;
                ang_fused_y = Kalman_Filter(&kalmanY, ang_y_g, ang_y) + 3.75;
                ang_fused_z = Kalman_Filter(&kalmanZ, ang_z_g + init_pos, ang_z);
            }

            firstSample = 0;

           	sprintf(str,"%.3f, %.3f, %.3f \r\n", ang_fused_x, ang_fused_y, ang_z);
	        //uart_putstr(str);
			Usart_writeStr(str);
		
			PORTB ^= (1<<PORTB1);
		}
    }
}


