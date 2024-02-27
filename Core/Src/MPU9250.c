/*
 * MPU9250.c
 *
 *  Created on: Feb 27, 2024
 *      Author: Nguyen Lam Anh Vu
 */

#include "MPU9250.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c1;
extern uint16_t status;

static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

//low pass filter
float x_Low, y_Low, z_Low, x_old, y_old, z_old;
float x2_Low, y2_Low, z2_Low, x2_old, y2_old, z2_old;
float xg2_Low, yg2_Low, zg2_Low, xg2_old, yg2_old, zg2_old;
const float dtt2 = (1.0 / 200.0); // sample rate
const float RC2 = 0.35;
const float alpha2 = dtt2 / (RC2 + dtt2);

void MPU9250_Writebyte(uint8_t reg_addr, uint8_t val)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 1);
}

void MPU9250_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU9250_Readbyte(uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 1);
}

void MPU9250_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU9250_Initialization(void)
{
	x2_old = 0;
	y2_old = 0;
	z2_old = 0;

	HAL_Delay(50);
	uint8_t who_am_i = 0;

	MPU9250_Readbyte(MPU9250_WHO_AM_I, &who_am_i);
	if(who_am_i == 0x71)		//default value is 0x71
	{
		status = 1;			// who_am_i correct
	}
	else
	{
		status = 0;			// who_am_i incorrect
		while(1)
		{
			HAL_Delay(100);
		}
	}

	//Reset the whole module before initialization
	//Reset the internal registers and restores the default settings.  Write a 1 to set the reset, the bit will auto clear.
	MPU9250_Writebyte(MPU9250_PWR_MGMT_1, 0x1<<7);
	HAL_Delay(100);

	//Power Management setting
	/* Default is sleep mode
	 * necessary to wake up MPU6050*/
	MPU9250_Writebyte(MPU9250_PWR_MGMT_1, 0x00);
	HAL_Delay(50);

	//Sample rate divider
	/*Sample Rate = GYRO output rate / (1 + SMPRT_DIV) */
	//	MPU9250_Writebyte(MPU9250_SMPLRT_DIV, 0x00); // ACC output rate is 1kHz, GYRO output rate is 8kHz (Normal mode)
	MPU9250_Writebyte(MPU9250_SMPLRT_DIV, 39); // Sample Rate = 200Hz		//**********************
	HAL_Delay(50);

	//FSYNC and DLPF setting
	//fchoice[1:0]=2'b11 ; fchoice_b[1:0]=2'b00
	/*DLPF is set to 0*/
	MPU9250_Writebyte(MPU9250_CONFIG, 0x00);								//**********************
	HAL_Delay(50);

	//GYRO FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-250 degree/s
	  1		+-500 degree/s
	  2		+-1000 degree/s
	  3		+-2000 degree/s	*/
	uint8_t FS_SCALE_GYRO = 0x03;
	MPU9250_Writebyte(MPU9250_GYRO_CONFIG, FS_SCALE_GYRO<<3);
	HAL_Delay(50);

	//ACCEL FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-2g
	  1		+-4g
	  2		+-8g
	  3		+-16g	*/
	uint8_t FS_SCALE_ACC = 0x0;
	MPU9250_Writebyte(MPU9250_ACCEL_CONFIG, FS_SCALE_ACC<<3);
	HAL_Delay(50);

	MPU9250_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);

//	//Interrupt PIN setting
//	uint8_t INT_LEVEL = 0x0; //0 - active high, 1 - active low
//	uint8_t LATCH_INT_EN = 0x0; //0 - INT 50us pulse, 1 - interrupt clear required
//	uint8_t INT_RD_CLEAR = 0x1; //0 - INT flag cleared by reading INT_STATUS, 1 - INT flag cleared by any read operation
//	MPU9250_Writebyte(MPU9250_INT_PIN_CFG, (INT_LEVEL<<7)|(LATCH_INT_EN<<5)|(INT_RD_CLEAR<<4)); //
//	HAL_Delay(50);
//
//	//Interrupt enable setting
//	uint8_t DATA_RDY_EN = 0x1; // 1 - enable, 0 - disable
//	MPU9250_Writebyte(MPU9250_INT_ENABLE, DATA_RDY_EN);
//	HAL_Delay(50);

}

/*Get Raw Data from sensor*/
void MPU9250_Get6AxisRawData(Struct_MPU9250* pMPU9250)
{
	uint8_t data[14];
	MPU9250_Readbytes(MPU9250_ACCEL_XOUT_H, 14, data);

	pMPU9250->acc_x_raw = (data[0] << 8) | data[1];
	pMPU9250->acc_y_raw = (data[2] << 8) | data[3];
	pMPU9250->acc_z_raw = (data[4] << 8) | data[5];

	pMPU9250->temperature_raw = (data[6] << 8) | data[7];

	pMPU9250->gyro_x_raw = ((data[8] << 8) | data[9]);
	pMPU9250->gyro_y_raw = ((data[10] << 8) | data[11]);
	pMPU9250->gyro_z_raw = ((data[12] << 8) | data[13]);
}

void MPU9250_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
	switch(FS_SCALE_GYRO)
	{
	case 0:
		LSB_Sensitivity_GYRO = 131.f;
		break;
	case 1:
		LSB_Sensitivity_GYRO = 65.5f;
		break;
	case 2:
		LSB_Sensitivity_GYRO = 32.8f;
		break;
	case 3:
		LSB_Sensitivity_GYRO = 16.4f;
		break;
	}
	switch(FS_SCALE_ACC)
	{
	case 0:
		LSB_Sensitivity_ACC = 16384.f;
		break;
	case 1:
		LSB_Sensitivity_ACC = 8192.f;
		break;
	case 2:
		LSB_Sensitivity_ACC = 4096.f;
		break;
	case 3:
		LSB_Sensitivity_ACC = 2048.f;
		break;
	}
}

/// @brief Calculate the real world sensor values
/// @param pMPU9250 Pointer to master MPU9250 struct
void MPU_readProcessedData(Struct_MPU9250 *pMPU9250)
{
    // Get raw values from the IMU
	MPU9250_Get6AxisRawData(pMPU9250);

    // Convert accelerometer values to g's
    pMPU9250->acc_x = pMPU9250->acc_x_raw / LSB_Sensitivity_ACC;
    pMPU9250->acc_y = pMPU9250->acc_y_raw / LSB_Sensitivity_ACC;
    pMPU9250->acc_z = pMPU9250->acc_z_raw / LSB_Sensitivity_ACC;
    // Low pass filter of accelerometer
    x2_Low = ((alpha2 * pMPU9250->acc_x) + (1.0 - alpha2) * x2_old);
	y2_Low = ((alpha2 * pMPU9250->acc_y) + (1.0 - alpha2) * y2_old);
	z2_Low = ((alpha2 * pMPU9250->acc_z) + (1.0 - alpha2) * z2_old);

	x2_old = x2_Low;
	y2_old = y2_Low;
	z2_old = z2_Low;

	pMPU9250->Filt_accx = x2_Low;
	pMPU9250->Filt_accy = y2_Low;
	pMPU9250->Filt_accz = z2_Low;

    // Compensate for gyro offset
    pMPU9250->gyro_x = pMPU9250->gyro_x_raw - pMPU9250->cal_gyx;
    pMPU9250->gyro_y = pMPU9250->gyro_y_raw - pMPU9250->cal_gyy;
    pMPU9250->gyro_z = pMPU9250->gyro_z_raw - pMPU9250->cal_gyz;

    // Convert gyro values to deg/s
    pMPU9250->gyro_x /= LSB_Sensitivity_GYRO;
    pMPU9250->gyro_y /= LSB_Sensitivity_GYRO;
    pMPU9250->gyro_z /= LSB_Sensitivity_GYRO;
    // Low pass filter of accelerometer
    xg2_Low = ((alpha2 * pMPU9250->gyro_x) + (1.0 - alpha2) * xg2_old);
	yg2_Low = ((alpha2 * pMPU9250->gyro_y) + (1.0 - alpha2) * yg2_old);
	zg2_Low = ((alpha2 * pMPU9250->gyro_z) + (1.0 - alpha2) * zg2_old);

	xg2_old = xg2_Low;
	yg2_old = yg2_Low;
	zg2_old = zg2_Low;

	pMPU9250->Filt_gyx = xg2_Low;
	pMPU9250->Filt_gyy = yg2_Low;
	pMPU9250->Filt_gyz = zg2_Low;
}

/// @brief Find offsets for each axis of gyroscope
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param numCalPoints Number of data points to average
void MPU_calibrateGyro(Struct_MPU9250 *pMPU9250, uint16_t numCalPoints)
{
    // Init
    int32_t xx = 0;
    int32_t yy = 0;
    int32_t zz = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
    	MPU9250_Get6AxisRawData(pMPU9250);
        xx += pMPU9250->gyro_x_raw;
        yy += pMPU9250->gyro_y_raw;
        zz += pMPU9250->gyro_z_raw;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    pMPU9250->cal_gyx = (float)xx / (float)numCalPoints;
    pMPU9250->cal_gyy = (float)yy / (float)numCalPoints;
    pMPU9250->cal_gyz = (float)zz / (float)numCalPoints;
}

void CalculateAccAngle(Struct_Angle* Angle, Struct_MPU9250 *pMPU9250)
{
	Angle->acc_roll  = atan(-pMPU9250->acc_x / sqrt(pow(pMPU9250->acc_y,2) + pow(pMPU9250->acc_z,2))) * RADIAN_TO_DEGREE;
	Angle->acc_pitch = atan(pMPU9250->acc_y / sqrt(pow(pMPU9250->acc_x,2) + pow(pMPU9250->acc_z,2))) * RADIAN_TO_DEGREE;
	Angle->acc_yaw = atan(sqrt(pow(pMPU9250->acc_x, 2) + pow(pMPU9250->acc_y, 2)) / pMPU9250->acc_z) * RADIAN_TO_DEGREE;
	//Can't use Angle->acc_yaw there is no reliability. It's based on my personal experimental view.

    x_Low = ((alpha2 * pMPU9250->acc_x) + (1.0 - alpha2) * x_old);
    y_Low = ((alpha2 * pMPU9250->acc_y) + (1.0 - alpha2) * y_old);
    z_Low = ((alpha2 * pMPU9250->acc_z) + (1.0 - alpha2) * z_old);

    x_old = x_Low;
    y_old = y_Low;
    z_old = z_Low;

    Angle->Filt_roll  = atan(-x_Low / sqrt(pow(y_Low,2) + pow(z_Low,2))) * RADIAN_TO_DEGREE;
	Angle->Filt_pitch = atan(y_Low / sqrt(pow(x_Low,2) + pow(z_Low,2))) * RADIAN_TO_DEGREE;
}

void CalculateGyroAngle(Struct_Angle* Angle, Struct_MPU9250 *pMPU9250)
{
	Angle->gyro_roll  += pMPU9250->Filt_gyx * dtt2;
	Angle->gyro_pitch += pMPU9250->Filt_gyy * dtt2;
	Angle->gyro_yaw   += pMPU9250->Filt_gyz * dtt2;
}

void CalculateCompliFilter(Struct_Angle* Angle, Struct_MPU9250 *pMPU9250)
{
	CalculateAccAngle(Angle, pMPU9250); //Prepare Acc Angle before using Complimentary Filter.

	static float alpha = 0.955f;
	Angle->ComFilt_roll  = alpha*(pMPU9250->gyro_y * dtt2 + Angle->ComFilt_roll) + (1-alpha) * Angle->Filt_roll;
	Angle->ComFilt_pitch = alpha*(pMPU9250->gyro_x * dtt2 + Angle->ComFilt_pitch) + (1-alpha) * Angle->Filt_pitch;
	Angle->ComFilt_yaw   = Angle->ComFilt_yaw + pMPU9250->Filt_gyz * dtt2 ; //0.033, sau do nhan 1.2
}
