/*
 *By Jarred Beilke
 *12/25/16
 *Adapted from the generous example code provided by Sparkfun's Jim Lindblom
 *
 */

#include "spi_functions.h"
#include "lsm_registers.h"
#include "lsm_functions.h"
#include <math.h>

float magSensitivity[4] = {0.00014, 0.00029, 0.00043, 0.00058};

int16_t mx, my, mz, ax, ay, az, gx, gy, gz;
int16_t temperature;
float mRes, aRes, gRes;
int16_t mBiasRaw[3], aBiasRaw[3], gBiasRaw[3];
float mBias[3], aBias[3], gBias[3];
bool _autoCalc;
const float PI =  3.14159;
magSettings mag_settings;
accelSettings accel_settings;
gyroSettings gyro_settings;

void initLSM()
{

    //! Todo: don't use _xgAddress or _mAddress, duplicating memory	
    	
    mag_settings.scale = 4;
    mag_settings.sampleRate = 7;
    mag_settings.tempCompensationEnable = true;
    mag_settings.XYPerformance = 3;
    mag_settings.ZPerformance = 3;
    mag_settings.operatingMode = 0;

    gyro_settings.enabled = true;
    gyro_settings.enableX = true;
    gyro_settings.enableY = true;
    gyro_settings.enableZ = true;
    gyro_settings.scale = 245;
    gyro_settings.sampleRate = 6;
    gyro_settings.bandwidth = 0;
    gyro_settings.lowPowerEnable = false;
    gyro_settings.HPFEnable = false;
    gyro_settings.HPFCutoff = 0;
    gyro_settings.flipX = false;
    gyro_settings.flipY = false;
    gyro_settings.flipZ = false;
    gyro_settings.orientation = 0;
    gyro_settings.latchInterrupt = true;

    accel_settings.enabled = true;
    accel_settings.enableX = true;
    accel_settings.enableY = true;
    accel_settings.enableZ = true;
    accel_settings.scale = 2;
    accel_settings.sampleRate = 6;
    accel_settings.bandwidth = -1;
    accel_settings.highResEnable = false;
    accel_settings.highResBandwidth = 0;



    // Gyro initialization stuff:
    initGyro(gyro_settings);	// This will "turn on" the gyro. Setting up interrupts, etc.
	
    // Accelerometer initialization stuff:
    initAccel(accel_settings); // "Turn on" all axes of the accel. Set up interrupts, etc.
	
    // Magnetometer initialization stuff:
    initMag(mag_settings); // "Turn on" all axes of the mag. Set up interrupts, etc.
    constrainScales();
    // Once we have the scale values, we can calculate the resolution
    // of each sensor. That's what these functions are for. One for each sensor
    calcgRes(gyro_settings); // Calculate DPS / ADC tick, stored in gRes variable
    calcmRes(mag_settings); // Calculate Gs / ADC tick, stored in mRes variable
    calcaRes(accel_settings); // Calculate g / ADC tick, stored in aRes variable
}

/*This function has been adapted from c++ to take a struct of settings
 *the setttings are documented within the function itself where they
 *occur. It serves to initialize the magnetometer returns nothing.
 */
void initMag(magSettings settings)
{
    uint8_t tempRegValue = 0;
	
    // CTRL_REG1_M (Default value: 0x10)
    // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
    // TEMP_COMP - Temperature compensation
    // OM[1:0] - X & Y axes op mode selection
    //	00:low-power, 01:medium performance
    //	10: high performance, 11:ultra-high performance
    // DO[2:0] - Output data rate selection
    // ST - Self-test enable
    if (settings.tempCompensationEnable) tempRegValue |= (1<<7);
    tempRegValue |= (settings.XYPerformance & 0x3) << 5;
    tempRegValue |= (settings.sampleRate & 0x7) << 2;
    magWriteByte(CTRL_REG1_M, tempRegValue);
	
    // CTRL_REG2_M (Default value 0x00)
    // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
    // FS[1:0] - Full-scale configuration
    // REBOOT - Reboot memory content (0:normal, 1:reboot)
    // SOFT_RST - Reset config and user registers (0:default, 1:reset)
    tempRegValue = 0;
    switch (settings.scale)
    {
    case 8:
	tempRegValue |= (0x1 << 5);
	break;
    case 12:
	tempRegValue |= (0x2 << 5);
	break;
    case 16:
	tempRegValue |= (0x3 << 5);
	break;
	// Otherwise we'll default to 4 gauss (00)
    }
    magWriteByte(CTRL_REG2_M, tempRegValue); // +/-4Gauss
	
    // CTRL_REG3_M (Default value: 0x03)
    // [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
    // I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
    // LP - Low-power mode cofiguration (1:enable)
    // SIM - SPI mode selection (0:write-only, 1:read/write enable)
    // MD[1:0] - Operating mode
    //	00:continuous conversion, 01:single-conversion,
    //  10,11: Power-down
    tempRegValue = 0;
    if (settings.lowPowerEnable) tempRegValue |= (1<<5);
    tempRegValue |= (settings.operatingMode & 0x3);
    magWriteByte(CTRL_REG3_M, tempRegValue); // Continuous conversion mode
	
    // CTRL_REG4_M (Default value: 0x00)
    // [0][0][0][0][OMZ1][OMZ0][BLE][0]
    // OMZ[1:0] - Z-axis operative mode selection
    //	00:low-power mode, 01:medium performance
    //	10:high performance, 10:ultra-high performance
    // BLE - Big/little endian data
    tempRegValue = 0;
    tempRegValue = (settings.ZPerformance & 0x3) << 2;
    magWriteByte(CTRL_REG4_M, tempRegValue);
	
    // CTRL_REG5_M (Default value: 0x00)
    // [0][BDU][0][0][0][0][0][0]
    // BDU - Block data update for magnetic data
    //	0:continuous, 1:not updated until MSB/LSB are read
    tempRegValue = 0;
    magWriteByte(CTRL_REG5_M, tempRegValue);
}

void readMag()
{
    uint8_t temp[6]; // We'll read six bytes from the mag into temp	
    magReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
    mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
    my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
    mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}
/*
//ability to read a specific axis not used at this time
int16_t readMag(lsm9ds1_axis axis)
{
uint8_t temp[2];
mReadBytes(OUT_X_L_M + (2 * axis), temp, 2);
return (temp[1] << 8) | temp[0];
}*/

void setGyroScale(uint16_t gScl, gyroSettings settings)
{
    // Read current value of CTRL_REG1_G:
    uint8_t ctrl1RegValue = xgReadByte(CTRL_REG1_G);
    // Mask out scale bits (3 & 4):
    ctrl1RegValue &= 0xE7;
    switch (gScl)
    {
    case 500:
	ctrl1RegValue |= (0x1 << 3);
	settings.scale = 500;
	break;
    case 2000:
	ctrl1RegValue |= (0x3 << 3);
	settings.scale = 2000;
	break;
    default: // Otherwise we'll set it to 245 dps (0x0 << 4)
	settings.scale = 245;
	break;
    }
    xgWriteByte(CTRL_REG1_G, ctrl1RegValue);
	
    calcgRes();	
}

void setAccelScale(uint8_t aScl, accelSettings settings)
{
    // We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
    uint8_t tempRegValue = xgReadByte(CTRL_REG6_XL);
    // Mask out accel scale bits:
    tempRegValue &= 0xE7;
	
    switch (aScl)
    {
    case 4:
	tempRegValue |= (0x2 << 3);
	settings.scale = 4;
	break;
    case 8:
	tempRegValue |= (0x3 << 3);
	settings.scale = 8;
	break;
    case 16:
	tempRegValue |= (0x1 << 3);
	settings.scale = 16;
	break;
    default: // Otherwise it'll be set to 2g (0x0 << 3)
	settings.scale = 2;
	break;
    }
    xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
    // Then calculate a new aRes, which relies on aScale being set correctly:
    calcaRes();
}

void setMagScale(uint8_t mScl, magSettings settings)
{
    // We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
    uint8_t temp = magReadByte(CTRL_REG2_M);
    // Then mask out the mag scale bits:
    temp &= 0xFF^(0x3 << 5);
	
    switch (mScl)
    {
    case 8:
	temp |= (0x1 << 5);
	settings.scale = 8;
	break;
    case 12:
	temp |= (0x2 << 5);
	settings.scale = 12;
	break;
    case 16:
	temp |= (0x3 << 5);
	settings.scale = 16;
	break;
    default: // Otherwise we'll default to 4 gauss (00)
	settings.scale = 4;
	break;
    }	
	
    // And write the new register value back into CTRL_REG6_XM:
    magWriteByte(CTRL_REG2_M, temp);
	
    // We've updated the sensor, but we also need to update our class variables
    // First update mScale:
    //mScale = mScl;
    // Then calculate a new mRes, which relies on mScale being set correctly:
    calcmRes(settings);
}

void calcgRes(gyroSettings settings)
{
    gRes = 0.00875;//((float) settings.scale) / 32768.0;
}

void calcaRes(accelSettings settings)
{
    aRes = 0.183;//((float) settings.scale) / 32768.0;
}

void calcmRes(magSettings settings)
{
    //mRes = ((float) settings.mag.scale) / 32768.0;
    switch (settings.scale)
    {
    case 4:
	mRes = magSensitivity[0];
	break;
    case 8:
	mRes = magSensitivity[1];
	break;
    case 12:
	mRes = magSensitivity[2];
	break;
    case 16:
	mRes = magSensitivity[3];
	break;
    }
	
}
float pitch(float ax, float ay, float az)
{
    float pitch = atanf(-ax/ sqrt(ay*ay+az*az));
    pitch *= (180.0/PI);
    return pitch;
}
float roll(float ax, float ay, float az)
{
    float roll = atanf(ay/az);
    roll *= (180.0/PI);
    return roll;
}

float heading(float mx, float my, float mz)
{
    float heading, rv;

    if(my == 0)
	heading = (mx < 0) ? 180.0 : 0;
    else
	heading = atanf(mx/my);
    heading -= 33.7*(PI/180);
    if(heading > PI) heading -= (2*PI);
    else if(heading < -PI) heading += (2*PI);
    else if(heading < 0) heading += (2 * PI);

    rv = heading * (180.0/PI);
    return rv;
}

void initGyro(gyroSettings settings)
{
    uint8_t tempRegValue = 0;
	
    // CTRL_REG1_G (Default value: 0x00)
    // [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
    // ODR_G[2:0] - Output data rate selection
    // FS_G[1:0] - Gyroscope full-scale selection
    // BW_G[1:0] - Gyroscope bandwidth selection
	
    // To disable gyro, set sample rate bits to 0. We'll only set sample
    // rate if the gyro is enabled.
    if (settings.enabled)
    {
	tempRegValue = (settings.sampleRate & 0x07) << 5;
    }
    switch (settings.scale)
    {
    case 500:
	tempRegValue |= (0x1 << 3);
	break;
    case 2000:
	tempRegValue |= (0x3 << 3);
	break;
	// Otherwise we'll set it to 245 dps (0x0 << 4)
    case 245:
    	tempRegValue |= (0x0 << 4);
	break;
    }
    tempRegValue |= (settings.bandwidth & 0x3);
    xgWriteByte(CTRL_REG1_G, tempRegValue);
	
    // CTRL_REG2_G (Default value: 0x00)
    // [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
    // INT_SEL[1:0] - INT selection configuration
    // OUT_SEL[1:0] - Out selection configuration
    xgWriteByte(CTRL_REG2_G, 0x00);	
	
    // CTRL_REG3_G (Default value: 0x00)
    // [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
    // LP_mode - Low-power mode enable (0: disabled, 1: enabled)
    // HP_EN - HPF enable (0:disabled, 1: enabled)
    // HPCF_G[3:0] - HPF cutoff frequency
    tempRegValue = settings.lowPowerEnable ? (1<<7) : 0;
    if (settings.HPFEnable)
    {
	tempRegValue |= (1<<6) | (settings.HPFCutoff & 0x0F);
    }
    xgWriteByte(CTRL_REG3_G, tempRegValue);
	
    // CTRL_REG4 (Default value: 0x38)
    // [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
    // Zen_G - Z-axis output enable (0:disable, 1:enable)
    // Yen_G - Y-axis output enable (0:disable, 1:enable)
    // Xen_G - X-axis output enable (0:disable, 1:enable)
    // LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
    // 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
    tempRegValue = 0;
    if (settings.enableZ) tempRegValue |= (1<<5);
    if (settings.enableY) tempRegValue |= (1<<4);
    if (settings.enableX) tempRegValue |= (1<<3);
    if (settings.latchInterrupt) tempRegValue |= (1<<1);
    xgWriteByte(CTRL_REG4, tempRegValue);
	
    // ORIENT_CFG_G (Default value: 0x00)
    // [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
    // SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
    // Orient [2:0] - Directional user orientation selection
    tempRegValue = 0;
    if (settings.flipX) tempRegValue |= (1<<5);
    if (settings.flipY) tempRegValue |= (1<<4);
    if (settings.flipZ) tempRegValue |= (1<<3);
    xgWriteByte(ORIENT_CFG_G, tempRegValue);
}

void initAccel(accelSettings settings)
{
    uint8_t tempRegValue = 0;
	
    //	CTRL_REG5_XL (0x1F) (Default value: 0x38)
    //	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
    //	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
    //		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
    //	Zen_XL - Z-axis output enabled
    //	Yen_XL - Y-axis output enabled
    //	Xen_XL - X-axis output enabled
    if (settings.enableZ) tempRegValue |= (1<<5);
    if (settings.enableY) tempRegValue |= (1<<4);
    if (settings.enableX) tempRegValue |= (1<<3);
	
    xgWriteByte(CTRL_REG5_XL, tempRegValue);
	
    // CTRL_REG6_XL (0x20) (Default value: 0x00)
    // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
    // ODR_XL[2:0] - Output data rate & power mode selection
    // FS_XL[1:0] - Full-scale selection
    // BW_SCAL_ODR - Bandwidth selection
    // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
    tempRegValue = 0;
    // To disable the accel, set the sampleRate bits to 0.
    if (settings.enabled)
    {
	tempRegValue |= (settings.sampleRate & 0x07) << 5;
    }
    switch (settings.scale)
    {
    case 4:
	tempRegValue |= (0x2 << 3);
	break;
    case 8:
	tempRegValue |= (0x3 << 3);
	break;
    case 16:
	tempRegValue |= (0x1 << 3);
	break;
	// Otherwise it'll be set to 2g (0x0 << 3)
    }
    if (settings.bandwidth >= 0)
    {
	tempRegValue |= (1<<2); // Set BW_SCAL_ODR
	tempRegValue |= (settings.bandwidth & 0x03);
    }
    xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
    // CTRL_REG7_XL (0x21) (Default value: 0x00)
    // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
    // HR - High resolution mode (0: disable, 1: enable)
    // DCF[1:0] - Digital filter cutoff frequency
    // FDS - Filtered data selection
    // HPIS1 - HPF enabled for interrupt function
    tempRegValue = 0;
    if (settings.highResEnable)
    {
	tempRegValue |= (1<<7); // Set HR bit
	tempRegValue |= (settings.highResBandwidth & 0x3) << 5;
    }
    xgWriteByte(CTRL_REG7_XL, tempRegValue);
}

void calibrate(bool autoCalc)
{  
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    uint8_t samples = 0;
    int ii;
    int32_t aBiasRawTemp[3] = {0, 0, 0};
    int32_t gBiasRawTemp[3] = {0, 0, 0};
	
    // Turn on FIFO and set threshold to 32 samples
    enableFIFO(true);
    setFIFO(FIFO_THS, 0x1F);
    while (samples < 0x1F)
    {
	samples = (xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
    }
    for(ii = 0; ii < samples ; ii++) 
    {	// Read the gyro data stored in the FIFO
	readGyro();
	gBiasRawTemp[0] += gx;
	gBiasRawTemp[1] += gy;
	gBiasRawTemp[2] += gz;
	readAccel();
	aBiasRawTemp[0] += ax;
	aBiasRawTemp[1] += ay;
	aBiasRawTemp[2] += az - (int16_t)(1./aRes); // Assumes sensor facing up!
    }  
    for (ii = 0; ii < 3; ii++)
    {
	gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
	gBias[ii] = calcGyro(gBiasRaw[ii]);
	aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
	aBias[ii] = calcAccel(aBiasRaw[ii]);
    }
	
    enableFIFO(false);
    setFIFO(FIFO_OFF, 0x00);
	
    if (autoCalc) _autoCalc = true;
}

void calibrateMag(bool loadIn)
{
    int i, j;
    int16_t magMin[3] = {0, 0, 0};
    int16_t magMax[3] = {0, 0, 0}; // The road warrior
	
    for (i=0; i<128; i++)
    {
	while (!magAvailable())
	    ;
	readMag();
	int16_t magTemp[3] = {0, 0, 0};
	magTemp[0] = mx;		
	magTemp[1] = my;
	magTemp[2] = mz;
	for (j = 0; j < 3; j++)
	{
	    if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
	    if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
	}
    }
    for (j = 0; j < 3; j++)
    {
	mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
	mBias[j] = calcMag(mBiasRaw[j]);
	if (loadIn)
	    magOffset(j, mBiasRaw[j]);
    }
	
}

void magOffset(uint8_t axis, int16_t offset)
{
    if (axis > 2)
	return;
    uint8_t msb, lsb;
    msb = (offset & 0xFF00) >> 8;
    lsb = offset & 0x00FF;
    magWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
    magWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
}

uint8_t accelAvailable()
{
    uint8_t status = xgReadByte(STATUS_REG_1);
	
    return (status & (1<<0));
}

uint8_t gyroAvailable()
{
    uint8_t status = xgReadByte(STATUS_REG_1);
	
    return ((status & (1<<1)) >> 1);
}

uint8_t tempAvailable()
{
    uint8_t status = xgReadByte(STATUS_REG_1);
	
    return ((status & (1<<2)) >> 2);
}

uint8_t magAvailable()
{
    uint8_t status;
    status = magReadByte(STATUS_REG_M);
	
    return ((status & (1<<4)) >> 4);
}

void readAccel()
{
    uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
    xgReadBytes(OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
    ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
    ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
    az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
    if (_autoCalc)
    {
	ax -= aBiasRaw[X_AXIS];
	ay -= aBiasRaw[Y_AXIS];
	az -= aBiasRaw[Z_AXIS];
    }
}

void readTemp()
{
    uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
    xgReadBytes(OUT_TEMP_L, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L
    temperature = ((int16_t)temp[1] << 8) | temp[0];
}

void readGyro()
{
    uint8_t temp[6]; // We'll read six bytes from the gyro into temp
    xgReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
    gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
    gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
    gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
    if (_autoCalc)
    {
	gx -= gBiasRaw[X_AXIS];
	gy -= gBiasRaw[Y_AXIS];
	gz -= gBiasRaw[Z_AXIS];
    }
}

float calcGyro(int16_t gyro)
{
    // Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
    return gRes * gyro; 
}

float calcAccel(int16_t accel)
{
    // Return the accel raw reading times our pre-calculated g's / (ADC tick):
    return aRes * accel;
}

float calcMag(int16_t mag)
{
    // Return the mag raw reading times our pre-calculated Gs / (ADC tick):
    return mRes * mag;
}

void setAccelODR(uint8_t aRate, accelSettings settings)
{
    // Only do this if aRate is not 0 (which would disable the accel)
    if ((aRate & 0x07) != 0)
    {
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG6_XL);
	// Then mask out the accel ODR bits:
	temp &= 0x1F;
	// Then shift in our new ODR bits:
	temp |= ((aRate & 0x07) << 5);
	settings.sampleRate = aRate & 0x07;
	// And write the new register value back into CTRL_REG1_XM:
	xgWriteByte(CTRL_REG6_XL, temp);
    }
}

void setMagODR(uint8_t mRate, magSettings settings)
{
    // We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
    uint8_t temp = magReadByte(CTRL_REG1_M);
    // Then mask out the mag ODR bits:
    temp &= 0xFF^(0x7 << 2);
    // Then shift in our new ODR bits:
    temp |= ((mRate & 0x07) << 2);
    settings.sampleRate = mRate & 0x07;
    // And write the new register value back into CTRL_REG5_XM:
    magWriteByte(CTRL_REG1_M, temp);
}

void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
    // Limit threshold - 0x1F (31) is the maximum. If more than that was asked
    // limit it to the maximum.
    uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
    xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t getFIFOSamples()
{
    return (xgReadByte(FIFO_SRC) & 0x3F);
}

void enableFIFO(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	xgWriteByte(CTRL_REG9, temp);
}

void constrainScales(magSettings msettings, accelSettings asettings,
		     gyroSettings gsettings)
{
    if ((gsettings.scale != 245) && (gsettings.scale != 500) && 
	(gsettings.scale != 2000))
    {
	gsettings.scale = 245;
    }
		
    if ((asettings.scale != 2) && (asettings.scale != 4) &&
	(asettings.scale != 8) && (asettings.scale != 16))
    {
	asettings.scale = 2;
    }
		
    if ((msettings.scale != 4) && (msettings.scale != 8) &&
	(msettings.scale != 12) && (msettings.scale != 16))
    {
	msettings.scale = 4;
    }
}
