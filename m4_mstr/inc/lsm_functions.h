#include "chip.h"
#include "board.h"



//settings structure for the magnetometer
typedef struct
{
    // Magnetometer settings:
    uint8_t enabled;
    uint8_t scale;
    uint8_t sampleRate;
    // New mag stuff:
    uint8_t tempCompensationEnable;
    uint8_t XYPerformance;
    uint8_t ZPerformance;
    uint8_t lowPowerEnable;
    uint8_t operatingMode;
}magSettings;

//settings structure for the gyro
typedef struct
{
    // Gyroscope settings:
    uint8_t enabled;
    uint16_t scale;	// Changed this to 16-bit
    uint8_t sampleRate;
    // New gyro stuff:
    uint8_t bandwidth;
    uint8_t lowPowerEnable;
    uint8_t HPFEnable;
    uint8_t HPFCutoff;
    uint8_t flipX;
    uint8_t flipY;
    uint8_t flipZ;
    uint8_t orientation;
    uint8_t enableX;
    uint8_t enableY;
    uint8_t enableZ;
    uint8_t latchInterrupt;
}gyroSettings;

//settings structure for the accelerometer
typedef struct 
{
    // Accelerometer settings:
    uint8_t enabled;
    uint8_t scale;
    uint8_t sampleRate;
    // New accel stuff:
    uint8_t enableX;
    uint8_t enableY;
    uint8_t enableZ;
    int8_t  bandwidth;
    uint8_t highResEnable;
    uint8_t highResBandwidth;
}accelSettings;

typedef enum
{
    FIFO_OFF = 0,
    FIFO_THS = 1,
    FIFO_CONT_TRIGGER = 3,
    FIFO_OFF_TRIGGER = 4,
    FIFO_CONT = 5
}fifoMode_type;

enum lsm9ds1_axis
{
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
};
//raw signed 16 bit values from magnetometer
extern int16_t mx, my, mz,ax, ay, az, gx, gy, gz; 
extern int16_t temperature; //chip temperature

//make settings structure variables available globally as they
//may need to be passed into other functions
extern magSettings mag_settings;
extern accelSettings accel_settings;
extern gyroSettings gyro_settings;

// begin() -- Initialize the gyro, accelerometer, and magnetometer.
// This will set up the scale and output rate of each sensor. The values set
// in the IMUSettings struct will take effect after calling this function.
void initLSM();
	
//void calibrate(bool autoCalc = true);
//void calibrateMag(bool loadIn = true);
//void magOffset(uint8_t axis, int16_t offset);

// initMag() -- Sets up the magnetometer to begin reading.
// This function steps through all magnetometer-related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
//		requests don't latch. Temperature sensor disabled.
//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
void initMag(magSettings settings);
void initGyro(gyroSettings settings);
void initAccel(accelSettings settings);

void magOffset(uint8_t axis, int16_t offset);

// accelAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t accelAvailable();
	
// gyroAvailable() -- Polls the gyroscope status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t gyroAvailable();

// magAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Input:
//	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
//	  on one specific axis. Or ALL_AXIS (default) to check for new data
//	  on all axes.
// Output:	1 - New data available
//			0 - No new data available
uint8_t magAvailable();

// tempAvailable() -- Polls the temperature status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t tempAvailable();

// calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
// This function reads in a signed 16-bit value and returns the scaled
// Gs. This function relies on mScale and mRes being correct.
// Input:
//	- mag = A signed 16-bit raw reading from the magnetometer.
float calcMag(int16_t mag);

// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
// This function reads in a signed 16-bit value and returns the scaled
// DPS. This function relies on gScale and gRes being correct.
// Input:
//	- gyro = A signed 16-bit raw reading from the gyroscope.
float calcGyro(int16_t gyro);
	
// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
// This function reads in a signed 16-bit value and returns the scaled
// g's. This function relies on aScale and aRes being correct.
// Input:
//	- accel = A signed 16-bit raw reading from the accelerometer.
float calcAccel(int16_t accel);

void setGyroScale(uint16_t gScl, gyroSettings settings);
	
// setAccelScale() -- Set the full-scale range of the accelerometer.
// This function can be called to set the scale of the accelerometer to
// 2, 4, 6, 8, or 16 g's.
// Input:
// 	- aScl = The desired accelerometer scale. Must be one of five possible
//		values from the accel_scale.
void setAccelScale(uint8_t aScl, accelSettings settings);

// setMagScale() -- Set the full-scale range of the magnetometer.
// This function can be called to set the scale of the magnetometer to
// 2, 4, 8, or 12 Gs.
// Input:
// 	- mScl = The desired magnetometer scale. Must be one of four possible
//
void setMagScale(uint8_t mScl, magSettings settings);

// setMagODR() -- Set the output data rate of the magnetometer
// Input:
//	- mRate = The desired output rate of the mag.
void setMagODR(uint8_t mRate, magSettings settings);

// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
// Input:
//	- gRate = The desired output rate and cutoff frequency of the gyro.
void setGyroODR(uint8_t gRate, gyroSettings settings);
	
// setAccelODR() -- Set the output data rate of the accelerometer
// Input:
//	- aRate = The desired output rate of the accel.
void setAccelODR(uint8_t aRate, accelSettings settings); 

// calcgRes() -- Calculate the resolution of the gyroscope.
// This function will set the value of the gRes variable. gScale must
// be set prior to calling this function.
void calcgRes();

// calcaRes() -- Calculate the resolution of the accelerometer.
// This function will set the value of the aRes variable. aScale must
// be set prior to calling this function.
void calcaRes();

// calcmRes() -- Calculate the resolution of the magnetometer.
// This function will set the value of the mRes variable. mScale must
// be set prior to calling this function.
void calcmRes(magSettings settings);

// readGyro() -- Read the gyroscope output registers.
// This function will read all six gyroscope output registers.
// The readings are stored in the class' gx, gy, and gz variables. Read
// those _after_ calling readGyro().
void readGyro();

// readAccel() -- Read the accelerometer output registers.
// This function will read all six accelerometer output registers.
// The readings are stored in the class' ax, ay, and az variables. Read
// those _after_ calling readAccel().
void readAccel();

// readTemp() -- Read the temperature output register.
// This function will read two temperature output registers.
// The combined readings are stored in the class' temperature variables. Read
// those _after_ calling readTemp().
void readTemp();

// readMag() -- Read the magnetometer output registers.
// This function will read all six magnetometer output registers.
// The readings are stored in the class' mx, my, and mz variables. Read
// those _after_ calling readMag().
void readMag();

float heading(float mx, float my, float mz);
float pitch(float ax, float ay, float az);
float roll(float ax, float ay, float az);
	
// sleepGyro() -- Sleep or wake the gyroscope
// Input:
//	- enable: True = sleep gyro. False = wake gyro.
//void sleepGyro(bool enable);
	
// enableFIFO() - Enable or disable the FIFO
// Input:
//	- enable: true = enable, false = disable.
//void enableFIFO(bool enable);
	
// setFIFO() - Configure FIFO mode and Threshold
// Input:
//	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
//	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
//	- fifoThs: FIFO threshold level setting
//	  Any value from 0-0x1F is acceptable.
void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);
	
// getFIFOSamples() - Get number of FIFO samples
uint8_t getFIFOSamples();

//enable FIFO
void enableFIFO(bool enable);

//
void calibrate(bool);

void calibrateMag(bool);
