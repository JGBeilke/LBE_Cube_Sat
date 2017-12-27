/*
===============================================================================
 Name        : M4_mstr.c
 Author      : $Jarred Beilke
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#if defined (__MULTICORE_MASTER_SLAVE_M0SLAVE) || \
    defined (__MULTICORE_MASTER_SLAVE_M4SLAVE)
#include "boot_multicore_slave.h"
#endif

// TODO: insert other include files here
#include "tempPinFuncs.h"
#include "spi_functions.h"
#include "lsm_functions.h"
#include "lsm_registers.h"
//#include "retarget.h"
//#include <string.h>
// TODO: insert other definitions and declarations here
bool pinState = false;
uint32_t counter = 100;
uint8_t addressToRead = 0x0F, info_from_LSM = 0;
//magSettings mag_settings;
//accelSettings accel_settings;
//gyroSettings gyro_settings;

unsigned int j=0;

int main(void) {

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
#if defined (__MULTICORE_MASTER) || defined (__MULTICORE_NONE)
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    //Board_Init();
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SCT0);
    Chip_GPIO_Init(LPC_GPIO);
    //Board_UART_Init();
    DEBUGINIT();

    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 29);
    Chip_GPIO_SetPinState(LPC_GPIO, 0, 29, true);

    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 31);
    Chip_GPIO_SetPinState(LPC_GPIO, 0, 31, true);




#endif
    // Set the LED to the state of "On"
    Board_LED_Set(0, false);
#endif
#endif

#if defined (__MULTICORE_MASTER_SLAVE_M0SLAVE) || \
    defined (__MULTICORE_MASTER_SLAVE_M4SLAVE)
    boot_multicore_slave();
#endif




    //Board_UARTPutSTR(pinState);

    spi_pin_mux();
    spi_master_init();
    initLSM();

    info_from_LSM = magReadByte(addressToRead);
    DEBUGOUT("Mag who am I register: %x\r\n", info_from_LSM);
    info_from_LSM = xgReadByte(WHO_AM_I_XG);
    DEBUGOUT("Accel/Gyro Who am I register: %x\r\n", info_from_LSM);
    calibrate(true);
    calibrateMag(true);


    while(1)
    {

    	readAccel();
    	readGyro();
    	readMag();
    	DEBUGOUT("Heading: %f\n\r", heading(mx,my,mz));
    	DEBUGOUT("pitch:  %f\n\r", pitch(ax, ay, az));
    	DEBUGOUT("roll: %f\n\r", roll(ax, ay, az));
    	DEBUGOUT("Accelerometer raw %i, %i, %i\n\r", calcAccel(ax)/1000.0, calcAccel(ay)/1000.0, calcAccel(az)/1000.0);
    	DEBUGOUT("Gyroscope  %i, %i, %i\n\r", calcGyro(gx)/1000.0, calcGyro(gy)/1000.0, calcGyro(gz)/1000.0);
    	//DEBUGOUT("Magnetometer reading x,y,z %i, %i, %i\n\r", mx, my, mz);
    	for(int i = 0; i<10000000; i++)
    	{
    		i++;
    		i--;
    		i++;
    	}
    }




   /* volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1)
    {
    	i++;
    }
    return 0 ;*/
}
