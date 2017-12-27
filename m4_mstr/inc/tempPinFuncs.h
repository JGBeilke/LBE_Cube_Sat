#include "chip.h"
#include "board.h"

void initGPIOblock();

void setPinDigitalOut(const uint8_t portn, const uint8_t pinn);

void setPinDigitalIn(const uint8_t portn, const uint8_t pinn);

void digitalWrite(unsigned int portn, unsigned int pinn, bool staten);

bool digitalRead(unsigned int portn,unsigned int pinn);

void setPinPWM(unsigned int portn, uint8_t ppin, uint8_t pSCTpin, uint8_t index);

void startPwm(uint8_t pindex, uint32_t pticks );

//void stopPwm();

void sctPwmInit(uint32_t pfrequency);
