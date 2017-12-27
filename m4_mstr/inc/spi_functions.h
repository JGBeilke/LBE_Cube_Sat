#include "chip.h"
#ifndef _rv
#define _rv

void spi_pin_mux();

void spi_master_init();

uint8_t magReadByte();

void magReadBytes();

void magWriteByte();

uint8_t xgReadByte();

void xgReadBytes();

void xgWriteByte();

void multRead();

void multWrite();

uint8_t Read();

void Write();
extern char rv[16];
#endif
