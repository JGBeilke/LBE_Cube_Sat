/*
 * By Jarred Beilke
 * 9/16/16
 * Adapted from the generous example code provided by NXP
 *
 */

#include "chip.h"
#include "board.h"
#include "spi_common_5410x.h"
//Parts of this code are specific to communication with the LSM9DS1
//Some definitions from the spi example for readability.
//At this time the SPI1 will be set up as default.

#define SPI0IRQHANDLER                 SPI0_IRQHandler
#define LPC_SPI0PORT                   LPC_SPI0
#define LPC_SPI0IRQNUM                 SPI0_IRQn
#define SPI1IRQHANDLER                 SPI1_IRQHandler
#define LPC_SPI1PORT                   LPC_SPI1
#define LPC_SPI1IRQNUM                 SPI1_IRQn

/* Flags For Determing When master transfers end */
static volatile bool mEnd;
//this is the clock rate from the example may need to be adjusted
//depending on slave devices
#define LPCMASTERCLOCKRATE                  (4000000)
//transfer size is the size of each transfer, in this case 16 bits are used
uint32_t transferSize = 16;
char rv[16];
//spi setup structure
SPI_CFGSETUP_T spiSetup;

/* Buffer size is the total number of bytes to be transferred
each cell of the buffer array constitutes 16 bits to be transferred */
#define BUFFER_SIZE 1

/* Single transmit and receive buffers */
uint16_t masterRXBuffer[BUFFER_SIZE], masterTXBuffer[BUFFER_SIZE];

/* SPI master transfer descriptor */
SPIM_XFER_T spiMasterXfer;
//Pin configuration for SPI0 and SPI1 includes one chip select others can be uncommented SPI1 is
//chosen as default at this time

void spi_pin_mux()
{
    /* Connect the SPI0 signals to port pins
       Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI0_SCK
       Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI0_MOSI
       Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI0_MISO
       Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI0_SSEL0*/
    //Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 15, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)); //SPI0 SSEL1
    //Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	//SPI0 SSEL2
    //Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)); //SPI0 SSEL3

    //Connect the SPI1 signals to port pins
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 15,  (IOCON_FUNC4 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI1_SSEL0
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 6,   (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI1_SCK
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 7,   (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI1_MOSI
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 14,  (IOCON_FUNC4 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI1_MISO
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 4,  (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI1_SSEL1
    //Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 3,  (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI1_SSEL2
    //Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 2,  (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	// SPI1_SSEL3

    /*
     *For both SPI0 and SPI0 there are other pins that may serve the same function if the pins configured
     * here are need for other functions in the future
     */
}

/* Callback for master SPI transfers */
static int master_cb(SPIM_EVENT_T event, struct SPIM_XFER *xfer)
{
	switch (event) {
	case SPIM_EVT_SSELASSERT:
		/* Handle Master select here */
		break;

	case SPIM_EVT_SSELDEASSERT:
		/* Handle Master de-select here */
		break;

	case SPIM_EVT_TXDONE:
		/* Handle Master transfer done here */
		break;

	case SPIM_EVT_RXDONE:
		/* Handle Master receive done here */
		break;
	}
	return 0;
}

//Initialize SPI For
void spi_master_init()
{


    SPIM_DELAY_CONFIG_T masterDelay;

    /* Initialize SPI controller */
    Chip_SPI_Init(LPC_SPI1);

    /* Call to initialize first SPI controller for mode0, master mode,
       MSB first */
    Chip_SPI_Enable(LPC_SPI1);
    spiSetup.master = 1;
    spiSetup.lsbFirst = 0;	//most significant bit first
    
    Chip_SPI_ConfigureSPI(LPC_SPI1, &spiSetup);

    /* Setup master controller SSEL0 for active low select */
    Chip_SPI_SetCSPolLow(LPC_SPI1, 0);
    Chip_SPI_SetCSPolLow(LPC_SPI1, 1);

    /* Setup clock rate */
    Chip_SPIM_SetClockRate(LPC_SPI1, LPCMASTERCLOCKRATE);
		

    /* Setup master delay (all chip selects) */
    masterDelay.PreDelay = 0xD;
    masterDelay.PostDelay = 0xD;
    masterDelay.FrameDelay = 0xD;
    masterDelay.TransferDelay = 0xD;
    Chip_SPIM_DelayConfig(LPC_SPI1, &masterDelay);

    /*Set TX data size to 16 bits*/
    Chip_SPI_SetXferSize(LPC_SPI1, transferSize);

    /* For the SPI controller configured in master mode, enable SPI master interrupts
       for interrupt service. Do not enable SPI_INTENSET_TXDYEN. */
    Chip_SPI_EnableInts(LPC_SPI1, (SPI_INTENSET_RXDYEN |
      SPI_INTENSET_RXOVEN | SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN |
      SPI_INTENSET_SSDEN));

}

void SPI1IRQHANDLER(void)
{
    uint32_t ints = Chip_SPI_GetPendingInts(LPC_SPI1);

    /* Handle SPI master interrupts only */
    if ((ints & (SPI_INTENSET_RXDYEN | SPI_INTENSET_RXOVEN |
		 SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN |
		 SPI_INTENSET_SSDEN)) != 0) {
	/* SPI master handler */
	Chip_SPIM_XferHandler(LPC_SPI1, &spiMasterXfer);
    }
}



/*
 * This function "reads" data from the megnetometer of the LSM chip by transferring an address then
 * receiving the information from that address. Addresses can be found on page 38 of
 * the LSM9DS1 datasheet This function will need to be optimized as we learn what
 * needs to be called every time and what can remain from read to read.
 *
 */

uint8_t  magReadByte(uint8_t regAddress)
{
    uint8_t SSEL0 = 0, rv = 0; //chip select for magnetometer, return value
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)
    
    //mode is different for lsm and rfm
    spiSetup.mode = SPI_CLOCK_MODE1; //CPHA0 (capture on rising edge) CPOL1 (base     value is high)
    
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler
    //Initialize the TX buffer with address to be read and RX buffer to zero
    //Set msb of TXbuff to 1 and next bit to 0 for single read
    masterTXBuffer[0] = (((uint16_t)regAddress) << 8) | 0x8000;
//DEBUGOUT("MasterTXBuffer %x\n\r", masterTXBuffer[0]);
    masterRXBuffer[0] = 0;

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = masterTXBuffer;
    spiMasterXfer.txCount = 1;
    spiMasterXfer.rxBuff = masterRXBuffer;
    spiMasterXfer.rxCount = 1;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL0;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
	__WFI();
    }

//DEBUGOUT("RX buffer before 0x%X\n\r", masterRXBuffer[0]);
    
    rv = masterRXBuffer[0] & 0xFF;
    return rv;
  
}


/*read multiple bytes from the magnetometer given a starting address, a data
 *array, and a count of the number of items to be read. Size of Data
 *array must be equal number of items to be read.
 */
void magReadBytes(uint8_t regAddress, uint8_t *data, uint8_t count)
{
    uint16_t txBuffer[count], rxBuffer[count];
    uint8_t SSEL0 = 0; //chip select for magnetometer, return value
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)

    //mode is different for lsm and rfm
    spiSetup.mode = SPI_CLOCK_MODE1; //CPHA0 (capture on rising edge) CPOL1 (base     value is high)
    
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler
    //Initialize the TX buffer with address to be read and RX buffer to zero
    //Set bit 0(msb) of TXbuff to 1 and bit 1 to 1 for multiple read.
    txBuffer[0] = (((uint16_t)regAddress) << 8) | 0xC000;
//DEBUGOUT("MasterTXBuffer %x\n\r", masterTXBuffer[0]);
    for(int i = 0; i < count; i++)
    {
	rxBuffer[i] = 0;
	if(i != 0)
	    txBuffer[i] = 0;
    }
    

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = txBuffer;
    spiMasterXfer.txCount = count;
    spiMasterXfer.rxBuff = rxBuffer;
    spiMasterXfer.rxCount = count;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL0;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
	__WFI();
    }
    for(int i = 0; i < count; i++)
    {
	data[i] = rxBuffer[i] & 0xFF;
    }
}

/*write single byte into the magnetometer given the register address
 *and the data to be written.
 *There is a seperate function to write to the accel/gyro
 */
void magWriteByte(uint8_t regAddress, uint8_t data)
{
    uint8_t SSEL0 = 0; //chip select for magnetometer
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)

    //mode is different for lsm and rfm
    spiSetup.mode = SPI_CLOCK_MODE1; //CPHA0 (capture on rising edge) CPOL1 (base     value is high)
    
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler
    //Initialize the TX buffer with address to be read and RX buffer to zero
    //Set msb of TXbuff to 0 and next bit to 0 for single write
    masterTXBuffer[0] = (((uint16_t)regAddress) << 8) | 0x0000 | (uint16_t)data;
//DEBUGOUT("MasterTXBuffer %x\n\r", masterTXBuffer[0]);
    masterRXBuffer[0] = 0;

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = masterTXBuffer;
    spiMasterXfer.txCount = 1;
    spiMasterXfer.rxBuff = masterRXBuffer;
    spiMasterXfer.rxCount = 1;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL0;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
	__WFI();
    }
//DEBUGOUT("Write completed.");
    
}

uint8_t  xgReadByte(uint8_t regAddress)
{
    uint8_t SSEL1 = 1, rv = 0; //chip select for magnetometer, return value
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)

    //mode is different for lsm and rfm
    spiSetup.mode = SPI_CLOCK_MODE1; //CPHA0 (capture on rising edge) CPOL1 (base     value is high)
    
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler
    //Initialize the TX buffer with address to be read and RX buffer to zero
    //Set msb of TXbuff to 1 and next bit to 0 for single read
    masterTXBuffer[0] = (((uint16_t)regAddress) << 8) | 0x8000;
//DEBUGOUT("MasterTXBuffer %x\n\r", masterTXBuffer[0]);
    masterRXBuffer[0] = 0;

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = masterTXBuffer;
    spiMasterXfer.txCount = 1;
    spiMasterXfer.rxBuff = masterRXBuffer;
    spiMasterXfer.rxCount = 1;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL1;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
	__WFI();
    }

//DEBUGOUT("RX buffer before 0x%X\n\r", masterRXBuffer[0]);
    
    rv = masterRXBuffer[0] & 0xFF;
    return rv;
  
}

/*read multiple bytes from the accelerometer or gyroscope given a starting
 *address, a dataarray, and a count of the number of items to be read.
 *Size of data array must be equal to number of items to be read.
 */
void xgReadBytes(uint8_t regAddress, uint8_t *data, uint8_t count)
{
    uint16_t txBuffer[count], rxBuffer[count];
    uint8_t SSEL1 = 1; //chip select for magnetometer, return value
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)

    //mode is different for lsm and rfm
    spiSetup.mode = SPI_CLOCK_MODE1; //CPHA0 (capture on rising edge) CPOL1 (base     value is high)
    
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler
    //Initialize the TX buffer with address to be read and RX buffer to zero
    //Set bit 0(msb) of TXbuff to 1 the increasing address function is
    //controlled by the IF_ADD_INC register
    txBuffer[0] = (((uint16_t)regAddress) << 8) | 0x8000;
//DEBUGOUT("MasterTXBuffer %x\n\r", masterTXBuffer[0]);
    for(int i = 0; i < count; i++)
    {
	rxBuffer[i] = 0;
	if(i != 0)
	    txBuffer[i] = 0;
    }
    
    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = txBuffer;
    spiMasterXfer.txCount = count;
    spiMasterXfer.rxBuff = rxBuffer;
    spiMasterXfer.rxCount = count;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL1;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
	__WFI();
    }
    for(int i = 0; i < count; i++)
    {
	data[i] = rxBuffer[i] & 0xFF;
    }
}

/*write single byte into the accel/gyro given the register address
 *and the data to be written.
 */
void xgWriteByte(uint8_t regAddress, uint8_t data)
{
    uint8_t SSEL1 = 1; //chip select for magnetometer
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler

    //mode is different for lsm and rfm
    spiSetup.mode = SPI_CLOCK_MODE1; //CPHA0 (capture on rising edge) CPOL1 (base     value is high)
    
    //Initialize the TX buffer with address to be read and RX buffer to zero
    //Set msb of TXbuff to 0 and next bit to 0 for single write
    masterTXBuffer[0] = (((uint16_t)regAddress) << 8) | 0x0000 | (uint16_t)data;
//DEBUGOUT("MasterTXBuffer %x\n\r", masterTXBuffer[0]);
    masterRXBuffer[0] = 0;

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = masterTXBuffer;
    spiMasterXfer.txCount = 1;
    spiMasterXfer.rxBuff = masterRXBuffer;
    spiMasterXfer.rxCount = 1;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL1;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
	__WFI();
    }
//DEBUGOUT("Write completed.");
    
}


/*write single byte into the inAir9B given the register address
 *and the data to be written. Though the name "Write" may be ambiguous it will 
 *will be kept to make porting the inAir9B libraries easier.
 */
void Write(uint8_t regAddress, uint8_t data)
{
    uint8_t SSEL3 = 3; //chip select for magnetometer
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler

    //mode is different for lsm and rfm
    spiSetup.mode = SPI_CLOCK_MODE0; //CPHA0 (capture on rising edge) CPOL0 (base     value is low?)
    
    //Initialize the TX buffer with address to write and data to write, also
    //intialize RX buffer to zero
    //Set msb of TXbuff to 1 for write
    masterTXBuffer[0] = (((uint16_t)regAddress) << 8) | 0x8000 | (uint16_t)data;
//DEBUGOUT("MasterTXBuffer %x\n\r", masterTXBuffer[0]);
    masterRXBuffer[0] = 0;

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = masterTXBuffer;
    spiMasterXfer.txCount = 1;
    spiMasterXfer.rxBuff = masterRXBuffer;
    spiMasterXfer.rxCount = 1;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL3;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
	__WFI();
    }
//DEBUGOUT("Write completed.");
    
}


/*Read single byte from the inAir9B given the register address.
 *Though the name "Read" may be ambiguous it will be kept to make 
 *porting the inAir9B libraries easier.
 */
uint8_t Read(uint8_t regAddress)
{
    uint8_t SSEL3 = 3, rv = 0; //chip select for magnetometer, return value
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler

    //mode is different for lsm(mode1) and rfm(mode0)
    spiSetup.mode = SPI_CLOCK_MODE0; //CPHA0 (capture on rising edge) CPOL0 (base     value is low?)
    
    //Initialize the TX buffer with address to write and data to write, also
    //intialize RX buffer to zero
    //Set msb of TXbuff to 0 for read
    masterTXBuffer[0] = (((uint16_t)regAddress) << 8) | 0x0000;
//DEBUGOUT("MasterTXBuffer %x\n\r", masterTXBuffer[0]);
    masterRXBuffer[0] = 0;

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = masterTXBuffer;
    spiMasterXfer.txCount = 1;
    spiMasterXfer.rxBuff = masterRXBuffer;
    spiMasterXfer.rxCount = 1;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL3;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
	__WFI();
    }

    rv = masterRXBuffer[0] & 0xFF;
    return rv;
//DEBUGOUT("Write completed.");
    
}

/*Read multiple bytes from the inAir9B given the starting register address, a
 *data array and a count of the number of items to be read. The size of the data 
 *array must be equal to the number of items to be received(count)
 *Though the name "Read" may be ambiguous it will be kept to make 
 *porting the inAir9B libraries easier.
 */

void multRead(uint8_t regAddress, uint8_t *data, uint8_t count)
{
    uint16_t txBuffer[count], rxBuffer[count];
    uint8_t SSEL3 = 3; //chip select for magnetometer, return value
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler

    //mode is different for lsm(mode1) and rfm(mode0)
    spiSetup.mode = SPI_CLOCK_MODE0; //CPHA0 (capture on rising edge) CPOL0 (base     value is low?)
    
    //Initialize the TX buffer with starting address to be read from, also
    //intialize RX buffer to zero
    //Set msb of TXbuff to 0 for read
    txBuffer[0] = (((uint16_t)regAddress) << 8) | 0x0000 | (uint16_t)data;
    for(int i = 0; i<count; i++)
    {
	rxBuffer[i] = 0;
    }
    

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = txBuffer;
    spiMasterXfer.txCount = count;
    spiMasterXfer.rxBuff = rxBuffer;
    spiMasterXfer.rxCount = count;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL3;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
    	__WFI();
    }

    for(int i = 0; i < count; i++)
    {
    	data[i] = rxBuffer[i] & 0xFF;
    }
    
//DEBUGOUT("Write completed.");
    
}

/*Write multiple bytes from the inAir9B given the starting register address, a
 *data array and a count of the number of items to be sent. The size of the data 
 *array must be equal to the number of items to be sent(count)
 *Though the name "Write" may be ambiguous it will be kept to make 
 *porting the inAir9B libraries easier.
 */

void multWrite(uint8_t regAddress, uint8_t *data, uint8_t count)
{
    uint16_t txBuffer[count], rxBuffer[count];
    uint8_t SSEL3 = 3, rv = 0; //chip select for magnetometer, return value
    volatile uint8_t *mstate; //pointer to state of master spi (the lpc chip)
    NVIC_EnableIRQ(LPC_SPI1IRQNUM);//enable interupt handler

    //mode is different for lsm(mode1) and rfm(mode0)
    spiSetup.mode = SPI_CLOCK_MODE0; //CPHA0 (capture on rising edge) CPOL0 (base     value is low?)
    
    //Initialize the TX buffer with the starting address to write and data to write, also
    //intialize RX buffer to zero
    //Set msb of TXbuff to 1 for write
    txBuffer[0] = (((uint16_t)regAddress) << 8) | 0x8000 | (uint16_t)data;
    for(int i = 1; i<count; i++)
    {
	rxBuffer[i] = data[i]; 
    }

    spiMasterXfer.cbFunc = master_cb;
    spiMasterXfer.state = SPIS_XFER_STATE_IDLE;
    spiMasterXfer.txBuff = txBuffer;
    spiMasterXfer.txCount = count;
    spiMasterXfer.rxBuff = rxBuffer;
    spiMasterXfer.rxCount = count;
    mstate = &spiMasterXfer.state;

    /* Setup master transfer options - 16 data bits per transfer, EOT, EOF */
    spiMasterXfer.options =
	SPIM_XFER_OPTION_SIZE(16) | SPIM_XFER_OPTION_EOT |  
	SPIM_XFER_OPTION_EOF |  0;

    spiMasterXfer.sselNum = SSEL3;

    Chip_SPI_FlushFifos(LPC_SPI1);

    /* Start master transfer */
    Chip_SPIM_Xfer(LPC_SPI1, &spiMasterXfer);

    while (*mstate != SPIM_XFER_STATE_DONE)
    {
    	__WFI();
    }

    
//DEBUGOUT("Write completed.");
    
}
