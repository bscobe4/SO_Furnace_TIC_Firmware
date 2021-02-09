//
//                   MSP430G2553
//                 -----------------
//            /|\ |             P2.3|->   ADC !RST (GPIO) [Active Low]
//             |  |                 |
//             ---|RST          P1.5|->   !CS (GPIO) [Active Low]
//                |                 |
//       SCL   -->|P1.6         P2.5|<-   !DRDY (GPIO) [Active Low]
//                |                 |
//       SDA  <-->|P1.7         P2.4|->   START/SYNC (GPIO)
//                |                 |
// I2CAddr_0   -->|P2.0         P1.2|->   Data Out (UCA0SIMO)
//                |                 |
// I2CAddr_1   -->|P2.1         P1.1|<-   Data In (UCA0SOMI)
//                |                 |
// I2CAddr_2   -->|P2.2         P1.4|->   Serial Clock Out (UCA0CLK)
//                |                 |
//********************************************************************************************

#include <msp430.h>
#include <stdint.h>

//******************************************************************************
// Definitions and Commands ****************************************************
//******************************************************************************

#define DUMMY     0x00 //Does this need to be 0xFF instead?

#define BAUDRATE_0  0x41  //UCBRA0 byte 0 (baudrate = SMCLK/(UCBRA0+UCBRA1) ) At 8 MHZ this is 9600 baud
#define BAUDRATE_1  0x06  //UCBRA1 byte 1
#define CONVERSION_DELAY    1000000    //Delay needed after conversion starts or has to restart following changes to DATARATE and INPMUX registers
#define SFOCAL_DELAY        4800000    //Delay needed after self offset calibration
#define RESET_DELAY         1000000    //Delay needed after ADC is reset

#define CS_OUT    P1OUT  //P1.5 CS Pin config
#define CS_DIR    P1DIR
#define CS_PIN    BIT5

#define I2CA_0  BIT0        //I2C Address bits
#define I2CA_1  BIT1
#define I2CA_2  BIT2

#define NOP                 0x00 //No operation
#define START_CMD           0x09 //Start conversion
#define STOP_CMD            0x0B //Stop conversion
#define RDATA_CMD           0x13 //Read data
#define SFOCAL_CMD          0x19 //Self offset calibration

#define CMD_LENGTH          1 //Length of single byte SPI commands

#define RDATA_TX_LENGTH     2 //This is a workaround so that the MSP430 is Tx'ing while
                              //the ADC is sending the previous byte in the data-holding register
#define RDATA_RX_LENGTH     3 //Don't forget this affects the burnout detect code too. Check to make sure
                              //It doesn't get screwed up if this is set to 4 or 5 to accomodate
                              //Status or CRC bytes

#define RREG_TX_LENGTH      3 //Number of bytes transmitted in RREG command
#define RREG_RX_LENGTH      1 //Number of bytes expected to be received from ADC after RREG command (this assumes that the second byte is 0x00)
#define RREG_RX_LENGTH_ALL  10//Number of bytes expected to be received from ADC after RREG command (this assumes that the second byte is 0x0A

#define WREG_TX_LENGTH      3

#define STATUS_WORD_LENGTH  4  //Length of output data status word
//#define CCMM_DATA_LENGTH    //48 //Output data buffer for CCMM. 9 integers plus 4 byte bitfield

#define MAX_BUFFER_SIZE     10 //Maximum size of SPI TX and RX buffers

#define FS_1                0xFF //Full-scale ADC measurement bits 1-3
#define FS_2                0xFF
#define FS_3                0x7F

//******I2C Slave definitions******
//#define SLAVE_ADDR  0x09 //********still need to base this on address pins ********

#define CCMM_RESET_CMD      BIT0    //Bit 0 high requires MSP430 to run reset command
#define CCMM_SFOCAL_CMD     BIT1    //Bit 1 high requires MSP430 to run self offset calibration
#define CCMM_BURNOUT        BIT2    //Bit 2 high requires MSP430 to run burnout detection

#define CMD_LENGTH          1       //Length of I2C commands sent by CCMM to MSP430

#define CCMM_RX_BUFFER_SIZE 2       //Size of buffer used to store data received from CCMM. Not used
#define CCMM_TX_BUFFER_SIZE 40      //Size of data buffer transmitted to CCMM. Assumes 9 4-bit integers plus 4 byte bitfield

/*Command Definitions */

/*
 *     RDATA: Tx -> 0x12 or 0x13   Rx -> 3 bytes (unless SENDSTAT or CRC bits of SYS high)
 *
 *     RREG [Status]:   Tx -> 0x21 0x00 (0x00) Rx -> 1 byte (default 0x80)
 *
 *     RREG [Datarate]: Tx -> 0x24 0x00 (0x00) Rx -> 1 byte (default 0x14)
 *
 *     WREG [Datarate]: TX -> 0x44 0x00 0x94 (ADC Setup)
 */

/*Command Buffers*/
uint8_t SLAVE_ADDR = 0;             //Global variable to store I2C slave address

uint8_t started = 0;                //Toggled when power-on startup tasks have been performed

uint8_t start_cmd [CMD_LENGTH] = {0x09};    //Command byte sent to the ADC to start conversion. Needs a delay afterwards if sent using CMD()
uint8_t sfocal_cmd [CMD_LENGTH] = {0x19};   //Command byte sent to ADC to perform self offset calibration. Needs a delay afterwards if sent using CMD()


uint8_t RData_Tx [RDATA_TX_LENGTH] = {0x12}; //Command byte sent to the ADC to request that data in the ADC data holding register be sent to the MSP430
uint8_t RData_Rx_Ch0 [RDATA_RX_LENGTH] = {0};//Buffers to store data read from the ADC for channels 0 through 5.
uint8_t RData_Rx_Ch1 [RDATA_RX_LENGTH] = {0};
uint8_t RData_Rx_Ch2 [RDATA_RX_LENGTH] = {0};
uint8_t RData_Rx_Ch3 [RDATA_RX_LENGTH] = {0};
uint8_t RData_Rx_Ch4 [RDATA_RX_LENGTH] = {0};
uint8_t RData_Rx_Ch5 [RDATA_RX_LENGTH] = {0};
uint8_t RData_Rx_AVDD [RDATA_RX_LENGTH] = {0}; //Analog supply monitor data
uint8_t RData_Rx_DVDD [RDATA_RX_LENGTH] = {0}; //Digital supply monitor data
uint8_t RData_Rx_Temperature [RDATA_RX_LENGTH] = {0}; //Internal temperature data
uint8_t RReg_Datarate_Tx [RREG_TX_LENGTH] = {0x24, 0x00}; //Need an additional 0x00?
uint8_t RReg_Datarate_Rx [RREG_RX_LENGTH] = {0};
uint8_t RReg_All_Tx [RREG_TX_LENGTH] = {0x20, 0x0A}; //Command sent to ADC to request the values of 10 registers starting at address 0x00
uint8_t RReg_All_Rx [RREG_RX_LENGTH_ALL] = {0};      //Buffer to store value of ADC buffers 0x00 through 0x09

uint8_t Status_bitfield [STATUS_WORD_LENGTH] = {0}; //Byte0: Bits[0:4] Burnout status channels 0 to 4
                                                    //       Bit5: Reset command acknowledged by MSP430
                                                    //       Bit6: Self calibration acknowledged by MSP430
                                                    //       Bit7: Burnout detect acknowledged by MSP430
                                                    //Byte1:
                                                    //Byte2:
                                                    //Byte3:
uint8_t ADC_Setup [WREG_TX_LENGTH] = {0x44, 0x00, 0x94}; //Sets ADC DATARATE register to required startup condition
uint8_t En_PGA_32 [WREG_TX_LENGTH] = {0x43, 0x00, 0x0D}; //Enable PGA and set gain to 32V/V
uint8_t En_PGA_4 [WREG_TX_LENGTH] = {0x43, 0x00, 0x0A}; //Enable PGA and set gain to 4V/V
uint8_t Chop_Mode_Off [WREG_TX_LENGTH] = {0x44, 0x00, 0x14}; //Disable global chop
uint8_t Burnout_On [WREG_TX_LENGTH] = {0x49, 0x00, 0xD0}; //Turn on burnout current sources (1uA)
uint8_t Burnout_Off [WREG_TX_LENGTH] = {0x49, 0x00, 0x10}; //Turn off burnout current sources
uint8_t Channel_0 [WREG_TX_LENGTH] = {0x42, 0x00, 0x01};        //Set input mux to channel 0
uint8_t Channel_1 [WREG_TX_LENGTH] = {0x42, 0x00, 0x23};        //Set input mux to channel 1
uint8_t Channel_2 [WREG_TX_LENGTH] = {0x42, 0x00, 0x45};        //Set input mux to channel 2
uint8_t Channel_3 [WREG_TX_LENGTH] = {0x42, 0x00, 0x67};        //Set input mux to channel 3
uint8_t Channel_4 [WREG_TX_LENGTH] = {0x42, 0x00, 0x89};        //Set input mux to channel 4
uint8_t Channel_5 [WREG_TX_LENGTH] = {0x42, 0x00, 0xAB};        //Set input mux to channel 5
uint8_t V_analog_mon [WREG_TX_LENGTH] = {0x49,0x00,0x70};       //Set analog supply voltage monitor
uint8_t V_digital_mon [WREG_TX_LENGTH] = {0x49,0x00,0x90};      //Set digital supply voltage monitor
uint8_t Chip_temperature [WREG_TX_LENGTH] = {0x49,0x00,0x50};   //Set On-Chip temperature monitor

//******I2C Command******
uint8_t CCMM_CMD[CMD_LENGTH] = {0}; //Buffer to store CCMM command


//******************************************************************************
// SPI State Machine ***********************************************************
//******************************************************************************

typedef enum SPI_ModeEnum{
    IDLE_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    TIMEOUT_MODE
} SPI_Mode;

SPI_Mode MasterMode = IDLE_MODE; //Variable to track current state of state machine

//uint8_t TransmitRegAddr = 0; //Need this if only doing whole buffers?

/*Variables to handle Rx and Tx buffers*/

uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};   //Buffer to receive data in ISR
uint8_t RXByteCtr = 0;                          //Number of bytes left to receive
uint8_t ReceiveIndex = 0;                       //Index of next byte to be received by ReceiveBuffer
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};  //Buffer to transmit data in ISR
uint8_t TXByteCtr = 0;                          //Number of bytes left to transmit
uint8_t TransmitIndex = 0;                      //Index of next byte to be transmitted by TransmitBuffer


/*Function Declarations*/

/* Write to ADC register(s)
 *
 * cmd: sequence of bytes to execute WREG command. See commands above.
 * tx_count: number of bytes to be transmitted. If one register is being read, this will be
 *           three bytes. Note that the number of bytes that the ADC expects to receive
 *           is determined by the second byte of cmd*/
SPI_Mode WREG(uint8_t *cmd, uint8_t tx_count);

/*Requests data from the ADC. The ADC will send whatever is in the conversion buffer,
 * which is measured from whatever + and - inputs are defined by the input mux.
 * By default, the ADC should only send 3 bytes unless the status and crc bytes are enabled
 * by setting their respective bits high in the SYS register, in which case it could send 4 or 5
 * bytes. This function only expects 3 bytes, so data will be truncated if the status byte is turned on.*/
SPI_Mode RDATA();

/*Read Register(s) from ADC
 * cmd: sequence of bytes to execute RREG command. See commands above.
 * tx_count: number of bytes in the command. Reading 1 register needs 3 bytes.
 * rx_count: number of registers that will be read. Note that this is defined
 *           for the ADC by the second byte of cmd. rx_count is the number of
 *           bytes read back by the MSP430*/
SPI_Mode RREG(uint8_t *cmd, uint8_t tx_count, uint8_t rx_count);

/*Send a single command byte to the CCMM
 * cmd: value of command byte*/
SPI_Mode CMD(uint8_t *cmd);

/*Copy an array into another array, starting with the lowest-value index
 * source:  array that is being copied
 * dest:    destination of copied array
 * count:   how many bytes are to be copied from source to dest */
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count); //Copy source array to dest array

/*Copies and array into another array, starting at the lowest index of the source and a chosen
 *  index of the destination.
 *  dest: destination of copied array
 *  source: array that is being copied
 *  count: how many bytes are to be copied from source to dest
 *  startIndex: index of destination array at which the source array will start being copied.
 */
void CopytoDataBuffer(uint8_t *dest, uint8_t *source, uint8_t count, uint8_t startIndex);

/* Places byte val into the SPI UCA0TX buffer*/
void SendUCA0Data(uint8_t val); //Add val to UCA0TXBUF

/*Performs self offset calibration, burnout detect, and reads registers 0x00 to 0x09 from the ADC*/
void PowerOnSetup();

/*Sends self offset calibration command to ADC and gives a delay for the procedure to finish*/
void SelfCalibration();

/*Sends command to turn on burnout current sources and measures channels 0 through 4. If a channel measurement
 * is EQUAL to the full-scale values defined above, then the corresponding bits 0 through 4 of Byte 1 in the
 * output status word will be set high. Turns off burnout current sources when finished.
 */
void BurnoutDetect();

/*  Sends commands to set PGA gain to 32 V/V, turn on global-chop and turn off burnout current sources
 * For each channel 1 through 5, analog and digital supplies, and on-chip temperature sensor:
 *  +Set input mux to appropriate channel or SYS register to internal measurement
 *  +Read three bytes of data and copy into corresponding channel data buffer on MSP430
 */
void ReadADCData();

/*Hold Reset pin low and then bring high after appropriate delay. Delay to allow startup
 * Send command to start data conversion
 * Setup ADC basic parameters (turn on global chop is the only difference from power on default)
 */
void ADCReset();

/*Sends a few commands to make sure the SPI interface is working. Only use for debugging*/
void SPI_Debug();

/*Checks if measured voltage is EQUAL to full scale values defined above*/
uint8_t BurnoutCheck(uint8_t *data);


/*Function Definitions*/

SPI_Mode WREG(uint8_t *cmd, uint8_t tx_count){

    MasterMode = TX_DATA_MODE;

    CopyArray(cmd, TransmitBuffer, tx_count);

    TXByteCtr = tx_count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    CS_OUT &= ~(CS_PIN);    //Set CS pin low

    SendUCA0Data(TransmitBuffer[TransmitIndex++]);
    TXByteCtr--;

    __bis_SR_register(CPUOFF + GIE);              // Enter LPM0 w/ interrupts
    CS_OUT |= CS_PIN;

    return MasterMode;
}

SPI_Mode RDATA()
{
    MasterMode = TX_DATA_MODE;

    CopyArray(RData_Tx, TransmitBuffer, RDATA_TX_LENGTH);   //Copy RDATA command byte to tx buffer

    TXByteCtr = RDATA_TX_LENGTH;                            //Set tx and rx counters
    RXByteCtr = RDATA_RX_LENGTH;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    CS_OUT &= ~(CS_PIN);                //Set CS Low

    SendUCA0Data(TransmitBuffer[TransmitIndex++]);
    TXByteCtr--;

    __bis_SR_register(CPUOFF + GIE);          // Enter LPM0 w/ interrupts
    CS_OUT |= CS_PIN;                   //Set CS High

    return MasterMode;
}

SPI_Mode RREG(uint8_t *cmd, uint8_t tx_count, uint8_t rx_count){
    MasterMode = TX_DATA_MODE;

    CopyArray(cmd, TransmitBuffer, tx_count);  //Copy RREG command bytes into tx buffer

    TXByteCtr = tx_count;                      //Set counters
    RXByteCtr = rx_count;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    CS_OUT &= ~(CS_PIN);        //Set CS Low

    SendUCA0Data(TransmitBuffer[TransmitIndex++]);  //Add first byte to be transmitted to UCA0TXBUF
    TXByteCtr--;

     __bis_SR_register(CPUOFF + GIE);              // Enter LPM0 w/ interrupts

     CS_OUT |= CS_PIN;          //Set CS High

    return MasterMode;
}

SPI_Mode CMD(uint8_t *cmd){
    MasterMode = TX_DATA_MODE;

    CopyArray(cmd, TransmitBuffer, 1);

    TXByteCtr = 1;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    CS_OUT &= ~(CS_PIN);        //Set CS Low

    SendUCA0Data(TransmitBuffer[TransmitIndex++]);
    TXByteCtr--;

     __bis_SR_register(CPUOFF + GIE);              // Enter LPM0 w/ interrupts

     CS_OUT |= CS_PIN;          //Set CS High

    return MasterMode;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

void CopytoDataBuffer(uint8_t *dest, uint8_t *source, uint8_t count, uint8_t startIndex){
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex + startIndex] = source[copyIndex];
    }
}

void SendUCA0Data(uint8_t val)
{
    while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
    UCA0TXBUF = val;
}

void PowerOnSetup(){

    SelfCalibration(); //ADC self offset calibration

    BurnoutDetect(); //Run Burnout Detect

    RREG(RReg_All_Tx, RREG_TX_LENGTH, RREG_RX_LENGTH_ALL);
    CopyArray(ReceiveBuffer, RReg_All_Rx, RREG_RX_LENGTH_ALL);
}

void SelfCalibration(){
    CMD(sfocal_cmd); //ADC self offset calibration
    __delay_cycles(SFOCAL_DELAY); //Need delay to let conversion restart
}

void BurnoutDetect(){

    uint8_t burnedOut = 0;

    WREG(En_PGA_32, WREG_TX_LENGTH); //Enable PGA and set gain to 32V/V
    WREG(Chop_Mode_Off, WREG_TX_LENGTH);
    WREG(Burnout_On, WREG_TX_LENGTH);

    WREG(Channel_0, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    burnedOut = BurnoutCheck(ReceiveBuffer);
    if(burnedOut == 1){
        Status_bitfield[0] |= BIT0;
    }

    WREG(Channel_1, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    burnedOut = BurnoutCheck(ReceiveBuffer);
    if(burnedOut == 1){
        Status_bitfield[0] |= BIT1;
    }

    WREG(Channel_2, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    burnedOut = BurnoutCheck(ReceiveBuffer);
    if(burnedOut == 1){
        Status_bitfield[0] |= BIT2;
    }

    WREG(Channel_3, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    burnedOut = BurnoutCheck(ReceiveBuffer);
    if(burnedOut == 1){
        Status_bitfield[0] |= BIT3;
    }

    WREG(Channel_4, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    burnedOut = BurnoutCheck(ReceiveBuffer);
    if(burnedOut == 1){
        Status_bitfield[0] |= BIT4;
    }

    WREG(Burnout_Off, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); // Necessary?
}

void ReadADCData(){
    WREG(En_PGA_32, WREG_TX_LENGTH); //Enable PGA and set gain to 32V/V
    WREG(ADC_Setup, WREG_TX_LENGTH);
    WREG(Burnout_Off, WREG_TX_LENGTH);

    WREG(Channel_0, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_Ch0, RDATA_RX_LENGTH);

    WREG(Channel_1, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_Ch1, RDATA_RX_LENGTH);

    WREG(Channel_2, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_Ch2, RDATA_RX_LENGTH);

    WREG(Channel_3, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_Ch3, RDATA_RX_LENGTH);

    WREG(Channel_4, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_Ch4, RDATA_RX_LENGTH);

    WREG(Channel_5, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_Ch5, RDATA_RX_LENGTH);

    WREG(V_analog_mon, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_AVDD, RDATA_RX_LENGTH);

    WREG(V_digital_mon, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_DVDD, RDATA_RX_LENGTH);


    WREG(En_PGA_4, WREG_TX_LENGTH); //PGA gain is capped at 4v/v to remain within absolute maximum electrical characteristics
    WREG(Chip_temperature, WREG_TX_LENGTH);
    __delay_cycles(CONVERSION_DELAY); //Need delay to let conversion restart
    RDATA();
    CopyArray(ReceiveBuffer, RData_Rx_Temperature, RDATA_RX_LENGTH);

    WREG(Burnout_Off, WREG_TX_LENGTH);//Stop reading voltage and temperature monitors (same as burnout off)

    __delay_cycles(CONVERSION_DELAY);
    //P1OUT &= ~(BIT0);
}

uint8_t BurnoutCheck(uint8_t *data){
    uint8_t isBurnedOut = 0;
    if((data[0] == FS_3) && (data[1] == FS_2) && (data[2] == FS_1)){//switch to bitwise & AND ?
        isBurnedOut = 1;
    }

    return isBurnedOut;
}

void ADCReset(){
    //P1OUT &= ~BIT0;                            //LED off
    P2OUT &= ~BIT3;                           // With SPI signals initialized,
    __delay_cycles(RESET_DELAY); //Half this?      //
    P2OUT |= BIT3;                            // reset slave
    __delay_cycles(RESET_DELAY);                   // Wait for ADC to initialize (4096*tclk = 1ms)

    CMD(start_cmd);             //Start conversion

    WREG(ADC_Setup, WREG_TX_LENGTH); //Setup ADC basic parameters
    __delay_cycles(CONVERSION_DELAY); //Not sure if this delay is needed in addition to the one after selfcalibration

    //RREG(RReg_All_Tx, RREG_TX_LENGTH, RREG_RX_LENGTH_ALL);
    //CopyArray(ReceiveBuffer, RReg_All_Rx, RREG_RX_LENGTH_ALL);

    //P1OUT |= BIT0;                            //LED on
}

void SPI_Debug(){

    BurnoutDetect();

    ReadADCData();

    RREG(RReg_All_Tx, RREG_TX_LENGTH, RREG_RX_LENGTH_ALL);
    CopyArray(ReceiveBuffer, RReg_All_Rx, RREG_RX_LENGTH_ALL);
}
//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************


typedef enum I2C_ModeEnum{
    I2C_IDLE_MODE,
    I2C_NACK_MODE,
    I2C_TX_REG_ADDRESS_MODE,
    I2C_RX_REG_ADDRESS_MODE,
    I2C_TX_DATA_MODE,
    I2C_RX_DATA_MODE,
    I2C_SWITCH_TO_RX_MODE,
    I2C_SWITCH_TO_TX_MODE,
    I2C_TIMEOUT_MODE
} I2C_Mode;

/* Used to track the state of the I2C state machine*/
I2C_Mode SlaveMode = I2C_RX_REG_ADDRESS_MODE;

/* The Register Address/Command to use*/
uint8_t i2cReceiveRegAddr = 0;                      //Command byte send by CCMM

/*Variables to handle Rx and Tx buffers*/
uint8_t i2cReceiveBuffer[CCMM_RX_BUFFER_SIZE]={0};  //Buffer to receive data from I2C master. Not used in this application
uint8_t i2cRXByteCtr = 0;                           //Counter to be decremented to keep track of how many bytes have been received from master while reading data. Not used
uint8_t i2cReceiveIndex = 0;                        //Index of received data. Not used
uint8_t i2cTransmitBuffer[CCMM_TX_BUFFER_SIZE]={0}; //40-byte buffer to contain data and status bytes to be sent to CCMM
uint8_t i2cTXByteCtr = 0;                           //Counter to keep track of how many bytes have been sent to CCMM
uint8_t i2cTransmitIndex = 0;                       //Index to keep track of which byte in the i2c transmit buffer should be added to the UCB0 tx buffer upon request by the master


/********Function Declariations********/

/*Respond to received command byte from CCMM depending on which command bits are high:
 * Bit 0: Execute reset procedure, set byte 1 bit 5 of the status word high
 * Bit 1: Execute self offset calibration, set byte 1 bit 6 of the status word high
 * Bit 2: Execute burnout detect procedure, set byte 1 bit 7 of the status word high
 *
 * Read ADC data, publish to I2C Transmit buffer, and wait for master to request data
 * (This is only called during I2C interrupt, so the MSP430 should be in Low-Power Mode
 */
void I2C_Slave_ProcessCMD(uint8_t cmd);

/* Publish Data from individual channel data-holding registers to the
 * I2C Transmit Buffer to be read by the Master. Data is not processed.*/
void Publish_Data();


/********Function Definitions********/

void I2C_Slave_ProcessCMD(uint8_t cmd)
{
    i2cReceiveIndex = 0;
    i2cTransmitIndex = 0;
    i2cRXByteCtr = 0;
    i2cTXByteCtr = 0;
    Status_bitfield[0] = 0;

    //DEBUGWhoops, should copy to status bitfield, not directly to i2c TransmitBuffer. Also does the status bitfield actually get reset every cycle?

    if (cmd & BIT0){                            //Reset
        ADCReset();
        Status_bitfield[0] |= BIT5;  //Set status bit
        //P1OUT &= ~(BIT0);
    }
    if (cmd & BIT1){                            //Self Offset Calibration
        SelfCalibration();
        Status_bitfield[0] |= BIT6;  //Set status bit
    }
    if (cmd & BIT2){                            //Burnout Detect
        BurnoutDetect();
        Status_bitfield[0] |= BIT7;  //Set status bit
    }

    SlaveMode = I2C_TX_DATA_MODE;
    i2cTXByteCtr = CCMM_TX_BUFFER_SIZE;
    //ReadADCData();
    ReadADCData();
    //Fill out the TransmitBuffer
    Publish_Data();
    IE2 &= ~UCB0RXIE;                       // Disable RX interrupt
    IE2 |= UCB0TXIE;                        // Enable TX interrupt


}

void Publish_Data(){
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_Ch0, RDATA_RX_LENGTH, 0);          //Channel 0 (P: Ain0 N:Ain1)
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_Ch1, RDATA_RX_LENGTH, 4);          //Channel 1 (P: Ain0 N:Ain1)
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_Ch2, RDATA_RX_LENGTH, 8);          //Channel 2 (P: Ain0 N:Ain1)
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_Ch3, RDATA_RX_LENGTH, 12);         //Channel 3 (P: Ain0 N:Ain1)
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_Ch4, RDATA_RX_LENGTH, 16);         //Channel 4 (P: Ain0 N:Ain1)
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_Ch5, RDATA_RX_LENGTH, 20);         //Channel 5 (P: Ain0 N:Ain1)
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_AVDD, RDATA_RX_LENGTH, 24);        //Analog Supply Status
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_DVDD, RDATA_RX_LENGTH, 28);        //Digital Supply Status
    CopytoDataBuffer(i2cTransmitBuffer, RData_Rx_Temperature, RDATA_RX_LENGTH, 32); //On-Chip Temperature
    CopytoDataBuffer(i2cTransmitBuffer, Status_bitfield, STATUS_WORD_LENGTH, 36);   //Status Word

}


//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

/*Calibrate Clock to 8MHz*/
void initClockTo8MHz()
{
    if (CALBC1_8MHZ==0xFF)                  // If calibration constant erased
    {
        while(1);                               // Trap CPU
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_8MHZ;                    // Set DCO
    DCOCTL = CALDCO_8MHZ;
}


//
//                   MSP430G2553
//                 -----------------
//            /|\ |             P2.3|->   ADC !RST (GPIO) [Active Low]
//             |  |                 |
//             ---|RST          P1.5|->   !CS (GPIO) [Active Low]
//                |                 |
//       SCL   -->|P1.6         P2.5|<-   !DRDY (GPIO) [Active Low]
//                |                 |
//       SDA  <-->|P1.7         P2.4|->   START/SYNC (GPIO)
//                |                 |
// I2CAddr_0   -->|P2.0         P1.2|->   Data Out (UCA0SIMO)
//                |                 |
// I2CAddr_1   -->|P2.1         P1.1|<-   Data In (UCA0SOMI)
//                |                 |
// I2CAddr_2   -->|P2.2         P1.4|->   Serial Clock Out (UCA0CLK)
//                |                 |

/*Initialize Reset, DRDY, START/SYNC, SPI and I2C Pins*/
void initGPIO()
{
  //LEDs
  P1OUT = 0x00;                             // P1 setup for debug LED
  //P1DIR |= BIT0;// + BIT6;

  //ADC Reset, DRDY and START/SYNC (Active Low)
  P2DIR |= BIT3 + BIT4;                       //P2.3, P2.4 Output
  P2OUT |= BIT3;                              //P2.3 High --> ADC !RST
  P2OUT &= ~BIT4;                             //P2.4 Low  --> START/SYNC
  P2DIR &= ~(I2CA_0 + I2CA_1 + I2CA_2 + BIT5);//P2.0, 2.1, 2.2, 2.5 Input --> I2C Pins, !DRDY


  //SPI Pins
  P1SEL = BIT1 + BIT2 + BIT4;               //Assign SPI pins to USCI_A0
  P1SEL2 = BIT1 + BIT2 + BIT4;

  //I2C Pins
  P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
  P1SEL2|= BIT6 + BIT7;
}

/*Initialize SPI configuration*/
void initSPI()
{
  //Clock Polarity: The inactive state is low
  //MSB First, 8-bit, Master, 3-pin mode, Synchronous
  UCA0CTL0 |= UCMSB + UCMST + UCSYNC;
  UCA0CTL0 &= ~(UCCKPL);

  UCA0CTL1 |= UCSSEL_2;                     // SMCLK selected for SCLK
  UCA0BR0 |= 0x41;                          // SMCLK / 1778                             //0xF2 1MHZ
  UCA0BR1 = 0x06;                           // 9600 Baud assuming SMCLK = 8MHz          //0x06 1MHZ
  UCA0MCTL = 0;                             // No modulation
  UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
  IE2 |= UCA0RXIE;                          // Enable USCI0 RX interrupt

  CS_DIR |= CS_PIN;                         //CS pin set Output
  CS_OUT |= CS_PIN;                         //CS pin set High

}

/*Initialize I2C configuration*/
void initI2C()
{
    SLAVE_ADDR = (P2IN & (I2CA_0 + I2CA_1 + I2CA_2));// Slave Address determined by slave address pins
    UCB0CTL1 |= UCSWRST;                             // Enable SW reset
    UCB0CTL0 = UCMODE_3 + UCSYNC;                    // I2C Slave, synchronous mode
    UCB0I2COA |= SLAVE_ADDR ;                        // Own Address
    UCB0CTL1 &= ~UCSWRST;                            // Clear SW reset, resume operation
    UCB0I2CIE |= UCSTPIE + UCSTTIE;                  // Enable STT and STP interrupt
    IE2 |= UCB0RXIE;                                 // Enable RX interrupt
}

//******************************************************************************
// Main ************************************************************************
//******************************************************************************
int main(void)
{

  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer




  for(;;){
  //Power-on tasks for MSP430 (init clock, GPIO, SPI and I2C; perform reset,
  //Self offset calibration, and burnout detect.

  if(started == 1){
      initI2C();                               //Initialize I2C
      __delay_cycles(30);

  }

  if(started == 0){
      initClockTo8MHz();                       //Initialize Sub-Main Clock (SMCLK) to 8MHz
      initGPIO();                              //Initialize GPIO
      initSPI();                               //Initialize SPI
      initI2C();                               //Initialize I2C

      //Perform startup tasks
      ADCReset();                               //Reset ADC
      PowerOnSetup();                           //Run Power-on procedure
      started = 1;                              //Flag power-on tasks are complete

      //SPI_Debug();
      //Publish_Data();
      //P1OUT &= ~(BIT0);
  }


  //Wait in low-power mode until I2C rx interrupt
  __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts and wait for commands from master
}
  /****************TRAP IN LOOP, AND STOP SPI INTERRUPT FROM EXITING LPMO AFTER PUBLISHING DATA BECAUSE IT STOPS I2C FROM EXECUTING********
   **DISABLE I2C INTERRUPTS DURING SPI AND SPI INTERRUPTS DURING I2C
   **ADD DELAY IF STUCK IN I2C TX MODE FOR TOO LONG
   */

  //return 0;
}

//******************************************************************************
// SPI Interrupt ***************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCIA0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{

    /*I2C Interrupt for Start, Restart, Nack, Stop*/
        if (UCB0STAT & UCSTPIFG)                        //Stop or NACK Interrupt
        {
            UCB0STAT &=
                ~(UCSTTIFG + UCSTPIFG + UCNACKIFG);     //Clear START/STOP/NACK Flags
        }
        if (UCB0STAT & UCSTTIFG)
        {
            UCB0STAT &= ~(UCSTTIFG);                    //Clear START Flags
        }

    /*SPI Interrupt*/
    if (IFG2 & UCA0RXIFG)
    {
        uint8_t uca0_rx_val = UCA0RXBUF;
        switch (MasterMode)
        {

            case TX_DATA_MODE:
                if (TXByteCtr)
                {
                  SendUCA0Data(TransmitBuffer[TransmitIndex++]);
                  TXByteCtr--;
                }
                else
                {
                  if (RXByteCtr)
                  {
                      MasterMode = RX_DATA_MODE;
                      __delay_cycles(5);             // Delay 25 us
                      SendUCA0Data(DUMMY);
                      ReceiveBuffer[ReceiveIndex++] = uca0_rx_val;
                      RXByteCtr--;

                  }
                  else
                  {
                      //Done with transmission
                      MasterMode = IDLE_MODE;
                      __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
                  }
                }
                break;

            case RX_DATA_MODE:
                if (RXByteCtr)
                {
                    //SendUCA0Data(DUMMY);              //Debug RDATA index problem
                    ReceiveBuffer[ReceiveIndex++] = uca0_rx_val;
                    RXByteCtr--;
                }
                if (RXByteCtr == 0)
                {
                    MasterMode = IDLE_MODE;
                    __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
                }
                else
                {
                    SendUCA0Data(DUMMY);
                }
                break;

            default:
                __no_operation();
                break;
        }
        __delay_cycles(5);
    }


}

//******************************************************************************
// I2C Interrupt For Received and Transmitted Data******************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  if (IFG2 & UCB0RXIFG)                 // Receive Data Interrupt
  {
      //Must read from UCB0RXBUF
      uint8_t rx_val = UCB0RXBUF;
      switch (SlaveMode)
      {
          case (I2C_RX_REG_ADDRESS_MODE):
              i2cReceiveRegAddr = rx_val;
              I2C_Slave_ProcessCMD(i2cReceiveRegAddr);
              break;
          case (I2C_RX_DATA_MODE):
              i2cReceiveBuffer[i2cReceiveIndex++] = rx_val;
              i2cRXByteCtr--;
              if (i2cRXByteCtr == 0)
              {
                  //Done Receiving MSG
                  SlaveMode = I2C_RX_REG_ADDRESS_MODE;
                  IE2 &= ~(UCB0TXIE);
                  IE2 |= UCB0RXIE;                          // Enable RX interrupt
              }
              break;
          default:
              __no_operation();
              break;
      }

  }
  else if (IFG2 & UCB0TXIFG)            // Transmit Data Interrupt
  {
      //Must write to UCB0TXBUF
      switch (SlaveMode)
      {
          case (I2C_TX_DATA_MODE):
              UCB0TXBUF = i2cTransmitBuffer[i2cTransmitIndex++];
              i2cTXByteCtr--;
              if (i2cTXByteCtr == 0)
              {
                  //Done Transmitting MSG
                  SlaveMode = I2C_RX_REG_ADDRESS_MODE;
                  IE2 &= ~(UCB0TXIE);
                  IE2 |= UCB0RXIE;                          // Enable RX interrupt
                  __bic_SR_register_on_exit(CPUOFF);      // DEBUG Exit LPM0
              }
              break;
          default:
              __no_operation();
              break;
      }

  }
}
