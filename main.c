
#include "msp430f5244.h"

// Clock 1,048576 MHz

// MAX30100 definitions
#define MAX30100_ID     0xAE >> 1    // MAX30100 slave ID, shifted once right, as the slave address register is right justified
// Status
#define MAX30100_IS     0x00    // Interrupt Status
#define MAX30100_IE     0x01    // Interrupt Enable
// FIFO
#define MAX30100_FWP    0x02    // FIFO Write Pointer
#define MAX30100_OFC    0x03    // Over Flow Counter
#define MAX30100_FRP    0x04    // FIFO Read Pointer
#define MAX30100_FDR    0x05    // FIFO Data Register
// Configuration
#define MAX30100_MC     0x06    // Mode Configuration
#define MAX30100_SP02C  0X07    // SPO2 Configuration
#define MAX30100_LEDC   0X09    // LED Configuration
// Temperature
#define MAX30100_TI     0x16    // Temp_Integer
#define MAX30100_TF     0x17    // Temp_Fraction
// Part ID
#define MAX30100_RID    0xFE    // Revision ID
#define MAX30100_PID    0xFF    // Part ID

#define SAMPLES         512     // samples count

unsigned char InterruptStatus,
              InterruptEnable,
              FIFOWritePointer,
              OverFlowCounter,
              FIFOReadPointer,
              FIFODataRegister,
              ModeConfiguration,
              SPO2Configuration,
              LEDConfiguration,
              Temp_Integer,
              Temp_Fraction,
              RevisionID,
              PartID;           // part id and revision id variables
unsigned int RED_DATA, IR_DATA; // FIFO read data for RED and IR diodes

unsigned int RED_ARRAY[SAMPLES];   // RED diode array of datta
unsigned int IR_ARRAY[SAMPLES];    // IR diode array of datta

unsigned char InterruptStatus_Array[SAMPLES],  // IS array
              FIFOWritePointer_Array[SAMPLES], // WP array
              OverFlowCounter_Array[SAMPLES],  // OFC array
              FIFOReadPointer_Array[SAMPLES];; // RP array

int i;     // used for loops


//==============================================================================
// Init USCI_B0 in I2C mode
// P3.1 - SCL
// P3.0 - SDA
//
// Taken from the user guide
// The recommended USCI initialization/reconfiguration process is:
// 1. Set UCSWRST (BIS.B #UCSWRST,&UCxCTL1).
// 2. Initialize all USCI registers with UCSWRST = 1.
// 3. Configure ports.
// 4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCxCTL1).
// 5. Enable interrupts (optional).
//==============================================================================
void i2c_init(void) {
  UCB0CTL1 |= UCSWRST;           // Reset the module
  
  UCB0CTL0 |= UCMST + UCMODE1 + UCMODE0 + UCSYNC;  // 7-bit address, Master mode, I2C mode, Synchronous mode
  UCB0CTL1 |= UCSSEL__SMCLK;     // USCI clock source select SMCLK
  UCB0BR0 = 4;                  // Set SCL frequency to SMCLK/10 = 104,8576 kHz
  UCB0BR1 = 0;
  
  UCB0I2CSA = MAX30100_ID;       // Set slave ID
  
  P3SEL |= BIT0 + BIT1;          // Select I2C function on SCL and SDA
  P3DIR |= BIT0 + BIT1;          // Select SCL and SDA as outputs (just in case)
  
  UCB0CTL1 &= ~UCSWRST;          // Clear reset
}

//==============================================================================
// I2C write one byte of datta
// addr     - address to which to be written data
// data     - data to be written
//==============================================================================
void i2c_write(unsigned char addr, unsigned char data) {
  //UCB0I2CSA = slave_id;                // Set slave ID
  
  UCB0CTL1 |= UCTR;                    // Transmit
  UCB0CTL1 |= UCTXSTT;                 // Generate START condition
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for START, SLAVE ID to be sent
  
  UCB0TXBUF = addr;                    // write addr to the buffer
  while(UCB0CTL1 & UCTXSTT != 0x00);   // check bit for 0 - wait for ACK by slave
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for ADDR to be sent
  __delay_cycles(100);
  
  UCB0TXBUF = data;                    // write addr to the buffer
  while(UCB0CTL1 & UCTXSTT != 0x00);   // check bit for 0 - wait for ACK by slave
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be sent
  __delay_cycles(100);
  
  UCB0CTL1 |= UCTXSTP;                 // send STOP condition
  while(UCB0CTL1 & UCTXSTP != 0x00);   // check bit for 0 - wait for STOP to be generated
  __delay_cycles(100);
  
}

//==============================================================================
// I2C read one byte of data
// addr     - address from which data to be read
// 
// return   -> data
//==============================================================================
unsigned char i2c_read(unsigned char addr) {
  unsigned char data;
  //UCB0I2CSA = slave_id;                // Set slave ID
  
  UCB0CTL1 |= UCTR;                    // Transmit
  UCB0CTL1 |= UCTXSTT;                 // Generate START condition
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for START, SLAVE ID to be sent
  
  UCB0TXBUF = addr;                    // write addr to the buffer
  while(UCB0CTL1 & UCTXSTT != 0x00);   // check bit for 0 - wait for ACK by slave
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for ADDR to be sent
  __delay_cycles(100);
  
  UCB0CTL1 &= ~UCTR;                   // Receive
  UCB0CTL1 |= UCTXSTT;                 // Generate START condition
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
  __delay_cycles(100);
  
  UCB0CTL1 |= UCTXSTP + UCTXNACK;      // send NACK and STOP condition
  while(UCB0CTL1 & UCTXSTP != 0x00);   // check bit for 0 - wait for STOP to be generated
  __delay_cycles(100);
  
  data = UCB0RXBUF;                    // read receive buffer - this is the read data
  return data;
}

//==============================================================================
// I2C read 4 bytes of data
// addr     - address from which data to be read
// 
// return   -> data
//==============================================================================
void i2c_read4(unsigned char addr) {
  
  IR_DATA = 0x0000;
  RED_DATA = 0x0000;
  
  UCB0CTL1 |= UCTR;                    // Transmit
  UCB0CTL1 |= UCTXSTT;                 // Generate START condition
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for START, SLAVE ID to be sent
  
  UCB0TXBUF = addr;                    // write addr to the buffer
  while(UCB0CTL1 & UCTXSTT != 0x00);   // check bit for 0 - wait for ACK by slave
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for ADDR to be sent
  __delay_cycles(100);
  
  UCB0CTL1 &= ~UCTR;                   // Receive
  UCB0CTL1 |= UCTXSTT;                 // Generate START condition
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
  __delay_cycles(100);
  
  IR_DATA = UCB0RXBUF << 8;            // Read 8 MSB of IR adc data
  
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
  __delay_cycles(100);
  
  IR_DATA += UCB0RXBUF;                // Read 8 LSB of IR adc data
  
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
  __delay_cycles(100);
  
  RED_DATA = UCB0RXBUF << 8;           // Read 8 MSB of RED adc data
  
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
  __delay_cycles(100);
  
  RED_DATA += UCB0RXBUF;               // Read 8 LSB of RED adc data
  
  UCB0CTL1 |= UCTXSTP + UCTXNACK;      // send NACK and STOP condition
  while(UCB0CTL1 & UCTXSTP != 0x00);   // check bit for 0 - wait for STOP to be generated
  __delay_cycles(100);

}

//==============================================================================
// I2C read FIFO - 4 bytes of data, 15 times
// addr     - address from which data to be read
// 
// return   -> data
//==============================================================================
void i2c_read_fifo() {
  
  //IR_DATA = 0x0000;
  //RED_DATA = 0x0000;
  
  int loop;                            // used to loop 15 times
  
  UCB0CTL1 |= UCTR;                    // Transmit
  UCB0CTL1 |= UCTXSTT;                 // Generate START condition
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for START, SLAVE ID to be sent
  
  UCB0TXBUF = MAX30100_FDR;            // write fifo address to the buffer
  while(UCB0CTL1 & UCTXSTT != 0x00);   // check bit for 0 - wait for ACK by slave
  while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for ADDR to be sent
  __delay_cycles(100);
  
  UCB0CTL1 &= ~UCTR;                   // Receive
  UCB0CTL1 |= UCTXSTT;                 // Generate START condition
  
  for(loop=0; loop<16; loop++) {
    while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
    __delay_cycles(100);
    
    IR_DATA = UCB0RXBUF << 8;            // Read 8 MSB of IR adc data
    
    while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
    __delay_cycles(100);
    
    IR_DATA += UCB0RXBUF;                // Read 8 LSB of IR adc data
    
    while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
    __delay_cycles(100);
    
    RED_DATA = UCB0RXBUF << 8;           // Read 8 MSB of RED adc data
    
    while(UCB0IFG & UCTXIFG == 0x00);    // check bit for 1 - wait for DATA to be received
    __delay_cycles(100);
    
    RED_DATA += UCB0RXBUF;               // Read 8 LSB of RED adc data
    
    RED_ARRAY[i*16 + loop] = RED_DATA;   // store data in the array
    IR_ARRAY[i*16 + loop] = IR_DATA;     // store data in the array
  }
  
  UCB0CTL1 |= UCTXSTP + UCTXNACK;      // send NACK and STOP condition
  while(UCB0CTL1 & UCTXSTP != 0x00);   // check bit for 0 - wait for STOP to be generated
  __delay_cycles(100);
  
  InterruptStatus = i2c_read(MAX30100_IS); // read interupt status register to clear power ready (bit 0)
  FIFOWritePointer = i2c_read(MAX30100_FWP);
  OverFlowCounter = i2c_read(MAX30100_OFC);
  FIFOReadPointer = i2c_read(MAX30100_FRP);

}

//==============================================================================
// MAX30100 init function
// 
// Init parameters:
// 1.) SPO2 mode
// 2.) SPO2 high resolution
// 3.) 100 samples per second
// 4.) 1,6us pulse width
// 5.) 11mA IR and RED current
//
// 6.) Enable SPO2 interrupt
//==============================================================================
void max30100_init() {
  
  // Configure MAX30100
  
  i2c_write(MAX30100_MC, 0x40);       // reset the sensor
  while(i2c_read(MAX30100_MC) & 0x40 != 0x00); // wait for reset bit to be cleared
  
  ModeConfiguration = i2c_read(MAX30100_MC);  // read mode config
  i2c_write(MAX30100_MC, 0x03);       // enable SPO2 mode
  
  SPO2Configuration = i2c_read(MAX30100_SP02C);  // read SPO2 config
  i2c_write(MAX30100_SP02C, 0x47);    // SPO2 high resolution, 100SPS, 1,6us pulse width
  
  LEDConfiguration = i2c_read(MAX30100_LEDC);  // read LED current settings
  i2c_write(MAX30100_LEDC, 0x66);     // set RED current - 50mA and IR current to 24mA
  
  //INTERUPT_STATUS = i2c_read(MAX30100_IS); // read interupt status register to clear power ready (bit 0)
  
  i2c_write(MAX30100_IE, 0x10);       // enable RED and IR interupts on INT# pin
  
  // Read part id and revision id to verify the i2c and max30100 are working properly
  //PART_ID = i2c_read(MAX30100_PID);   // Read MAX30100 part id
  //PART_RID = i2c_read(MAX30100_RID);  // Read MAX30100 revision id
  
}

//int main( void )
void main(void)
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  
  //P1DIR = BIT2;                  // P1.2 is output
  //P1OUT |= BIT2;                 // Set P1.2
  
/*  // ===========================================================================
  // LED
  // ===========================================================================
  P1DIR |= BIT2;                 // P1.2 pwm output
  P1SEL |= BIT2;                 // P1.2 TA0.1
  TA0CCR0 = 65000;               // WPM period = 1,048576MHz/650000 = ~16Hz
  TA0CCTL1 = CAP + OUTMOD_3;     // Capture mode, Toggle
  TA0CCR1 = 64500;               // Duty cycle ~0,7%
  TA0CTL |= TASSEL__SMCLK + ID_0 + MC_1 + TACLR;  // SMCLK source, UP mode, clear timer
*/
  
  // Init INT# input
  P2DIR &= ~BIT7;                     // P2.7 is input
  
  i2c_init();                         // init I2C module for MAX30100
  
  max30100_init();                    // init MAX30100 SoC
  
  //MODE_CFG = i2c_read(MAX30100_MC);   // read mode config
  
  //FIFO_WR_PTR = i2c_read(MAX30100_FWP);
  //FIFO_RD_PTR = i2c_read(MAX30100_FRP);
  
  // Clear FIFO pointers
  i2c_write(MAX30100_FWP, 0x00);      // Clear FIFO write pointer
  i2c_write(MAX30100_OFC, 0x00);      // Clear overflow counter
  i2c_write(MAX30100_FRP, 0x00);      // Clear FIFO read pointer
  
  //FIFO_RD_PTR = i2c_read(MAX30100_FRP);
  //i2c_read4(MAX30100_FDR);            // Read one sample from FIFO
  //FIFO_RD_PTR2 = i2c_read(MAX30100_FRP);
  
  //for(i=0; i<16; i++) {
  //  i2c_read4(MAX30100_FDR);          // Read one sample from FIFO
  //  RED_ARRAY[i] = RED_DATA;          // write in RED array the current sample
  //  IR_ARRAY[i] = IR_DATA;            // write in IR array the current sample
  //}
  
  do {
    //do {
    //while (P2IN & BIT7 != 0x00);        // while INT# is set high (all data read)
    //INTERUPT_STATUS = i2c_read(MAX30100_IS); // read interupt status register to clear power ready (bit 0)
    //FIFO_WR_PTR = i2c_read(MAX30100_FWP);
    //OverFlowCounter = i2c_read(MAX30100_OFC);
    //FIFO_RD_PTR = i2c_read(MAX30100_FRP);
    //for(int j=0; j<FIFO_RD_PTR; j++) { // loop until the fifo is empty
    if(i2c_read(MAX30100_IS) == 0x10) {
      InterruptStatus_Array[i] =  i2c_read(MAX30100_IS);
      FIFOWritePointer_Array[i] = i2c_read(MAX30100_FWP);
      OverFlowCounter_Array[i] = i2c_read(MAX30100_OFC);
      FIFOReadPointer_Array[i] = i2c_read(MAX30100_FRP);
      i2c_read4(MAX30100_FDR);          // Read one sample from FIFO
      RED_ARRAY[i] = RED_DATA;          // write in RED array the current sample
      IR_ARRAY[i] = IR_DATA;            // write in IR array the current sample
      //i2c_read_fifo();
      i++;
    }
    //}
    //} while (P2IN & BIT7 != 0x00);      // while INT# is set high (all data read)
  } while (i<SAMPLES);                     // while the arrays are filled
  
//  return 0;
} 

/**
 * @page example Example
 *
 * @par Example created using the EasyMX Pro v7
 *
 * @code
 *
 * #include <stdint.h>
 * #include "heartrate1_hw.h"
 *
 * #define DataIsReady()     ( dataReady == 0 )
 * #define DataIsNotReady()  ( dataReady != 0 )
 * #define SAMPLES 750
 *
 * sbit dataReady at GPIOD_IDR.B10;
 * int ir_screen[16], red_screen[16];
 * int cnt_samples, cnt;
 * uint16_t ir_buffer[2000], red_buffer[2000];
 *
 * *****************************************************************************
 * * TFT module connections
 * *****************************************************************************
 * uint16_t TFT_DataPort at GPIOE_ODR;
 * sbit     TFT_RST at GPIOE_ODR.B8;
 * sbit     TFT_RS at GPIOE_ODR.B12;
 * sbit     TFT_CS at GPIOE_ODR.B15;
 * sbit     TFT_RD at GPIOE_ODR.B10;
 * sbit     TFT_WR at GPIOE_ODR.B11;
 * sbit     TFT_BLED at GPIOE_ODR.B9;
 *
 * // Resources
 * char const extern Arial_Black24x33_Regular[], Arial_Black27x38_Regular[],
 *                   Arial_Black64x90_Regular[], Tahoma19x23_Regular[],
 *                   active_jpg[6823], idle_red_jpg[6089];
 *
 * // Display Initialize
 * // Set the background on TFT
 * static void DisplayInit()
 * {
 *     TFT_Init_ILI9341_8bit( 320, 240 );
 *     TFT_BLED = 1;
 *     TFT_Image_Jpeg( 0, 0, idle_red_jpg );
 *     TFT_Set_Font( Tahoma19x23_Regular, CL_WHITE, FO_HORIZONTAL );
 *     TFT_Write_Text( "PLACE YOUR FINGER ON THE CLICK", 10, 194 );
 * }
 *
 * // Update Values on TFT screen
 * // Erase old IR data from sensor, then write new IR data
 * void UpdateValues ( uint16_t ir_val, uint16_t red_val )
 * {
 *     TFT_Set_Font( Tahoma19x23_Regular, 0xE8C4, FO_HORIZONTAL );
 *     TFT_Write_Text( ir_screen, 10, 50 );
 *     WordToStr( ir_val, ir_screen );
 *     TFT_Set_Font( Tahoma19x23_Regular, CL_WHITE, FO_HORIZONTAL );
 *     TFT_Write_Text( ir_screen, 10, 50 );
 *     TFT_Set_Font( Tahoma19x23_Regular, 0xE8C4, FO_HORIZONTAL );
 *     TFT_Write_Text( red_screen, 10, 80 );
 *     WordToStr( red_val, red_screen );
 *     TFT_Set_Font( Tahoma19x23_Regular, CL_WHITE, FO_HORIZONTAL );
 *     TFT_Write_Text( red_screen, 10, 80 );
 * }
 *
 * *****************************************************************************
 * * System Initialize
 * *****************************************************************************
 * static void System_Init()
 * {
 *     GPIO_Digital_Input( &GPIOD_BASE, _GPIO_PINMASK_10 );
 *     I2C1_Init_Advanced( 400000, &_GPIO_MODULE_I2C1_PB67 );
 *     Delay_ms( 100 );
 *     DisplayInit();
 *     heartrate1_init( MAX30100_I2C_ADR );
 *     Delay_ms( 100 );
 *     UART1_Init(115200);
 *     UART_Write_Text("test\r\n");
 * }
 *
 * *****************************************************************************
 * * Main program
 * *****************************************************************************
 * * sample_num -  number of read samples
 * * ir_buff, red_buff - Raw values from LED diodes
 * * ir_average, red_average - averaged values
 * * cnt_samples - samples counter
 * *****************************************************************************
 * void main()
 * {
 *     int i, j = 0;
 *     char sample_num;
 *     unsigned long temp_value,
 *                   ir_buff[16]  = {0},
 *                   red_buff[16] = {0},
 *                   ir_average,
 *                   red_average;
 *
 *     System_Init();
 *
 *     while ( 1 )
 *     {
 *         if ( DataIsReady() && (( heartrate1_get_status() & 0x20 ) != 0) )
 *         {
 *             // Read IR and RED sensor data and store it in sample_num
 *             sample_num = heartrate1_read_ir_red( ir_buff, red_buff );
 *             // Average data
 *             if ( sample_num >= 1 )
 *             {
 *                 ir_average = 0;
 *                 red_average = 0;
 *                 for ( i = 0; i < sample_num; i++ )
 *                 {
 *                     ir_average += ir_buff[i];
 *                     red_average += red_buff[i];
 *                 }
 *                 ir_average  /= sample_num;
 *                 red_average /= sample_num;
 *                 UpdateValues( ir_average, red_average );
 *
 *                 ir_buffer[j] = ir_average;
 *                 j++;
 *                 if(j > SAMPLES) break;
 *             }
 *         }
 *     }
 *
 *     for( cnt = 0; cnt < SAMPLES ;cnt++ )
 *     {
 *         WordToStr( ir_buffer[cnt],ir_screen );
 *         UART1_Write_Text( ir_screen );
 *         UART1_Write( 13 );
 *         UART1_Write( 10 );
 *     }
 *
 *     // Moving average
 *     // Searching max
 * }
 * 
 * @endcode
 *  */

/*

#include "MAX30100_Defs.h"
#include "stdint.h"

//#if defined(DEBUG_LOG)
//uint8_t
//  debugText[16];
//#endif

//
// write data to the sensor
// @param wrAddr address to write to
// @param wrData data to write
//

int debug;
// MAX30100_SendToSensor
static void MAX30100_SendToSensor (uint8_t wrAddr, uint8_t wrData) {
  uint8_t dataToSend[2];
  dataToSend[0] = wrAddr;
  dataToSend[1] = wrData;
  I2C1_Start();  // I2C start signal
  // send address to write to and then data to be written
  I2C1_Write(MAX30100_I2C_ADR, dataToSend, 2, END_MODE_STOP); // MAX30100_I2C_ADR
}

// 
//  * read data from the sensor
//  * @param rAddr   register address to read from
//  * @param rxBuff  buffer where read data are to be placed
//  * @param dataNum number of bytes to read
//  

// MAX30100_ReadFromSensor
static void MAX30100_ReadFromSensor (uint8_t rAddr, uint8_t* rxBuff, uint8_t dataNum)
{
  I2C1_Start(); // I2C start signal
  I2C1_Write(MAX30100_I2C_ADR, &rAddr, 1, END_MODE_RESTART);   // send the register address
  I2C1_Read(MAX30100_I2C_ADR, rxBuff, dataNum, END_MODE_STOP); // receive data
}

//
// getters
//

// get wanted ID
// @return wanted ID
// MAX30100_Get
uint8_t MAX30100_Get(uint8_t anID) {
  uint8_t readData;
  MAX30100_ReadFromSensor(anID, &readData, 1);
  return readData;
}


// get revision ID
// @return revision ID
// MAX30100_GetRevisionID()
uint8_t MAX30100_GetRevisionID() {
  uint8_t readData;
  MAX30100_ReadFromSensor(REVISION_ID, &readData, 1);
  return readData;
}


// get part ID
// @return part ID
// MAX30100_GetPartID()
uint8_t MAX30100_GetPartID() {
  uint8_t readData;
  MAX30100_ReadFromSensor(PART_ID, &readData, 1);
  return readData;
}

// get config
// @return config
// MAX30100_GetConfig()
uint8_t MAX30100_GetConfig() {
  uint8_t readData;
  MAX30100_ReadFromSensor(MODE_CONFIG, &readData, 1);
  return readData;
}

// get status
// @return status
// MAX30100_GetStatus()
uint8_t MAX30100_GetStatus() {
  uint8_t readData;
  MAX30100_ReadFromSensor(INT_STATUS, &readData, 1);
  return readData;
}

//
// setters
//

// send configuration
// @param cfg desired config value
// MAX30100_SetConfig ()
void MAX30100_SetConfig (uint8_t cfg) {
  MAX30100_SendToSensor(MODE_CONFIG, cfg); // MAX30100_SendToSensor
}

// set interrupts
// @param intrpts desired interrupts
// MAX30100_SetInterrupt()
void MAX30100_SetInterrupt(uint8_t intrpts) {
  MAX30100_SendToSensor(MODE_CONFIG, intrpts); // MAX30100_SendToSensor
}

// read data from the sensor
// @param irValue  data from IR LED
// @param redValue data from red LED
// MAX30100_Read()
uint8_t MAX30100_Read (uint16_t* irBuff,uint16_t* redBuff) {
  uint8_t i, sampleNum = 0;
  uint8_t volatile wrPtr, rdPtr, ovPtr, samples[4];

  wrPtr = MAX30100_Get(FIFO_WRITE_PTR);
  rdPtr = MAX30100_Get(FIFO_READ_PTR);
  
  sampleNum = abs( 16 + wrPtr-rdPtr ) % 16;

//  #if defined(DEBUG_LOG)
//  if ( sampleNum > 1 )
//  {
//    asm nop;
//  }
//  #endif
  
  if ( sampleNum >= 1 )
  {
    for ( i = 0; i < sampleNum; ++i )
    {
      // read data
      MAX30100_ReadFromSensor(FIFO_DATA_REG, samples, 4);
      ( (uint8_t*)irBuff   )[0]  = samples[1];
      ( (uint8_t*)irBuff++ )[1]  = samples[0];

      ( (uint8_t*)redBuff   )[0]  = samples[3];
      ( (uint8_t*)redBuff++ )[1]  = samples[2];
    }
  }
    
  return sampleNum;
}

// read calibration temperature from the sensor
// @param tempValue data from temperature sensor
// MAX30100_ReadTemp()
void MAX30100_ReadTemp(uint16_t* tempValue) {
  uint8_t tempInt, tempFrac;
  MAX30100_ReadFromSensor(TEMP_INTEGER, &tempInt, 1); // read temperature - integer part
  MAX30100_ReadFromSensor(TEMP_FRACTION, &tempFrac, 1); // read temperature - fraction part

  ( (uint8_t*)tempValue )[1]  = tempInt;
  ( (uint8_t*)tempValue )[0]  = tempFrac;
}

// reset the sensor
// MAX30100_Reset()
void MAX30100_Reset() {
  char cfg;
  MAX30100_ReadFromSensor(MODE_CONFIG, &cfg, 1);
  cfg.B6 = 1;
  MAX30100_SendToSensor(MODE_CONFIG, cfg);
  
  // wait for the RESET bit to clear itself
  while (1) {
    MAX30100_ReadFromSensor(MODE_CONFIG, &cfg, 1);
    if ( cfg.B6 == 0 )
    {
      return;
    }
  }
}

// initialize the sensor
// MAX30100_Init()
void MAX30100_Init()
{
  uint8_t cfg = 0;

  // read old and set new mode
  MAX30100_ReadFromSensor(MODE_CONFIG, &cfg, 1);
  cfg = ( cfg & ~0x07 ) | HR_ONLY;
  cfg = ( cfg & ~0x07 ) | SPO2_EN | TEMP_EN;

  MAX30100_SendToSensor(MODE_CONFIG, cfg);
  MAX30100_ReadFromSensor(SPO2_CONFIG, &cfg, 1); // read SpO2 configuration
  cfg |= SPO2_HI_RES_EN; // enable High Resolution SpO2
  cfg |= SAMPLES_50; // set samples per second
  cfg |= PULSE_WIDTH_1600; // set 16-bit ADC resolution
  MAX30100_SendToSensor(SPO2_CONFIG, cfg); // send new SpO2 configuration


  MAX30100_ReadFromSensor(LED_CONFIG, &cfg, 1); // read LED current control config
  cfg |= IR_CURRENT_500; // set IR LED current to be 50 mA
  cfg |= RED_CURRENT_240; // set RED LED current to be 24 mA
  MAX30100_SendToSensor(LED_CONFIG, cfg); // send new LED current control config

  // interrupt settings
  MAX30100_ReadFromSensor(INT_ENABLE, &cfg, 1);
  cfg |= ENA_HR_RDY;
  cfg |= ENA_SO2_RDY;
  cfg |= ENA_TEP_RDY;

  // send new interrupt settings
  MAX30100_SendToSensor(INT_ENABLE, cfg);
  debug=cfg;
}

*/
