
#include "msp430f5244.h"

// Clock 1,048576 MHz

// MAX30100 definitions
#define MAX30100_ID     0xAE >> 1    // MAX30100 slave ID, shifted once right, as the slave address register is right justified
// Status
#define MAX30100_IS     0x00    // Interrupt Status
#define MAX30100_IE     0x01    // Interrupt Enable
// FIFO
#define MAX30100_FWP    0x02    // FIFO Write Pointer
#define MAX30100_OVC    0x03    // Over Flow Counter
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
// slave_id - slave ID to be sent
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
// I2C read one byte of datta
// slave_id - slave ID to be sent
// addr     - address from wrich data to be read
// 
// return data
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

int main( void )
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
  
  unsigned char PART_ID;
  
  i2c_init();                      // init I2C module for MAX30100
  
  //i2c_write(MAX30100_ID, MAX30100_IE, 0xFF);    // Write to MAX30100 interupt enable register
  //__delay_cycles(100);
  
  i2c_write(MAX30100_MC, 0x03);    // enable SPO2 mode
  
  i2c_write(MAX30100_LEDC, 0x33);  // set red and ir leds current to 11mA
  
  PART_ID = i2c_read(MAX30100_PID);// Read MAX30100 part id
  //PART_ID = i2c_read(MAX30100_ID, MAX30100_IE); // Read MAX30100 interrupt enable
  
  PART_ID++;

  return 0;
}
