#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "hal_LCD.h"
#include "driverlib.h"
#include "main.h"
#include "MPU6050.h"
#include "I2C.h"

#define LED_OUT     P1OUT
#define LED_DIR     P1DIR
#define LED0_PIN    BIT0
#define LED1_PIN    BIT1

//
//#define MAX_BUFFER_SIZE     20
I2C_Mode MasterMode = IDLE_MODE;

uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

uint8_t TransmitRegAddr = 0;

// Variables to hold acclerometer values
int             ACCEL_XOUT = 0,
                ACCEL_YOUT = 0,
                ACCEL_ZOUT = 0,
                ACCEL_XOUT_PAST=0,          //
                ACCEL_YOUT_PAST=0,
                ACCEL_ZOUT_PAST=0,
                ACCEL_XA    =0,
                ACCEL_YA    =0,
                ACCEL_ZA    =0,
                ACCEL_XV    = 0,
                ACCEL_YV    = 0,
                ACCEL_ZV    = 0,
                ACCEL_XDECAC=0,
                ACCEL_YDECAC=0,
                ACCEL_ZDECAC=0;

//Gyroscope variables
int         GYRO_XOUT = 0,
            GYRO_YOUT = 0,
            GYRO_ZOUT = 0,
            PREV_GYRO_XOUT_1 = 0,
            PREV_GYRO_YOUT_1 = 0,
            PREV_GYRO_ZOUT_1 = 0,
            PREV_GYRO_XOUT_2 = 0,
            PREV_GYRO_YOUT_2 = 0,
            PREV_GYRO_ZOUT_2 = 0,
            GYRO_XOUT_OFFSET = 250,//-237,//104,
            GYRO_YOUT_OFFSET = -170,//-9,
            GYRO_ZOUT_OFFSET = 8,//-197,
            GYRO_XRATE = 0,
            GYRO_YRATE = 0,
            GYRO_ZRATE = 0,
            GYRO_XANGLE = 0,
            GYRO_YANGLE = 0,
            GYRO_ZANGLE = 0,
            gyro_ratio = 4;

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************


void initGPIO()
{
    // Configure GPIO
    LED_OUT &= ~(LED0_PIN | LED1_PIN); // P1 setup for LED & reset output
    LED_DIR |= (LED0_PIN | LED1_PIN);

    // I2C pins
    P5SEL0 |= BIT2 | BIT3;

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);
    //PA.x output
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN5
        );

    GPIO_setOutputLowOnPin(
         GPIO_PORT_P1,
         GPIO_PIN5
         );

    GPIO_setAsOutputPin(
           GPIO_PORT_P2,
           GPIO_PIN5
           );

       GPIO_setOutputHighOnPin(
            GPIO_PORT_P2,
            GPIO_PIN5
            );
}

void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    __bis_SR_register(SCG0);                           // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source
    CSCTL0 = 0;                                        // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                            // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                               // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                           // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked
}

void blink_LED(){
    GPIO_setOutputHighOnPin(
                GPIO_PORT_P1,
                GPIO_PIN5
                );

    __delay_cycles(1000000);

    //Set all PA pins HI
     GPIO_setOutputLowOnPin(
         GPIO_PORT_P1,
         GPIO_PIN5
         );
     __delay_cycles(1000000);
}
//******************************************************************************
// Main ************************************************************************
// Send and receive three messages containing the example commands *************
//******************************************************************************

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    initClockTo16MHz();
    initGPIO();
    initI2C();
    initLCD();
    Setup_MPU6050(SLAVE_ADDR_1);
    Setup_MPU6050(SLAVE_ADDR_2);
    Calibrate_Gyros(SLAVE_ADDR_1);
    Calibrate_Gyros(SLAVE_ADDR_2);

    displayScrollText("CALIBRATION COMPLETE");
    displayScrollText("BEGINNING BACK INJURY DETECTION");

    while(1){


        int counter = 0;

        while(counter < 2000){
            Get_Gyro_Values(SLAVE_ADDR_1);
            counter += Compare_Values_1();

            Get_Gyro_Values(SLAVE_ADDR_2);
            counter += Compare_Values_2();

            if(counter < 0){
                counter = 0;
            }

            __delay_cycles(100000);
        }

        displayScrollText("YOU ARE AT RISK OF HURTING YOUR BACK");

        while((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1)) {
            GPIO_setOutputHighOnPin(
                GPIO_PORT_P1,
                GPIO_PIN5
                );
        }
        GPIO_setOutputLowOnPin(
            GPIO_PORT_P1,
            GPIO_PIN5
            );
        displayScrollText("RESETTING LOOP");
    }

    return 0;
}


//******************************************************************************
// I2C Interrupt ***************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //Must read from UCB0RXBUF
  uint8_t rx_val = 0;
  switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      break;
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB0RXBUF;
        if (RXByteCtr)
        {
          ReceiveBuffer[ReceiveIndex++] = rx_val;
          RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
          UCB0CTLW0 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
          UCB0IE &= ~UCRXIE;
          MasterMode = IDLE_MODE;
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
        break;
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        switch (MasterMode)
        {
          case TX_REG_ADDRESS_MODE:
              UCB0TXBUF = TransmitRegAddr;
              if (RXByteCtr)
                  MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
              else
                  MasterMode = TX_DATA_MODE;        // Continue to transmision with the data in Transmit Buffer
              break;

          case SWITCH_TO_RX_MODE:
              UCB0IE |= UCRXIE;              // Enable RX interrupt
              UCB0IE &= ~UCTXIE;             // Disable TX interrupt
              UCB0CTLW0 &= ~UCTR;            // Switch to receiver
              MasterMode = RX_DATA_MODE;    // State state is to receive data
              UCB0CTLW0 |= UCTXSTT;          // Send repeated start
              if (RXByteCtr == 1)
              {
                  //Must send stop since this is the N-1 byte
                  while((UCB0CTLW0 & UCTXSTT));
                  UCB0CTLW0 |= UCTXSTP;      // Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (TXByteCtr)
              {
                  UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                  TXByteCtr--;
              }
              else
              {
                  //Done with transmission
                  UCB0CTLW0 |= UCTXSTP;     // Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB0IE &= ~UCTXIE;                       // disable TX interrupt
                  __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
              }
              break;

          default:
              __no_operation();
              break;
        }
        break;
    default: break;
  }
}

