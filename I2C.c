#include "I2C.h"
#include "msp430fr4133.h"

void initI2C()
{
    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0BRW = 40;                            // fSCL = SMCLK/160 = ~100kHz
//    UCB0I2CSA = dev_addr;                   // Slave Address
    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0IE |= UCNACKIE;
}


I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;

}


I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{

    /* Initialize state machine */
        MasterMode = TX_REG_ADDRESS_MODE;
        TransmitRegAddr = reg_addr;

        //Copy register data to TransmitBuffer
        CopyArray(reg_data, TransmitBuffer, count);

        TXByteCtr = count;
        RXByteCtr = 0;
        ReceiveIndex = 0;
        TransmitIndex = 0;

        /* Initialize slave address and interrupts */
        UCB0I2CSA = dev_addr;
        UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
       // UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
        UCB0IE |= UCTXIE;                        // Enable TX interrupt

        UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
//        while(!(UCB0IFG&UCTXIFG));
//        UCB0TXBUF = 0x68 + 0;
//        __no_operation();
        __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

        return MasterMode;
}


I2C_Mode I2C_Master_WriteReg_Value(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_data, uint8_t count)
{

    /* Initialize state machine */
        MasterMode = TX_REG_ADDRESS_MODE;
        TransmitRegAddr = reg_addr;

        uint8_t buffer [TYPE_0_LENGTH] = { reg_data};
        //Copy register data to TransmitBuffer
        CopyArray(buffer, TransmitBuffer, count);

        TXByteCtr = count;
        RXByteCtr = 0;
        ReceiveIndex = 0;
        TransmitIndex = 0;

        /* Initialize slave address and interrupts */
        UCB0I2CSA = dev_addr;
        UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
       // UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
        UCB0IE |= UCTXIE;                        // Enable TX interrupt

        UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
//        while(!(UCB0IFG&UCTXIFG));
//        UCB0TXBUF = 0x68 + 0;
//        __no_operation();
        __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

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
