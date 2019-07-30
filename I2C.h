#include <stdint.h>
#include "main.h"
//Constants
#define MAX_BUFFER_SIZE     20


#define CMD_TYPE_0_SLAVE      0
#define CMD_TYPE_1_SLAVE      1
#define CMD_TYPE_2_SLAVE      2

#define CMD_TYPE_0_MASTER      3
#define CMD_TYPE_1_MASTER      4
#define CMD_TYPE_2_MASTER      5

#define TYPE_0_LENGTH   1
#define TYPE_1_LENGTH   2
#define TYPE_2_LENGTH   6

// I2C transaction values
typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;


/* Used to track the state of the software state machine*/
extern I2C_Mode MasterMode;

/* The Register Address/Command to use*/
extern uint8_t TransmitRegAddr;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * */
extern uint8_t ReceiveBuffer[MAX_BUFFER_SIZE];
extern uint8_t RXByteCtr;
extern uint8_t ReceiveIndex;
extern uint8_t TransmitBuffer[MAX_BUFFER_SIZE];
extern uint8_t TXByteCtr;
extern uint8_t TransmitIndex;


/* I2C Write and Read Functions */

/* For slave device with dev_addr, writes the data specified in *reg_data
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 * *reg_data: The buffer to write
 * count: The length of *reg_data
 *  */
void initI2C();
I2C_Mode I2C_Master_WriteReg(uint8_t, uint8_t, uint8_t *, uint8_t);
I2C_Mode I2C_Master_ReadReg(uint8_t, uint8_t, uint8_t);
void CopyArray(uint8_t *, uint8_t *, uint8_t);
I2C_Mode I2C_Master_WriteReg_Value(uint8_t, uint8_t, uint8_t, uint8_t);
