#include "MPU6050.h"
#include "I2C.h"

void Setup_MPU6050(uint8_t dev_addr){
    // IFG set
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_SIGNAL_PATH_RESET, 0x07, TYPE_0_LENGTH);
    __delay_cycles(1000);

    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_SMPLRT_DIV, 9, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_CONFIG, 0x03, TYPE_0_LENGTH);

    //Disable gyro self tests, scale of 500 degrees/s
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_GYRO_CONFIG, 0x18, TYPE_0_LENGTH);
    //Disable accel self tests, scale of +-4g, no DHPF
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_ACCEL_CONFIG, 0x18, TYPE_0_LENGTH);

    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_TEMP_OUT_H,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_TEMP_OUT_L,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_GYRO_XOUT_H,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_GYRO_XOUT_L,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_GYRO_YOUT_H,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_GYRO_YOUT_L,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_GYRO_ZOUT_H,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_GYRO_ZOUT_L,0x00, TYPE_0_LENGTH);

    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_ACCEL_XOUT_H,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_ACCEL_XOUT_L,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_ACCEL_YOUT_H,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_ACCEL_YOUT_L,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_ACCEL_ZOUT_H,0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_ACCEL_ZOUT_L,0x00, TYPE_0_LENGTH);

    //Freefall threshold of <|0mg|
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_FF_THR, 0x00, TYPE_0_LENGTH);
    //Freefall duration limit of 0
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_FF_DUR, 0x00, TYPE_0_LENGTH);
    //Motion threshold of >0mg
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_MOT_THR, 0x00, TYPE_0_LENGTH);
    //Motion duration of >0s
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_MOT_DUR, 0x00, TYPE_0_LENGTH);
    //Zero motion threshold
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_ZRMOT_THR, 0x00, TYPE_0_LENGTH);
    //Zero motion duration threshold
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_ZRMOT_DUR, 0x00, TYPE_0_LENGTH);
    //Disable sensor output to FIFO buffer
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_FIFO_EN, 0x00, TYPE_0_LENGTH);

    //AUX I2C setup
    //Sets AUX I2C to single master control, plus other config
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_MST_CTRL, 0x00, TYPE_0_LENGTH);
    //Setup AUX I2C slaves
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV0_ADDR, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV0_REG, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV0_CTRL, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV1_ADDR, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV1_REG, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV1_CTRL, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV2_ADDR, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV2_REG, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV2_CTRL, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV3_ADDR, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV3_REG, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV3_CTRL, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV4_ADDR, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV4_REG, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV4_DO, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV4_CTRL, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_I2C_SLV4_DI, 0x00, TYPE_0_LENGTH);

    //MPU6050_RA_I2C_MST_STATUS //Read-only
    //Setup INT pin and AUX I2C pass through
    I2C_Master_WriteReg_Value(dev_addr,  MPU6050_RA_INT_PIN_CFG, 0x00, TYPE_0_LENGTH);
    //Enable data ready interrupt
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_INT_ENABLE, 0x01, TYPE_0_LENGTH);

    //Slave out, dont care
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV0_DO, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV1_DO, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV2_DO, 0x00, TYPE_0_LENGTH);
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_I2C_SLV3_DO, 0x00, TYPE_0_LENGTH);
    //More slave config
    I2C_Master_WriteReg_Value( dev_addr, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00, TYPE_0_LENGTH);
    //Reset sensor signal paths
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_SIGNAL_PATH_RESET, 0x00, TYPE_0_LENGTH);
    //Motion detection control
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_MOT_DETECT_CTRL, 0x00, TYPE_0_LENGTH);
    //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_USER_CTRL, 0x00, TYPE_0_LENGTH);
    //Sets clock source to gyro reference w/ PLL
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_PWR_MGMT_1, 0x00, TYPE_0_LENGTH);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    I2C_Master_WriteReg_Value(dev_addr, MPU6050_RA_PWR_MGMT_2, 0x00, TYPE_0_LENGTH);

    __delay_cycles(1000);
}

// MPU6050 FUNCTIONS
void Get_Accel_Values(uint8_t dev_addr){
    I2C_Master_ReadReg(dev_addr, MPU6050_RA_ACCEL_XOUT_H, 6);
    ACCEL_XOUT = ((ReceiveBuffer[1]<<8)|ReceiveBuffer[0]);
    ACCEL_YOUT = ((ReceiveBuffer[3]<<8)|ReceiveBuffer[2]);
    ACCEL_ZOUT = ((ReceiveBuffer[5]<<8)|ReceiveBuffer[4]);
}

void Get_Gyro_Values(uint8_t dev_addr)
{
    I2C_Master_ReadReg(dev_addr, MPU6050_RA_GYRO_XOUT_H, 6);
    GYRO_XOUT = ((ReceiveBuffer[1]<<8)|ReceiveBuffer[0]) + GYRO_XOUT_OFFSET;
    GYRO_YOUT = ((ReceiveBuffer[3]<<8)|ReceiveBuffer[2]) + GYRO_YOUT_OFFSET;
    GYRO_ZOUT = ((ReceiveBuffer[5]<<8)|ReceiveBuffer[4]) + GYRO_ZOUT_OFFSET;
}

int Compare_Values_1(){
    int ret;

    if(((PREV_GYRO_XOUT_1 >= 0) ^ (GYRO_XOUT < 0)) & ((PREV_GYRO_YOUT_1 >= 0) ^ (GYRO_YOUT < 0)) & ((PREV_GYRO_ZOUT_1 >= 0) ^ (GYRO_ZOUT < 0))){
        ret = 1;
    }
    else{
        ret = -10;
    }
    PREV_GYRO_XOUT_1 = GYRO_XOUT;
    PREV_GYRO_YOUT_1 = GYRO_YOUT;
    PREV_GYRO_ZOUT_1 = GYRO_ZOUT;

    return ret;
}


int Compare_Values_2(){
    int ret;

    if(((PREV_GYRO_XOUT_2 >= 0) ^ (GYRO_XOUT < 0)) & ((PREV_GYRO_YOUT_2 >= 0) ^ (GYRO_YOUT < 0)) & ((PREV_GYRO_ZOUT_2 >= 0) ^ (GYRO_ZOUT < 0))){
        ret = 1;
    }
    else{
        blink_LED();
        ret = -10;
    }
    PREV_GYRO_XOUT_2 = GYRO_XOUT;
    PREV_GYRO_YOUT_2 = GYRO_YOUT;
    PREV_GYRO_ZOUT_2 = GYRO_ZOUT;

    return ret;
}
void Filters_Gyro()
{
    GYRO_XFilter[3] = GYRO_XFilter[2];
    GYRO_XFilter[2] = GYRO_XFilter[1];
    GYRO_XFilter[1] = GYRO_XFilter[0];
    GYRO_XFilter[0] = GYRO_XOUT;

    GYRO_XOUT =      ((long)GYRO_XFilter[3]
                    +GYRO_XFilter[2]
                    +GYRO_XFilter[1]
                    +GYRO_XFilter[0])>>2;

    GYRO_YFilter[3] = GYRO_YFilter[2];
    GYRO_YFilter[2] = GYRO_YFilter[1];
    GYRO_YFilter[1] = GYRO_YFilter[0];
    GYRO_YFilter[0] = GYRO_YOUT;

    GYRO_YOUT =      ((long)GYRO_YFilter[3]
                    +GYRO_YFilter[2]
                    +GYRO_YFilter[1]
                    +GYRO_YFilter[0])>>2;

    GYRO_ZFilter[3] = GYRO_ZFilter[2];
    GYRO_ZFilter[2] = GYRO_ZFilter[1];
    GYRO_ZFilter[1] = GYRO_ZFilter[0];
    GYRO_ZFilter[0] = GYRO_ZOUT;

    GYRO_ZOUT =      ((long)GYRO_ZFilter[3]
                    +GYRO_ZFilter[2]
                    +GYRO_ZFilter[1]
                    +GYRO_ZFilter[0])>>2;
}

void Calibrate_Gyros(uint8_t dev_addr)
{
    int i = 0;
    long    GYRO_XOUT_OFFSET_4096SUM=0,
            GYRO_YOUT_OFFSET_4096SUM=0,
            GYRO_ZOUT_OFFSET_4096SUM=0,
            GYRO_XOFFSET = 0,
            GYRO_YOFFSET = 0,
            GYRO_ZOFFSET = 0;

    __delay_cycles(1000);
    __delay_cycles(1000);
    for(i = 0; i<4096; i++)
    {
        I2C_Master_ReadReg(dev_addr, MPU6050_RA_GYRO_XOUT_H, 6);

        GYRO_XOUT_OFFSET_4096SUM += ((ReceiveBuffer[1]<<8)|ReceiveBuffer[0]);
        GYRO_YOUT_OFFSET_4096SUM += ((ReceiveBuffer[3]<<8)|ReceiveBuffer[2]);
        GYRO_ZOUT_OFFSET_4096SUM += ((ReceiveBuffer[5]<<8)|ReceiveBuffer[4]);
        GYRO_XOUT_OFFSET_4096SUM += GYRO_XOUT;
        GYRO_YOUT_OFFSET_4096SUM += GYRO_YOUT;
        GYRO_ZOUT_OFFSET_4096SUM += GYRO_ZOUT;

       __delay_cycles(16000);

    }
    GYRO_XOFFSET = GYRO_XOUT_OFFSET_4096SUM>>12;
    GYRO_YOFFSET = GYRO_YOUT_OFFSET_4096SUM>>12;
    GYRO_ZOFFSET = GYRO_ZOUT_OFFSET_4096SUM>>12;

    __delay_cycles(10000);
}
