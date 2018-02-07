#include "include.h"


int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;


//向I2C设备写入一个字节数据
void Single_WriteI2C_MPU6050(uint8 REG_Address,uint8 REG_data)
{
    IIC_start();                //起始信号
    send_ch(MPU6050_ADDR);      //发送设备地址+写信号
    send_ch(REG_Address);       //内部寄存器地址，
    send_ch(REG_data);          //内部寄存器数据，
    IIC_stop();                 //发送停止信号
}
//从I2C设备读取一个字节数据
uint8 Single_ReadI2C_MPU6050(uint8 REG_Address)
{
    uint8 REG_data;
    IIC_start();                //起始信号
    send_ch(MPU6050_ADDR);      //发送设备地址+写信号
    send_ch(REG_Address);       //发送存储单元地址，从0开始	
    IIC_start();                //起始信号
    send_ch(MPU6050_ADDR+1);    //发送设备地址+读信号
    REG_data=read_ch();         //读出寄存器数据
    //I2C_SendACK();            //接收应答信号
    IIC_stop();                 //停止信号
    return REG_data;
}
//初始化MPU6050
void InitMPU6050(void)
{
    Single_WriteI2C_MPU6050(PWR_MGMT_1, 0x00);	    //解除休眠状态
    Single_WriteI2C_MPU6050(SMPLRT_DIV, 0x04);      //200HZ采样率
    Single_WriteI2C_MPU6050(CONFIG, 0x01);          //
    Single_WriteI2C_MPU6050(GYRO_CONFIG, 0x18);     //2000
    Single_WriteI2C_MPU6050(ACCEL_CONFIG, 0x10);    //8g
}
//读取数据
int16 GetData(uint8 REG_Address)
{
    uint8 L;   uint16 H ;
    H=Single_ReadI2C_MPU6050(REG_Address);
    L=Single_ReadI2C_MPU6050(REG_Address+1);
    return (H<<8)+L;   //合成数据
}

void Get_AccData(void)
{
    mpu_acc_x = GetData(ACCEL_XOUT_H)>>2;
    mpu_acc_y = GetData(ACCEL_YOUT_H)>>2;
    mpu_acc_z = GetData(ACCEL_ZOUT_H)>>2;
}

uint8 Gyro_y_bijiao(int16 base_zhi)
{
  
     mpu_gyro_y = GetData(GYRO_YOUT_H)>>3;

    //处理 一下
    
    if((base_zhi>0&&mpu_gyro_y>base_zhi)
       ||(base_zhi<0&&mpu_gyro_y<base_zhi))
    {
        mpu_gyro_z = GetData(GYRO_ZOUT_H)>>3;
        if(ABS(mpu_gyro_y)>ABS(mpu_gyro_z))
        {
           mpu_gyro_x = GetData(GYRO_XOUT_H)>>3;
           if(ABS(mpu_gyro_y)>ABS(mpu_gyro_x))
           {
              return 1;
           }
        }
    }
    return 0;
}

int16 Get_Gyro_y()
{
    mpu_gyro_y = GetData(GYRO_YOUT_H)>>3;
    return mpu_gyro_y;
}
int16 Get_Gyro_z()
{
    mpu_gyro_z = GetData(GYRO_ZOUT_H)>>3;
    return mpu_gyro_z;
}
int16 Get_Gyro_x()
{
   mpu_gyro_x = GetData(GYRO_XOUT_H)>>3;
    return mpu_gyro_x;
}