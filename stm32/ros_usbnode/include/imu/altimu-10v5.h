
#ifndef __ALTIMU_10V5_H
#define __ALTIMU_10V5_H

/* Calibration, Conversion factors */

#define DS33_G_FACTOR           1.0/(65536/2/2)     // LSM6DS33 datasheet (page 15)  0.061 mg/LSB
#define DS33_DPS_FACTOR         1.0/(65536/2/245)   // LSM6DS33 °/sec/LSB 
#define LIS3MDL_GAUSS_FACTOR    1.0/6842            // LIS3MDL datasheet (page8)  Gauss/LSB 

/* Gyro / Accelerometer */
#define DS33_ADDRESS 0b1101011
#define DS33_WHO_ID    0x69         // WHO_AM_I will report this value

/* Magnetometer */
#define LIS3MDL_ADDRESS 0b0011110
#define LIS3MDL_WHO_ID  0x3D        // WHO_AM_I will report this value

/* Barometer */
#define LPS25H_ADDRESS  0b1011101
#define LPS25H_WHO_ID   0xBD        // WHO_AM_I will report this value



#define DS33_FUNC_CFG_ACCESS    0x01
#define DS33_FIFO_CTRL1         0x06
#define DS33_FIFO_CTRL2         0x07
#define DS33_FIFO_CTRL3         0x08
#define DS33_FIFO_CTRL4         0x09
#define DS33_FIFO_CTRL5         0x0A
#define DS33_ORIENT_CFG_G       0x0B
#define DS33_INT1_CTRL          0x0D
#define DS33_INT2_CTRL          0x0E
#define DS33_WHO_AM_I           0x0F
#define DS33_CTRL1_XL           0x10
#define DS33_CTRL2_G            0x11
#define DS33_CTRL3_C            0x12
#define DS33_CTRL4_C            0x13
#define DS33_CTRL5_C            0x14
#define DS33_CTRL6_C            0x15
#define DS33_CTRL7_G            0x16
#define DS33_CTRL8_XL           0x17
#define DS33_CTRL9_XL           0x18
#define DS33_CTRL10_C           0x19
#define DS33_WAKE_UP_SRC        0x1B
#define DS33_TAP_SRC            0x1C
#define DS33_D6D_SRC            0x1D
#define DS33_STATUS_REG         0x1E
#define DS33_OUT_TEMP_L         0x20
#define DS33_OUT_TEMP_H         0x21
#define DS33_OUTX_L_G           0x22
#define DS33_OUTX_H_G           0x23
#define DS33_OUTY_L_G           0x24
#define DS33_OUTY_H_G           0x25
#define DS33_OUTZ_L_G           0x26
#define DS33_OUTZ_H_G           0x27
#define DS33_OUTX_L_XL          0x28
#define DS33_OUTX_H_XL          0x29
#define DS33_OUTY_L_XL          0x2A
#define DS33_OUTY_H_XL          0x2B
#define DS33_OUTZ_L_XL          0x2C
#define DS33_OUTZ_H_XL          0x2D
#define DS33_FIFO_STATUS1       0x3A
#define DS33_FIFO_STATUS2       0x3B
#define DS33_FIFO_STATUS3       0x3C
#define DS33_FIFO_STATUS4       0x3D
#define DS33_FIFO_DATA_OUT_L    0x3E
#define DS33_FIFO_DATA_OUT_H    0x3F
#define DS33_TIMESTAMP0_REG     0x40
#define DS33_TIMESTAMP1_REG     0x41
#define DS33_TIMESTAMP2_REG     0x42
#define DS33_STEP_TIMESTAMP_L   0x49
#define DS33_STEP_TIMESTAMP_H   0x4A
#define DS33_STEP_COUNTER_L     0x4B
#define DS33_STEP_COUNTER_H     0x4C
#define DS33_FUNC_SRC           0x53
#define DS33_TAP_CFG            0x58
#define DS33_TAP_THS_6D         0x59
#define DS33_INT_DUR2           0x5A
#define DS33_WAKE_UP_THS        0x5B
#define DS33_WAKE_UP_DUR        0x5C
#define DS33_FREE_FALL          0x5D
#define DS33_MD1_CFG            0x5E
#define DS33_MD2_CFG            0x5F

#define LIS3MDL_WHO_AM_I           0x0F
#define LIS3MDL_CTRL_REG1          0x20
#define LIS3MDL_CTRL_REG2          0x21
#define LIS3MDL_CTRL_REG3          0x22
#define LIS3MDL_CTRL_REG4          0x23
#define LIS3MDL_CTRL_REG5          0x24
#define LIS3MDL_STATUS_REG         0x27
#define LIS3MDL_OUT_X_L            0x28
#define LIS3MDL_OUT_X_H            0x29
#define LIS3MDL_OUT_Y_L            0x2A
#define LIS3MDL_OUT_Y_H            0x2B
#define LIS3MDL_OUT_Z_L            0x2C
#define LIS3MDL_OUT_Z_H            0x2D
#define LIS3MDL_TEMP_OUT_L         0x2E
#define LIS3MDL_TEMP_OUT_H         0x2F
#define LIS3MDL_INT_CFG            0x30
#define LIS3MDL_INT_SRC            0x31
#define LIS3MDL_INT_THS_L          0x32
#define LIS3MDL_INT_THS_H          0x33


#define LPS25H_REF_P_XL            0x08
#define LPS25H_REF_P_L             0x09
#define LPS25H_REF_P_H             0x0A                              
#define LPS25H_WHO_AM_I            0x0F                              
#define LPS25H_RES_CONF            0x10                              
#define LPS25H_CTRL_REG1           0x20
#define LPS25H_CTRL_REG2           0x21
#define LPS25H_CTRL_REG3           0x22
#define LPS25H_CTRL_REG4           0x23
#define LPS25H_STATUS_REG          0x27                            
#define LPS25H_PRESS_OUT_XL        0x28
#define LPS25H_PRESS_OUT_L         0x29
#define LPS25H_PRESS_OUT_H         0x2A
#define LPS25H_TEMP_OUT_L          0x2B
#define LPS25H_TEMP_OUT_H          0x2C      
#define LPS25H_FIFO_CTRL           0x2E 
#define LPS25H_FIFO_STATUS         0x2F 
#define LPS25H_RPDS_L              0x39 
#define LPS25H_RPDS_H              0x3A      
#define LPS25H_INTERRUPT_CFG       0x24
#define LPS25H_INT_SOURCE          0x25
#define LPS25H_THS_P_L             0x30
#define LPS25H_THS_P_H             0x31

#endif /* __ALTIMU_10V5_H */