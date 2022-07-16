#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

/* defines */
//#define GPIO_SW_I2C1_SCL           GPIOC
//#define GPIO_SW_I2C1_SCL_PIN   GPIO_Pin_0
//#define GPIO_SW_I2C1_SDA           GPIOB
//#define GPIO_SW_I2C1_SDA_PIN   GPIO_Pin_14

#define SW_I2C1		1
#define SW_I2C2		2
#define SW_I2C3		3
#define SW_I2C4		4
#define SW_I2C5		5
#define SW_I2C6		6
#define SW_I2C7		7
#define SW_I2C8		8
#define SW_I2C9		9
#define SW_I2C10	10

/* functions */
void SW_I2C_Init(void);
void SW_I2C_DeInit(void);

void i2c_port_initial(void);		

uint8_t SW_I2C_ReadVal_SDA(void);

void SW_I2C_Write_Data(uint8_t data);
uint8_t SW_I2C_Read_Data(void);

uint8_t SW_I2C_WriteControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t data);
uint8_t SW_I2C_WriteControl_8Bit_OnlyRegAddr(uint8_t IICID, uint8_t regaddr);
uint8_t SW_I2C_WriteControl_16Bit(uint8_t IICID, uint8_t regaddr, uint16_t data);

uint8_t SW_I2C_ReadControl_8Bit_OnlyRegAddr(uint8_t IICID, uint8_t regaddr);
uint8_t SW_I2C_ReadControl_8Bit_OnlyData(uint8_t IICID);
uint16_t SW_I2C_ReadControl_16Bit_OnlyData(uint8_t IICID);
uint8_t SW_I2C_ReadControl_8Bit(uint8_t IICID, uint8_t regaddr);
uint16_t SW_I2C_ReadControl_16Bit(uint8_t IICID, uint8_t regaddr);

uint8_t SW_I2C_ReadnControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata));
uint8_t SW_I2C_Multi_ReadnControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata));
uint8_t SW_I2C_Check_SlaveAddr(uint8_t IICID);

uint8_t SW_I2C_UTIL_WRITE(uint8_t IICID, uint8_t regaddr, uint8_t data);
uint8_t SW_I2C_UTIL_Read(uint8_t IICID, uint8_t regaddr);
uint8_t SW_I2C_UTIL_Read_Multi(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata));

#endif  /* __SOFT_I2C_H */