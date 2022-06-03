/**
  ******************************************************************************
  * @file    soft_i2c.c
  * @author  Georg Swoboda <cn@warp.at>
  * @brief   Software I2C driver for pins on J18
  ******************************************************************************
  * @attention
  *
  * mostly reworked code from: https://schkorea.tistory.com/437
  * note that you need to turn of JTAG (but not SWD) to free the pins that go to
  * J18 !
  ******************************************************************************
  */

#include "stm32f1xx_hal.h"
#include "soft_i2c.h"
#include "board.h"



#define  SW_I2C_WAIT_TIME  25	//(11.0us)
//#define  SW_I2C_WAIT_TIME  23	//(10.4us)
//#define  SW_I2C_WAIT_TIME  22	//100Khz(10.0us)	100Khz	==	10us
//#define  SW_I2C_WAIT_TIME  10	//195Khz
//#define  SW_I2C_WAIT_TIME  9	//205Khz	200Khz	==	5us
//#define  SW_I2C_WAIT_TIME  8	//237Khz
//#define  SW_I2C_WAIT_TIME  7	//240Khz	250Khz	==	4us
//#define  SW_I2C_WAIT_TIME  6	//275Khz
//#define  SW_I2C_WAIT_TIME  5	//305Khz
//#define  SW_I2C_WAIT_TIME  4	//350Khz(3.84us)
//#define  SW_I2C_WAIT_TIME  3	//400Khz(3.44us)
//#define  SW_I2C_WAIT_TIME  2	//425Khz(3.04us)	333Khz	==	3us
//#define  SW_I2C_WAIT_TIME  1	//425Khz(2.64us)	400Khz	==	2.5us

#define TRUE 1
#define FALSE 0

#define  I2C_READ       0x01
#define  READ_CMD       1
#define  WRITE_CMD      0

// map from board.h to what soft_i2c uses
#define SW_I2C1_SCL_GPIO  SOFT_I2C_SCL_PORT
#define SW_I2C1_SDA_GPIO  SOFT_I2C_SDA_PORT
#define SW_I2C1_SCL_PIN   SOFT_I2C_SCL_PIN
#define SW_I2C1_SDA_PIN   SOFT_I2C_SDA_PIN



void TIMER__Wait_us(uint32_t nCount)
{
    for (; nCount != 0;nCount--);
}

/* init soft i2c pins */
void SW_I2C_Init(void)
{
    
    /* PB3, PB4 are used by the JTAG - we need to disable it, as we use SWD anyhow we dont need it */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable A.F. clock
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // JTAG is disabled, SWD is enabled


    SOFT_I2C_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;


    GPIO_InitStruct.Pin   = SW_I2C1_SCL_PIN;
    HAL_GPIO_Init(SW_I2C1_SCL_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = SW_I2C1_SDA_PIN;
    HAL_GPIO_Init(SW_I2C1_SDA_GPIO, &GPIO_InitStruct);    

}

// SDA High
void sda_high(void)
{ 
    //debug_printf("sda_1\r\n");
    // GPIO_SetBits(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN);    
    HAL_GPIO_WritePin(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN, GPIO_PIN_SET);
    //  HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_3 , 1);
}

// SDA low
void sda_low(void)
{
    //debug_printf("sda_0\r\n");
    // GPIO_ResetBits(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN);  
    HAL_GPIO_WritePin(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_3 , 0);
}

// SCL High
void scl_high(void)
{   
    //debug_printf("scl_1\r\n"); 
    // GPIO_SetBits(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN);  
    HAL_GPIO_WritePin(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN, GPIO_PIN_SET);
}

// SCL low
void scl_low(void)
{    
    //debug_printf("scl_0\r\n");
    // GPIO_ResetBits(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN);
    HAL_GPIO_WritePin(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN, GPIO_PIN_RESET);
}

void sda_out(uint8_t out)
{
    if (out)
    {
        sda_high();
    }
    else
    {
        sda_low();
    }
}

void sda_in_mode(void)
{
  //  debug_printf("sda_in_mode()\r\n");
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;	//IPD->IPU
	GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin   = SW_I2C1_SDA_PIN;
    HAL_GPIO_Init(SW_I2C1_SDA_GPIO, &GPIO_InitStruct);  
}

void sda_out_mode(void)
{
    //debug_printf("sda_out_mode()\r\n");

    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;


    GPIO_InitStruct.Pin   = SW_I2C1_SDA_PIN;
    HAL_GPIO_Init(SW_I2C1_SDA_GPIO, &GPIO_InitStruct);  
}

void scl_in_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;	//IPD->IPU
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin   = SW_I2C1_SCL_PIN;
    HAL_GPIO_Init(SW_I2C1_SCL_GPIO, &GPIO_InitStruct);
  
}

void scl_out_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;		// error point GPIO_Mode_Out_PP	

    GPIO_InitStruct.Pin   = SW_I2C1_SCL_PIN;
    HAL_GPIO_Init(SW_I2C1_SCL_GPIO, &GPIO_InitStruct);
}

void i2c_clk_data_out(void)
{
    scl_high();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_low();
//    TIMER__Wait_us(SW_I2C_WAIT_TIME>>2);
}

void i2c_port_initial(void)
{
    sda_high();
    scl_high();
}

void i2c_start_condition(void)
{
    sda_high();
    scl_high();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    sda_low();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_low();

    TIMER__Wait_us(SW_I2C_WAIT_TIME << 1);
}

void i2c_stop_condition(void)
{
    sda_low();
    scl_high();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    sda_high();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
}

uint8_t i2c_check_ack(void)
{
    uint8_t         ack;
    int             i;
    unsigned int    temp;

    sda_in_mode();

    scl_high();

    ack = FALSE;
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    for (i = 10; i > 0; i--)
    {
        temp = !(SW_I2C_ReadVal_SDA());	//0=ack , 1=nack
        if (temp)	// if ack, enter
        {
            ack = TRUE;
            break;
        }
    }
    scl_low();
    sda_out_mode();	//during setting, sda signal high

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    return ack;
}

void i2c_check_not_ack(void)
{
    sda_in_mode();
    i2c_clk_data_out();
    sda_out_mode();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
}

void i2c_check_not_ack_continue(void)
{
//    sda_in_mode();
    i2c_clk_data_out();
//    sda_out_mode();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);
}

void i2c_slave_address(uint8_t IICID, uint8_t readwrite)
{
    int x;

    if (readwrite)
    {
        IICID |= I2C_READ;
    }
    else
    {
        IICID &= ~I2C_READ;
    }

    scl_low();

    for (x = 7; x >= 0; x--)
    {
        sda_out(IICID & (1 << x));
        TIMER__Wait_us(SW_I2C_WAIT_TIME);
        i2c_clk_data_out();
//        TIMER__Wait_us(SW_I2C_WAIT_TIME);
    }
}

void i2c_register_address(uint8_t addr)
{
    int  x;

    scl_low();

    for (x = 7; x >= 0; x--)
    {
        sda_out(addr & (1 << x));
        TIMER__Wait_us(SW_I2C_WAIT_TIME);
        i2c_clk_data_out();
//        TIMER__Wait_us(SW_I2C_WAIT_TIME);
    }
}

void i2c_send_ack(void)
{
    sda_out_mode();
    sda_low();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
    scl_high();

    TIMER__Wait_us(SW_I2C_WAIT_TIME << 1);

    sda_low();
    TIMER__Wait_us(SW_I2C_WAIT_TIME << 1);

    scl_low();

    sda_out_mode();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
}

/* external functions */
uint8_t SW_I2C_ReadVal_SDA(void)
{    
    return HAL_GPIO_ReadPin(SW_I2C1_SDA_GPIO, SW_I2C1_SDA_PIN);       
}

uint8_t SW_I2C_ReadVal_SCL(void)
{ 
    return HAL_GPIO_ReadPin(SW_I2C1_SCL_GPIO, SW_I2C1_SCL_PIN);    
}

void SW_I2C_Write_Data(uint8_t data)
{
    int  x;

    scl_low();

    for (x = 7; x >= 0; x--)
    {
        sda_out(data & (1 << x));
        TIMER__Wait_us(SW_I2C_WAIT_TIME);
        i2c_clk_data_out();
//        TIMER__Wait_us(SW_I2C_WAIT_TIME);
    }
}

uint8_t SW_I2C_Read_Data(void)
{
    int      x;
    uint8_t  readdata = 0;

    sda_in_mode();

    for (x = 8; x--;)
    {
        scl_high();

        readdata <<= 1;
        if (SW_I2C_ReadVal_SDA())
            readdata |= 0x01;

        TIMER__Wait_us(SW_I2C_WAIT_TIME);
        scl_low();

        TIMER__Wait_us(SW_I2C_WAIT_TIME);
    }

    sda_out_mode();
    return readdata;
}

uint8_t SW_I2C_WriteControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t data)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_register_address(regaddr);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    SW_I2C_Write_Data(data);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_WriteControl_8Bit_OnlyRegAddr(uint8_t IICID, uint8_t regaddr)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    i2c_register_address(regaddr);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

//    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_WriteControl_16Bit(uint8_t IICID, uint8_t regaddr, uint16_t data)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_register_address(regaddr);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    SW_I2C_Write_Data((data >> 8) & 0xFF);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    SW_I2C_Write_Data(data & 0xFF);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_ReadControl_8Bit_OnlyRegAddr(uint8_t IICID, uint8_t regaddr)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_register_address(regaddr);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_ReadControl_8Bit_OnlyData(uint8_t IICID)
{
    uint8_t  readdata = 0;

    i2c_port_initial();

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    i2c_check_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readdata = SW_I2C_Read_Data();

    i2c_check_not_ack();

    i2c_stop_condition();

    return readdata;
}

uint16_t SW_I2C_ReadControl_16Bit_OnlyData(uint8_t IICID)
{
    uint8_t  readimsi = 0;
    uint16_t  readdata = 0;

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    i2c_check_not_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readimsi = SW_I2C_Read_Data();
    i2c_check_not_ack_continue();

    readdata = readimsi<<8;

    readimsi = SW_I2C_Read_Data();
    i2c_check_not_ack();


    readdata |= readimsi;

    i2c_stop_condition();

    return readdata;
}

uint8_t SW_I2C_ReadControl_8Bit(uint8_t IICID, uint8_t regaddr)
{
    uint8_t  readdata = 0;

    i2c_port_initial();

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    i2c_check_ack();

    i2c_register_address(regaddr);
    i2c_check_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    i2c_check_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readdata = SW_I2C_Read_Data();

    i2c_check_not_ack();

    i2c_stop_condition();

    return readdata;
}

uint16_t SW_I2C_ReadControl_16Bit(uint8_t IICID, uint8_t regaddr)
{
    uint16_t  readdata = 0;

    i2c_port_initial();

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    i2c_check_ack();

    i2c_register_address(regaddr);
    i2c_check_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    i2c_check_ack();

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readdata = SW_I2C_Read_Data();
    i2c_send_ack();
    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    readdata = ((readdata << 8) | SW_I2C_Read_Data());

    i2c_check_not_ack();

    i2c_stop_condition();

    return readdata;
}

uint8_t SW_I2C_ReadnControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata))
{
    uint8_t   returnack = TRUE;
    uint8_t  index;

    i2c_port_initial();

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack()) { returnack = FALSE; }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
	
    i2c_register_address(regaddr);
    if (!i2c_check_ack()) { returnack = FALSE; }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    if (!i2c_check_ack()) { returnack = FALSE; }

    for ( index = 0 ; index < rcnt ; index++){
    	TIMER__Wait_us(SW_I2C_WAIT_TIME);
    	pdata[index] = SW_I2C_Read_Data();
    }

    pdata[rcnt-1] = SW_I2C_Read_Data();
	
    i2c_check_not_ack();

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_Multi_ReadnControl_8Bit(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata))
{
    uint8_t   returnack = TRUE;
    uint8_t  index;

    i2c_port_initial();

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack()) { returnack = FALSE; }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);
	
    i2c_register_address(regaddr);
    if (!i2c_check_ack()) { returnack = FALSE; }

    TIMER__Wait_us(SW_I2C_WAIT_TIME);

    i2c_start_condition();

    i2c_slave_address(IICID, READ_CMD);
    if (!i2c_check_ack()) { returnack = FALSE; }

    for ( index = 0 ; index < (rcnt-1) ; index++){
    	TIMER__Wait_us(SW_I2C_WAIT_TIME);
    	pdata[index] = SW_I2C_Read_Data();
	i2c_send_ack();
	//if (!i2c_check_ack()) { returnack = FALSE; }
    }

    pdata[rcnt-1] = SW_I2C_Read_Data();
	
    i2c_check_not_ack();

    i2c_stop_condition();

    return returnack;
}

uint8_t SW_I2C_Check_SlaveAddr(uint8_t IICID)
{
    uint8_t   returnack = TRUE;

    i2c_start_condition();

    i2c_slave_address(IICID, WRITE_CMD);
    if (!i2c_check_ack())
    {
        returnack = FALSE;
    }
	
    return returnack;
}

uint8_t SW_I2C_UTIL_WRITE(uint8_t IICID, uint8_t regaddr, uint8_t data)
{
	return SW_I2C_WriteControl_8Bit(IICID<<1, regaddr, data);
}

uint8_t SW_I2C_UTIL_Read(uint8_t IICID, uint8_t regaddr)
{
	return SW_I2C_ReadControl_8Bit(IICID<<1, regaddr);
}

uint8_t SW_I2C_UTIL_Read_Multi(uint8_t IICID, uint8_t regaddr, uint8_t rcnt, uint8_t (*pdata))
{
	return SW_I2C_Multi_ReadnControl_8Bit(IICID<<1, regaddr, rcnt, pdata);
}
