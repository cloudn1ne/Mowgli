EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 9
Title "Project Mowgli"
Date "2022-05-22"
Rev "1.0.1"
Comp ""
Comment1 "(c) CyberNet / cn@warp.at"
Comment2 "https://github.com/cloudn1ne/Mowgli"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_ST_STM32F1:STM32F103VCTx U?
U 1 1 628DE3A0
P 5450 3800
F 0 "U?" H 5450 911 50  0000 C CNN
F 1 "STM32F103VCT6" H 5450 820 50  0000 C CNN
F 2 "Package_QFP:LQFP-100_14x14mm_P0.5mm" H 4650 1200 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00191185.pdf" H 5450 3800 50  0001 C CNN
	1    5450 3800
	1    0    0    -1  
$EndComp
Text HLabel 3700 4500 0    50   Output ~ 0
PAC5223_nRESET1
Wire Wire Line
	4450 4500 3700 4500
Text HLabel 3700 5500 0    50   UnSpc ~ 0
PAC5210_PC4
Wire Wire Line
	4450 5500 3700 5500
Text HLabel 3700 4600 0    50   Output ~ 0
PAC5210_74366_nOE
Wire Wire Line
	4450 4600 3700 4600
Text HLabel 3700 5600 0    50   UnSpc ~ 0
PAC5210_PC3
Wire Wire Line
	4450 5600 3700 5600
Text HLabel 7200 5300 2    50   Output ~ 0
TF4_24V
Wire Wire Line
	6450 5300 7200 5300
Text HLabel 7200 3300 2    50   Output ~ 0
LED
Wire Wire Line
	7200 3300 6450 3300
Text HLabel 3700 4100 0    50   Input ~ 0
HC245_STM32_PE10
Wire Wire Line
	4450 4100 3700 4100
Text HLabel 3700 4300 0    50   Input ~ 0
HC245_STM32_PE12
Text HLabel 3700 4400 0    50   Input ~ 0
HC245_STM32_PE13
Wire Wire Line
	4450 4200 3700 4200
Text HLabel 3700 4200 0    50   Input ~ 0
HC245_STM32_PE11
Wire Wire Line
	4450 4300 3700 4300
Wire Wire Line
	4450 4400 3700 4400
Text HLabel 7200 4500 2    50   Input ~ 0
HC245_STM32_PB14
Wire Wire Line
	7200 4500 6450 4500
Text HLabel 3650 6100 0    50   Output ~ 0
HC245_STM32_PD13
Wire Wire Line
	4450 6100 3650 6100
Text HLabel 7250 3900 2    50   Output ~ 0
HC245_STM32_PB8
Wire Wire Line
	7250 3900 6450 3900
Text HLabel 7250 4000 2    50   Output ~ 0
HC245_STM32_PB9
Wire Wire Line
	7250 4000 6450 4000
Text HLabel 7250 4300 2    50   Output ~ 0
HC245_STM32_PB12
Wire Wire Line
	7250 4300 6450 4300
Text Notes 1750 5600 0    50   ~ 0
(HIGH for Drive Motor PAC enable)
Text Notes 1250 4650 0    50   ~ 0
(LOW for Drive Motor ULN Driver enable)
Text Notes 1500 4550 0    50   ~ 0
(HIGH for Blade Motor PAC enable)
Text HLabel 3700 5300 0    50   Output ~ 0
HC245_STM32_PD5
Text HLabel 3700 5400 0    50   Input ~ 0
HC245_STM32_PD6
Wire Wire Line
	4450 5300 3700 5300
Wire Wire Line
	4450 5400 3700 5400
Text Notes 2150 5400 0    50   ~ 0
(PAC5210 USART 2)
Text HLabel 7250 4100 2    50   Output ~ 0
HC245_STM32_PB10
Text HLabel 7250 4200 2    50   Input ~ 0
HC245_STM32_PB11
Wire Wire Line
	7250 4200 6450 4200
Wire Wire Line
	6450 4100 7250 4100
Text Notes 8100 4200 0    50   ~ 0
(PAC5223 USART 3)
Text HLabel 7200 5800 2    50   Output ~ 0
HC245_STM32_PC10
Text HLabel 7200 5900 2    50   Input ~ 0
HC245_STM32_PC11
Wire Wire Line
	7200 5900 6450 5900
Wire Wire Line
	7200 5800 6450 5800
Text Notes 8100 5900 0    50   ~ 0
(J18 (red) UART 4)
Text Notes 2500 4300 0    50   ~ 0
(4x PWM)
Text HLabel 7300 2300 2    50   Output ~ 0
J6_Pin_1
Text HLabel 7300 2400 2    50   Input ~ 0
J6_Pin_3
Wire Wire Line
	7300 2400 6450 2400
Wire Wire Line
	6450 2300 7300 2300
Text Notes 7750 2400 0    50   ~ 0
(Control Panel USART 1)
Text HLabel 7200 3400 2    50   Output ~ 0
SPI3_CLK
Text HLabel 7200 3500 2    50   Output ~ 0
SPI3_DO
Text HLabel 7200 3600 2    50   BiDi ~ 0
SPI3_DIO
Wire Wire Line
	7200 3400 6450 3400
Wire Wire Line
	7200 3500 6450 3500
Wire Wire Line
	7200 3600 6450 3600
Text HLabel 7300 2900 2    50   Output ~ 0
SPI3_FLASH_nCS
Wire Wire Line
	7300 2900 6450 2900
Text HLabel 7200 3700 2    50   Output ~ 0
ACCEL_SCL
Wire Wire Line
	7200 3700 6450 3700
Text HLabel 7200 3800 2    50   Output ~ 0
ACCEL_SDA
Wire Wire Line
	7200 3800 6450 3800
Text Notes 7700 3800 0    50   ~ 0
(I2C Accelerometer)
Text Notes 7650 3550 0    50   ~ 0
(SPI3 Flash)
$EndSCHEMATC
