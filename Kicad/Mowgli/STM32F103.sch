EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 13
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
Text HLabel 3350 4500 0    50   Output ~ 0
PAC5223_nRESET1
Text HLabel 3350 5500 0    50   UnSpc ~ 0
PAC5210_PC4
Text HLabel 3350 4600 0    50   Output ~ 0
PAC5210_74366_nOE
Text HLabel 3350 5600 0    50   UnSpc ~ 0
PAC5210_PC3
Text HLabel 7200 5300 2    50   Output ~ 0
TF4_24V
Wire Wire Line
	6450 5300 7200 5300
Text HLabel 7200 3300 2    50   Output ~ 0
LED
Wire Wire Line
	7200 3300 6450 3300
Text HLabel 3350 4100 0    50   Input ~ 0
HC245_STM32_PE10
Text HLabel 3350 4300 0    50   Input ~ 0
HC245_STM32_PE12
Text HLabel 3350 4400 0    50   Input ~ 0
HC245_STM32_PE13
Text HLabel 3350 4200 0    50   Input ~ 0
HC245_STM32_PE11
Wire Wire Line
	4450 4400 3800 4400
Text HLabel 7200 4500 2    50   Input ~ 0
HC245_STM32_PB14
Wire Wire Line
	7200 4500 6450 4500
Text HLabel 3350 6100 0    50   Output ~ 0
HC245_STM32_PD13
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
Text Notes 1400 5600 0    50   ~ 0
(HIGH for Drive Motor PAC enable)
Text Notes 900  4650 0    50   ~ 0
(LOW for Drive Motor ULN Driver enable)
Text Notes 1150 4550 0    50   ~ 0
(HIGH for Blade Motor PAC enable)
Text HLabel 3350 5300 0    50   Output ~ 0
HC245_STM32_PD5
Text HLabel 3350 5400 0    50   Input ~ 0
HC245_STM32_PD6
Text Notes 1800 5400 0    50   ~ 0
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
Text Notes 2150 4300 0    50   ~ 0
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
(SPI3 Flash / J18)
Text HLabel 3350 3900 0    50   Output ~ 0
CHARGE_LOW_SIDE_DRIVER
Text HLabel 3350 4000 0    50   Output ~ 0
CHARGE_HIGHSIDE_DRIVER
Wire Wire Line
	7300 1700 6450 1700
Text HLabel 7300 2200 2    50   Input ~ 0
(ghidra:input,pulldown)
Wire Wire Line
	7300 2200 6450 2200
Text HLabel 3350 5000 0    50   Input ~ 0
(ghidra:input,pulldown)
Text HLabel 3350 5100 0    50   Input ~ 0
(ghidra:input,pulldown)
Text HLabel 3350 4900 0    50   Input ~ 0
(ghidra:input,pulldown)
Text HLabel 3350 4800 0    50   Input ~ 0
(ghidra:input,pulldown)
Text HLabel 7300 1400 2    50   Output ~ 0
(ghidra:output,pushpull,alternatefunc?)
Text HLabel 7300 1500 2    50   Output ~ 0
(ghidra:output,pushpull,alternatefunc?)-also_analog_in?
Wire Wire Line
	6450 1400 7300 1400
Wire Wire Line
	7300 1600 6450 1600
Wire Wire Line
	7300 1500 6450 1500
Text HLabel 7300 1900 2    50   Input ~ 0
(ghidra:ANALOG,input)
Text HLabel 7300 2000 2    50   Input ~ 0
(ghidra:ANALOG,input;PERIMETERSENSE)
Text HLabel 7300 2100 2    50   Input ~ 0
(ghidra:ANALOG,input)
Wire Wire Line
	7300 2100 6450 2100
Wire Wire Line
	6450 2000 7300 2000
Wire Wire Line
	7300 1900 6750 1900
Text HLabel 7200 5000 2    50   Input ~ 0
(ghidra:ANALOG,input)
Text HLabel 7200 5100 2    50   Input ~ 0
(ghidra:ANALOG,input)
Text HLabel 7200 5200 2    50   Input ~ 0
(ghidra:ANALOG,input)
Wire Wire Line
	7200 5200 6450 5200
Wire Wire Line
	6450 5100 7200 5100
Wire Wire Line
	7200 5000 6450 5000
Text HLabel 7200 4400 2    50   Input ~ 0
(ghidra:input,pullup)
Wire Wire Line
	7200 4400 6450 4400
Text HLabel 7200 5500 2    50   Input ~ 0
(ghidra:input,pullup)
Wire Wire Line
	7200 5500 6450 5500
Text HLabel 7200 3200 2    50   Output ~ 0
(ghidra:output,pushpull)
Wire Wire Line
	7200 3200 6450 3200
Text HLabel 7200 4900 2    50   Input ~ 0
(ghidra:input,pulldown)
Wire Wire Line
	7200 4900 6450 4900
Text HLabel 7200 4800 2    50   Input ~ 0
(ghidra:input,pulldown)
Wire Wire Line
	7200 4800 6450 4800
Text HLabel 7200 5600 2    50   Input ~ 0
(ghidra:input,pulldown)
Wire Wire Line
	7200 5600 6450 5600
Text HLabel 3350 3300 0    50   Input ~ 0
RAIN_Sensor_PE2
Text HLabel 7200 6000 2    50   Input ~ 0
(ghidra:input,pullup)
Wire Wire Line
	7200 6000 6450 6000
Text HLabel 3350 6200 0    50   Output ~ 0
(ghidra:output,pushpull)
Text HLabel 3350 5900 0    50   Output ~ 0
(ghidra:output,pushpull)
Text HLabel 3350 3400 0    50   Output ~ 0
(CHARGE?;ghidra:output,pushpull)
Wire Wire Line
	4450 3400 4350 3400
Text HLabel 3350 3500 0    50   Output ~ 0
(ghidra:output,pushpull)
Text HLabel 7300 1800 2    50   Input ~ 0
(ghidra:input,floating)
Wire Wire Line
	7300 1800 6500 1800
Text HLabel 7300 1700 2    50   Input ~ 0
BatteryVoltage
Text HLabel 7300 1600 2    50   Input ~ 0
Charge_Voltage
$Comp
L Connector:Conn_01x01_Male TP6
U 1 1 62BD479C
P 3800 2700
F 0 "TP6" V 3862 2744 50  0000 L CNB
F 1 "PE13" V 3750 2600 50  0000 L CNN
F 2 "" H 3800 2700 50  0001 C CNN
F 3 "~" H 3800 2700 50  0001 C CNN
	1    3800 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	3800 2900 3800 4400
$Comp
L Connector:Conn_01x01_Male TP1
U 1 1 62BDB441
P 6500 800
F 0 "TP1" V 6562 844 50  0000 L CNB
F 1 "PA4" V 6450 700 50  0000 L CNN
F 2 "" H 6500 800 50  0001 C CNN
F 3 "~" H 6500 800 50  0001 C CNN
	1    6500 800 
	0    1    1    0   
$EndComp
Wire Wire Line
	6500 1000 6500 1800
Connection ~ 6500 1800
Wire Wire Line
	6500 1800 6450 1800
$Comp
L Connector:Conn_01x01_Male TP2
U 1 1 62BDDD40
P 6750 800
F 0 "TP2" V 6812 844 50  0000 L CNB
F 1 "PA5" V 6700 700 50  0000 L CNN
F 2 "" H 6750 800 50  0001 C CNN
F 3 "~" H 6750 800 50  0001 C CNN
	1    6750 800 
	0    1    1    0   
$EndComp
Wire Wire Line
	6750 1000 6750 1900
Connection ~ 6750 1900
Wire Wire Line
	6750 1900 6450 1900
$Comp
L Connector:Conn_01x01_Male TP3
U 1 1 62BE0448
P 4350 6850
F 0 "TP3" V 4504 6762 50  0000 R CNN
F 1 "PD15" V 4413 6762 50  0000 R CNN
F 2 "" H 4350 6850 50  0001 C CNN
F 3 "~" H 4350 6850 50  0001 C CNN
	1    4350 6850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4450 6300 4350 6300
Wire Wire Line
	4350 6300 4350 6650
$Comp
L Connector:Conn_01x01_Male TP5
U 1 1 62BE67AB
P 4100 2700
F 0 "TP5" V 4162 2744 50  0000 L CNB
F 1 "PE4" V 4050 2600 50  0000 L CNN
F 2 "" H 4100 2700 50  0001 C CNN
F 3 "~" H 4100 2700 50  0001 C CNN
	1    4100 2700
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x01_Male TP4
U 1 1 62BE9166
P 4350 2700
F 0 "TP4" V 4412 2744 50  0000 L CNB
F 1 "PE3" V 4300 2600 50  0000 L CNN
F 2 "" H 4350 2700 50  0001 C CNN
F 3 "~" H 4350 2700 50  0001 C CNN
	1    4350 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 2900 4350 3400
Wire Wire Line
	4100 3500 4450 3500
Wire Wire Line
	3350 4400 3800 4400
Connection ~ 3800 4400
Wire Wire Line
	3350 4300 4450 4300
Wire Wire Line
	3350 4200 4450 4200
Wire Wire Line
	3350 4100 4450 4100
Wire Wire Line
	3350 4000 4450 4000
Wire Wire Line
	3350 3900 4450 3900
Wire Wire Line
	3350 3500 4100 3500
Connection ~ 4100 3500
Wire Wire Line
	3350 3400 4350 3400
Connection ~ 4350 3400
Wire Wire Line
	3350 3300 4450 3300
Wire Wire Line
	4100 2900 4100 3500
Wire Wire Line
	3350 4500 4450 4500
Wire Wire Line
	3350 4600 4450 4600
Wire Wire Line
	3350 4800 4450 4800
Wire Wire Line
	3350 4900 4450 4900
Wire Wire Line
	3350 5000 4450 5000
Wire Wire Line
	3350 5100 4450 5100
Wire Wire Line
	3350 5300 4450 5300
Wire Wire Line
	3350 5400 4450 5400
Wire Wire Line
	3350 5500 4450 5500
Wire Wire Line
	3350 5600 4450 5600
Wire Wire Line
	3350 6200 4450 6200
Wire Wire Line
	3350 6100 4450 6100
Wire Wire Line
	3350 5900 4450 5900
$EndSCHEMATC
