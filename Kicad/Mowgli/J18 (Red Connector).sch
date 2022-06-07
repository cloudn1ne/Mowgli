EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 12
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
L Connector:Conn_01x09_Male J18
U 1 1 6292746F
P 5000 3350
F 0 "J18" H 5108 3931 50  0000 C CNN
F 1 "(Red Connector)" H 5108 3840 50  0000 C CNN
F 2 "" H 5000 3350 50  0001 C CNN
F 3 "~" H 5000 3350 50  0001 C CNN
	1    5000 3350
	1    0    0    -1  
$EndComp
Text HLabel 6250 2950 2    50   Input ~ 0
J18_Pin_1
Wire Wire Line
	6250 2950 5200 2950
Text HLabel 6250 3550 2    50   Output ~ 0
J18_Pin_7
Text HLabel 6250 3650 2    50   Input ~ 0
J18_Pin_8
Text Notes 6750 3650 0    50   Italic 10
UART 4
Wire Wire Line
	6250 3650 5200 3650
Wire Wire Line
	5200 3550 6250 3550
$Comp
L power:GND #PWR0109
U 1 1 629291C3
P 6250 3750
F 0 "#PWR0109" H 6250 3500 50  0001 C CNN
F 1 "GND" H 6255 3577 50  0000 C CNN
F 2 "" H 6250 3750 50  0001 C CNN
F 3 "" H 6250 3750 50  0001 C CNN
	1    6250 3750
	1    0    0    -1  
$EndComp
Text HLabel 6250 3050 2    50   UnSpc ~ 0
J18_Pin_2
Wire Wire Line
	6250 3050 5200 3050
Text HLabel 6250 3450 2    50   UnSpc ~ 0
J18_Pin_6
Wire Wire Line
	5200 3450 6250 3450
Text Notes 7100 3300 0    50   Italic 10
SPI 3
Wire Wire Line
	6250 3150 5200 3150
Wire Wire Line
	5200 3250 6250 3250
Wire Wire Line
	6250 3350 5200 3350
Text HLabel 6250 3350 2    50   Input ~ 0
J18_Pin_5_SPI3_CLK
Text HLabel 6250 3250 2    50   Input ~ 0
J18_Pin_4_SPI3_DO
Text HLabel 6250 3150 2    50   Output ~ 0
J18_Pin_3_SPI3_DI
Wire Wire Line
	5200 3750 6250 3750
$EndSCHEMATC
