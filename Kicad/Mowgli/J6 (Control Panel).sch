EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 9
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
L Connector:Conn_01x16_Male J6
U 1 1 629324E6
P 4900 2850
F 0 "J6" H 5008 3731 50  0000 C CNN
F 1 "Control Panel" H 5008 3640 50  0000 C CNN
F 2 "" H 4900 2850 50  0001 C CNN
F 3 "~" H 4900 2850 50  0001 C CNN
	1    4900 2850
	1    0    0    -1  
$EndComp
Text HLabel 5800 2950 2    50   UnSpc ~ 0
J6_Pin_9
Wire Wire Line
	5800 2950 5100 2950
Text HLabel 5800 2150 2    50   Input ~ 0
J6_Pin_1
Text HLabel 5800 2350 2    50   Output ~ 0
J6_Pin_3
Wire Wire Line
	5800 2150 5100 2150
Wire Wire Line
	5800 2350 5100 2350
Text Notes 6200 2400 0    50   ~ 0
USART 1 (RX)
Text Notes 6200 2200 0    50   ~ 0
USART 1 (TX)
$EndSCHEMATC
