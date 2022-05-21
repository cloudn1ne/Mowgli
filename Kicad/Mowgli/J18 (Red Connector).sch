EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 9
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Conn_01x09_Male J?
U 1 1 6292746F
P 5000 3350
F 0 "J?" H 5108 3931 50  0000 C CNN
F 1 "J18 (Red Connector)" H 5108 3840 50  0000 C CNN
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
L power:GND #PWR?
U 1 1 629291C3
P 5450 3850
F 0 "#PWR?" H 5450 3600 50  0001 C CNN
F 1 "GND" H 5455 3677 50  0000 C CNN
F 2 "" H 5450 3850 50  0001 C CNN
F 3 "" H 5450 3850 50  0001 C CNN
	1    5450 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 3850 5450 3750
Wire Wire Line
	5450 3750 5200 3750
Text HLabel 6250 3050 2    50   UnSpc ~ 0
J18_Pin_2
Wire Wire Line
	6250 3050 5200 3050
Text HLabel 6250 3450 2    50   UnSpc ~ 0
J18_Pin_6
Wire Wire Line
	5200 3450 6250 3450
$EndSCHEMATC
