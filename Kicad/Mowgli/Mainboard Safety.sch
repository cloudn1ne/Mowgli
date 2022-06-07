EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 13 13
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 7100 1350 2    50   Input ~ 0
J6_Pin_16_Wheel_red
Text HLabel 1550 900  0    50   Input ~ 0
J6_Pin_10_Wheel_blue
$Comp
L 4xxx:4075 U3
U 1 1 62BC3F53
P 2400 2050
F 0 "U3" H 2400 2375 50  0000 C CNN
F 1 "4075" H 2400 2284 50  0000 C CNN
F 2 "" H 2400 2050 50  0001 C CNN
F 3 "http://www.intersil.com/content/dam/Intersil/documents/cd40/cd4071bms-72bms-75bms.pdf" H 2400 2050 50  0001 C CNN
	1    2400 2050
	1    0    0    -1  
$EndComp
$Comp
L 4xxx:4075 U3
U 2 1 62BC6CC0
P 2400 2600
F 0 "U3" H 2400 2925 50  0000 C CNN
F 1 "4075" H 2400 2834 50  0000 C CNN
F 2 "" H 2400 2600 50  0001 C CNN
F 3 "http://www.intersil.com/content/dam/Intersil/documents/cd40/cd4071bms-72bms-75bms.pdf" H 2400 2600 50  0001 C CNN
	2    2400 2600
	1    0    0    -1  
$EndComp
$Comp
L 4xxx:4075 U3
U 3 1 62BC7874
P 2400 3150
F 0 "U3" H 2400 3475 50  0000 C CNN
F 1 "4075" H 2400 3384 50  0000 C CNN
F 2 "" H 2400 3150 50  0001 C CNN
F 3 "http://www.intersil.com/content/dam/Intersil/documents/cd40/cd4071bms-72bms-75bms.pdf" H 2400 3150 50  0001 C CNN
	3    2400 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Network04_Split RP6
U 1 1 62BCA3FA
P 4000 1350
F 0 "RP6" H 4088 1396 50  0000 L CNN
F 1 "4.7k" H 4088 1305 50  0000 L CNN
F 2 "Resistor_THT:R_Array_SIP5" V 3920 1350 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 4000 1350 50  0001 C CNN
	1    4000 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Network04_Split RP6
U 2 1 62BCADFA
P 4350 1350
F 0 "RP6" H 4448 1388 50  0000 L CNN
F 1 "4.7k" H 4448 1297 50  0000 L CNN
F 2 "Resistor_THT:R_Array_SIP5" V 4270 1350 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 4350 1350 50  0001 C CNN
	2    4350 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Network04_Split RP6
U 3 1 62BCB8D2
P 4750 1350
F 0 "RP6" H 4848 1388 50  0000 L CNN
F 1 "4.7k" H 4848 1297 50  0000 L CNN
F 2 "Resistor_THT:R_Array_SIP5" V 4670 1350 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 4750 1350 50  0001 C CNN
	3    4750 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Network04_Split RP6
U 4 1 62BCC566
P 5150 1350
F 0 "RP6" H 5248 1388 50  0000 L CNN
F 1 "4.7k" H 5248 1297 50  0000 L CNN
F 2 "Resistor_THT:R_Array_SIP5" V 5070 1350 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 5150 1350 50  0001 C CNN
	4    5150 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 1050 4750 1050
Wire Wire Line
	4000 1050 4000 1200
Wire Wire Line
	4350 1200 4350 1050
Connection ~ 4350 1050
Wire Wire Line
	4350 1050 4000 1050
Wire Wire Line
	4750 1200 4750 1050
Connection ~ 4750 1050
Wire Wire Line
	4750 1050 4550 1050
Wire Wire Line
	5150 1200 5150 1050
$Comp
L power:+5V #PWR?
U 1 1 62BD1505
P 4550 800
F 0 "#PWR?" H 4550 650 50  0001 C CNN
F 1 "+5V" H 4565 973 50  0000 C CNN
F 2 "" H 4550 800 50  0001 C CNN
F 3 "" H 4550 800 50  0001 C CNN
	1    4550 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 800  4550 1050
Connection ~ 4550 1050
Wire Wire Line
	4550 1050 4350 1050
Wire Wire Line
	5150 1500 5150 1700
Wire Wire Line
	5150 1700 5550 1700
$Comp
L Device:R R121
U 1 1 62BD2C00
P 6900 1550
F 0 "R121" H 6970 1596 50  0000 L CNN
F 1 "22" H 6970 1505 50  0000 L CNN
F 2 "" V 6830 1550 50  0001 C CNN
F 3 "~" H 6900 1550 50  0001 C CNN
	1    6900 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1350 6900 1350
Wire Wire Line
	6900 1350 6900 1400
$EndSCHEMATC
