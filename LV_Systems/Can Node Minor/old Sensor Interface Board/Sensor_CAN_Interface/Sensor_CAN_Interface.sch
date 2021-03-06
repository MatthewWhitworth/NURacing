EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L teensy:Teensy4.0 U?
U 1 1 609E88F9
P 5950 3050
F 0 "U?" H 5950 4665 50  0000 C CNN
F 1 "Teensy4.0" H 5950 4574 50  0000 C CNN
F 2 "" H 5550 3250 50  0001 C CNN
F 3 "" H 5550 3250 50  0001 C CNN
	1    5950 3050
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM7805_TO220 U?
U 1 1 609EB91C
P 1450 900
F 0 "U?" H 1450 1142 50  0000 C CNN
F 1 "LM7805_TO220" H 1450 1051 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 1450 1125 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 1450 850 50  0001 C CNN
	1    1450 900 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 609EE13D
P 1900 1050
F 0 "C?" H 2015 1096 50  0000 L CNN
F 1 "0.1u" H 2015 1005 50  0000 L CNN
F 2 "" H 1938 900 50  0001 C CNN
F 3 "~" H 1900 1050 50  0001 C CNN
	1    1900 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 609EEF5E
P 1000 1050
F 0 "C?" H 1115 1096 50  0000 L CNN
F 1 "0.22u" H 1115 1005 50  0000 L CNN
F 2 "" H 1038 900 50  0001 C CNN
F 3 "~" H 1000 1050 50  0001 C CNN
	1    1000 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 609EF8AB
P 1450 1200
F 0 "#PWR?" H 1450 950 50  0001 C CNN
F 1 "GND" H 1455 1027 50  0000 C CNN
F 2 "" H 1450 1200 50  0001 C CNN
F 3 "" H 1450 1200 50  0001 C CNN
	1    1450 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 1200 1450 1200
Connection ~ 1450 1200
Wire Wire Line
	1450 1200 1000 1200
Wire Wire Line
	1900 900  1750 900 
Wire Wire Line
	1150 900  1000 900 
$Comp
L power:+12V #PWR?
U 1 1 609F0511
P 900 900
F 0 "#PWR?" H 900 750 50  0001 C CNN
F 1 "+12V" H 915 1073 50  0000 C CNN
F 2 "" H 900 900 50  0001 C CNN
F 3 "" H 900 900 50  0001 C CNN
	1    900  900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 900  1900 900 
Connection ~ 1900 900 
Wire Wire Line
	1000 900  900  900 
Connection ~ 1000 900 
$Comp
L Regulator_Switching:LM2576HVS-5 U?
U 1 1 609F8D48
P 1650 2000
F 0 "U?" H 1650 2367 50  0000 C CNN
F 1 "LM2576HVS-5" H 1650 2276 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-5_TabPin3" H 1650 1750 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2576.pdf" H 1650 2000 50  0001 C CNN
	1    1650 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 609F9C48
P 1650 2300
F 0 "#PWR?" H 1650 2050 50  0001 C CNN
F 1 "GND" H 1655 2127 50  0000 C CNN
F 2 "" H 1650 2300 50  0001 C CNN
F 3 "" H 1650 2300 50  0001 C CNN
	1    1650 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 609FA3CB
P 900 2100
F 0 "#PWR?" H 900 1850 50  0001 C CNN
F 1 "GND" H 905 1927 50  0000 C CNN
F 2 "" H 900 2100 50  0001 C CNN
F 3 "" H 900 2100 50  0001 C CNN
	1    900  2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 609FC2D3
P 900 2000
F 0 "C?" H 992 2046 50  0000 L CNN
F 1 "100u" H 992 1955 50  0000 L CNN
F 2 "" H 900 2000 50  0001 C CNN
F 3 "~" H 900 2000 50  0001 C CNN
	1    900  2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1900 900  1900
$Comp
L power:+12V #PWR?
U 1 1 609FE2F1
P 900 1900
F 0 "#PWR?" H 900 1750 50  0001 C CNN
F 1 "+12V" V 915 2028 50  0000 L CNN
F 2 "" H 900 1900 50  0001 C CNN
F 3 "" H 900 1900 50  0001 C CNN
	1    900  1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 609FE3F6
P 1150 2100
F 0 "#PWR?" H 1150 1850 50  0001 C CNN
F 1 "GND" H 1155 1927 50  0000 C CNN
F 2 "" H 1150 2100 50  0001 C CNN
F 3 "" H 1150 2100 50  0001 C CNN
	1    1150 2100
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N5822 D?
U 1 1 609FEE95
P 2150 2250
F 0 "D?" V 2104 2330 50  0000 L CNN
F 1 "1N5822" V 2195 2330 50  0000 L CNN
F 2 "Diode_THT:D_DO-201AD_P15.24mm_Horizontal" H 2150 2075 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 2150 2250 50  0001 C CNN
	1    2150 2250
	0    1    1    0   
$EndComp
$Comp
L Device:L_Small L?
U 1 1 609FFDDF
P 2300 2100
F 0 "L?" V 2119 2100 50  0000 C CNN
F 1 "330u" V 2210 2100 50  0000 C CNN
F 2 "" H 2300 2100 50  0001 C CNN
F 3 "~" H 2300 2100 50  0001 C CNN
	1    2300 2100
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 60A011A3
P 2600 2300
F 0 "C?" H 2692 2346 50  0000 L CNN
F 1 "330u" H 2692 2255 50  0000 L CNN
F 2 "" H 2600 2300 50  0001 C CNN
F 3 "~" H 2600 2300 50  0001 C CNN
	1    2600 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A0215A
P 2150 2400
F 0 "#PWR?" H 2150 2150 50  0001 C CNN
F 1 "GND" H 2155 2227 50  0000 C CNN
F 2 "" H 2150 2400 50  0001 C CNN
F 3 "" H 2150 2400 50  0001 C CNN
	1    2150 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A027F2
P 2600 2400
F 0 "#PWR?" H 2600 2150 50  0001 C CNN
F 1 "GND" H 2605 2227 50  0000 C CNN
F 2 "" H 2600 2400 50  0001 C CNN
F 3 "" H 2600 2400 50  0001 C CNN
	1    2600 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 2100 2200 2100
Connection ~ 2150 2100
Wire Wire Line
	2400 2100 2600 2100
Wire Wire Line
	2600 2100 2600 2200
Wire Wire Line
	2600 2100 2600 1900
Wire Wire Line
	2600 1900 2150 1900
Connection ~ 2600 2100
Connection ~ 900  1900
$Comp
L power:+5V #PWR?
U 1 1 60A04108
P 2600 1900
F 0 "#PWR?" H 2600 1750 50  0001 C CNN
F 1 "+5V" H 2615 2073 50  0000 C CNN
F 2 "" H 2600 1900 50  0001 C CNN
F 3 "" H 2600 1900 50  0001 C CNN
	1    2600 1900
	1    0    0    -1  
$EndComp
Connection ~ 2600 1900
$Comp
L power:+5V #PWR?
U 1 1 60A04976
P 2000 900
F 0 "#PWR?" H 2000 750 50  0001 C CNN
F 1 "+5V" H 2015 1073 50  0000 C CNN
F 2 "" H 2000 900 50  0001 C CNN
F 3 "" H 2000 900 50  0001 C CNN
	1    2000 900 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60A05384
P 1650 3250
F 0 "R?" H 1720 3296 50  0000 L CNN
F 1 "R" H 1720 3205 50  0000 L CNN
F 2 "" V 1580 3250 50  0001 C CNN
F 3 "~" H 1650 3250 50  0001 C CNN
	1    1650 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60A05F1D
P 1650 3650
F 0 "R?" H 1720 3696 50  0000 L CNN
F 1 "R" H 1720 3605 50  0000 L CNN
F 2 "" V 1580 3650 50  0001 C CNN
F 3 "~" H 1650 3650 50  0001 C CNN
	1    1650 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A06521
P 1650 3800
F 0 "#PWR?" H 1650 3550 50  0001 C CNN
F 1 "GND" H 1655 3627 50  0000 C CNN
F 2 "" H 1650 3800 50  0001 C CNN
F 3 "" H 1650 3800 50  0001 C CNN
	1    1650 3800
	1    0    0    -1  
$EndComp
Text Label 1650 3050 0    50   ~ 0
Sense1
Wire Wire Line
	1650 3050 1650 3100
Wire Wire Line
	1650 3400 1650 3450
Text Label 1800 3450 0    50   ~ 0
Analog1
Wire Wire Line
	1800 3450 1650 3450
Connection ~ 1650 3450
Wire Wire Line
	1650 3450 1650 3500
$Comp
L Connector:Conn_01x04_Female J?
U 1 1 60A07FBB
P 10300 750
F 0 "J?" H 10328 726 50  0000 L CNN
F 1 "Conn_01x04_Female" H 10328 635 50  0000 L CNN
F 2 "" H 10300 750 50  0001 C CNN
F 3 "~" H 10300 750 50  0001 C CNN
	1    10300 750 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x12_Female J?
U 1 1 60A091EB
P 10300 1600
F 0 "J?" H 10328 1576 50  0000 L CNN
F 1 "Conn_01x12_Female" H 10328 1485 50  0000 L CNN
F 2 "" H 10300 1600 50  0001 C CNN
F 3 "~" H 10300 1600 50  0001 C CNN
	1    10300 1600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J?
U 1 1 60A0DF5D
P 10300 2550
F 0 "J?" H 10328 2526 50  0000 L CNN
F 1 "Conn_01x06_Female" H 10328 2435 50  0000 L CNN
F 2 "" H 10300 2550 50  0001 C CNN
F 3 "~" H 10300 2550 50  0001 C CNN
	1    10300 2550
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 60A15349
P 10100 650
F 0 "#PWR?" H 10100 500 50  0001 C CNN
F 1 "+12V" V 10115 778 50  0000 L CNN
F 2 "" H 10100 650 50  0001 C CNN
F 3 "" H 10100 650 50  0001 C CNN
	1    10100 650 
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A15DDA
P 10100 750
F 0 "#PWR?" H 10100 500 50  0001 C CNN
F 1 "GND" V 10105 622 50  0000 R CNN
F 2 "" H 10100 750 50  0001 C CNN
F 3 "" H 10100 750 50  0001 C CNN
	1    10100 750 
	0    1    1    0   
$EndComp
Text Label 10100 850  2    50   ~ 0
Can_H
Text Label 10100 950  2    50   ~ 0
CAN_L
Text Label 10100 1100 2    50   ~ 0
Sense_1
Text Label 10100 1200 2    50   ~ 0
Sense_2
Text Label 10100 1300 2    50   ~ 0
Sense_3
Text Label 10100 1400 2    50   ~ 0
Sense_4
Text Label 10100 2350 2    50   ~ 0
Sense_5
Text Label 10100 2450 2    50   ~ 0
Sense_6
$Comp
L power:GND #PWR?
U 1 1 60A18915
P 10100 2200
F 0 "#PWR?" H 10100 1950 50  0001 C CNN
F 1 "GND" V 10105 2072 50  0000 R CNN
F 2 "" H 10100 2200 50  0001 C CNN
F 3 "" H 10100 2200 50  0001 C CNN
	1    10100 2200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A18FF4
P 10100 2100
F 0 "#PWR?" H 10100 1850 50  0001 C CNN
F 1 "GND" V 10105 1972 50  0000 R CNN
F 2 "" H 10100 2100 50  0001 C CNN
F 3 "" H 10100 2100 50  0001 C CNN
	1    10100 2100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A19C82
P 10100 2850
F 0 "#PWR?" H 10100 2600 50  0001 C CNN
F 1 "GND" V 10105 2722 50  0000 R CNN
F 2 "" H 10100 2850 50  0001 C CNN
F 3 "" H 10100 2850 50  0001 C CNN
	1    10100 2850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A19C88
P 10100 2750
F 0 "#PWR?" H 10100 2500 50  0001 C CNN
F 1 "GND" V 10105 2622 50  0000 R CNN
F 2 "" H 10100 2750 50  0001 C CNN
F 3 "" H 10100 2750 50  0001 C CNN
	1    10100 2750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A1A2E1
P 10100 2000
F 0 "#PWR?" H 10100 1750 50  0001 C CNN
F 1 "GND" V 10105 1872 50  0000 R CNN
F 2 "" H 10100 2000 50  0001 C CNN
F 3 "" H 10100 2000 50  0001 C CNN
	1    10100 2000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60A1A2E7
P 10100 1900
F 0 "#PWR?" H 10100 1650 50  0001 C CNN
F 1 "GND" V 10105 1772 50  0000 R CNN
F 2 "" H 10100 1900 50  0001 C CNN
F 3 "" H 10100 1900 50  0001 C CNN
	1    10100 1900
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60A1A892
P 10100 2550
F 0 "#PWR?" H 10100 2400 50  0001 C CNN
F 1 "+5V" V 10115 2678 50  0000 L CNN
F 2 "" H 10100 2550 50  0001 C CNN
F 3 "" H 10100 2550 50  0001 C CNN
	1    10100 2550
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60A1B003
P 10100 2650
F 0 "#PWR?" H 10100 2500 50  0001 C CNN
F 1 "+5V" V 10115 2778 50  0000 L CNN
F 2 "" H 10100 2650 50  0001 C CNN
F 3 "" H 10100 2650 50  0001 C CNN
	1    10100 2650
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60A1BE18
P 10100 1700
F 0 "#PWR?" H 10100 1550 50  0001 C CNN
F 1 "+5V" V 10115 1828 50  0000 L CNN
F 2 "" H 10100 1700 50  0001 C CNN
F 3 "" H 10100 1700 50  0001 C CNN
	1    10100 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60A1BE1E
P 10100 1800
F 0 "#PWR?" H 10100 1650 50  0001 C CNN
F 1 "+5V" V 10115 1928 50  0000 L CNN
F 2 "" H 10100 1800 50  0001 C CNN
F 3 "" H 10100 1800 50  0001 C CNN
	1    10100 1800
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60A1C5EB
P 10100 1500
F 0 "#PWR?" H 10100 1350 50  0001 C CNN
F 1 "+5V" V 10115 1628 50  0000 L CNN
F 2 "" H 10100 1500 50  0001 C CNN
F 3 "" H 10100 1500 50  0001 C CNN
	1    10100 1500
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60A1C5F1
P 10100 1600
F 0 "#PWR?" H 10100 1450 50  0001 C CNN
F 1 "+5V" V 10115 1728 50  0000 L CNN
F 2 "" H 10100 1600 50  0001 C CNN
F 3 "" H 10100 1600 50  0001 C CNN
	1    10100 1600
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
