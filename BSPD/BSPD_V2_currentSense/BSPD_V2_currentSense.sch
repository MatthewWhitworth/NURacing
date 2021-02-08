EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "BSPD"
Date "2019-09-05"
Rev "V01"
Comp "NURacing"
Comment1 "Matthew Whitworth"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 1450 750  0    79   ~ 0
I/O
$Comp
L power:GND #PWR07
U 1 1 5D72DEB9
P 1150 1150
F 0 "#PWR07" H 1150 900 50  0001 C CNN
F 1 "GND" V 1155 1022 50  0000 R CNN
F 2 "" H 1150 1150 50  0001 C CNN
F 3 "" H 1150 1150 50  0001 C CNN
	1    1150 1150
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x02_Female J2
U 1 1 5D71C6D0
P 950 1250
F 0 "J2" H 850 1050 50  0000 C CNN
F 1 "Conn_01x02_Female" H 400 1050 50  0000 C CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-0212_2x01_P3.00mm_Vertical" H 950 1250 50  0001 C CNN
F 3 "~" H 950 1250 50  0001 C CNN
	1    950  1250
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR014
U 1 1 5E6C471C
P 2850 1250
F 0 "#PWR014" H 2850 1100 50  0001 C CNN
F 1 "+5V" V 2865 1423 50  0000 C CNN
F 2 "" H 2850 1250 50  0001 C CNN
F 3 "" H 2850 1250 50  0001 C CNN
	1    2850 1250
	0    -1   -1   0   
$EndComp
$Comp
L Device:Q_PMOS_DGS Q1
U 1 1 5E6CDF2B
P 3050 1350
F 0 "Q1" H 3256 1396 50  0000 L CNN
F 1 "Q_PMOS_DGS" H 3256 1305 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3250 1450 50  0001 C CNN
F 3 "~" H 3050 1350 50  0001 C CNN
	1    3050 1350
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5E6CFE10
P 3050 1550
F 0 "#PWR08" H 3050 1300 50  0001 C CNN
F 1 "GND" H 3055 1422 50  0000 R CNN
F 2 "" H 3050 1550 50  0001 C CNN
F 3 "" H 3050 1550 50  0001 C CNN
	1    3050 1550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR015
U 1 1 5E6D270A
P 3250 1250
F 0 "#PWR015" H 3250 1100 50  0001 C CNN
F 1 "VCC" V 3268 1423 50  0000 C CNN
F 2 "" H 3250 1250 50  0001 C CNN
F 3 "" H 3250 1250 50  0001 C CNN
	1    3250 1250
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 5E6B7FDF
P 1150 1250
F 0 "#PWR03" H 1150 1100 50  0001 C CNN
F 1 "+5V" V 1165 1423 50  0000 C CNN
F 2 "" H 1150 1250 50  0001 C CNN
F 3 "" H 1150 1250 50  0001 C CNN
	1    1150 1250
	0    1    1    0   
$EndComp
$Comp
L Amplifier_Operational:MCP6002-xSN U1
U 1 1 602168CE
P 3950 2400
F 0 "U1" H 3950 2767 50  0000 C CNN
F 1 "MCP6002-xSN" H 3950 2676 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 3950 2400 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 3950 2400 50  0001 C CNN
	1    3950 2400
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP6002-xSN U1
U 2 1 602189CA
P 3950 3050
F 0 "U1" H 3950 3417 50  0000 C CNN
F 1 "MCP6002-xSN" H 3950 3326 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 3950 3050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 3950 3050 50  0001 C CNN
	2    3950 3050
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP6002-xSN U1
U 3 1 6021B63F
P 1350 3050
F 0 "U1" H 1308 3096 50  0000 L CNN
F 1 "MCP6002-xSN" H 1308 3005 50  0000 L CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 1350 3050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 1350 3050 50  0001 C CNN
	3    1350 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 6021E946
P 1000 3050
F 0 "C1" H 1115 3096 50  0000 L CNN
F 1 "0.1u" H 1115 3005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1038 2900 50  0001 C CNN
F 3 "~" H 1000 3050 50  0001 C CNN
	1    1000 3050
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR04
U 1 1 6021F3D5
P 1250 2750
F 0 "#PWR04" H 1250 2600 50  0001 C CNN
F 1 "VCC" H 1265 2923 50  0000 C CNN
F 2 "" H 1250 2750 50  0001 C CNN
F 3 "" H 1250 2750 50  0001 C CNN
	1    1250 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 6021FFEF
P 1250 3350
F 0 "#PWR05" H 1250 3100 50  0001 C CNN
F 1 "GND" H 1255 3177 50  0000 C CNN
F 2 "" H 1250 3350 50  0001 C CNN
F 3 "" H 1250 3350 50  0001 C CNN
	1    1250 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2750 1000 2750
Wire Wire Line
	1000 2750 1000 2900
Connection ~ 1250 2750
Wire Wire Line
	1000 3350 1250 3350
Wire Wire Line
	1000 3200 1000 3350
Connection ~ 1250 3350
Text Label 4250 2400 0    50   ~ 0
currentOut
Text Label 4250 3050 0    50   ~ 0
refOut
Wire Wire Line
	4250 3050 4250 3300
Wire Wire Line
	4250 3300 3650 3300
Wire Wire Line
	3650 3300 3650 3150
Wire Wire Line
	3650 2500 3650 2650
Wire Wire Line
	3650 2650 4250 2650
Wire Wire Line
	4250 2650 4250 2400
$Comp
L Device:D_Zener D1
U 1 1 60223032
P 2950 3150
F 0 "D1" V 2904 3230 50  0000 L CNN
F 1 "2.4V" V 2995 3230 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2950 3150 50  0001 C CNN
F 3 "~" H 2950 3150 50  0001 C CNN
	1    2950 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 60224BC9
P 2700 2950
F 0 "R1" V 2493 2950 50  0000 C CNN
F 1 "10k?" V 2584 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2630 2950 50  0001 C CNN
F 3 "~" H 2700 2950 50  0001 C CNN
	1    2700 2950
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR06
U 1 1 60226720
P 2550 2950
F 0 "#PWR06" H 2550 2800 50  0001 C CNN
F 1 "VCC" V 2565 3077 50  0000 L CNN
F 2 "" H 2550 2950 50  0001 C CNN
F 3 "" H 2550 2950 50  0001 C CNN
	1    2550 2950
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 602271C8
P 2950 3300
F 0 "#PWR09" H 2950 3050 50  0001 C CNN
F 1 "GND" H 2955 3127 50  0000 C CNN
F 2 "" H 2950 3300 50  0001 C CNN
F 3 "" H 2950 3300 50  0001 C CNN
	1    2950 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2950 2950 2950
Wire Wire Line
	2950 2950 2950 3000
Wire Wire Line
	2850 2950 2950 2950
Connection ~ 2950 2950
Text Label 1150 1850 0    50   ~ 0
currentOut
Text Label 1150 1950 0    50   ~ 0
refOut
$Comp
L power:GND #PWR01
U 1 1 60228B92
P 1150 1550
F 0 "#PWR01" H 1150 1300 50  0001 C CNN
F 1 "GND" V 1155 1422 50  0000 R CNN
F 2 "" H 1150 1550 50  0001 C CNN
F 3 "" H 1150 1550 50  0001 C CNN
	1    1150 1550
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 60228B98
P 1150 1650
F 0 "#PWR02" H 1150 1500 50  0001 C CNN
F 1 "+5V" V 1165 1823 50  0000 C CNN
F 2 "" H 1150 1650 50  0001 C CNN
F 3 "" H 1150 1650 50  0001 C CNN
	1    1150 1650
	0    1    1    0   
$EndComp
Text Label 1150 1750 0    50   ~ 0
currentIn
Text Label 3250 2300 2    50   ~ 0
currentIn
$Comp
L Connector:Conn_01x06_Female J1
U 1 1 60229ED2
P 950 1850
F 0 "J1" H 842 1325 50  0000 C CNN
F 1 "Conn_01x06_Female" H 842 1416 50  0000 C CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-0612_2x03_P3.00mm_Vertical" H 950 1850 50  0001 C CNN
F 3 "~" H 950 1850 50  0001 C CNN
	1    950  1850
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 60234524
P 3350 2450
F 0 "R2" H 3420 2496 50  0000 L CNN
F 1 "10k?" H 3420 2405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3280 2450 50  0001 C CNN
F 3 "~" H 3350 2450 50  0001 C CNN
	1    3350 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 60235396
P 3350 2600
F 0 "#PWR010" H 3350 2350 50  0001 C CNN
F 1 "GND" H 3355 2427 50  0000 C CNN
F 2 "" H 3350 2600 50  0001 C CNN
F 3 "" H 3350 2600 50  0001 C CNN
	1    3350 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 2300 3650 2300
Wire Wire Line
	3250 2300 3350 2300
Connection ~ 3350 2300
$Comp
L power:GND #PWR?
U 1 1 6023CEF0
P 1150 2050
F 0 "#PWR?" H 1150 1800 50  0001 C CNN
F 1 "GND" V 1150 1900 50  0000 R CNN
F 2 "" H 1150 2050 50  0001 C CNN
F 3 "" H 1150 2050 50  0001 C CNN
	1    1150 2050
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
