EESchema Schematic File Version 4
LIBS:DashDriver-V1-cache
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Dashboard Display Driver"
Date "2020-01-13"
Rev "V0.1"
Comp "NuRacing"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L teensy:Teensy4.0 U2
U 1 1 5E1B9978
P 4500 3800
F 0 "U2" H 4500 5415 50  0000 C CNN
F 1 "Teensy4.0" H 4500 5324 50  0000 C CNN
F 2 "teensy:Teensy40" H 4100 4000 50  0001 C CNN
F 3 "" H 4100 4000 50  0001 C CNN
	1    4500 3800
	1    0    0    -1  
$EndComp
Text Notes 650  750  0    50   ~ 0
Power connection, polarity protection
$Comp
L power:GND #PWR0101
U 1 1 5E1BD32A
P 1250 2450
F 0 "#PWR0101" H 1250 2200 50  0001 C CNN
F 1 "GND" H 1255 2277 50  0000 C CNN
F 2 "" H 1250 2450 50  0001 C CNN
F 3 "" H 1250 2450 50  0001 C CNN
	1    1250 2450
	1    0    0    -1  
$EndComp
Entry Wire Line
	2250 1900 2150 2000
Text Label 2000 2000 0    50   ~ 0
+5V
$Comp
L dk_PMIC-Voltage-Regulators-Linear:LM1117MPX-3_3_NOPB U1
U 1 1 5E1BF74F
P 3300 1250
F 0 "U1" H 3250 1537 60  0000 C CNN
F 1 "LM1117MPX-3_3_NOPB" H 3250 1431 60  0000 C CNN
F 2 "digikey-footprints:SOT-223" H 3500 1450 60  0001 L CNN
F 3 "http://www.ti.com/general/docs/suppproductinfo.tsp?distId=10&gotoUrl=http%3A%2F%2Fwww.ti.com%2Flit%2Fgpn%2Flm1117" H 3500 1550 60  0001 L CNN
F 4 "LM1117MPX-3.3/NOPBCT-ND" H 3500 1650 60  0001 L CNN "Digi-Key_PN"
F 5 "LM1117MPX-3.3/NOPB" H 3500 1750 60  0001 L CNN "MPN"
F 6 "Integrated Circuits (ICs)" H 3500 1850 60  0001 L CNN "Category"
F 7 "PMIC - Voltage Regulators - Linear" H 3500 1950 60  0001 L CNN "Family"
F 8 "http://www.ti.com/general/docs/suppproductinfo.tsp?distId=10&gotoUrl=http%3A%2F%2Fwww.ti.com%2Flit%2Fgpn%2Flm1117" H 3500 2050 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/texas-instruments/LM1117MPX-3.3-NOPB/LM1117MPX-3.3-NOPBCT-ND/1010516" H 3500 2150 60  0001 L CNN "DK_Detail_Page"
F 10 "IC REG LIN 3.3V 800MA SOT223-4" H 3500 2250 60  0001 L CNN "Description"
F 11 "Texas Instruments" H 3500 2350 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3500 2450 60  0001 L CNN "Status"
	1    3300 1250
	1    0    0    -1  
$EndComp
Entry Wire Line
	2550 850  2650 950 
Entry Wire Line
	3800 850  3900 950 
Wire Wire Line
	2650 950  2650 1250
Wire Wire Line
	2650 1250 2900 1250
Wire Wire Line
	3600 1350 3600 1250
Wire Wire Line
	3600 1250 3900 1250
Wire Wire Line
	3900 1250 3900 950 
Connection ~ 3600 1250
Wire Wire Line
	3200 1650 3200 1800
$Comp
L power:GND #PWR0102
U 1 1 5E1C0C7C
P 3200 1800
F 0 "#PWR0102" H 3200 1550 50  0001 C CNN
F 1 "GND" H 3205 1627 50  0000 C CNN
F 2 "" H 3200 1800 50  0001 C CNN
F 3 "" H 3200 1800 50  0001 C CNN
	1    3200 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5E1C1112
P 2650 1400
F 0 "C1" H 2765 1446 50  0000 L CNN
F 1 "C" H 2765 1355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2688 1250 50  0001 C CNN
F 3 "~" H 2650 1400 50  0001 C CNN
	1    2650 1400
	1    0    0    -1  
$EndComp
Connection ~ 2650 1250
$Comp
L Device:C C2
U 1 1 5E1C1759
P 3900 1400
F 0 "C2" H 4015 1446 50  0000 L CNN
F 1 "C" H 4015 1355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3938 1250 50  0001 C CNN
F 3 "~" H 3900 1400 50  0001 C CNN
	1    3900 1400
	1    0    0    -1  
$EndComp
Connection ~ 3900 1250
Wire Wire Line
	2650 1550 2650 1650
Wire Wire Line
	2650 1650 3200 1650
Connection ~ 3200 1650
Wire Wire Line
	3200 1650 3900 1650
Wire Wire Line
	3900 1650 3900 1550
Text Label 2650 950  0    50   ~ 0
+5V
Text Label 3900 950  0    50   ~ 0
+3V3
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5E1C73C9
P 850 1850
F 0 "J1" H 850 1550 50  0000 C CNN
F 1 "Power" H 850 2050 50  0000 C CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-0412_2x02_P3.00mm_Vertical" H 850 1850 50  0001 C CNN
F 3 "~" H 850 1850 50  0001 C CNN
	1    850  1850
	-1   0    0    1   
$EndComp
$Comp
L dk_Transistors-FETs-MOSFETs-Single:AO3401A Q3
U 1 1 5E1C8960
P 1650 2000
F 0 "Q3" V 1917 2000 60  0000 C CNN
F 1 "AO3401A" V 1811 2000 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 1850 2200 60  0001 L CNN
F 3 "http://aosmd.com/res/data_sheets/AO3401A.pdf" H 1850 2300 60  0001 L CNN
F 4 "785-1001-1-ND" H 1850 2400 60  0001 L CNN "Digi-Key_PN"
F 5 "AO3401A" H 1850 2500 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 1850 2600 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 1850 2700 60  0001 L CNN "Family"
F 8 "http://aosmd.com/res/data_sheets/AO3401A.pdf" H 1850 2800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/alpha-omega-semiconductor-inc/AO3401A/785-1001-1-ND/1855943" H 1850 2900 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET P-CH 30V 4A SOT23" H 1850 3000 60  0001 L CNN "Description"
F 11 "Alpha & Omega Semiconductor Inc." H 1850 3100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 1850 3200 60  0001 L CNN "Status"
	1    1650 2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 2300 1250 2300
Wire Wire Line
	1250 2450 1250 2300
Connection ~ 1250 2300
Wire Wire Line
	1250 2300 1050 2300
Wire Wire Line
	2150 2000 1850 2000
Entry Wire Line
	2250 2350 2350 2450
Entry Wire Line
	2250 4550 2350 4650
Entry Wire Line
	2250 4650 2350 4750
Wire Wire Line
	2350 4650 3400 4650
Wire Wire Line
	2350 4750 3400 4750
Wire Wire Line
	3400 2450 2350 2450
Text Label 6100 4950 0    50   ~ 0
+3V3
Entry Wire Line
	2150 2300 2250 2400
Wire Wire Line
	1750 2300 2150 2300
Connection ~ 1750 2300
Text Label 2000 2300 0    50   ~ 0
GND
Text Label 2350 2450 0    50   ~ 0
GND
Text Label 2350 4650 0    50   ~ 0
I2C_C
Text Label 2350 4750 0    50   ~ 0
I2C_D
Text Label 2350 5050 0    50   ~ 0
LED_Data
Wire Wire Line
	2350 5050 3400 5050
Entry Wire Line
	2250 4950 2350 5050
Entry Wire Line
	2250 2450 2350 2550
Entry Wire Line
	2250 2550 2350 2650
Wire Wire Line
	2350 2550 3400 2550
Wire Wire Line
	3400 2650 2350 2650
Text Label 2350 2550 0    50   ~ 0
CRX2
Text Label 2350 2650 0    50   ~ 0
CTX2
Wire Wire Line
	5600 5050 6350 5050
Wire Wire Line
	5600 5150 6350 5150
Entry Wire Line
	6350 5050 6450 5150
Entry Wire Line
	6350 5150 6450 5250
Text Label 6100 5150 0    50   ~ 0
CTX1
Text Label 6100 5050 0    50   ~ 0
CRX1
Text Notes 1200 6050 0    50   ~ 0
I2C and WS2811 Outputs\nLevel shifted to 5V
$Comp
L dk_Transistors-FETs-MOSFETs-Single:2N7002 Q11
U 1 1 5E1C45A9
P 3050 7150
F 0 "Q11" V 3211 7150 60  0000 C CNN
F 1 "2N7002" V 3317 7150 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 3250 7350 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 3250 7450 60  0001 L CNN
F 4 "2N7002NCT-ND" H 3250 7550 60  0001 L CNN "Digi-Key_PN"
F 5 "2N7002" H 3250 7650 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 3250 7750 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 3250 7850 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 3250 7950 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/2N7002/2N7002NCT-ND/244664" H 3250 8050 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 60V 115MA SOT-23" H 3250 8150 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 3250 8250 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3250 8350 60  0001 L CNN "Status"
	1    3050 7150
	0    1    1    0   
$EndComp
$Comp
L dk_Transistors-FETs-MOSFETs-Single:2N7002 Q9
U 1 1 5E1C5D5B
P 3050 6300
F 0 "Q9" V 3211 6300 60  0000 C CNN
F 1 "2N7002" V 3317 6300 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 3250 6500 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 3250 6600 60  0001 L CNN
F 4 "2N7002NCT-ND" H 3250 6700 60  0001 L CNN "Digi-Key_PN"
F 5 "2N7002" H 3250 6800 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 3250 6900 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 3250 7000 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 3250 7100 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/2N7002/2N7002NCT-ND/244664" H 3250 7200 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 60V 115MA SOT-23" H 3250 7300 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 3250 7400 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3250 7500 60  0001 L CNN "Status"
	1    3050 6300
	0    1    1    0   
$EndComp
$Comp
L dk_Transistors-FETs-MOSFETs-Single:2N7002 Q13
U 1 1 5E1C73AA
P 3050 8000
F 0 "Q13" V 3211 8000 60  0000 C CNN
F 1 "2N7002" V 3317 8000 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 3250 8200 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 3250 8300 60  0001 L CNN
F 4 "2N7002NCT-ND" H 3250 8400 60  0001 L CNN "Digi-Key_PN"
F 5 "2N7002" H 3250 8500 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 3250 8600 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 3250 8700 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 3250 8800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/2N7002/2N7002NCT-ND/244664" H 3250 8900 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 60V 115MA SOT-23" H 3250 9000 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 3250 9100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3250 9200 60  0001 L CNN "Status"
	1    3050 8000
	0    1    1    0   
$EndComp
$Comp
L Device:R R19
U 1 1 5E1C7E42
P 2650 6150
F 0 "R19" H 2720 6196 50  0000 L CNN
F 1 "10k" H 2720 6105 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 2580 6150 50  0001 C CNN
F 3 "~" H 2650 6150 50  0001 C CNN
	1    2650 6150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R23
U 1 1 5E1C8508
P 2650 7000
F 0 "R23" H 2720 7046 50  0000 L CNN
F 1 "10k" H 2720 6955 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 2580 7000 50  0001 C CNN
F 3 "~" H 2650 7000 50  0001 C CNN
	1    2650 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R27
U 1 1 5E1C8814
P 2650 7850
F 0 "R27" H 2720 7896 50  0000 L CNN
F 1 "10k" H 2720 7805 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 2580 7850 50  0001 C CNN
F 3 "~" H 2650 7850 50  0001 C CNN
	1    2650 7850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R20
U 1 1 5E1C8C3A
P 3450 6150
F 0 "R20" H 3520 6196 50  0000 L CNN
F 1 "1k" H 3520 6105 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3380 6150 50  0001 C CNN
F 3 "~" H 3450 6150 50  0001 C CNN
	1    3450 6150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R24
U 1 1 5E1C9195
P 3450 7000
F 0 "R24" H 3520 7046 50  0000 L CNN
F 1 "1k" H 3520 6955 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3380 7000 50  0001 C CNN
F 3 "~" H 3450 7000 50  0001 C CNN
	1    3450 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R28
U 1 1 5E1C9614
P 3450 7850
F 0 "R28" H 3520 7896 50  0000 L CNN
F 1 "1k" H 3520 7805 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3380 7850 50  0001 C CNN
F 3 "~" H 3450 7850 50  0001 C CNN
	1    3450 7850
	1    0    0    -1  
$EndComp
Entry Wire Line
	2250 5800 2350 5900
Entry Wire Line
	2250 5700 2350 5800
Entry Wire Line
	2250 6650 2350 6750
Entry Wire Line
	2250 6550 2350 6650
Entry Wire Line
	2250 7500 2350 7600
Entry Wire Line
	2250 7400 2350 7500
Wire Wire Line
	2350 6650 3450 6650
Wire Wire Line
	3450 6650 3450 6850
Wire Wire Line
	2350 6750 2650 6750
Wire Wire Line
	2950 6750 2950 6850
Wire Wire Line
	2650 6750 2650 6850
Connection ~ 2650 6750
Wire Wire Line
	2650 6750 2950 6750
Wire Wire Line
	2350 7150 2650 7150
Connection ~ 2650 7150
Wire Wire Line
	2650 7150 2850 7150
Wire Wire Line
	3250 7150 3450 7150
Wire Wire Line
	3450 6300 3250 6300
Wire Wire Line
	2850 6300 2650 6300
Connection ~ 2650 6300
Wire Wire Line
	2650 6300 2350 6300
Wire Wire Line
	2350 5900 2650 5900
Wire Wire Line
	2650 5900 2650 6000
Wire Wire Line
	2650 5900 2950 5900
Wire Wire Line
	2950 5900 2950 6000
Connection ~ 2650 5900
Wire Wire Line
	2350 5800 3450 5800
Wire Wire Line
	3450 5800 3450 6000
Wire Wire Line
	2350 7500 3450 7500
Wire Wire Line
	3450 7500 3450 7700
Wire Wire Line
	2950 7700 2950 7600
Wire Wire Line
	2950 7600 2650 7600
Wire Wire Line
	2650 7700 2650 7600
Connection ~ 2650 7600
Wire Wire Line
	2650 7600 2350 7600
Wire Wire Line
	2350 8000 2650 8000
Connection ~ 2650 8000
Wire Wire Line
	2650 8000 2850 8000
Wire Wire Line
	3250 8000 3450 8000
Entry Wire Line
	2250 6200 2350 6300
Entry Wire Line
	2250 7050 2350 7150
Entry Wire Line
	2250 7900 2350 8000
Text Label 2350 5800 0    50   ~ 0
+5V
Text Label 2350 6650 0    50   ~ 0
+5V
Text Label 2350 7500 0    50   ~ 0
+5V
Text Label 2350 5900 0    50   ~ 0
+3V3
Text Label 2350 6750 0    50   ~ 0
+3V3
Text Label 2350 7600 0    50   ~ 0
+3V3
Text Label 2650 8000 3    50   ~ 0
LED_Data
Text Label 2350 6300 0    50   ~ 0
I2C_D
Text Label 2350 7150 0    50   ~ 0
I2C_C
Wire Wire Line
	3450 8000 4200 8000
Connection ~ 3450 8000
Wire Wire Line
	3450 7150 4100 7150
Connection ~ 3450 7150
Wire Wire Line
	3450 6300 4250 6300
Connection ~ 3450 6300
Wire Wire Line
	3450 6650 4150 6650
Connection ~ 3450 6650
Text Label 3950 6300 0    50   ~ 0
I2C_D-5V
Text Label 3600 7150 0    50   ~ 0
I2C_C-5V
Text Label 3600 8000 0    50   ~ 0
LED_Data-5V
$Comp
L Connector_Generic:Conn_01x06 J4
U 1 1 5E1DB0C4
P 4550 6800
F 0 "J4" H 4630 6792 50  0000 L CNN
F 1 "Dash Conn" H 4630 6701 50  0000 L CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-0612_2x03_P3.00mm_Vertical" H 4550 6800 50  0001 C CNN
F 3 "~" H 4550 6800 50  0001 C CNN
	1    4550 6800
	1    0    0    -1  
$EndComp
NoConn ~ 4350 7000
NoConn ~ 5600 4650
NoConn ~ 5600 4750
NoConn ~ 5600 4850
NoConn ~ 5600 3350
NoConn ~ 5600 3250
NoConn ~ 5600 3150
NoConn ~ 5600 3050
NoConn ~ 5600 2950
NoConn ~ 5600 2850
NoConn ~ 5600 2750
NoConn ~ 5600 2650
NoConn ~ 5600 2550
NoConn ~ 5600 2450
Entry Wire Line
	2250 4450 2350 4550
Entry Wire Line
	2250 4350 2350 4450
Entry Wire Line
	2250 3450 2350 3550
Entry Wire Line
	2250 3350 2350 3450
Entry Wire Line
	2250 3250 2350 3350
Entry Wire Line
	2250 3150 2350 3250
Entry Wire Line
	2250 3050 2350 3150
Entry Wire Line
	2250 2950 2350 3050
Entry Wire Line
	2250 2850 2350 2950
Entry Wire Line
	2250 2750 2350 2850
Entry Wire Line
	2250 2650 2350 2750
Wire Wire Line
	2350 2750 3400 2750
Wire Wire Line
	2350 2850 3400 2850
Wire Wire Line
	3400 2950 2350 2950
Wire Wire Line
	2350 3050 3400 3050
Wire Wire Line
	3400 3150 2350 3150
Wire Wire Line
	2350 3250 3400 3250
Wire Wire Line
	3400 3350 2350 3350
Wire Wire Line
	2350 3450 3400 3450
Wire Wire Line
	3400 3550 2350 3550
Wire Wire Line
	3400 4450 2350 4450
Text Label 2350 2750 0    50   ~ 0
SD_Inertia
Text Label 2350 2850 0    50   ~ 0
SD_BOTS
Text Label 2350 2950 0    50   ~ 0
SD_Dash
Wire Wire Line
	2350 4550 3400 4550
Entry Wire Line
	6350 4950 6450 5050
Wire Wire Line
	6350 4950 5600 4950
Text Label 2350 4450 0    50   ~ 0
Throttle_Pos
Text Label 2350 4550 0    50   ~ 0
AIn_Spare
Text Label 2350 3050 0    50   ~ 0
APPS_Trail
Text Label 2350 3150 0    50   ~ 0
APPS_Bound
Text Label 2350 3250 0    50   ~ 0
APPS_Dis
Text Label 2350 3350 0    50   ~ 0
Brake_Sw
Text Label 2350 3450 0    50   ~ 0
RTD_Gate
Text Label 2350 3550 0    50   ~ 0
Spare_D2
NoConn ~ 3400 4250
NoConn ~ 3400 4150
NoConn ~ 3400 4050
NoConn ~ 3400 3950
NoConn ~ 3400 3850
NoConn ~ 5600 4350
NoConn ~ 5600 4250
NoConn ~ 5600 4150
NoConn ~ 5600 4050
NoConn ~ 5600 3950
NoConn ~ 5600 3850
NoConn ~ 5600 3750
NoConn ~ 5600 3650
NoConn ~ 5600 3550
NoConn ~ 5600 3450
$Comp
L Interface_CAN_LIN:TJA1051T-3 U3
U 1 1 5E242040
P 3050 9100
F 0 "U3" H 3050 9681 50  0000 C CNN
F 1 "TJA1051T-3" H 3050 9590 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3050 8600 50  0001 C CIN
F 3 "http://www.nxp.com/documents/data_sheet/TJA1051.pdf" H 3050 9100 50  0001 C CNN
	1    3050 9100
	1    0    0    -1  
$EndComp
$Comp
L Interface_CAN_LIN:TJA1051T-3 U4
U 1 1 5E242B95
P 3050 10300
F 0 "U4" H 3050 10881 50  0000 C CNN
F 1 "TJA1051T-3" H 3050 10790 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3050 9800 50  0001 C CIN
F 3 "http://www.nxp.com/documents/data_sheet/TJA1051.pdf" H 3050 10300 50  0001 C CNN
	1    3050 10300
	1    0    0    -1  
$EndComp
Text Notes 1700 8600 0    50   ~ 0
CAN Interface\nCAN1: RCU\nCAN2: Spare
Entry Wire Line
	2250 8800 2350 8900
Entry Wire Line
	2250 8900 2350 9000
Entry Wire Line
	2250 9100 2350 9200
NoConn ~ 2550 9300
NoConn ~ 2550 10500
Entry Wire Line
	2250 10000 2350 10100
Entry Wire Line
	2250 10100 2350 10200
Entry Wire Line
	2250 10300 2350 10400
Wire Wire Line
	2350 10400 2550 10400
Wire Wire Line
	2350 10200 2550 10200
Wire Wire Line
	2350 10100 2550 10100
Wire Wire Line
	2350 8900 2550 8900
Wire Wire Line
	2350 9000 2550 9000
Wire Wire Line
	2350 9200 2550 9200
Text Label 2350 9200 0    50   ~ 0
+3V3
Text Label 2350 10400 0    50   ~ 0
+3V3
Text Label 2350 8900 0    50   ~ 0
CTX1
Text Label 2350 9000 0    50   ~ 0
CRX1
Text Label 2350 10100 0    50   ~ 0
CTX2
Text Label 2350 10200 0    50   ~ 0
CRX2
Wire Wire Line
	3050 8700 2350 8700
Wire Wire Line
	3050 9500 2350 9500
Wire Wire Line
	3050 10700 2350 10700
Wire Wire Line
	3050 9900 2350 9900
Entry Wire Line
	2250 8600 2350 8700
Entry Wire Line
	2250 9400 2350 9500
Entry Wire Line
	2250 9800 2350 9900
Entry Wire Line
	2250 10600 2350 10700
Text Label 2350 8700 0    50   ~ 0
+5V
Text Label 2350 9900 0    50   ~ 0
+5V
Text Label 2350 9500 0    50   ~ 0
GND
Text Label 2350 10700 0    50   ~ 0
GND
$Comp
L Device:R R32
U 1 1 5E26C50B
P 4400 9100
F 0 "R32" H 4470 9146 50  0000 L CNN
F 1 "60" H 4470 9055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 4330 9100 50  0001 C CNN
F 3 "~" H 4400 9100 50  0001 C CNN
	1    4400 9100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R31
U 1 1 5E26CCAD
P 4050 9100
F 0 "R31" H 3980 9054 50  0000 R CNN
F 1 "60" H 3980 9145 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3980 9100 50  0001 C CNN
F 3 "~" H 4050 9100 50  0001 C CNN
	1    4050 9100
	-1   0    0    1   
$EndComp
Wire Wire Line
	3550 9000 3850 9000
Wire Wire Line
	3850 9000 3850 8950
Wire Wire Line
	3850 8950 4050 8950
Wire Wire Line
	4050 9250 4250 9250
Wire Wire Line
	4250 9250 4250 8950
Wire Wire Line
	4250 8950 4400 8950
Wire Wire Line
	3550 9200 3850 9200
Wire Wire Line
	3850 9200 3850 9300
Wire Wire Line
	3850 9300 4400 9300
Wire Wire Line
	4400 9300 4400 9250
$Comp
L Device:C C6
U 1 1 5E277A2C
P 3700 9400
F 0 "C6" V 3850 9400 50  0000 C CNN
F 1 "50n" V 3550 9400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3738 9250 50  0001 C CNN
F 3 "~" H 3700 9400 50  0001 C CNN
	1    3700 9400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 9400 4250 9400
Wire Wire Line
	4250 9400 4250 9250
Connection ~ 4250 9250
Wire Wire Line
	3550 9400 3550 9500
Wire Wire Line
	3550 9500 3050 9500
Connection ~ 3050 9500
Wire Wire Line
	4050 8950 4050 8900
Connection ~ 4050 8950
$Comp
L Device:R R36
U 1 1 5E28F2CB
P 4400 10300
F 0 "R36" H 4470 10346 50  0000 L CNN
F 1 "60" H 4470 10255 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 4330 10300 50  0001 C CNN
F 3 "~" H 4400 10300 50  0001 C CNN
	1    4400 10300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R35
U 1 1 5E28F2D5
P 4050 10300
F 0 "R35" H 3980 10254 50  0000 R CNN
F 1 "60" H 3980 10345 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 3980 10300 50  0001 C CNN
F 3 "~" H 4050 10300 50  0001 C CNN
	1    4050 10300
	-1   0    0    1   
$EndComp
Wire Wire Line
	3550 10200 3850 10200
Wire Wire Line
	3850 10200 3850 10150
Wire Wire Line
	3850 10150 4050 10150
Wire Wire Line
	4050 10450 4250 10450
Wire Wire Line
	4250 10450 4250 10150
Wire Wire Line
	4250 10150 4400 10150
Wire Wire Line
	3550 10400 3850 10400
Wire Wire Line
	3850 10400 3850 10500
Wire Wire Line
	3850 10500 4400 10500
Wire Wire Line
	4400 10500 4400 10450
$Comp
L Device:C C8
U 1 1 5E28F2E9
P 3700 10600
F 0 "C8" V 3850 10600 50  0000 C CNN
F 1 "50n" V 3550 10600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3738 10450 50  0001 C CNN
F 3 "~" H 3700 10600 50  0001 C CNN
	1    3700 10600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 10600 4250 10600
Wire Wire Line
	4250 10600 4250 10450
Connection ~ 4250 10450
Wire Wire Line
	4050 10150 4050 10100
Connection ~ 4050 10150
Connection ~ 4400 10500
Wire Wire Line
	3550 10600 3550 10700
Wire Wire Line
	3550 10700 3050 10700
Connection ~ 3050 10700
Text Notes 3700 9750 0    50   ~ 0
Split termination used - can \nforego capacitors if desired.
$Comp
L Connector_Generic:Conn_01x04 J5
U 1 1 5E29A47E
P 5250 9050
F 0 "J5" H 5330 9042 50  0000 L CNN
F 1 "CAN" H 5330 8951 50  0000 L CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-0412_2x02_P3.00mm_Vertical" H 5250 9050 50  0001 C CNN
F 3 "~" H 5250 9050 50  0001 C CNN
	1    5250 9050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 8900 4950 8900
Wire Wire Line
	4050 10100 4850 10100
Wire Wire Line
	4400 10500 4950 10500
$Comp
L Device:C C5
U 1 1 5E2AF4EF
P 3700 8700
F 0 "C5" V 3850 8700 50  0000 C CNN
F 1 "100n" V 3550 8700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3738 8550 50  0001 C CNN
F 3 "~" H 3700 8700 50  0001 C CNN
	1    3700 8700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 8700 3050 8700
Connection ~ 3050 8700
Wire Wire Line
	3850 8700 3900 8700
Wire Wire Line
	3900 8700 3900 8900
Wire Wire Line
	3900 8900 3600 8900
Wire Wire Line
	3600 8900 3600 9350
Wire Wire Line
	3600 9350 3550 9350
Wire Wire Line
	3550 9350 3550 9400
Connection ~ 3550 9400
$Comp
L Device:C C7
U 1 1 5E2BAACF
P 3700 9900
F 0 "C7" V 3750 10000 50  0000 C CNN
F 1 "100n" V 3550 9900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3738 9750 50  0001 C CNN
F 3 "~" H 3700 9900 50  0001 C CNN
	1    3700 9900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 9900 3050 9900
Connection ~ 3050 9900
Wire Wire Line
	3850 9900 3900 9900
Wire Wire Line
	3900 9900 3900 10100
Wire Wire Line
	3900 10100 3600 10100
Wire Wire Line
	3600 10100 3600 10550
Wire Wire Line
	3600 10550 3550 10550
Wire Wire Line
	3550 10550 3550 10600
Connection ~ 3550 10600
Connection ~ 6450 850 
Text Notes 6800 1150 0    50   ~ 0
General IO Connectors - level shifted.\nAnalog inputs protected by zener\n
Text Label 6550 1750 0    50   ~ 0
SD_Inertia
Entry Wire Line
	6450 1650 6550 1750
Entry Wire Line
	6450 2000 6550 2100
Entry Wire Line
	6450 2500 6550 2600
Text Label 6550 3450 0    50   ~ 0
SD_Dash
Entry Wire Line
	6450 2850 6550 2950
Entry Wire Line
	6450 3350 6550 3450
Entry Wire Line
	6450 3700 6550 3800
Entry Wire Line
	6450 4200 6550 4300
Text Label 6550 5150 0    50   ~ 0
APPS_Bound
Entry Wire Line
	6450 4550 6550 4650
Entry Wire Line
	6450 5400 6550 5500
Entry Wire Line
	6450 5050 6550 5150
$Comp
L dk_Transistors-FETs-MOSFETs-Single:2N7002 Q8
U 1 1 5E2302FF
P 7250 6400
F 0 "Q8" V 7411 6400 60  0000 C CNN
F 1 "2N7002" V 7517 6400 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 7450 6600 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 6700 60  0001 L CNN
F 4 "2N7002NCT-ND" H 7450 6800 60  0001 L CNN "Digi-Key_PN"
F 5 "2N7002" H 7450 6900 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 7000 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 7450 7100 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 7200 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/2N7002/2N7002NCT-ND/244664" H 7450 7300 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 60V 115MA SOT-23" H 7450 7400 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 7450 7500 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 7600 60  0001 L CNN "Status"
	1    7250 6400
	0    1    1    0   
$EndComp
$Comp
L Device:R R17
U 1 1 5E230309
P 6850 6250
F 0 "R17" H 6920 6296 50  0000 L CNN
F 1 "10k" H 6920 6205 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6780 6250 50  0001 C CNN
F 3 "~" H 6850 6250 50  0001 C CNN
	1    6850 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 5E230313
P 7500 6250
F 0 "R18" H 7570 6296 50  0000 L CNN
F 1 "100k" H 7570 6205 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7430 6250 50  0001 C CNN
F 3 "~" H 7500 6250 50  0001 C CNN
	1    7500 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 6400 6850 6400
Connection ~ 6850 6400
Wire Wire Line
	6850 6400 6550 6400
Wire Wire Line
	6550 6000 6850 6000
Wire Wire Line
	6850 6000 6850 6100
Wire Wire Line
	6850 6000 7150 6000
Wire Wire Line
	7150 6000 7150 6100
Connection ~ 6850 6000
Wire Wire Line
	6550 5900 7500 5900
Wire Wire Line
	7500 5900 7500 6100
Text Label 6550 6000 0    50   ~ 0
+3V3
Text Label 6650 6400 1    50   ~ 0
APPS_Dis
Text Label 6550 5900 0    50   ~ 0
+5V
Wire Wire Line
	7500 6400 7450 6400
Wire Wire Line
	7500 6400 7700 6400
Connection ~ 7500 6400
Entry Wire Line
	6450 5800 6550 5900
Entry Wire Line
	6450 5900 6550 6000
Entry Wire Line
	6450 6300 6550 6400
$Comp
L dk_Transistors-FETs-MOSFETs-Single:2N7002 Q10
U 1 1 5E23D18C
P 7250 7250
F 0 "Q10" V 7411 7250 60  0000 C CNN
F 1 "2N7002" V 7517 7250 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 7450 7450 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 7550 60  0001 L CNN
F 4 "2N7002NCT-ND" H 7450 7650 60  0001 L CNN "Digi-Key_PN"
F 5 "2N7002" H 7450 7750 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 7850 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 7450 7950 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 8050 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/2N7002/2N7002NCT-ND/244664" H 7450 8150 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 60V 115MA SOT-23" H 7450 8250 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 7450 8350 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 8450 60  0001 L CNN "Status"
	1    7250 7250
	0    1    1    0   
$EndComp
$Comp
L Device:R R21
U 1 1 5E23D196
P 6850 7100
F 0 "R21" H 6920 7146 50  0000 L CNN
F 1 "10k" H 6920 7055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6780 7100 50  0001 C CNN
F 3 "~" H 6850 7100 50  0001 C CNN
	1    6850 7100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R22
U 1 1 5E23D1A0
P 7500 7100
F 0 "R22" H 7570 7146 50  0000 L CNN
F 1 "100k" H 7570 7055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7430 7100 50  0001 C CNN
F 3 "~" H 7500 7100 50  0001 C CNN
	1    7500 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 7250 6850 7250
Connection ~ 6850 7250
Wire Wire Line
	6850 7250 6550 7250
Wire Wire Line
	6550 6850 6850 6850
Wire Wire Line
	6850 6850 6850 6950
Wire Wire Line
	6850 6850 7150 6850
Wire Wire Line
	7150 6850 7150 6950
Connection ~ 6850 6850
Wire Wire Line
	6550 6750 7500 6750
Wire Wire Line
	7500 6750 7500 6950
Text Label 6550 6850 0    50   ~ 0
+3V3
Text Label 6550 6750 0    50   ~ 0
+5V
Wire Wire Line
	7500 7250 7450 7250
Wire Wire Line
	7500 7250 7700 7250
Connection ~ 7500 7250
Entry Wire Line
	6450 6650 6550 6750
Entry Wire Line
	6450 6750 6550 6850
Entry Wire Line
	6450 7150 6550 7250
$Comp
L dk_Transistors-FETs-MOSFETs-Single:2N7002 Q12
U 1 1 5E24FAEF
P 7250 8100
F 0 "Q12" V 7411 8100 60  0000 C CNN
F 1 "2N7002" V 7517 8100 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 7450 8300 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 8400 60  0001 L CNN
F 4 "2N7002NCT-ND" H 7450 8500 60  0001 L CNN "Digi-Key_PN"
F 5 "2N7002" H 7450 8600 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 8700 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 7450 8800 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 8900 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/2N7002/2N7002NCT-ND/244664" H 7450 9000 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 60V 115MA SOT-23" H 7450 9100 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 7450 9200 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 9300 60  0001 L CNN "Status"
	1    7250 8100
	0    1    1    0   
$EndComp
$Comp
L Device:R R25
U 1 1 5E24FAF9
P 6850 7950
F 0 "R25" H 6920 7996 50  0000 L CNN
F 1 "10k" H 6920 7905 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6780 7950 50  0001 C CNN
F 3 "~" H 6850 7950 50  0001 C CNN
	1    6850 7950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R26
U 1 1 5E24FB03
P 7500 7950
F 0 "R26" H 7570 7996 50  0000 L CNN
F 1 "100k" H 7570 7905 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7430 7950 50  0001 C CNN
F 3 "~" H 7500 7950 50  0001 C CNN
	1    7500 7950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 8100 6850 8100
Connection ~ 6850 8100
Wire Wire Line
	6850 8100 6550 8100
Wire Wire Line
	6550 7700 6850 7700
Wire Wire Line
	6850 7700 6850 7800
Wire Wire Line
	6850 7700 7150 7700
Wire Wire Line
	7150 7700 7150 7800
Connection ~ 6850 7700
Wire Wire Line
	6550 7600 7500 7600
Wire Wire Line
	7500 7600 7500 7800
Text Label 6550 7700 0    50   ~ 0
+3V3
Text Label 6550 7600 0    50   ~ 0
+5V
Wire Wire Line
	7500 8100 7450 8100
Wire Wire Line
	7500 8100 7700 8100
Connection ~ 7500 8100
Entry Wire Line
	6450 7500 6550 7600
Entry Wire Line
	6450 7600 6550 7700
Entry Wire Line
	6450 8000 6550 8100
$Comp
L dk_Transistors-FETs-MOSFETs-Single:2N7002 Q14
U 1 1 5E24FB29
P 7250 8950
F 0 "Q14" V 7411 8950 60  0000 C CNN
F 1 "2N7002" V 7517 8950 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 7450 9150 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 9250 60  0001 L CNN
F 4 "2N7002NCT-ND" H 7450 9350 60  0001 L CNN "Digi-Key_PN"
F 5 "2N7002" H 7450 9450 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 9550 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 7450 9650 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 9750 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/2N7002/2N7002NCT-ND/244664" H 7450 9850 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 60V 115MA SOT-23" H 7450 9950 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 7450 10050 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 10150 60  0001 L CNN "Status"
	1    7250 8950
	0    1    1    0   
$EndComp
$Comp
L Device:R R29
U 1 1 5E24FB33
P 6850 8800
F 0 "R29" H 6920 8846 50  0000 L CNN
F 1 "10k" H 6920 8755 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6780 8800 50  0001 C CNN
F 3 "~" H 6850 8800 50  0001 C CNN
	1    6850 8800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R30
U 1 1 5E24FB3D
P 7500 8800
F 0 "R30" H 7570 8846 50  0000 L CNN
F 1 "100k" H 7570 8755 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7430 8800 50  0001 C CNN
F 3 "~" H 7500 8800 50  0001 C CNN
	1    7500 8800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 8950 6850 8950
Connection ~ 6850 8950
Wire Wire Line
	6850 8950 6550 8950
Wire Wire Line
	6550 8550 6850 8550
Wire Wire Line
	6850 8550 6850 8650
Wire Wire Line
	6850 8550 7150 8550
Wire Wire Line
	7150 8550 7150 8650
Connection ~ 6850 8550
Wire Wire Line
	6550 8450 7500 8450
Wire Wire Line
	7500 8450 7500 8650
Text Label 6550 8550 0    50   ~ 0
+3V3
Text Label 6550 8450 0    50   ~ 0
+5V
Wire Wire Line
	7500 8950 7450 8950
Wire Wire Line
	7500 8950 7700 8950
Connection ~ 7500 8950
Entry Wire Line
	6450 8350 6550 8450
Entry Wire Line
	6450 8450 6550 8550
Entry Wire Line
	6450 8850 6550 8950
$Comp
L dk_Transistors-FETs-MOSFETs-Single:2N7002 Q15
U 1 1 5E262BF5
P 7250 9800
F 0 "Q15" V 7411 9800 60  0000 C CNN
F 1 "2N7002" V 7517 9800 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 7450 10000 60  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 10100 60  0001 L CNN
F 4 "2N7002NCT-ND" H 7450 10200 60  0001 L CNN "Digi-Key_PN"
F 5 "2N7002" H 7450 10300 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7450 10400 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 7450 10500 60  0001 L CNN "Family"
F 8 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 7450 10600 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/2N7002/2N7002NCT-ND/244664" H 7450 10700 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 60V 115MA SOT-23" H 7450 10800 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 7450 10900 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7450 11000 60  0001 L CNN "Status"
	1    7250 9800
	0    1    1    0   
$EndComp
$Comp
L Device:R R33
U 1 1 5E262BFF
P 6850 9650
F 0 "R33" H 6920 9696 50  0000 L CNN
F 1 "10k" H 6920 9605 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6780 9650 50  0001 C CNN
F 3 "~" H 6850 9650 50  0001 C CNN
	1    6850 9650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R34
U 1 1 5E262C09
P 7500 9650
F 0 "R34" H 7570 9696 50  0000 L CNN
F 1 "100k" H 7570 9605 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7430 9650 50  0001 C CNN
F 3 "~" H 7500 9650 50  0001 C CNN
	1    7500 9650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 9800 6850 9800
Connection ~ 6850 9800
Wire Wire Line
	6850 9800 6550 9800
Wire Wire Line
	6550 9400 6850 9400
Wire Wire Line
	6850 9400 6850 9500
Wire Wire Line
	6850 9400 7150 9400
Wire Wire Line
	7150 9400 7150 9500
Connection ~ 6850 9400
Wire Wire Line
	6550 9300 7500 9300
Wire Wire Line
	7500 9300 7500 9500
Text Label 6550 9400 0    50   ~ 0
+3V3
Text Label 6650 9800 1    50   ~ 0
RTD_Gate
Text Label 6550 9300 0    50   ~ 0
+5V
Wire Wire Line
	7500 9800 7450 9800
Wire Wire Line
	7500 9800 7700 9800
Connection ~ 7500 9800
Entry Wire Line
	6450 9200 6550 9300
Entry Wire Line
	6450 9300 6550 9400
Entry Wire Line
	6450 9700 6550 9800
Entry Wire Line
	7700 1750 7800 1850
Entry Wire Line
	7700 2600 7800 2700
Entry Wire Line
	7700 3450 7800 3550
Entry Wire Line
	7700 4300 7800 4400
Entry Wire Line
	7700 5150 7800 5250
Entry Wire Line
	7700 6400 7800 6500
Entry Wire Line
	7700 7250 7800 7350
Entry Wire Line
	7700 8100 7800 8200
Entry Wire Line
	7700 8950 7800 9050
Entry Wire Line
	7700 9800 7800 9900
Entry Wire Line
	2250 3550 2350 3650
Wire Wire Line
	3400 3650 2350 3650
Text Label 2350 3650 0    50   ~ 0
Spare_D3
Text Label 6650 8100 1    50   ~ 0
Spare_D2
Text Label 6650 8950 1    50   ~ 0
Spare_D3
Text Label 7700 1750 3    50   ~ 0
SD_Inertia_
Text Label 7700 2600 3    50   ~ 0
SD_BOTS_
Text Label 7700 3450 3    50   ~ 0
SD_Dash_
Text Label 7700 4300 3    50   ~ 0
APPS_Trail_
Text Label 7700 5150 3    50   ~ 0
APPS_Bound_
Text Label 7700 6400 3    50   ~ 0
APPS_Dis_
Text Label 7700 7250 3    50   ~ 0
Brake_Sw_
Text Label 7700 8100 3    50   ~ 0
Spare_D2_
Text Label 7700 8950 3    50   ~ 0
Spare_D3_
Text Label 7700 9800 3    50   ~ 0
RTD_Gate_
Text Label 2250 850  0    50   ~ 0
MainBus
Text Label 7800 2100 3    50   ~ 0
IO_OutBus
Entry Wire Line
	7800 1950 7900 2050
$Comp
L Device:D_Zener D3
U 1 1 5E31A186
P 8300 2250
F 0 "D3" V 8254 2329 50  0000 L CNN
F 1 "D_Zener" V 8345 2329 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 8300 2250 50  0001 C CNN
F 3 "~" H 8300 2250 50  0001 C CNN
	1    8300 2250
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5E31B78B
P 8650 2300
F 0 "R6" H 8720 2346 50  0000 L CNN
F 1 "20k" H 8720 2255 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8580 2300 50  0001 C CNN
F 3 "~" H 8650 2300 50  0001 C CNN
	1    8650 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5E31BE7B
P 8100 2100
F 0 "R5" H 8170 2146 50  0000 L CNN
F 1 "10k3" H 8170 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8030 2100 50  0001 C CNN
F 3 "~" H 8100 2100 50  0001 C CNN
	1    8100 2100
	0    -1   1    0   
$EndComp
Wire Wire Line
	8650 2450 9000 2450
Connection ~ 8650 2450
Wire Wire Line
	9400 2050 9000 2050
Entry Wire Line
	9400 2050 9500 2150
Entry Wire Line
	9400 2450 9500 2550
Text Label 7900 2050 0    50   ~ 0
Throttle_Pos_
Text Label 7900 3050 0    50   ~ 0
AIn_Spare_
Text Label 9400 2450 3    50   ~ 0
GND
Text Label 9400 2050 1    50   ~ 0
Throttle_Pos
Entry Wire Line
	7800 2950 7900 3050
$Comp
L Device:R R12
U 1 1 5E3A3330
P 8650 3300
F 0 "R12" H 8720 3346 50  0000 L CNN
F 1 "20k" H 8720 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8580 3300 50  0001 C CNN
F 3 "~" H 8650 3300 50  0001 C CNN
	1    8650 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 3050 9000 3050
Entry Wire Line
	9400 3050 9500 3150
Entry Wire Line
	9400 3450 9500 3550
Text Label 9400 3450 3    50   ~ 0
GND
Text Label 9400 3050 1    50   ~ 0
AIn_Spare
Wire Bus Line
	6450 850  9500 850 
$Comp
L Connector_Generic:Conn_01x12 J3
U 1 1 5E3CC37E
P 8650 5200
F 0 "J3" H 8730 5192 50  0000 L CNN
F 1 "MainIO" H 8730 5101 50  0000 L CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-1212_2x06_P3.00mm_Vertical" H 8650 5200 50  0001 C CNN
F 3 "~" H 8650 5200 50  0001 C CNN
	1    8650 5200
	1    0    0    -1  
$EndComp
Entry Wire Line
	7800 4600 7900 4700
Entry Wire Line
	7800 4700 7900 4800
Entry Wire Line
	7800 4800 7900 4900
Entry Wire Line
	7800 4900 7900 5000
Entry Wire Line
	7800 5000 7900 5100
Entry Wire Line
	7800 5100 7900 5200
Entry Wire Line
	7800 5200 7900 5300
Entry Wire Line
	7800 5300 7900 5400
Entry Wire Line
	7800 5400 7900 5500
Entry Wire Line
	7800 5500 7900 5600
Entry Wire Line
	7800 5600 7900 5700
Entry Wire Line
	7800 5700 7900 5800
Wire Wire Line
	7900 4700 8450 4700
Wire Wire Line
	8450 4800 7900 4800
Wire Wire Line
	7900 4900 8450 4900
Wire Wire Line
	8450 5000 7900 5000
Wire Wire Line
	7900 5100 8450 5100
Wire Wire Line
	8450 5200 7900 5200
Wire Wire Line
	7900 5300 8450 5300
Wire Wire Line
	8450 5400 7900 5400
Wire Wire Line
	7900 5500 8450 5500
Wire Wire Line
	8450 5600 7900 5600
Wire Wire Line
	7900 5700 8450 5700
Wire Wire Line
	8450 5800 7900 5800
$Comp
L Device:LED D1
U 1 1 5E1E2ADC
P 10000 1150
F 0 "D1" V 10039 1033 50  0000 R CNN
F 1 "LED" V 9948 1033 50  0000 R CNN
F 2 "Diode_SMD:D_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10000 1150 50  0001 C CNN
F 3 "~" H 10000 1150 50  0001 C CNN
	1    10000 1150
	0    -1   -1   0   
$EndComp
Connection ~ 9500 850 
Entry Wire Line
	9900 850  10000 950 
Entry Wire Line
	9700 850  9800 950 
$Comp
L Device:R R1
U 1 1 5E1FA826
P 10000 1550
F 0 "R1" H 10070 1596 50  0000 L CNN
F 1 "1k" H 10070 1505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 9930 1550 50  0001 C CNN
F 3 "~" H 10000 1550 50  0001 C CNN
	1    10000 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 950  10000 1000
Wire Wire Line
	10000 1300 10000 1400
Wire Wire Line
	10000 1700 9800 1700
Wire Wire Line
	9800 1700 9800 950 
$Comp
L Device:LED D2
U 1 1 5E239BED
P 10500 1150
F 0 "D2" V 10539 1033 50  0000 R CNN
F 1 "LED" V 10448 1033 50  0000 R CNN
F 2 "Diode_SMD:D_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10500 1150 50  0001 C CNN
F 3 "~" H 10500 1150 50  0001 C CNN
	1    10500 1150
	0    -1   -1   0   
$EndComp
Entry Wire Line
	10400 850  10500 950 
Entry Wire Line
	10200 850  10300 950 
$Comp
L Device:R R2
U 1 1 5E239BF9
P 10500 1550
F 0 "R2" H 10570 1596 50  0000 L CNN
F 1 "250" H 10570 1505 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 10430 1550 50  0001 C CNN
F 3 "~" H 10500 1550 50  0001 C CNN
	1    10500 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10500 950  10500 1000
Wire Wire Line
	10500 1300 10500 1400
Wire Wire Line
	10500 1700 10300 1700
Wire Wire Line
	10300 1700 10300 950 
Text Label 10000 950  3    50   ~ 0
+5V
Text Label 10500 950  0    50   ~ 0
+3V3
Text Label 10300 950  3    50   ~ 0
GND
Text Label 9800 950  3    50   ~ 0
GND
Entry Wire Line
	2250 7150 2350 7250
Wire Wire Line
	2350 7250 4050 7250
Wire Wire Line
	4050 7250 4050 6900
Wire Wire Line
	4050 6900 4350 6900
Text Label 2350 7250 0    50   ~ 0
GND
Text Label 7900 5300 0    50   ~ 0
SD_Inertia_
Text Label 6550 2600 0    50   ~ 0
SD_BOTS
Text Label 7900 5400 0    50   ~ 0
SD_BOTS_
Text Label 7900 4700 0    50   ~ 0
SD_Dash_
Text Label 7900 4800 0    50   ~ 0
APPS_Trail_
Text Label 7900 5500 0    50   ~ 0
APPS_Bound_
Text Label 7900 4900 0    50   ~ 0
APPS_Dis_
Text Label 6650 7250 1    50   ~ 0
Brake_Sw
Text Label 7900 5800 0    50   ~ 0
Throttle_Pos_
Text Label 7900 5200 0    50   ~ 0
AIn_Spare_
Text Label 7900 5600 0    50   ~ 0
Brake_Sw_
Text Label 7900 5100 0    50   ~ 0
Spare_D2_
Text Label 7900 5700 0    50   ~ 0
Spare_D3_
Text Label 7900 5000 0    50   ~ 0
RTD_Gate_
Entry Wire Line
	9400 3700 9500 3800
Entry Wire Line
	9400 3800 9500 3900
Entry Wire Line
	9400 3900 9500 4000
Entry Wire Line
	7800 4200 7900 4300
Entry Wire Line
	7800 4300 7900 4400
Entry Wire Line
	7800 4400 7900 4500
Wire Wire Line
	8450 3900 9400 3900
Text Label 7900 4400 0    50   ~ 0
Brake_Sw_
Text Label 7900 4500 0    50   ~ 0
Spare_D2_
Text Label 7900 4300 0    50   ~ 0
RTD_Gate_
Text Label 8450 3800 0    50   ~ 0
+5V
Text Label 8450 3900 0    50   ~ 0
GND
Text Label 8450 3700 0    50   ~ 0
+12V
$Comp
L dk_Transistors-FETs-MOSFETs-Single:AO3401A Q1
U 1 1 5E3643D6
P 1650 1350
F 0 "Q1" V 1917 1350 60  0000 C CNN
F 1 "AO3401A" V 1811 1350 60  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 1850 1550 60  0001 L CNN
F 3 "http://aosmd.com/res/data_sheets/AO3401A.pdf" H 1850 1650 60  0001 L CNN
F 4 "785-1001-1-ND" H 1850 1750 60  0001 L CNN "Digi-Key_PN"
F 5 "AO3401A" H 1850 1850 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 1850 1950 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 1850 2050 60  0001 L CNN "Family"
F 8 "http://aosmd.com/res/data_sheets/AO3401A.pdf" H 1850 2150 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/alpha-omega-semiconductor-inc/AO3401A/785-1001-1-ND/1855943" H 1850 2250 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET P-CH 30V 4A SOT23" H 1850 2350 60  0001 L CNN "Description"
F 11 "Alpha & Omega Semiconductor Inc." H 1850 2450 60  0001 L CNN "Manufacturer"
F 12 "Active" H 1850 2550 60  0001 L CNN "Status"
	1    1650 1350
	0    -1   -1   0   
$EndComp
Entry Wire Line
	2150 1350 2250 1250
Wire Wire Line
	1750 1650 1350 1650
Wire Wire Line
	1050 1850 1050 1950
Connection ~ 1050 1950
Wire Wire Line
	1050 1950 1050 2300
Wire Wire Line
	1450 2000 1150 2000
Text Label 1950 1350 0    50   ~ 0
+12V
$Comp
L Device:C C3
U 1 1 5E411E24
P 9000 2250
F 0 "C3" H 9115 2296 50  0000 L CNN
F 1 "10n" H 9115 2205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9038 2100 50  0001 C CNN
F 3 "~" H 9000 2250 50  0001 C CNN
	1    9000 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5E4127DC
P 9000 3250
F 0 "C4" H 9115 3296 50  0000 L CNN
F 1 "10n" H 9115 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9038 3100 50  0001 C CNN
F 3 "~" H 9000 3250 50  0001 C CNN
	1    9000 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2050 9000 2100
Connection ~ 9000 2050
Wire Wire Line
	9000 2050 8650 2050
Wire Wire Line
	9000 2400 9000 2450
Connection ~ 9000 2450
Wire Wire Line
	9000 2450 9400 2450
Wire Wire Line
	9000 3050 9000 3100
Connection ~ 9000 3050
Wire Wire Line
	9000 3050 8650 3050
Wire Wire Line
	9000 3400 9000 3450
Connection ~ 9000 3450
Wire Wire Line
	9000 3450 9400 3450
Text Notes 6850 1500 0    50   ~ 0
3V3 Zener Diodes\non inputs 1-5
NoConn ~ 3400 5150
NoConn ~ 3400 4950
NoConn ~ 3400 4850
Wire Wire Line
	8350 3800 8350 4100
Wire Wire Line
	8350 3800 9400 3800
Wire Wire Line
	8250 3700 9400 3700
$Comp
L Mechanical:MountingHole H1
U 1 1 5E5D5AD8
P 14250 1050
F 0 "H1" H 14350 1096 50  0000 L CNN
F 1 "MountingHole" H 14350 1005 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 14250 1050 50  0001 C CNN
F 3 "~" H 14250 1050 50  0001 C CNN
	1    14250 1050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5E5F02C2
P 14250 1250
F 0 "H2" H 14350 1296 50  0000 L CNN
F 1 "MountingHole" H 14350 1205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 14250 1250 50  0001 C CNN
F 3 "~" H 14250 1250 50  0001 C CNN
	1    14250 1250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5E5F047F
P 14250 1450
F 0 "H3" H 14350 1496 50  0000 L CNN
F 1 "MountingHole" H 14350 1405 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 14250 1450 50  0001 C CNN
F 3 "~" H 14250 1450 50  0001 C CNN
	1    14250 1450
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5E5F063C
P 14250 1650
F 0 "H4" H 14350 1696 50  0000 L CNN
F 1 "MountingHole" H 14350 1605 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 14250 1650 50  0001 C CNN
F 3 "~" H 14250 1650 50  0001 C CNN
	1    14250 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 9300 4800 9300
Connection ~ 4400 9300
Wire Wire Line
	4850 10100 4850 9250
Wire Wire Line
	1450 1350 1350 1350
Wire Wire Line
	1200 1350 1200 1750
Wire Wire Line
	1050 1750 1200 1750
Wire Wire Line
	1150 2000 1150 1650
Wire Wire Line
	1150 1650 1050 1650
Wire Wire Line
	4150 6650 4150 6600
Wire Wire Line
	4150 6600 4350 6600
Text Notes 8050 2950 0    50   ~ 0
Divider values for 5V logic\nChange for other logic inputs\n
Wire Wire Line
	4800 8950 5050 8950
Wire Wire Line
	4800 8950 4800 9300
Wire Wire Line
	5050 9050 4950 9050
Wire Wire Line
	4950 9050 4950 8900
Wire Wire Line
	4950 9150 4950 10500
Wire Wire Line
	4950 9150 5050 9150
Wire Wire Line
	4850 9250 5050 9250
Wire Wire Line
	4100 7150 4100 6800
Wire Wire Line
	4100 6800 4350 6800
Wire Wire Line
	4250 6300 4250 7100
Wire Wire Line
	4250 7100 4350 7100
Wire Wire Line
	4200 6700 4350 6700
Wire Wire Line
	4200 6700 4200 8000
Text Label 4500 8900 0    50   ~ 0
C1+
Text Label 4500 9300 0    50   ~ 0
C1-
Text Label 4550 10100 0    50   ~ 0
C2+
Text Label 4550 10500 0    50   ~ 0
C2-
Wire Wire Line
	7900 4500 8850 4500
Wire Wire Line
	7900 4400 8850 4400
Wire Wire Line
	7900 4300 8850 4300
$Comp
L Connector_Generic:Conn_01x06 J2
U 1 1 5E2AFC46
P 9050 4200
F 0 "J2" H 9130 4192 50  0000 L CNN
F 1 "RTD_Daughter" H 9130 4101 50  0000 L CNN
F 2 "DashDriver-V0:RTD-Daughter" H 9050 4200 50  0001 C CNN
F 3 "~" H 9050 4200 50  0001 C CNN
	1    9050 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 4200 8850 4200
Wire Wire Line
	8450 3900 8450 4200
Wire Wire Line
	8850 4100 8350 4100
Wire Wire Line
	8850 4000 8250 4000
Wire Wire Line
	8250 4000 8250 3700
$Comp
L Connector_Generic:Conn_01x04 J6
U 1 1 5F885C71
P 8650 6100
F 0 "J6" H 8730 6092 50  0000 L CNN
F 1 "Analog 0-5V" H 8730 6001 50  0000 L CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43045-0412_2x02_P3.00mm_Vertical" H 8650 6100 50  0001 C CNN
F 3 "~" H 8650 6100 50  0001 C CNN
	1    8650 6100
	1    0    0    -1  
$EndComp
Entry Wire Line
	7800 5900 7900 6000
Entry Wire Line
	7800 6000 7900 6100
Entry Wire Line
	7800 6100 7900 6200
Entry Wire Line
	7800 6200 7900 6300
Wire Wire Line
	8450 6000 7900 6000
Wire Wire Line
	7900 6100 8450 6100
Wire Wire Line
	7900 6200 8450 6200
Wire Wire Line
	7900 6300 8450 6300
Text Label 7900 6000 0    50   ~ 0
AIn_Spare_
Text Label 7900 6200 0    50   ~ 0
Throttle_Pos_
Text Label 7900 6100 0    50   ~ 0
BrkPrs_F_
Text Label 7900 6300 0    50   ~ 0
BrkPrs_R_
Entry Wire Line
	7800 6850 7900 6950
$Comp
L Device:R R39
U 1 1 5F9030D8
P 8650 7200
F 0 "R39" H 8720 7246 50  0000 L CNN
F 1 "20k" H 8720 7155 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8580 7200 50  0001 C CNN
F 3 "~" H 8650 7200 50  0001 C CNN
	1    8650 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 7350 9000 7350
Wire Wire Line
	9400 6950 9000 6950
Entry Wire Line
	9400 6950 9500 7050
Entry Wire Line
	9400 7350 9500 7450
Text Label 7900 6950 0    50   ~ 0
BrkPrs_F_
Text Label 9400 7350 3    50   ~ 0
GND
Text Label 9400 6950 1    50   ~ 0
BrkPrs_F
$Comp
L Device:C C9
U 1 1 5F9030F8
P 9000 7150
F 0 "C9" H 9115 7196 50  0000 L CNN
F 1 "10n" H 9115 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9038 7000 50  0001 C CNN
F 3 "~" H 9000 7150 50  0001 C CNN
	1    9000 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 6950 9000 7000
Connection ~ 9000 6950
Wire Wire Line
	9000 6950 8650 6950
Wire Wire Line
	9000 7300 9000 7350
Connection ~ 9000 7350
Wire Wire Line
	9000 7350 9400 7350
Entry Wire Line
	7800 7950 7900 8050
$Comp
L Device:R R40
U 1 1 5F9241DF
P 8650 8300
F 0 "R40" H 8720 8346 50  0000 L CNN
F 1 "20k" H 8720 8255 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8580 8300 50  0001 C CNN
F 3 "~" H 8650 8300 50  0001 C CNN
	1    8650 8300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 8450 9000 8450
Wire Wire Line
	9400 8050 9000 8050
Entry Wire Line
	9400 8050 9500 8150
Entry Wire Line
	9400 8450 9500 8550
Text Label 7900 8050 0    50   ~ 0
BrkPrs_R_
Text Label 9400 8450 3    50   ~ 0
GND
Text Label 9400 8050 1    50   ~ 0
BrkPrs_R
$Comp
L Device:C C10
U 1 1 5F9241FF
P 9000 8250
F 0 "C10" H 9115 8296 50  0000 L CNN
F 1 "10n" H 9115 8205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9038 8100 50  0001 C CNN
F 3 "~" H 9000 8250 50  0001 C CNN
	1    9000 8250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 8050 9000 8100
Connection ~ 9000 8050
Wire Wire Line
	9000 8050 8650 8050
Wire Wire Line
	9000 8400 9000 8450
Connection ~ 9000 8450
Wire Wire Line
	9000 8450 9400 8450
Entry Wire Line
	2250 4250 2350 4350
Entry Wire Line
	2250 3650 2350 3750
Wire Wire Line
	3400 3750 2350 3750
Wire Wire Line
	2350 4350 3400 4350
Wire Wire Line
	8650 2050 8650 2150
Wire Wire Line
	7900 2050 7950 2050
Wire Wire Line
	7950 2050 7950 2100
Wire Wire Line
	8250 2100 8300 2100
Wire Wire Line
	8300 2100 8300 2050
Wire Wire Line
	8300 2050 8650 2050
Connection ~ 8650 2050
Connection ~ 8300 2100
Wire Wire Line
	8300 2400 8300 2450
Wire Wire Line
	8300 2450 8650 2450
Wire Wire Line
	8650 3050 8650 3150
$Comp
L Device:D_Zener D4
U 1 1 5FA794C7
P 8300 3250
F 0 "D4" V 8254 3329 50  0000 L CNN
F 1 "D_Zener" V 8345 3329 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 8300 3250 50  0001 C CNN
F 3 "~" H 8300 3250 50  0001 C CNN
	1    8300 3250
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 5FA794CD
P 8100 3100
F 0 "R9" H 8170 3146 50  0000 L CNN
F 1 "10k3" H 8170 3055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8030 3100 50  0001 C CNN
F 3 "~" H 8100 3100 50  0001 C CNN
	1    8100 3100
	0    -1   1    0   
$EndComp
Wire Wire Line
	7950 3050 7950 3100
Wire Wire Line
	8250 3100 8300 3100
Wire Wire Line
	8300 3100 8300 3050
Connection ~ 8300 3100
Wire Wire Line
	8300 3400 8300 3450
Wire Wire Line
	8300 3450 8650 3450
Wire Wire Line
	8300 3050 8650 3050
Connection ~ 8650 3050
Wire Wire Line
	7950 3050 7900 3050
Wire Wire Line
	8650 6950 8650 7050
$Comp
L Device:D_Zener D5
U 1 1 5FAFC0D4
P 8300 7150
F 0 "D5" V 8254 7229 50  0000 L CNN
F 1 "D_Zener" V 8345 7229 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 8300 7150 50  0001 C CNN
F 3 "~" H 8300 7150 50  0001 C CNN
	1    8300 7150
	0    1    1    0   
$EndComp
$Comp
L Device:R R37
U 1 1 5FAFC0DA
P 8100 7000
F 0 "R37" H 8170 7046 50  0000 L CNN
F 1 "10k3" H 8170 6955 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8030 7000 50  0001 C CNN
F 3 "~" H 8100 7000 50  0001 C CNN
	1    8100 7000
	0    -1   1    0   
$EndComp
Wire Wire Line
	7950 6950 7950 7000
Wire Wire Line
	8250 7000 8300 7000
Wire Wire Line
	8300 7000 8300 6950
Connection ~ 8300 7000
Wire Wire Line
	8300 7300 8300 7350
Wire Wire Line
	8650 8050 8650 8150
$Comp
L Device:D_Zener D6
U 1 1 5FB3BB65
P 8300 8250
F 0 "D6" V 8254 8329 50  0000 L CNN
F 1 "D_Zener" V 8345 8329 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 8300 8250 50  0001 C CNN
F 3 "~" H 8300 8250 50  0001 C CNN
	1    8300 8250
	0    1    1    0   
$EndComp
$Comp
L Device:R R38
U 1 1 5FB3BB6B
P 8100 8100
F 0 "R38" H 8170 8146 50  0000 L CNN
F 1 "10k3" H 8170 8055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8030 8100 50  0001 C CNN
F 3 "~" H 8100 8100 50  0001 C CNN
	1    8100 8100
	0    -1   1    0   
$EndComp
Wire Wire Line
	7950 8050 7950 8100
Wire Wire Line
	8250 8100 8300 8100
Wire Wire Line
	8300 8100 8300 8050
Connection ~ 8300 8100
Wire Wire Line
	8300 8400 8300 8450
Connection ~ 8650 3450
Wire Wire Line
	8650 3450 9000 3450
Wire Wire Line
	8300 7350 8650 7350
Connection ~ 8650 7350
Wire Wire Line
	8650 6950 8300 6950
Connection ~ 8650 6950
Wire Wire Line
	7950 6950 7900 6950
Wire Wire Line
	7900 8050 7950 8050
Wire Wire Line
	8300 8050 8650 8050
Connection ~ 8650 8050
Wire Wire Line
	8300 8450 8650 8450
Connection ~ 8650 8450
Text Label 2350 4350 0    50   ~ 0
BrkPrs_R
Text Label 2350 3750 0    50   ~ 0
BrkPrs_F
$Comp
L Device:R R41
U 1 1 5F89FB57
P 1350 1500
F 0 "R41" H 1420 1546 50  0000 L CNN
F 1 "10k" H 1420 1455 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1280 1500 50  0001 C CNN
F 3 "~" H 1350 1500 50  0001 C CNN
	1    1350 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R42
U 1 1 5F8A0A06
P 1250 1800
F 0 "R42" H 1320 1846 50  0000 L CNN
F 1 "10k" H 1320 1755 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1180 1800 50  0001 C CNN
F 3 "~" H 1250 1800 50  0001 C CNN
	1    1250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1950 1250 2300
Connection ~ 1350 1350
Wire Wire Line
	1350 1350 1200 1350
Connection ~ 1350 1650
Wire Wire Line
	1350 1650 1250 1650
Wire Wire Line
	1850 1350 2150 1350
$Comp
L Device:D_Zener D7
U 1 1 5F91B6D6
P 6800 1900
F 0 "D7" V 6754 1979 50  0000 L CNN
F 1 "D_Zener" V 6845 1979 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 6800 1900 50  0001 C CNN
F 3 "~" H 6800 1900 50  0001 C CNN
	1    6800 1900
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5F9FC089
P 7300 1750
F 0 "R3" H 7370 1796 50  0000 L CNN
F 1 "10k" H 7370 1705 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7230 1750 50  0001 C CNN
F 3 "~" H 7300 1750 50  0001 C CNN
	1    7300 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 1750 7450 1750
Wire Wire Line
	7150 1750 6800 1750
Connection ~ 6800 1750
Wire Wire Line
	6800 1750 6550 1750
Wire Wire Line
	6550 2100 6800 2100
Wire Wire Line
	6800 2100 6800 2050
Text Label 6550 2100 0    50   ~ 0
GND
Text Notes 8250 1500 0    50   ~ 0
5V Zener Diodes\non inputs 1-5
$Comp
L Device:D_Zener D8
U 1 1 5FA99799
P 6800 2750
F 0 "D8" V 6754 2829 50  0000 L CNN
F 1 "D_Zener" V 6845 2829 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 6800 2750 50  0001 C CNN
F 3 "~" H 6800 2750 50  0001 C CNN
	1    6800 2750
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5FA9979F
P 7300 2600
F 0 "R4" H 7370 2646 50  0000 L CNN
F 1 "10k" H 7370 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7230 2600 50  0001 C CNN
F 3 "~" H 7300 2600 50  0001 C CNN
	1    7300 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 2600 7450 2600
Wire Wire Line
	7150 2600 6800 2600
Connection ~ 6800 2600
Wire Wire Line
	6800 2600 6550 2600
Wire Wire Line
	6550 2950 6800 2950
Wire Wire Line
	6800 2950 6800 2900
Text Label 6550 2950 0    50   ~ 0
GND
Text Label 6550 4300 0    50   ~ 0
APPS_Trail
Wire Wire Line
	7450 5150 7700 5150
$Comp
L Device:D_Zener D9
U 1 1 5FB83315
P 6800 3600
F 0 "D9" V 6754 3679 50  0000 L CNN
F 1 "D_Zener" V 6845 3679 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 6800 3600 50  0001 C CNN
F 3 "~" H 6800 3600 50  0001 C CNN
	1    6800 3600
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5FB8331B
P 7300 3450
F 0 "R7" H 7370 3496 50  0000 L CNN
F 1 "10k" H 7370 3405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7230 3450 50  0001 C CNN
F 3 "~" H 7300 3450 50  0001 C CNN
	1    7300 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 3450 7450 3450
Wire Wire Line
	7150 3450 6800 3450
Connection ~ 6800 3450
Wire Wire Line
	6800 3450 6550 3450
Wire Wire Line
	6550 3800 6800 3800
Wire Wire Line
	6800 3800 6800 3750
Text Label 6550 3800 0    50   ~ 0
GND
$Comp
L DashDriver-V1-rescue:BZD27Cxx-Diode D10
U 1 1 5FB9F779
P 6800 4450
F 0 "D10" V 6754 4529 50  0000 L CNN
F 1 "D_Zener" V 6845 4529 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 6800 4450 50  0001 C CNN
F 3 "~" H 6800 4450 50  0001 C CNN
	1    6800 4450
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5FB9F77F
P 7300 4300
F 0 "R8" H 7370 4346 50  0000 L CNN
F 1 "10k" H 7370 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7230 4300 50  0001 C CNN
F 3 "~" H 7300 4300 50  0001 C CNN
	1    7300 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 4300 7450 4300
Wire Wire Line
	7150 4300 6800 4300
Connection ~ 6800 4300
Wire Wire Line
	6800 4300 6550 4300
Wire Wire Line
	6550 4650 6800 4650
Wire Wire Line
	6800 4650 6800 4600
Text Label 6550 4650 0    50   ~ 0
GND
$Comp
L Device:D_Zener D11
U 1 1 5FBBC967
P 6800 5300
F 0 "D11" V 6754 5379 50  0000 L CNN
F 1 "D_Zener" V 6845 5379 50  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 6800 5300 50  0001 C CNN
F 3 "~" H 6800 5300 50  0001 C CNN
	1    6800 5300
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 5FBBC96D
P 7300 5150
F 0 "R10" H 7370 5196 50  0000 L CNN
F 1 "10k" H 7370 5105 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7230 5150 50  0001 C CNN
F 3 "~" H 7300 5150 50  0001 C CNN
	1    7300 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	7150 5150 6800 5150
Connection ~ 6800 5150
Wire Wire Line
	6800 5150 6550 5150
Wire Wire Line
	6550 5500 6800 5500
Wire Wire Line
	6800 5500 6800 5450
Text Label 6550 5500 0    50   ~ 0
GND
$Comp
L Diode:BZX84Cxx D?
U 1 1 5F9EC17F
P 5850 5950
F 0 "D?" H 5850 6167 50  0000 C CNN
F 1 "BZX84C33" H 5850 6076 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 5850 5775 50  0001 C CNN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/bzx84c2v4.pdf" H 5850 5950 50  0001 C CNN
	1    5850 5950
	1    0    0    -1  
$EndComp
Text Notes 11500 3900 0    50   ~ 0
Q1 - PMos used as a polarity protection device \nR1,R2 - voltage divider to protect Q1 gate from voltage exceeding vDS\nD1 - in normal operation no current flow, if reverse polarity allows \nlarge current to flow, blows fuse up stream
Wire Bus Line
	2250 850  6450 850 
Wire Bus Line
	9500 850  11000 850 
Wire Bus Line
	9500 850  9500 8550
Wire Bus Line
	6450 850  6450 9700
Wire Bus Line
	7800 1850 7800 9900
Wire Bus Line
	2250 850  2250 10600
$EndSCHEMATC