EESchema Schematic File Version 4
LIBS:stm32f446_actuators-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
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
L Connector:USB_B_Micro J2
U 1 1 5E64BD64
P 1300 1450
F 0 "J2" H 1357 1917 50  0000 C CNN
F 1 "USB_B_Micro" H 1357 1826 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex_47346-0001" H 1450 1400 50  0001 C CNN
F 3 "~" H 1450 1400 50  0001 C CNN
	1    1300 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5E64DE2F
P 1300 2000
F 0 "#PWR07" H 1300 1750 50  0001 C CNN
F 1 "GND" H 1305 1827 50  0000 C CNN
F 2 "" H 1300 2000 50  0001 C CNN
F 3 "" H 1300 2000 50  0001 C CNN
	1    1300 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1850 1200 1900
Wire Wire Line
	1200 1900 1300 1900
Wire Wire Line
	1300 1900 1300 1850
Wire Wire Line
	1300 1900 1300 2000
Connection ~ 1300 1900
$Comp
L power:VBUS #PWR08
U 1 1 5E64E790
P 1650 1100
F 0 "#PWR08" H 1650 950 50  0001 C CNN
F 1 "VBUS" H 1665 1273 50  0000 C CNN
F 2 "" H 1650 1100 50  0001 C CNN
F 3 "" H 1650 1100 50  0001 C CNN
	1    1650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1100 1650 1250
Wire Wire Line
	1650 1250 1600 1250
$Comp
L Device:R R5
U 1 1 5E65BE7E
P 2100 1450
F 0 "R5" V 2050 1600 50  0000 C CNN
F 1 "22" V 2100 1450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2030 1450 50  0001 C CNN
F 3 "~" H 2100 1450 50  0001 C CNN
	1    2100 1450
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5E65D770
P 2100 1550
F 0 "R6" V 2050 1700 50  0000 C CNN
F 1 "22" V 2100 1550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2030 1550 50  0001 C CNN
F 3 "~" H 2100 1550 50  0001 C CNN
	1    2100 1550
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5E65DC47
P 2100 1650
F 0 "R7" V 2050 1800 50  0000 C CNN
F 1 "100k" V 2100 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2030 1650 50  0001 C CNN
F 3 "~" H 2100 1650 50  0001 C CNN
	1    2100 1650
	0    1    1    0   
$EndComp
Wire Wire Line
	1600 1550 1950 1550
$Comp
L power:GND #PWR010
U 1 1 5E65E4E6
P 2350 2000
F 0 "#PWR010" H 2350 1750 50  0001 C CNN
F 1 "GND" H 2355 1827 50  0000 C CNN
F 2 "" H 2350 2000 50  0001 C CNN
F 3 "" H 2350 2000 50  0001 C CNN
	1    2350 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 2000 2350 1650
Wire Wire Line
	2350 1650 2250 1650
Wire Wire Line
	1950 1650 1600 1650
Wire Wire Line
	1600 1450 1900 1450
$Comp
L Device:R R4
U 1 1 5E663560
P 1900 1250
F 0 "R4" H 1970 1296 50  0000 L CNN
F 1 "1k5" H 1970 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1830 1250 50  0001 C CNN
F 3 "~" H 1900 1250 50  0001 C CNN
	1    1900 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 1400 1900 1450
Connection ~ 1900 1450
Wire Wire Line
	1900 1450 1950 1450
Text Label 2350 1550 0    50   ~ 0
USB_DM
Text Label 2350 1450 0    50   ~ 0
USB_DP
Wire Wire Line
	2350 1450 2250 1450
Wire Wire Line
	2250 1550 2350 1550
$Comp
L Device:R R8
U 1 1 5E66F60C
P 3450 3250
F 0 "R8" H 3520 3296 50  0000 L CNN
F 1 "100k" H 3520 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3380 3250 50  0001 C CNN
F 3 "~" H 3450 3250 50  0001 C CNN
	1    3450 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5E66FB05
P 3450 3600
F 0 "C3" H 3565 3646 50  0000 L CNN
F 1 "100nF" H 3565 3555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3488 3450 50  0001 C CNN
F 3 "~" H 3450 3600 50  0001 C CNN
	1    3450 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5E670756
P 3450 3750
F 0 "#PWR011" H 3450 3500 50  0001 C CNN
F 1 "GND" H 3455 3577 50  0000 C CNN
F 2 "" H 3450 3750 50  0001 C CNN
F 3 "" H 3450 3750 50  0001 C CNN
	1    3450 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 3450 3450 3450
Wire Wire Line
	3450 3450 3450 3400
Connection ~ 3450 3450
$Comp
L Device:R R10
U 1 1 5E675B99
P 4300 3650
F 0 "R10" V 4250 3500 50  0000 C CNN
F 1 "100k" V 4300 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4230 3650 50  0001 C CNN
F 3 "~" H 4300 3650 50  0001 C CNN
	1    4300 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	4450 3650 4650 3650
$Comp
L power:GND #PWR012
U 1 1 5E676BB4
P 3950 3700
F 0 "#PWR012" H 3950 3450 50  0001 C CNN
F 1 "GND" H 3955 3527 50  0000 C CNN
F 2 "" H 3950 3700 50  0001 C CNN
F 3 "" H 3950 3700 50  0001 C CNN
	1    3950 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 3700 3950 3650
Wire Wire Line
	3950 3650 4150 3650
Text Label 4600 3850 2    50   ~ 0
OSC_IN
Text Label 4600 3950 2    50   ~ 0
OSC_OUT
Wire Wire Line
	4600 3950 4650 3950
Wire Wire Line
	4650 3850 4600 3850
$Comp
L Device:Crystal Y1
U 1 1 5E67B6C9
P 1700 3700
F 0 "Y1" V 1750 3550 50  0000 R CNN
F 1 "8MHz(12pF)" V 1650 3550 50  0000 R CNN
F 2 "Crystal:Crystal_SMD_HC49-SD" H 1700 3700 50  0001 C CNN
F 3 "~" H 1700 3700 50  0001 C CNN
	1    1700 3700
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 5E67CE01
P 1450 3900
F 0 "C2" V 1500 4000 50  0000 C CNN
F 1 "20pF" V 1500 3750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1488 3750 50  0001 C CNN
F 3 "~" H 1450 3900 50  0001 C CNN
	1    1450 3900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1300 3500 1200 3500
Wire Wire Line
	1200 3900 1300 3900
$Comp
L power:GND #PWR06
U 1 1 5E683A8A
P 1200 4000
F 0 "#PWR06" H 1200 3750 50  0001 C CNN
F 1 "GND" H 1205 3827 50  0000 C CNN
F 2 "" H 1200 4000 50  0001 C CNN
F 3 "" H 1200 4000 50  0001 C CNN
	1    1200 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 4000 1200 3900
Connection ~ 1200 3900
Text Label 1850 3500 0    50   ~ 0
OSC_IN
Text Label 1850 3900 0    50   ~ 0
OSC_OUT
Wire Wire Line
	1850 3900 1700 3900
Wire Wire Line
	1700 3500 1850 3500
Wire Wire Line
	1700 3850 1700 3900
Wire Wire Line
	1700 3550 1700 3500
$Comp
L Device:C C1
U 1 1 5E67BFA4
P 1450 3500
F 0 "C1" V 1400 3400 50  0000 C CNN
F 1 "20pF" V 1400 3650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1488 3350 50  0001 C CNN
F 3 "~" H 1450 3500 50  0001 C CNN
	1    1450 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	1600 3900 1700 3900
Connection ~ 1700 3900
Wire Wire Line
	1600 3500 1700 3500
Connection ~ 1700 3500
Wire Wire Line
	1200 3500 1200 3900
$Comp
L Device:R R11
U 1 1 5E69B3F0
P 4300 4150
F 0 "R11" V 4250 4000 50  0000 C CNN
F 1 "10k" V 4300 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4230 4150 50  0001 C CNN
F 3 "~" H 4300 4150 50  0001 C CNN
	1    4300 4150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5E69C028
P 3950 4200
F 0 "#PWR013" H 3950 3950 50  0001 C CNN
F 1 "GND" H 3955 4027 50  0000 C CNN
F 2 "" H 3950 4200 50  0001 C CNN
F 3 "" H 3950 4200 50  0001 C CNN
	1    3950 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4200 3950 4150
Wire Wire Line
	3950 4150 4150 4150
Wire Wire Line
	4450 4150 4650 4150
$Comp
L power:GND #PWR014
U 1 1 5E6A1705
P 3950 4800
F 0 "#PWR014" H 3950 4550 50  0001 C CNN
F 1 "GND" H 3955 4627 50  0000 C CNN
F 2 "" H 3950 4800 50  0001 C CNN
F 3 "" H 3950 4800 50  0001 C CNN
	1    3950 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4800 3950 4750
Wire Wire Line
	3950 4750 4650 4750
$Comp
L Device:C C6
U 1 1 5E6A37E6
P 7250 1650
F 0 "C6" H 7250 1750 50  0000 L CNN
F 1 "100nF" H 7250 1550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7288 1500 50  0001 C CNN
F 3 "~" H 7250 1650 50  0001 C CNN
	1    7250 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5E6A8D31
P 7550 1650
F 0 "C7" H 7550 1750 50  0000 L CNN
F 1 "100nF" H 7550 1550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7588 1500 50  0001 C CNN
F 3 "~" H 7550 1650 50  0001 C CNN
	1    7550 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5E6A9B28
P 7850 1650
F 0 "C8" H 7850 1750 50  0000 L CNN
F 1 "100nF" H 7850 1550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7888 1500 50  0001 C CNN
F 3 "~" H 7850 1650 50  0001 C CNN
	1    7850 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5E6AAAB1
P 8150 1650
F 0 "C9" H 8150 1750 50  0000 L CNN
F 1 "100nF" H 8150 1550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8188 1500 50  0001 C CNN
F 3 "~" H 8150 1650 50  0001 C CNN
	1    8150 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5E6ABE4A
P 7250 1900
F 0 "#PWR022" H 7250 1650 50  0001 C CNN
F 1 "GND" H 7255 1727 50  0000 C CNN
F 2 "" H 7250 1900 50  0001 C CNN
F 3 "" H 7250 1900 50  0001 C CNN
	1    7250 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 1800 7250 1850
Wire Wire Line
	7250 1850 7550 1850
Wire Wire Line
	8150 1850 8150 1800
Wire Wire Line
	7850 1800 7850 1850
Connection ~ 7850 1850
Wire Wire Line
	7850 1850 8150 1850
Wire Wire Line
	7550 1800 7550 1850
Connection ~ 7550 1850
Wire Wire Line
	7250 1500 7250 1450
Wire Wire Line
	7250 1450 7550 1450
Wire Wire Line
	8150 1450 8150 1500
Wire Wire Line
	7850 1500 7850 1450
Connection ~ 7850 1450
Wire Wire Line
	7850 1450 8150 1450
Wire Wire Line
	7550 1500 7550 1450
Connection ~ 7550 1450
Wire Wire Line
	7250 1900 7250 1850
Connection ~ 7250 1450
Wire Wire Line
	7550 1450 7850 1450
Connection ~ 7250 1850
Wire Wire Line
	7550 1850 7850 1850
Wire Wire Line
	5150 3200 5250 3200
Wire Wire Line
	5250 3200 5250 3250
Connection ~ 5150 3200
Wire Wire Line
	5150 3200 5150 3250
Wire Wire Line
	5250 3200 5350 3200
Wire Wire Line
	5350 3200 5350 3250
Connection ~ 5250 3200
Wire Wire Line
	5350 3200 5450 3200
Wire Wire Line
	5450 3200 5450 3250
Connection ~ 5350 3200
Wire Wire Line
	5450 3200 5550 3200
Wire Wire Line
	5550 3200 5550 3250
Connection ~ 5450 3200
$Comp
L Device:C C10
U 1 1 5E6CC946
P 8450 1650
F 0 "C10" H 8450 1750 50  0000 L CNN
F 1 "100nF" H 8450 1550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8488 1500 50  0001 C CNN
F 3 "~" H 8450 1650 50  0001 C CNN
	1    8450 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 1850 8450 1800
Wire Wire Line
	8150 1850 8450 1850
Wire Wire Line
	8450 1450 8450 1500
Wire Wire Line
	8150 1450 8450 1450
$Comp
L power:GND #PWR018
U 1 1 5E6CEC30
P 5300 6350
F 0 "#PWR018" H 5300 6100 50  0001 C CNN
F 1 "GND" H 5305 6177 50  0000 C CNN
F 2 "" H 5300 6350 50  0001 C CNN
F 3 "" H 5300 6350 50  0001 C CNN
	1    5300 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 6250 5150 6300
Wire Wire Line
	5150 6300 5250 6300
Wire Wire Line
	5450 6300 5450 6250
Wire Wire Line
	5350 6250 5350 6300
Connection ~ 5350 6300
Wire Wire Line
	5350 6300 5450 6300
Wire Wire Line
	5250 6250 5250 6300
Connection ~ 5250 6300
Wire Wire Line
	5250 6300 5300 6300
Wire Wire Line
	5300 6350 5300 6300
Connection ~ 5300 6300
Wire Wire Line
	5300 6300 5350 6300
$Comp
L Device:R R9
U 1 1 5E6E2E9A
P 4100 5750
F 0 "R9" V 3893 5750 50  0000 C CNN
F 1 "100" V 3984 5750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4030 5750 50  0001 C CNN
F 3 "~" H 4100 5750 50  0001 C CNN
	1    4100 5750
	0    1    1    0   
$EndComp
Text Label 3850 5950 2    50   ~ 0
T_JTMS
Text Label 4600 5850 2    50   ~ 0
T_JTCK
Wire Wire Line
	3850 5950 3950 5950
Wire Wire Line
	4650 5850 4600 5850
Wire Wire Line
	4250 5750 4650 5750
Wire Wire Line
	3950 5750 3950 5950
Connection ~ 3950 5950
Wire Wire Line
	3950 5950 4650 5950
$Comp
L Device:R R14
U 1 1 5E6F97C2
P 6700 4350
F 0 "R14" H 6770 4396 50  0000 L CNN
F 1 "4k7" H 6770 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6630 4350 50  0001 C CNN
F 3 "~" H 6700 4350 50  0001 C CNN
	1    6700 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 5E6F9E59
P 6700 4750
F 0 "R15" H 6770 4796 50  0000 L CNN
F 1 "4k7" H 6770 4705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6630 4750 50  0001 C CNN
F 3 "~" H 6700 4750 50  0001 C CNN
	1    6700 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5E6FF29C
P 6700 4900
F 0 "#PWR021" H 6700 4650 50  0001 C CNN
F 1 "GND" H 6705 4727 50  0000 C CNN
F 2 "" H 6700 4900 50  0001 C CNN
F 3 "" H 6700 4900 50  0001 C CNN
	1    6700 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 4600 6700 4550
Wire Wire Line
	5950 4550 6700 4550
Connection ~ 6700 4550
Wire Wire Line
	6700 4550 6700 4500
Text Label 6050 5450 0    50   ~ 0
LED_STLINK
Wire Wire Line
	6050 5450 5950 5450
$Comp
L Device:LED D2
U 1 1 5E71A25E
P 1900 5150
F 0 "D2" V 1939 5033 50  0000 R CNN
F 1 "LED_G" V 1848 5033 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 1900 5150 50  0001 C CNN
F 3 "~" H 1900 5150 50  0001 C CNN
	1    1900 5150
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D3
U 1 1 5E71AEFB
P 1900 5700
F 0 "D3" V 1939 5583 50  0000 R CNN
F 1 "LED_R" V 1848 5583 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 1900 5700 50  0001 C CNN
F 3 "~" H 1900 5700 50  0001 C CNN
	1    1900 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 5E71BA82
P 1650 5350
F 0 "R2" V 1600 5200 50  0000 C CNN
F 1 "220" V 1650 5350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1580 5350 50  0001 C CNN
F 3 "~" H 1650 5350 50  0001 C CNN
	1    1650 5350
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5E71C4E5
P 1650 5500
F 0 "R3" V 1600 5350 50  0000 C CNN
F 1 "220" V 1650 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1580 5500 50  0001 C CNN
F 3 "~" H 1650 5500 50  0001 C CNN
	1    1650 5500
	0    1    1    0   
$EndComp
Text Label 1300 5400 2    50   ~ 0
LED_STLINK
Wire Wire Line
	1300 5400 1400 5400
Wire Wire Line
	1400 5400 1400 5350
Wire Wire Line
	1400 5350 1500 5350
Wire Wire Line
	1400 5400 1400 5500
Wire Wire Line
	1400 5500 1500 5500
Connection ~ 1400 5400
Wire Wire Line
	1800 5350 1900 5350
Wire Wire Line
	1900 5350 1900 5300
Wire Wire Line
	1800 5500 1900 5500
Wire Wire Line
	1900 5500 1900 5550
$Comp
L power:GND #PWR09
U 1 1 5E72CD6F
P 1900 5850
F 0 "#PWR09" H 1900 5600 50  0001 C CNN
F 1 "GND" H 1905 5677 50  0000 C CNN
F 2 "" H 1900 5850 50  0001 C CNN
F 3 "" H 1900 5850 50  0001 C CNN
	1    1900 5850
	1    0    0    -1  
$EndComp
Text Label 6050 5650 0    50   ~ 0
USB_DM
Text Label 6050 5750 0    50   ~ 0
USB_DP
Wire Wire Line
	6050 5750 5950 5750
Wire Wire Line
	5950 5650 6050 5650
Wire Wire Line
	6050 5850 5950 5850
Wire Wire Line
	6050 5950 5950 5950
Text HLabel 8500 3150 2    50   Output ~ 0
STLK_TX
Text HLabel 8500 3250 2    50   Input ~ 0
STLK_RX
Wire Wire Line
	6050 4850 5950 4850
Wire Wire Line
	5950 4750 6050 4750
Wire Wire Line
	6050 5550 5950 5550
Text Label 6050 4750 0    50   ~ 0
STLINK_TX
Text Label 6050 4850 0    50   ~ 0
STLINK_RX
Text Label 6050 5550 0    50   ~ 0
T_SWO
Text Label 8250 3150 2    50   ~ 0
STLINK_TX
Text Label 8250 3250 2    50   ~ 0
STLINK_RX
Text Label 8250 3850 2    50   ~ 0
T_SWO
Text Label 8250 3550 2    50   ~ 0
T_JTCK
Text Label 8250 3650 2    50   ~ 0
T_JTMS
Text Label 4600 4550 2    50   ~ 0
T_NRST
Wire Wire Line
	4600 4550 4650 4550
Text Label 8250 3750 2    50   ~ 0
T_NRST
Text HLabel 8500 3550 2    50   UnSpc ~ 0
TCK
Text HLabel 8500 3650 2    50   UnSpc ~ 0
TMS
Text HLabel 8500 3750 2    50   UnSpc ~ 0
NRST
Text HLabel 8500 3850 2    50   UnSpc ~ 0
SWO
Wire Wire Line
	8250 3150 8500 3150
Wire Wire Line
	8500 3250 8250 3250
Wire Wire Line
	8250 3550 8500 3550
Wire Wire Line
	8250 3650 8500 3650
Wire Wire Line
	8250 3750 8500 3750
Wire Wire Line
	8500 3850 8250 3850
Text Label 6050 5950 0    50   ~ 0
STM_JTCK
Text Label 6050 5850 0    50   ~ 0
STM_JTMS
Text Label 8450 5050 2    50   ~ 0
STM_JTCK
Wire Wire Line
	8450 5050 8500 5050
Wire Wire Line
	8500 4950 8450 4950
Text Label 8450 4950 2    50   ~ 0
STM_JTMS
$Comp
L power:GND #PWR024
U 1 1 5E7E8753
P 8400 5200
F 0 "#PWR024" H 8400 4950 50  0001 C CNN
F 1 "GND" H 8405 5027 50  0000 C CNN
F 2 "" H 8400 5200 50  0001 C CNN
F 3 "" H 8400 5200 50  0001 C CNN
	1    8400 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 4850 8500 4850
Wire Wire Line
	8500 5150 8400 5150
Wire Wire Line
	8400 5150 8400 5200
Wire Wire Line
	6050 5050 5950 5050
Connection ~ 8150 1450
Connection ~ 8150 1850
$Comp
L Device:R R13
U 1 1 5E867A66
P 6650 5350
F 0 "R13" V 6700 5200 50  0000 C CNN
F 1 "100" V 6650 5350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6580 5350 50  0001 C CNN
F 3 "~" H 6650 5350 50  0001 C CNN
	1    6650 5350
	0    -1   -1   0   
$EndComp
Text HLabel 6900 5350 2    50   UnSpc ~ 0
MCO
Wire Wire Line
	6900 5350 6800 5350
Wire Wire Line
	6500 5350 5950 5350
$Comp
L Connector:TestPoint TP2
U 1 1 5E8F4C96
P 8500 4950
F 0 "TP2" V 8454 5138 50  0000 L CNN
F 1 "JTMS" V 8500 5300 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 8700 4950 50  0001 C CNN
F 3 "~" H 8700 4950 50  0001 C CNN
	1    8500 4950
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 5E8F9232
P 8500 5050
F 0 "TP3" V 8454 5238 50  0000 L CNN
F 1 "JTCK" V 8500 5400 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 8700 5050 50  0001 C CNN
F 3 "~" H 8700 5050 50  0001 C CNN
	1    8500 5050
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 5E8FD607
P 8500 5150
F 0 "TP4" V 8454 5338 50  0000 L CNN
F 1 "GND" V 8500 5500 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 8700 5150 50  0001 C CNN
F 3 "~" H 8700 5150 50  0001 C CNN
	1    8500 5150
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 5E8F3A34
P 8500 4850
F 0 "TP1" V 8454 5038 50  0000 L CNN
F 1 "+3V3" V 8500 5200 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 8700 4850 50  0001 C CNN
F 3 "~" H 8700 4850 50  0001 C CNN
	1    8500 4850
	0    1    1    0   
$EndComp
Text Label 4650 3650 2    50   ~ 0
BOOT0
Text Label 6050 4550 0    50   ~ 0
AIN_1
Text Label 4650 5750 2    50   ~ 0
T_SWDIO_IN
Text Label 4600 3450 2    50   ~ 0
STM_RESET
Text Label 6050 5050 0    50   ~ 0
T_JTCK
$Comp
L MCU_ST_STM32F1:STM32F103CBTx U2
U 1 1 5E843640
P 5350 4750
F 0 "U2" H 5900 6300 50  0000 C CNN
F 1 "STM32F103CBTx" H 5950 6200 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 4750 3350 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 5350 4750 50  0001 C CNN
	1    5350 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2000 4150 1900
Wire Wire Line
	5900 1300 5350 1300
Wire Wire Line
	5900 1350 5900 1300
Wire Wire Line
	5900 2000 5900 1950
$Comp
L power:GND #PWR020
U 1 1 5E66C98C
P 5900 2000
F 0 "#PWR020" H 5900 1750 50  0001 C CNN
F 1 "GND" H 5905 1827 50  0000 C CNN
F 2 "" H 5900 2000 50  0001 C CNN
F 3 "" H 5900 2000 50  0001 C CNN
	1    5900 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1900 5350 2000
Connection ~ 5350 1300
Wire Wire Line
	5350 1600 5350 1300
Wire Wire Line
	4150 1500 4150 1600
Connection ~ 4150 1500
Wire Wire Line
	4350 1500 4150 1500
Connection ~ 4150 1300
Wire Wire Line
	4150 1300 4150 1500
$Comp
L Device:R R12
U 1 1 5E666118
P 5900 1500
F 0 "R12" H 5970 1546 50  0000 L CNN
F 1 "56" H 5970 1455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5830 1500 50  0001 C CNN
F 3 "~" H 5900 1500 50  0001 C CNN
	1    5900 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 5E6650AB
P 5900 1800
F 0 "D4" V 5939 1683 50  0000 R CNN
F 1 "LED_R" V 5848 1683 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 5900 1800 50  0001 C CNN
F 3 "~" H 5900 1800 50  0001 C CNN
	1    5900 1800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4750 1700 4750 2000
$Comp
L Device:C C5
U 1 1 5E652C91
P 5350 1750
F 0 "C5" H 5465 1796 50  0000 L CNN
F 1 "1uF" H 5465 1705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5388 1600 50  0001 C CNN
F 3 "~" H 5350 1750 50  0001 C CNN
	1    5350 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5E651EED
P 4150 1750
F 0 "C4" H 4265 1796 50  0000 L CNN
F 1 "1uF" H 4265 1705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 1600 50  0001 C CNN
F 3 "~" H 4150 1750 50  0001 C CNN
	1    4150 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5E651E24
P 5350 2000
F 0 "#PWR019" H 5350 1750 50  0001 C CNN
F 1 "GND" H 5355 1827 50  0000 C CNN
F 2 "" H 5350 2000 50  0001 C CNN
F 3 "" H 5350 2000 50  0001 C CNN
	1    5350 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5E651936
P 4150 2000
F 0 "#PWR016" H 4150 1750 50  0001 C CNN
F 1 "GND" H 4155 1827 50  0000 C CNN
F 2 "" H 4150 2000 50  0001 C CNN
F 3 "" H 4150 2000 50  0001 C CNN
	1    4150 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5E650EE1
P 4750 2000
F 0 "#PWR017" H 4750 1750 50  0001 C CNN
F 1 "GND" H 4755 1827 50  0000 C CNN
F 2 "" H 4750 2000 50  0001 C CNN
F 3 "" H 4750 2000 50  0001 C CNN
	1    4750 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1300 5150 1300
Wire Wire Line
	4150 1300 4350 1300
Wire Wire Line
	4150 1150 4150 1300
$Comp
L power:VBUS #PWR015
U 1 1 5E64F2AF
P 4150 1150
F 0 "#PWR015" H 4150 1000 50  0001 C CNN
F 1 "VBUS" H 4165 1323 50  0000 C CNN
F 2 "" H 4150 1150 50  0001 C CNN
F 3 "" H 4150 1150 50  0001 C CNN
	1    4150 1150
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:MIC5504-3.3YM5 U1
U 1 1 5E64CBF7
P 4750 1400
F 0 "U1" H 4750 1767 50  0000 C CNN
F 1 "MIC5504-3.3YM5" H 4750 1676 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 4750 1000 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/MIC550X.pdf" H 4500 1650 50  0001 C CNN
	1    4750 1400
	1    0    0    -1  
$EndComp
Text HLabel 8500 2900 2    50   Output ~ 0
VBUS
$Comp
L power:VBUS #PWR023
U 1 1 5E84A0C8
P 8250 2900
F 0 "#PWR023" H 8250 2750 50  0001 C CNN
F 1 "VBUS" V 8265 3027 50  0000 L CNN
F 2 "" H 8250 2900 50  0001 C CNN
F 3 "" H 8250 2900 50  0001 C CNN
	1    8250 2900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8250 2900 8500 2900
Text Label 5350 1100 0    50   ~ 0
STLINK_3V3
Text Label 1900 1050 0    50   ~ 0
STLINK_3V3
Text Label 5150 3000 0    50   ~ 0
STLINK_3V3
Text Label 6700 4100 0    50   ~ 0
STLINK_3V3
Text Label 8400 4700 0    50   ~ 0
STLINK_3V3
Text Label 1900 4950 0    50   ~ 0
STLINK_3V3
Text Label 3450 3000 0    50   ~ 0
STLINK_3V3
Wire Wire Line
	1900 1050 1900 1100
Wire Wire Line
	5350 1100 5350 1300
Text Label 7250 1350 0    50   ~ 0
STLINK_3V3
Wire Wire Line
	7250 1350 7250 1450
Wire Wire Line
	5150 3000 5150 3200
Wire Wire Line
	3450 3000 3450 3100
Wire Wire Line
	1900 4950 1900 5000
Wire Wire Line
	6700 4100 6700 4200
Wire Wire Line
	8400 4700 8400 4850
$EndSCHEMATC
