EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ki_modules
LIBS:shieldboard-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "6 feb 2017"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_01X01 P?
U 1 1 5B1D9972
P 1500 2300
F 0 "P?" H 1500 2400 50  0000 C CNN
F 1 "CONN_01X01" V 1600 2300 50  0000 C CNN
F 2 "" H 1500 2300 50  0000 C CNN
F 3 "" H 1500 2300 50  0000 C CNN
	1    1500 2300
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1D99CC
P 1500 2900
F 0 "P?" H 1500 3000 50  0000 C CNN
F 1 "CONN_01X01" V 1600 2900 50  0000 C CNN
F 2 "" H 1500 2900 50  0000 C CNN
F 3 "" H 1500 2900 50  0000 C CNN
	1    1500 2900
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1D9B36
P 1500 2000
F 0 "P?" H 1500 2100 50  0000 C CNN
F 1 "CONN_01X01" V 1600 2000 50  0000 C CNN
F 2 "" H 1500 2000 50  0000 C CNN
F 3 "" H 1500 2000 50  0000 C CNN
	1    1500 2000
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1D9B5E
P 1500 1700
F 0 "P?" H 1500 1800 50  0000 C CNN
F 1 "CONN_01X01" V 1600 1700 50  0000 C CNN
F 2 "" H 1500 1700 50  0000 C CNN
F 3 "" H 1500 1700 50  0000 C CNN
	1    1500 1700
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1D9B91
P 1500 1400
F 0 "P?" H 1500 1500 50  0000 C CNN
F 1 "CONN_01X01" V 1600 1400 50  0000 C CNN
F 2 "" H 1500 1400 50  0000 C CNN
F 3 "" H 1500 1400 50  0000 C CNN
	1    1500 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1700 2900 1800 2900
Wire Wire Line
	1800 2900 1800 3050
$Comp
L GND #PWR?
U 1 1 5B1D9BE8
P 1800 3050
F 0 "#PWR?" H 1800 2800 50  0001 C CNN
F 1 "GND" H 1800 2900 50  0000 C CNN
F 2 "" H 1800 3050 50  0000 C CNN
F 3 "" H 1800 3050 50  0000 C CNN
	1    1800 3050
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5B1D9C1A
P 1750 1300
F 0 "#PWR?" H 1750 1150 50  0001 C CNN
F 1 "+5V" H 1750 1440 50  0000 C CNN
F 2 "" H 1750 1300 50  0000 C CNN
F 3 "" H 1750 1300 50  0000 C CNN
	1    1750 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1300 1750 1400
Wire Wire Line
	1750 1400 1700 1400
$Comp
L +3.3V #PWR?
U 1 1 5B1D9CBA
P 2000 1300
F 0 "#PWR?" H 2000 1150 50  0001 C CNN
F 1 "+3.3V" H 2000 1440 50  0000 C CNN
F 2 "" H 2000 1300 50  0000 C CNN
F 3 "" H 2000 1300 50  0000 C CNN
	1    2000 1300
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1D9A8A
P 1550 5050
F 0 "J?" H 1550 5200 50  0000 C TNN
F 1 "Screw_Terminal_1x01" V 1400 5050 50  0000 C TNN
F 2 "" H 1550 4925 50  0001 C CNN
F 3 "" H 1550 4950 50  0001 C CNN
	1    1550 5050
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1D9FDA
P 1500 2600
F 0 "P?" H 1500 2700 50  0000 C CNN
F 1 "CONN_01X01" V 1600 2600 50  0000 C CNN
F 2 "" H 1500 2600 50  0000 C CNN
F 3 "" H 1500 2600 50  0000 C CNN
	1    1500 2600
	-1   0    0    1   
$EndComp
Text Notes 1700 1650 0    60   ~ 0
BI_1
Text Notes 1700 1950 0    60   ~ 0
BI_2
Text Notes 1700 2250 0    60   ~ 0
BO_1
Text Notes 1700 2550 0    60   ~ 0
BO_2
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1DA223
P 1550 4700
F 0 "J?" H 1550 4850 50  0000 C TNN
F 1 "Screw_Terminal_1x01" V 1400 4700 50  0000 C TNN
F 2 "" H 1550 4575 50  0001 C CNN
F 3 "" H 1550 4600 50  0001 C CNN
	1    1550 4700
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1DA3BB
P 10500 2800
F 0 "J?" H 10500 2950 50  0000 C TNN
F 1 "GND" V 10350 2800 50  0000 C TNN
F 2 "" H 10500 2675 50  0001 C CNN
F 3 "" H 10500 2700 50  0001 C CNN
	1    10500 2800
	-1   0    0    -1  
$EndComp
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1DA56D
P 10500 2300
F 0 "J?" H 10500 2450 50  0000 C TNN
F 1 "+12V" V 10350 2300 50  0000 C TNN
F 2 "" H 10500 2175 50  0001 C CNN
F 3 "" H 10500 2200 50  0001 C CNN
	1    10500 2300
	-1   0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q?
U 1 1 5B1DA607
P 2100 5300
F 0 "Q?" H 2300 5350 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 2300 5250 50  0000 L CNN
F 2 "" H 2300 5400 50  0000 C CNN
F 3 "" H 2100 5300 50  0000 C CNN
	1    2100 5300
	-1   0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 5B1DA757
P 2000 4850
F 0 "D?" H 2000 4950 50  0000 C CNN
F 1 "D" H 2000 4750 50  0000 C CNN
F 2 "" H 2000 4850 50  0000 C CNN
F 3 "" H 2000 4850 50  0000 C CNN
	1    2000 4850
	0    1    1    0   
$EndComp
$Comp
L +12V #PWR?
U 1 1 5B1DA950
P 2000 4600
F 0 "#PWR?" H 2000 4450 50  0001 C CNN
F 1 "+12V" H 2000 4740 50  0000 C CNN
F 2 "" H 2000 4600 50  0000 C CNN
F 3 "" H 2000 4600 50  0000 C CNN
	1    2000 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4700 2000 4700
Wire Wire Line
	2000 4700 2000 4600
Wire Wire Line
	2000 5000 2000 5100
Wire Wire Line
	1750 5050 2000 5050
Connection ~ 2000 5050
$Comp
L GND #PWR?
U 1 1 5B1DAB19
P 2000 5600
F 0 "#PWR?" H 2000 5350 50  0001 C CNN
F 1 "GND" H 2000 5450 50  0000 C CNN
F 2 "" H 2000 5600 50  0000 C CNN
F 3 "" H 2000 5600 50  0000 C CNN
	1    2000 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 5500 2000 5600
$Comp
L R R?
U 1 1 5B1DAC0C
P 2550 5300
F 0 "R?" V 2630 5300 50  0000 C CNN
F 1 "100" V 2550 5300 50  0000 C CNN
F 2 "" V 2480 5300 50  0000 C CNN
F 3 "" H 2550 5300 50  0000 C CNN
	1    2550 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	2300 5300 2400 5300
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1DB303
P 1550 6550
F 0 "J?" H 1550 6700 50  0000 C TNN
F 1 "Screw_Terminal_1x01" V 1400 6550 50  0000 C TNN
F 2 "" H 1550 6425 50  0001 C CNN
F 3 "" H 1550 6450 50  0001 C CNN
	1    1550 6550
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1DB309
P 1550 6200
F 0 "J?" H 1550 6350 50  0000 C TNN
F 1 "Screw_Terminal_1x01" V 1400 6200 50  0000 C TNN
F 2 "" H 1550 6075 50  0001 C CNN
F 3 "" H 1550 6100 50  0001 C CNN
	1    1550 6200
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q?
U 1 1 5B1DB30F
P 2100 6800
F 0 "Q?" H 2300 6850 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 2300 6750 50  0000 L CNN
F 2 "" H 2300 6900 50  0000 C CNN
F 3 "" H 2100 6800 50  0000 C CNN
	1    2100 6800
	-1   0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 5B1DB315
P 2000 6350
F 0 "D?" H 2000 6450 50  0000 C CNN
F 1 "D" H 2000 6250 50  0000 C CNN
F 2 "" H 2000 6350 50  0000 C CNN
F 3 "" H 2000 6350 50  0000 C CNN
	1    2000 6350
	0    1    1    0   
$EndComp
$Comp
L +12V #PWR?
U 1 1 5B1DB31B
P 2000 6100
F 0 "#PWR?" H 2000 5950 50  0001 C CNN
F 1 "+12V" H 2000 6240 50  0000 C CNN
F 2 "" H 2000 6100 50  0000 C CNN
F 3 "" H 2000 6100 50  0000 C CNN
	1    2000 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 6200 2000 6200
Wire Wire Line
	2000 6200 2000 6100
Wire Wire Line
	2000 6500 2000 6600
Wire Wire Line
	1750 6550 2000 6550
Connection ~ 2000 6550
$Comp
L GND #PWR?
U 1 1 5B1DB326
P 2000 7100
F 0 "#PWR?" H 2000 6850 50  0001 C CNN
F 1 "GND" H 2000 6950 50  0000 C CNN
F 2 "" H 2000 7100 50  0000 C CNN
F 3 "" H 2000 7100 50  0000 C CNN
	1    2000 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 7000 2000 7100
$Comp
L R R?
U 1 1 5B1DB32D
P 2550 6800
F 0 "R?" V 2630 6800 50  0000 C CNN
F 1 "100" V 2550 6800 50  0000 C CNN
F 2 "" V 2480 6800 50  0000 C CNN
F 3 "" H 2550 6800 50  0000 C CNN
	1    2550 6800
	0    1    1    0   
$EndComp
Wire Wire Line
	2300 6800 2400 6800
Text Notes 10750 2350 0    60   ~ 0
+12V
Text Notes 10750 2850 0    60   ~ 0
-12V
$Comp
L CP C?
U 1 1 5B1DB7E4
P 9950 2550
F 0 "C?" H 9975 2650 50  0000 L CNN
F 1 "470,0x16" H 9975 2450 50  0000 L CNN
F 2 "" H 9988 2400 50  0000 C CNN
F 3 "" H 9950 2550 50  0000 C CNN
	1    9950 2550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5B1DB8AC
P 9650 2550
F 0 "C?" H 9675 2650 50  0000 L CNN
F 1 "0,1" H 9675 2450 50  0000 L CNN
F 2 "" H 9688 2400 50  0000 C CNN
F 3 "" H 9650 2550 50  0000 C CNN
	1    9650 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 2300 10300 2300
Wire Wire Line
	9650 2200 9650 2400
Wire Wire Line
	7200 2800 10300 2800
Wire Wire Line
	9650 2700 9650 2900
Wire Wire Line
	9950 1150 9950 2400
Connection ~ 9950 2300
Wire Wire Line
	9950 2700 9950 2800
Connection ~ 9950 2800
$Comp
L GND #PWR?
U 1 1 5B1DBE08
P 9650 2900
F 0 "#PWR?" H 9650 2650 50  0001 C CNN
F 1 "GND" H 9650 2750 50  0000 C CNN
F 2 "" H 9650 2900 50  0000 C CNN
F 3 "" H 9650 2900 50  0000 C CNN
	1    9650 2900
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 5B1DC0F7
P 9650 2200
F 0 "#PWR?" H 9650 2050 50  0001 C CNN
F 1 "+12V" H 9650 2340 50  0000 C CNN
F 2 "" H 9650 2200 50  0000 C CNN
F 3 "" H 9650 2200 50  0000 C CNN
	1    9650 2200
	1    0    0    -1  
$EndComp
Connection ~ 9650 2300
Connection ~ 9650 2800
$Comp
L LM7805CT U?
U 1 1 5B1DC57D
P 8200 2350
F 0 "U?" H 8000 2550 50  0000 C CNN
F 1 "LM7805CT" H 8200 2550 50  0000 L CNN
F 2 "TO-220" H 8200 2450 50  0000 C CIN
F 3 "" H 8200 2350 50  0000 C CNN
	1    8200 2350
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5B1DC73C
P 8700 2550
F 0 "C?" H 8725 2650 50  0000 L CNN
F 1 "0,1" H 8725 2450 50  0000 L CNN
F 2 "" H 8738 2400 50  0000 C CNN
F 3 "" H 8700 2550 50  0000 C CNN
	1    8700 2550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5B1DC7B2
P 7700 2550
F 0 "C?" H 7725 2650 50  0000 L CNN
F 1 "0,1" H 7725 2450 50  0000 L CNN
F 2 "" H 7738 2400 50  0000 C CNN
F 3 "" H 7700 2550 50  0000 C CNN
	1    7700 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2700 7700 2900
Wire Wire Line
	7700 2200 7700 2400
Wire Wire Line
	7550 2300 7800 2300
Wire Wire Line
	8200 2600 8200 2800
Connection ~ 8200 2800
Wire Wire Line
	8700 2700 8700 2800
Connection ~ 8700 2800
Wire Wire Line
	8700 2400 8700 2300
Connection ~ 8700 2300
$Comp
L GND #PWR?
U 1 1 5B1DC98C
P 7700 2900
F 0 "#PWR?" H 7700 2650 50  0001 C CNN
F 1 "GND" H 7700 2750 50  0000 C CNN
F 2 "" H 7700 2900 50  0000 C CNN
F 3 "" H 7700 2900 50  0000 C CNN
	1    7700 2900
	1    0    0    -1  
$EndComp
Connection ~ 7700 2800
$Comp
L TEST_1P W?
U 1 1 5B1DCAAA
P 9000 1600
F 0 "W?" H 9000 1870 50  0000 C CNN
F 1 "TEST_1P" H 9000 1800 50  0000 C CNN
F 2 "" H 9200 1600 50  0000 C CNN
F 3 "" H 9200 1600 50  0000 C CNN
	1    9000 1600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5B1DCF53
P 7700 2200
F 0 "#PWR?" H 7700 2050 50  0001 C CNN
F 1 "+5V" H 7700 2340 50  0000 C CNN
F 2 "" H 7700 2200 50  0000 C CNN
F 3 "" H 7700 2200 50  0000 C CNN
	1    7700 2200
	1    0    0    -1  
$EndComp
Connection ~ 7700 2300
Text Notes 8900 1250 0    60   ~ 0
-IN
$Comp
L TEST_1P W?
U 1 1 5B1DD58F
P 9350 1600
F 0 "W?" H 9350 1870 50  0000 C CNN
F 1 "TEST_1P" H 9350 1800 50  0000 C CNN
F 2 "" H 9550 1600 50  0000 C CNN
F 3 "" H 9550 1600 50  0000 C CNN
	1    9350 1600
	1    0    0    -1  
$EndComp
Text Notes 9250 1250 0    60   ~ 0
+IN
$Comp
L TEST_1P W?
U 1 1 5B1DD6C0
P 7200 1600
F 0 "W?" H 7200 1870 50  0000 C CNN
F 1 "TEST_1P" H 7200 1800 50  0000 C CNN
F 2 "" H 7400 1600 50  0000 C CNN
F 3 "" H 7400 1600 50  0000 C CNN
	1    7200 1600
	1    0    0    -1  
$EndComp
Text Notes 7100 1250 0    60   ~ 0
-OUT
$Comp
L TEST_1P W?
U 1 1 5B1DD6C7
P 7550 1600
F 0 "W?" H 7550 1870 50  0000 C CNN
F 1 "TEST_1P" H 7550 1800 50  0000 C CNN
F 2 "" H 7750 1600 50  0000 C CNN
F 3 "" H 7750 1600 50  0000 C CNN
	1    7550 1600
	1    0    0    -1  
$EndComp
Text Notes 7450 1250 0    60   ~ 0
+OUT
Wire Wire Line
	7550 2300 7550 1600
Wire Wire Line
	7200 2800 7200 1600
Wire Wire Line
	9000 1600 9000 2800
Connection ~ 9000 2800
Wire Wire Line
	9350 1600 9350 2300
Connection ~ 9350 2300
Wire Notes Line
	9700 1650 6950 1650
Wire Notes Line
	6950 1650 6950 850 
Wire Notes Line
	6950 850  9700 850 
Wire Notes Line
	9700 850  9700 1650
Text Notes 7800 1000 0    60   ~ 0
Optional DC-DC module
$Comp
L Maple_Mini M?
U 1 1 5B1F0024
P 6850 4800
F 0 "M?" H 6300 5900 60  0000 C CNN
F 1 "Maple_Mini" H 7350 5900 60  0000 C CNN
F 2 "" H 6850 4800 60  0001 C CNN
F 3 "" H 6850 4800 60  0001 C CNN
	1    6850 4800
	1    0    0    -1  
$EndComp
Text Notes 1100 2350 1    60   ~ 0
sensor_port_1
Text Notes 1250 5900 1    60   ~ 0
coil_port_1
Text Notes 2650 5200 0    60   ~ 0
Ao
Text Notes 2650 6700 0    60   ~ 0
Ai
$Comp
L CONN_01X01 P?
U 1 1 5B1F2531
P 3250 2350
F 0 "P?" H 3250 2450 50  0000 C CNN
F 1 "CONN_01X01" V 3350 2350 50  0000 C CNN
F 2 "" H 3250 2350 50  0000 C CNN
F 3 "" H 3250 2350 50  0000 C CNN
	1    3250 2350
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1F2537
P 3250 2950
F 0 "P?" H 3250 3050 50  0000 C CNN
F 1 "CONN_01X01" V 3350 2950 50  0000 C CNN
F 2 "" H 3250 2950 50  0000 C CNN
F 3 "" H 3250 2950 50  0000 C CNN
	1    3250 2950
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1F253D
P 3250 2050
F 0 "P?" H 3250 2150 50  0000 C CNN
F 1 "CONN_01X01" V 3350 2050 50  0000 C CNN
F 2 "" H 3250 2050 50  0000 C CNN
F 3 "" H 3250 2050 50  0000 C CNN
	1    3250 2050
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1F2543
P 3250 1750
F 0 "P?" H 3250 1850 50  0000 C CNN
F 1 "CONN_01X01" V 3350 1750 50  0000 C CNN
F 2 "" H 3250 1750 50  0000 C CNN
F 3 "" H 3250 1750 50  0000 C CNN
	1    3250 1750
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1F2549
P 3250 1450
F 0 "P?" H 3250 1550 50  0000 C CNN
F 1 "CONN_01X01" V 3350 1450 50  0000 C CNN
F 2 "" H 3250 1450 50  0000 C CNN
F 3 "" H 3250 1450 50  0000 C CNN
	1    3250 1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	3450 2950 3550 2950
Wire Wire Line
	3550 2950 3550 3100
$Comp
L GND #PWR?
U 1 1 5B1F2551
P 3550 3100
F 0 "#PWR?" H 3550 2850 50  0001 C CNN
F 1 "GND" H 3550 2950 50  0000 C CNN
F 2 "" H 3550 3100 50  0000 C CNN
F 3 "" H 3550 3100 50  0000 C CNN
	1    3550 3100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5B1F2557
P 3500 1350
F 0 "#PWR?" H 3500 1200 50  0001 C CNN
F 1 "+5V" H 3500 1490 50  0000 C CNN
F 2 "" H 3500 1350 50  0000 C CNN
F 3 "" H 3500 1350 50  0000 C CNN
	1    3500 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1350 3500 1450
Wire Wire Line
	3500 1450 3450 1450
$Comp
L +3.3V #PWR?
U 1 1 5B1F255F
P 3750 1350
F 0 "#PWR?" H 3750 1200 50  0001 C CNN
F 1 "+3.3V" H 3750 1490 50  0000 C CNN
F 2 "" H 3750 1350 50  0000 C CNN
F 3 "" H 3750 1350 50  0000 C CNN
	1    3750 1350
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1F2565
P 3300 5100
F 0 "J?" H 3300 5250 50  0000 C TNN
F 1 "Screw_Terminal_1x01" V 3150 5100 50  0000 C TNN
F 2 "" H 3300 4975 50  0001 C CNN
F 3 "" H 3300 5000 50  0001 C CNN
	1    3300 5100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P?
U 1 1 5B1F256B
P 3250 2650
F 0 "P?" H 3250 2750 50  0000 C CNN
F 1 "CONN_01X01" V 3350 2650 50  0000 C CNN
F 2 "" H 3250 2650 50  0000 C CNN
F 3 "" H 3250 2650 50  0000 C CNN
	1    3250 2650
	-1   0    0    1   
$EndComp
Text Notes 3450 1700 0    60   ~ 0
BI_1
Text Notes 3450 2000 0    60   ~ 0
BI_2
Text Notes 3450 2300 0    60   ~ 0
BO_1
Text Notes 3450 2600 0    60   ~ 0
BO_2
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1F2575
P 3300 4750
F 0 "J?" H 3300 4900 50  0000 C TNN
F 1 "Screw_Terminal_1x01" V 3150 4750 50  0000 C TNN
F 2 "" H 3300 4625 50  0001 C CNN
F 3 "" H 3300 4650 50  0001 C CNN
	1    3300 4750
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q?
U 1 1 5B1F257B
P 3850 5350
F 0 "Q?" H 4050 5400 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 4050 5300 50  0000 L CNN
F 2 "" H 4050 5450 50  0000 C CNN
F 3 "" H 3850 5350 50  0000 C CNN
	1    3850 5350
	-1   0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 5B1F2581
P 3750 4900
F 0 "D?" H 3750 5000 50  0000 C CNN
F 1 "D" H 3750 4800 50  0000 C CNN
F 2 "" H 3750 4900 50  0000 C CNN
F 3 "" H 3750 4900 50  0000 C CNN
	1    3750 4900
	0    1    1    0   
$EndComp
$Comp
L +12V #PWR?
U 1 1 5B1F2587
P 3750 4650
F 0 "#PWR?" H 3750 4500 50  0001 C CNN
F 1 "+12V" H 3750 4790 50  0000 C CNN
F 2 "" H 3750 4650 50  0000 C CNN
F 3 "" H 3750 4650 50  0000 C CNN
	1    3750 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4750 3750 4750
Wire Wire Line
	3750 4750 3750 4650
Wire Wire Line
	3750 5050 3750 5150
Wire Wire Line
	3500 5100 3750 5100
Connection ~ 3750 5100
$Comp
L GND #PWR?
U 1 1 5B1F2592
P 3750 5650
F 0 "#PWR?" H 3750 5400 50  0001 C CNN
F 1 "GND" H 3750 5500 50  0000 C CNN
F 2 "" H 3750 5650 50  0000 C CNN
F 3 "" H 3750 5650 50  0000 C CNN
	1    3750 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5550 3750 5650
$Comp
L R R?
U 1 1 5B1F2599
P 4300 5350
F 0 "R?" V 4380 5350 50  0000 C CNN
F 1 "100" V 4300 5350 50  0000 C CNN
F 2 "" V 4230 5350 50  0000 C CNN
F 3 "" H 4300 5350 50  0000 C CNN
	1    4300 5350
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 5350 4150 5350
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1F25A0
P 3300 6600
F 0 "J?" H 3300 6750 50  0000 C TNN
F 1 "Screw_Terminal_1x01" V 3150 6600 50  0000 C TNN
F 2 "" H 3300 6475 50  0001 C CNN
F 3 "" H 3300 6500 50  0001 C CNN
	1    3300 6600
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1F25A6
P 3300 6250
F 0 "J?" H 3300 6400 50  0000 C TNN
F 1 "Screw_Terminal_1x01" V 3150 6250 50  0000 C TNN
F 2 "" H 3300 6125 50  0001 C CNN
F 3 "" H 3300 6150 50  0001 C CNN
	1    3300 6250
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q?
U 1 1 5B1F25AC
P 3850 6850
F 0 "Q?" H 4050 6900 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 4050 6800 50  0000 L CNN
F 2 "" H 4050 6950 50  0000 C CNN
F 3 "" H 3850 6850 50  0000 C CNN
	1    3850 6850
	-1   0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 5B1F25B2
P 3750 6400
F 0 "D?" H 3750 6500 50  0000 C CNN
F 1 "D" H 3750 6300 50  0000 C CNN
F 2 "" H 3750 6400 50  0000 C CNN
F 3 "" H 3750 6400 50  0000 C CNN
	1    3750 6400
	0    1    1    0   
$EndComp
$Comp
L +12V #PWR?
U 1 1 5B1F25B8
P 3750 6150
F 0 "#PWR?" H 3750 6000 50  0001 C CNN
F 1 "+12V" H 3750 6290 50  0000 C CNN
F 2 "" H 3750 6150 50  0000 C CNN
F 3 "" H 3750 6150 50  0000 C CNN
	1    3750 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6250 3750 6250
Wire Wire Line
	3750 6250 3750 6150
Wire Wire Line
	3750 6550 3750 6650
Wire Wire Line
	3500 6600 3750 6600
Connection ~ 3750 6600
$Comp
L GND #PWR?
U 1 1 5B1F25C3
P 3750 7150
F 0 "#PWR?" H 3750 6900 50  0001 C CNN
F 1 "GND" H 3750 7000 50  0000 C CNN
F 2 "" H 3750 7150 50  0000 C CNN
F 3 "" H 3750 7150 50  0000 C CNN
	1    3750 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 7050 3750 7150
$Comp
L R R?
U 1 1 5B1F25CA
P 4300 6850
F 0 "R?" V 4380 6850 50  0000 C CNN
F 1 "100" V 4300 6850 50  0000 C CNN
F 2 "" V 4230 6850 50  0000 C CNN
F 3 "" H 4300 6850 50  0000 C CNN
	1    4300 6850
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 6850 4150 6850
Text Notes 2850 2400 1    60   ~ 0
sensor_port_2
Text Notes 3000 5950 1    60   ~ 0
coil_port_2
Text Notes 4400 5250 0    60   ~ 0
Ao
Text Notes 4400 6750 0    60   ~ 0
Ai
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1F3377
P 10500 1650
F 0 "J?" H 10500 1800 50  0000 C TNN
F 1 "GND" V 10350 1650 50  0000 C TNN
F 2 "" H 10500 1525 50  0001 C CNN
F 3 "" H 10500 1550 50  0001 C CNN
	1    10500 1650
	-1   0    0    -1  
$EndComp
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1F337D
P 10500 1150
F 0 "J?" H 10500 1300 50  0000 C TNN
F 1 "+12V" V 10350 1150 50  0000 C TNN
F 2 "" H 10500 1025 50  0001 C CNN
F 3 "" H 10500 1050 50  0001 C CNN
	1    10500 1150
	-1   0    0    -1  
$EndComp
Text Notes 10750 1200 0    60   ~ 0
+12V
Text Notes 10750 1700 0    60   ~ 0
-12V
Wire Wire Line
	10300 1150 9950 1150
$Comp
L GND #PWR?
U 1 1 5B1F36DC
P 10200 1750
F 0 "#PWR?" H 10200 1500 50  0001 C CNN
F 1 "GND" H 10200 1600 50  0000 C CNN
F 2 "" H 10200 1750 50  0000 C CNN
F 3 "" H 10200 1750 50  0000 C CNN
	1    10200 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 1650 10200 1650
Wire Wire Line
	10200 1650 10200 1750
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1F616B
P 10500 3850
F 0 "J?" H 10500 4000 50  0000 C TNN
F 1 "GND" V 10350 3850 50  0000 C TNN
F 2 "" H 10500 3725 50  0001 C CNN
F 3 "" H 10500 3750 50  0001 C CNN
	1    10500 3850
	-1   0    0    -1  
$EndComp
$Comp
L Screw_Terminal_1x01 J?
U 1 1 5B1F6171
P 10500 3350
F 0 "J?" H 10500 3500 50  0000 C TNN
F 1 "+5V" V 10350 3350 50  0000 C TNN
F 2 "" H 10500 3225 50  0001 C CNN
F 3 "" H 10500 3250 50  0001 C CNN
	1    10500 3350
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B1F6178
P 10200 3950
F 0 "#PWR?" H 10200 3700 50  0001 C CNN
F 1 "GND" H 10200 3800 50  0000 C CNN
F 2 "" H 10200 3950 50  0000 C CNN
F 3 "" H 10200 3950 50  0000 C CNN
	1    10200 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 3850 10200 3850
Wire Wire Line
	10200 3850 10200 3950
$Comp
L +5V #PWR?
U 1 1 5B1F7499
P 10200 3250
F 0 "#PWR?" H 10200 3100 50  0001 C CNN
F 1 "+5V" H 10200 3390 50  0000 C CNN
F 2 "" H 10200 3250 50  0000 C CNN
F 3 "" H 10200 3250 50  0000 C CNN
	1    10200 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 3350 10200 3350
Wire Wire Line
	10200 3350 10200 3250
Text Label 5650 4250 2    60   ~ 0
PWM_00
Text Label 5650 4350 2    60   ~ 0
PWM_01
Text Label 5650 5350 2    60   ~ 0
PWM_10
Text Label 5650 5450 2    60   ~ 0
PWM_11
Text Label 5650 3850 2    60   ~ 0
SENS_00
Text Label 5650 3950 2    60   ~ 0
SENS_01
Text Label 5650 4050 2    60   ~ 0
SENS_02
Text Label 5650 4150 2    60   ~ 0
SENS_03
Text Label 5650 5150 2    60   ~ 0
SENS_10
Text Label 5650 5250 2    60   ~ 0
SENS_11
Text Label 8050 4350 0    60   ~ 0
SENS_12
Text Label 8050 4250 0    60   ~ 0
SENS_13
Text Label 1700 1700 0    60   ~ 0
SENS_00
Text Label 1700 2000 0    60   ~ 0
SENS_01
Text Label 1700 2300 0    60   ~ 0
SENS_02
Text Label 1700 2600 0    60   ~ 0
SENS_03
Text Label 3450 1750 0    60   ~ 0
SENS_10
Text Label 3450 2050 0    60   ~ 0
SENS_11
Text Label 3450 2350 0    60   ~ 0
SENS_12
Text Label 3450 2650 0    60   ~ 0
SENS_13
Text Label 2700 5300 0    60   ~ 0
PWM_00
Text Label 2700 6800 0    60   ~ 0
PWM_01
Text Label 4450 6850 0    60   ~ 0
PWM_11
Text Label 4450 5350 0    60   ~ 0
PWM_10
$Comp
L GND #PWR?
U 1 1 5B209E40
P 8050 3850
F 0 "#PWR?" H 8050 3600 50  0001 C CNN
F 1 "GND" H 8050 3700 50  0000 C CNN
F 2 "" H 8050 3850 50  0000 C CNN
F 3 "" H 8050 3850 50  0000 C CNN
	1    8050 3850
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5B209EE2
P 8050 3950
F 0 "#PWR?" H 8050 3700 50  0001 C CNN
F 1 "GND" H 8050 3800 50  0000 C CNN
F 2 "" H 8050 3950 50  0000 C CNN
F 3 "" H 8050 3950 50  0000 C CNN
	1    8050 3950
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5B20A0CF
P 5650 5650
F 0 "#PWR?" H 5650 5400 50  0001 C CNN
F 1 "GND" H 5650 5500 50  0000 C CNN
F 2 "" H 5650 5650 50  0000 C CNN
F 3 "" H 5650 5650 50  0000 C CNN
	1    5650 5650
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR?
U 1 1 5B20D291
P 5650 5550
F 0 "#PWR?" H 5650 5400 50  0001 C CNN
F 1 "+5V" H 5650 5690 50  0000 C CNN
F 2 "" H 5650 5550 50  0000 C CNN
F 3 "" H 5650 5550 50  0000 C CNN
	1    5650 5550
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5B20E5A1
P 5500 4650
F 0 "R?" V 5580 4650 50  0000 C CNN
F 1 "1.5k" V 5500 4650 50  0000 C CNN
F 2 "" V 5430 4650 50  0000 C CNN
F 3 "" H 5500 4650 50  0000 C CNN
	1    5500 4650
	0    1    1    0   
$EndComp
Text Notes 6650 4850 0    60   ~ 0
DESOLDER R10!
Wire Notes Line
	6100 4650 6300 4650
Wire Notes Line
	6300 4650 6300 4850
Wire Notes Line
	6250 4850 6350 4850
Wire Notes Line
	6350 4850 6350 5000
Wire Notes Line
	6350 5000 6250 5000
Wire Notes Line
	6250 5000 6250 4850
Wire Notes Line
	6050 5750 6300 5750
Wire Notes Line
	6300 5750 6300 5000
Text Notes 6400 4950 0    60   ~ 0
R10
Wire Wire Line
	5350 4650 5350 4450
Wire Wire Line
	5350 4450 5650 4450
$EndSCHEMATC
