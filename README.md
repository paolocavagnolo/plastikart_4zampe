# PARTE UNO: CONFIGURAZIONE CONTROLLINO MEGA

## Blocco dei finecorsa

```c
#define ES1_CEN_LIM CONTROLLINO_A0
#define ES1_BWD_LIM CONTROLLINO_A1
#define ES1_FWD_LIM CONTROLLINO_A2

#define ES2_CEN_LIM CONTROLLINO_A3
#define ES2_BWD_LIM CONTROLLINO_A4
#define ES2_FWD_LIM CONTROLLINO_A5

#define ES3_CEN_LIM CONTROLLINO_A6
#define ES3_BWD_LIM CONTROLLINO_A7
#define ES3_FWD_LIM CONTROLLINO_A8

#define ES4_CEN_LIM CONTROLLINO_A9
#define ES4_BWD_LIM CONTROLLINO_A10
#define ES4_FWD_LIM CONTROLLINO_A11
```


## Blocco dei pulsanti
```c
#define SW0_AUTO CONTROLLINO_A12
#define SW0_MAN CONTROLLINO_A13

#define SW1_MOT1 CONTROLLINO_A14
#define SW1_MOT2 CONTROLLINO_A15
#define SW1_MOT3 CONTROLLINO_I16

#define SW2_BWD CONTROLLINO_I17
#define SW2_FWD CONTROLLINO_I18
```

## Ingressi di sicurezza
```c
#define INV_ON CONTROLLINO_IN0
#define NO_ANOM CONTROLLINO_IN1
```

## OUTPUT
```c
#define MOT1_PWM CONTROLLINO_D0
#define MOT2_PWM CONTROLLINO_D1
#define MOT3_PWM CONTROLLINO_D2
#define MOT4_PWM CONTROLLINO_D3

#define MOT1_FWD CONTROLLINO_D4
#define MOT1_BWD CONTROLLINO_D5

#define MOT2_FWD CONTROLLINO_D6
#define MOT2_BWD CONTROLLINO_D7

#define MOT3_FWD CONTROLLINO_D8
#define MOT3_BWD CONTROLLINO_D9

#define MOT4_FWD CONTROLLINO_D10
#define MOT4_BWD CONTROLLINO_D11
```


# PARTE DUE: CONFIGURAZIONE MODBUS RTU

ArduinoRS485 + ArduinoMODBUS: funzionano benissimo con qModMaster

## Generale
**baudrate** = 19200
**slave id** = 43

## Configurazione Coils
```c
ModbusRTUServer.configureCoils(0x00, 8);
```
- 0 1: mot1 fwd / bwd
- 2 3: mot2 fwd / bwd
- 4 5: mot3 fwd / bwd
- 6 7: mot4 fwd / bwd

0 = OFF
1 = ON

## Configurazione DiscreteInput
```c
ModbusRTUServer.configureDiscreteInputs(0x00, 22);
```
- 0 1 2: es1 fwd / cen / bwd
- 3 4 5: es1 fwd / cen / bwd
- 6 7 8: es1 fwd / cen / bwd
- 9 10 11: es1 fwd / cen / bwd
- 12 13: auto / man
- 14 15 16 17: man_mot1 / man_mot2 / man_mot3 / man_mot4 ready
- 18 19: mot choosen fwd / bwd
- 20: inverter on
- 21: no anomalies

0 = open - no contact

1 = closed


## Configurazione HoldingRegister
```c
ModbusRTUServer.configureHoldingRegisters(0x00, 4);
```
- 0 1 2 3: PWM velocity value mot1 / mot2 / mot3 / mot

velocity regulation from 0 to 255.

## Configurazione InputRegister
```c
ModbusRTUServer.configureInputRegisters(0x00, 1);
```
- 0: State of the system

0 = OFF

1 = MAN

2 = AUTO

11 = ANOMALIES




