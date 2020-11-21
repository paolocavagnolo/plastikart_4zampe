#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <Controllino.h>

//INPUT
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

#define SW0_AUTO CONTROLLINO_A12
#define SW0_MAN CONTROLLINO_A13

#define SW1_MOT1 CONTROLLINO_A14
#define SW1_MOT2 CONTROLLINO_A15
#define SW1_MOT3 CONTROLLINO_I16

#define SW2_BWD CONTROLLINO_I17
#define SW2_FWD CONTROLLINO_I18

#define INV_ON CONTROLLINO_IN0
#define NO_ANOM CONTROLLINO_IN1


//OUTPUT
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

#define VEL_TRESHOLD 10

void stopMot();
void actMot(uint8_t motNum, bool dir, uint8_t vel);

void updatePWM(uint8_t index, uint8_t value);
void updateDIR(uint8_t index, bool value);


void setup() {
  
  //INPUT
  pinMode(ES1_CEN_LIM,INPUT);
  pinMode(ES1_BWD_LIM,INPUT);
  pinMode(ES1_FWD_LIM,INPUT);
  
  pinMode(ES1_CEN_LIM,INPUT);
  pinMode(ES1_BWD_LIM,INPUT);
  pinMode(ES1_FWD_LIM,INPUT);
  
  pinMode(ES1_CEN_LIM,INPUT);
  pinMode(ES1_BWD_LIM,INPUT);
  pinMode(ES1_FWD_LIM,INPUT);
  
  pinMode(ES1_CEN_LIM,INPUT);
  pinMode(ES1_BWD_LIM,INPUT);
  pinMode(ES1_FWD_LIM,INPUT);
  
  pinMode(INV_ON,INPUT);
  pinMode(NO_ANOM,INPUT);
  
  pinMode(SW0_AUTO,INPUT);
  pinMode(SW0_MAN,INPUT);
  
  pinMode(SW1_MOT1,INPUT);
  pinMode(SW1_MOT2,INPUT);
  pinMode(SW1_MOT3,INPUT);
  
  pinMode(SW2_FWD,INPUT);
  pinMode(SW2_BWD,INPUT);
  
  
  //OUTPUT
  pinMode(MOT1_PWM,OUTPUT);
  pinMode(MOT2_PWM,OUTPUT);
  pinMode(MOT3_PWM,OUTPUT);
  pinMode(MOT4_PWM,OUTPUT);
  
  pinMode(MOT1_FWD,OUTPUT);
  pinMode(MOT1_BWD,OUTPUT);
  
  pinMode(MOT2_FWD,OUTPUT);
  pinMode(MOT2_BWD,OUTPUT);
  
  pinMode(MOT3_FWD,OUTPUT);
  pinMode(MOT3_BWD,OUTPUT);
  
  pinMode(MOT4_FWD,OUTPUT);
  pinMode(MOT4_BWD,OUTPUT);

  //MODBUS
  ModbusRTUServer.begin(43, 19200);

  ModbusRTUServer.configureCoils(0x00, 8);
  //0 1: mot1 fwd / bwd
  //2 3: mot2 fwd / bwd
  //4 5: mot3 fwd / bwd
  //6 7: mot4 fwd / bwd

  ModbusRTUServer.configureDiscreteInputs(0x00, 22);
  //0 1 2: es1 fwd / cen / bwd
  //3 4 5: es1 fwd / cen / bwd
  //6 7 8: es1 fwd / cen / bwd
  //9 10 11: es1 fwd / cen / bwd
  //12 13: auto / man
  //14 15 16 17: man_mot1 / man_mot2 / man_mot3 / man_mot4 ready
  //18 19: mot choosen fwd / bwd
  //20: inverter on
  //21: no anomalies

  ModbusRTUServer.configureHoldingRegisters(0x00, 4);
  //0 1 2 3: PWM velocity value mot1 / mot2 / mot3 / mot4

  ModbusRTUServer.configureInputRegisters(0x00, 1);

  Serial.begin(9600);
}

/*
 * ModbusRTUServer.discreteInputRead
  ModbusRTUServer.discreteInputWrite
  ModbusRTUServer.coilRead
  ModbusRTUServer.coilWrite
  ModbusRTUServer.holdingRegisterRead
  ModbusRTUServer.holdingRegisterWrite
  ModbusRTUServer.inputRegisterRead
  ModbusRTUServer.inputRegisterWrite
 */

uint8_t coils[8] = {0,0,0,0,0,0,0,0};
uint8_t vels[8] = {0,0,0,0,0,0,0,0};

uint8_t manVel = 0;
uint8_t state = 0;


void loop() {

  ModbusRTUServer.poll();
  ModbusRTUServer.inputRegisterWrite(0, state);
  
  // END_SWITCH STATE
  ModbusRTUServer.discreteInputWrite(0,digitalRead(ES1_FWD_LIM));
  ModbusRTUServer.discreteInputWrite(1,digitalRead(ES1_CEN_LIM));
  ModbusRTUServer.discreteInputWrite(2,digitalRead(ES1_BWD_LIM));

  ModbusRTUServer.discreteInputWrite(3,digitalRead(ES2_FWD_LIM));
  ModbusRTUServer.discreteInputWrite(4,digitalRead(ES2_CEN_LIM));
  ModbusRTUServer.discreteInputWrite(5,digitalRead(ES2_BWD_LIM));

  ModbusRTUServer.discreteInputWrite(6,digitalRead(ES3_FWD_LIM));
  ModbusRTUServer.discreteInputWrite(7,digitalRead(ES3_CEN_LIM));
  ModbusRTUServer.discreteInputWrite(8,digitalRead(ES3_BWD_LIM));

  ModbusRTUServer.discreteInputWrite(9,digitalRead(ES4_FWD_LIM));
  ModbusRTUServer.discreteInputWrite(10,digitalRead(ES4_CEN_LIM));
  ModbusRTUServer.discreteInputWrite(11,digitalRead(ES4_BWD_LIM));

  // BUTTON STATE
  ModbusRTUServer.discreteInputWrite(12,digitalRead(SW0_AUTO));
  ModbusRTUServer.discreteInputWrite(13,digitalRead(SW0_MAN)); 
  
  ModbusRTUServer.discreteInputWrite(14,digitalRead(SW1_MOT1));
  ModbusRTUServer.discreteInputWrite(15,digitalRead(SW1_MOT2));
  ModbusRTUServer.discreteInputWrite(16,digitalRead(SW1_MOT3));
  if (!digitalRead(SW1_MOT1) && !digitalRead(SW1_MOT2) && !digitalRead(SW1_MOT3)) {
    ModbusRTUServer.discreteInputWrite(17,1);
  }
  else {
    ModbusRTUServer.discreteInputWrite(17,0);
  }
  
  ModbusRTUServer.discreteInputWrite(18,digitalRead(SW2_FWD));
  ModbusRTUServer.discreteInputWrite(19,digitalRead(SW2_BWD));
  
  // SAFETY STATE
  ModbusRTUServer.discreteInputWrite(20,digitalRead(INV_ON));
  ModbusRTUServer.discreteInputWrite(21,digitalRead(NO_ANOM));  

  
  
  if (!digitalRead(INV_ON) or !digitalRead(NO_ANOM)) {
    state = 11;
    stopMot();
  }
  else  {
    // SW0_STATE
    if (digitalRead(SW0_AUTO)) {
      state = 1;
    }
    else if (digitalRead(SW0_MAN)) {
      state = 2;
    }
    else {
      state = 0;
    }
  
    
    //////////////////////////////////////////////////////////// MAIN
    
    if (state == 1) { //SW0_AUTO

      for (uint8_t i=0; i<4; i++) {
        vels[i] = ModbusRTUServer.holdingRegisterRead(i);
        updatePWM(i,vels[i]);
      }
      
      for (uint8_t i=0; i<8; i++) {
        coils[i] = ModbusRTUServer.coilRead(i);
        if ((i%2) && (coils[i]) && (coils[i-1])) {
          coils[i] = 0;
          ModbusRTUServer.coilWrite(i,0);
        }
        updateDIR(i,coils[i]);
      }
      
    }

    ////////////////////////////////////////////////////////////////
    
    else if (state == 2) { //SW0_MAN
      
      manVel = 150;
  
      if (digitalRead(SW2_BWD)) {
        if (digitalRead(SW1_MOT1)) {
          actMot(1,false,manVel);
        }
        else if (digitalRead(SW1_MOT2)){
          actMot(2,false,manVel);
        }
        else if (digitalRead(SW1_MOT3)){
          actMot(3,false,manVel);
        }
        else {
          actMot(4,false,manVel);
        }
      }
      else if (digitalRead(SW2_FWD)){
        if (digitalRead(SW1_MOT1)) {
          actMot(1,true,manVel);
        }
        else if (digitalRead(SW1_MOT2)){
          actMot(2,true,manVel);
        }
        else if (digitalRead(SW1_MOT3)){
          actMot(3,true,manVel);
        }
        else {
          actMot(4,true,manVel);
        }
      }
      else {
        stopMot(); 
      }
      
    }
    else { //SW0_0FF or ANOMALIES
      stopMot(); 
    }
  }
  
}




void updatePWM(uint8_t index, uint8_t value) {
  switch (index) {
    case 0:
      analogWrite(MOT1_PWM, value);
      break;
    case 1:
      analogWrite(MOT2_PWM, value);
      break;
    case 2:
      analogWrite(MOT3_PWM, value);
      break;
    case 3:
      analogWrite(MOT4_PWM, value);
      break;
  }
}

void updateDIR(uint8_t index, bool value) {
  Serial.print(index);
  Serial.print(" ");
  Serial.println(value);
  switch (index) {
    case 0:
      !digitalRead(ES1_FWD_LIM) ? digitalWrite(MOT1_FWD,value) : digitalWrite(MOT1_FWD,LOW); 
      break;
    case 1:
      !digitalRead(ES1_BWD_LIM) ? digitalWrite(MOT1_BWD,value) : digitalWrite(MOT1_BWD,LOW);
      break;
    case 2:
      !digitalRead(ES2_FWD_LIM) ? digitalWrite(MOT2_FWD,value) : digitalWrite(MOT2_FWD,LOW);
      break;
    case 3:
      !digitalRead(ES2_BWD_LIM) ? digitalWrite(MOT2_BWD,value) : digitalWrite(MOT2_BWD,LOW);
      break;
    case 4:
      !digitalRead(ES3_FWD_LIM) ? digitalWrite(MOT3_FWD,value) : digitalWrite(MOT3_FWD,LOW);
      break;
    case 5:
      !digitalRead(ES3_BWD_LIM) ? digitalWrite(MOT3_BWD,value) : digitalWrite(MOT3_BWD,LOW);
      break;
    case 6:
      !digitalRead(ES4_FWD_LIM) ? digitalWrite(MOT4_FWD,value) : digitalWrite(MOT4_FWD,LOW);
      break;
    case 7:
      !digitalRead(ES4_BWD_LIM) ? digitalWrite(MOT4_BWD,value) : digitalWrite(MOT4_BWD,LOW);
      break;
  }
}

void stopMot() {

  digitalWrite(MOT1_FWD,LOW);
  digitalWrite(MOT1_BWD,LOW);
  ModbusRTUServer.coilWrite(0,0);
  ModbusRTUServer.coilWrite(1,0);

  digitalWrite(MOT2_FWD,LOW);
  digitalWrite(MOT2_BWD,LOW);
  ModbusRTUServer.coilWrite(2,0);
  ModbusRTUServer.coilWrite(3,0);

  digitalWrite(MOT3_FWD,LOW);
  digitalWrite(MOT3_BWD,LOW);
  ModbusRTUServer.coilWrite(4,0);
  ModbusRTUServer.coilWrite(5,0);

  digitalWrite(MOT4_FWD,LOW);
  digitalWrite(MOT4_BWD,LOW);
  ModbusRTUServer.coilWrite(6,0);
  ModbusRTUServer.coilWrite(7,0);

  digitalWrite(MOT1_PWM,LOW);
  ModbusRTUServer.holdingRegisterWrite(0,0);
  digitalWrite(MOT2_PWM,LOW);
  ModbusRTUServer.holdingRegisterWrite(1,0);
  digitalWrite(MOT3_PWM,LOW);
  ModbusRTUServer.holdingRegisterWrite(2,0);
  digitalWrite(MOT4_PWM,LOW);
  ModbusRTUServer.holdingRegisterWrite(3,0);
 
}

void actMot(uint8_t motNum, bool dir, uint8_t vel) {

    if (motNum == 1) {
      if (dir) {
        if (!digitalRead(ES1_FWD_LIM)) {
          digitalWrite(MOT1_FWD,HIGH);
          analogWrite(MOT1_PWM,vel);
        }
        else {
          digitalWrite(MOT1_FWD,LOW);
          digitalWrite(MOT1_PWM,LOW);
        }
      }
      else {
        if (!digitalRead(ES1_BWD_LIM)) {
          digitalWrite(MOT1_BWD,HIGH);
          analogWrite(MOT1_PWM,vel);
        }
        else {
          digitalWrite(MOT1_BWD,LOW);
          digitalWrite(MOT1_PWM,LOW);
        }
      }
    }
  
    else if (motNum == 2) {
      if (dir) {
        if (!digitalRead(ES2_FWD_LIM)) {
          digitalWrite(MOT2_FWD,HIGH);
          analogWrite(MOT2_PWM,vel);
        }
        else {
          digitalWrite(MOT2_FWD,LOW);
          digitalWrite(MOT2_PWM,LOW);
        }
      }
      else {
        if (!digitalRead(ES2_BWD_LIM)) {
          digitalWrite(MOT2_BWD,HIGH);
          analogWrite(MOT2_PWM,vel);
        }
        else {
          digitalWrite(MOT2_BWD,LOW);
          digitalWrite(MOT2_PWM,LOW);
        }
      }
    }
  
    else if (motNum == 3) {
      if (dir) {
        if (!digitalRead(ES3_FWD_LIM)) {
          digitalWrite(MOT3_FWD,HIGH);
          analogWrite(MOT3_PWM,vel);
        }
        else {
          digitalWrite(MOT3_FWD,LOW);
          digitalWrite(MOT3_PWM,LOW);
        }
      }
      else {
        if (!digitalRead(ES3_BWD_LIM)) {
          digitalWrite(MOT3_BWD,HIGH);
          analogWrite(MOT3_PWM,vel);
        }
        else {
          digitalWrite(MOT3_BWD,LOW);
          digitalWrite(MOT3_PWM,LOW);
        }
      }
    }
  
    else if (motNum == 4) {
      if (dir) {
        if (!digitalRead(ES4_FWD_LIM)) {
          digitalWrite(MOT4_FWD,HIGH);
          analogWrite(MOT4_PWM,vel);
        }
        else {
          digitalWrite(MOT4_FWD,LOW);
          digitalWrite(MOT4_PWM,LOW);
        }
      }
      else {
        if (!digitalRead(ES4_BWD_LIM)) {
          digitalWrite(MOT4_BWD,HIGH);
          analogWrite(MOT4_PWM,vel);
        }
        else {
          digitalWrite(MOT4_BWD,LOW);
          digitalWrite(MOT4_PWM,LOW);
        }
      }
    }

}


