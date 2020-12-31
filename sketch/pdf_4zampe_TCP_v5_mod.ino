#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>
#include <Controllino.h>

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(10, 143, 132, 51);
EthernetServer server(502);

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

#define EMERG CONTROLLINO_IN0
#define NO_ANOM CONTROLLINO_IN1
#define LAMP_ANOM CONTROLLINO_R9
#define LAMP_LISTO CONTROLLINO_R8


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
void updateVAR();
void updateBaseVAR();

void checkES();

ModbusTCPServer modbusTCPServer;

bool st_1 = false;
bool st_2 = false;
bool st_3 = false;
bool st_4 = false;

bool loop_1 = false;
bool loop_2 = false;
bool loop_3 = false;
bool loop_4 = false;

void setup() {
  
  //INPUT
  pinMode(ES1_CEN_LIM,INPUT);
  pinMode(ES1_BWD_LIM,INPUT);
  pinMode(ES1_FWD_LIM,INPUT);
  
  pinMode(ES2_CEN_LIM,INPUT);
  pinMode(ES2_BWD_LIM,INPUT);
  pinMode(ES2_FWD_LIM,INPUT);
  
  pinMode(ES3_CEN_LIM,INPUT);
  pinMode(ES3_BWD_LIM,INPUT);
  pinMode(ES3_FWD_LIM,INPUT);
  
  pinMode(ES4_CEN_LIM,INPUT);
  pinMode(ES4_BWD_LIM,INPUT);
  pinMode(ES4_FWD_LIM,INPUT);
  
  pinMode(EMERG,INPUT);
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

  pinMode(LAMP_ANOM, OUTPUT);
  pinMode(LAMP_LISTO, OUTPUT);

  Ethernet.begin(mac, ip);
  delay(1000);
  
  server.begin();
  delay(1000);
  
  modbusTCPServer.begin();
  delay(1000);


  modbusTCPServer.configureCoils(0x00, 8);
  //0 1: mot1 fwd / bwd
  //2 3: mot2 fwd / bwd
  //4 5: mot3 fwd / bwd
  //6 7: mot4 fwd / bwd

  modbusTCPServer.configureDiscreteInputs(0x00, 24);
  //0 1 2: es1 fwd / cen / bwd
  //3 4 5: es1 fwd / cen / bwd
  //6 7 8: es1 fwd / cen / bwd
  //9 10 11: es1 fwd / cen / bwd
  //12 13: auto / man
  //14 15 16 17: man_mot1 / man_mot2 / man_mot3 / man_mot4 ready
  //18 19: mot choosen fwd / bwd
  //20: inverter on
  //21: no anomalies
  //22: lamp defaut
  //23: lamp listo

  modbusTCPServer.configureHoldingRegisters(0x00, 4);
  //0 1 2 3: PWM velocity value mot1 / mot2 / mot3 / mot4

  modbusTCPServer.configureInputRegisters(0x00, 1);

  st_1 = digitalRead(ES1_CEN_LIM);
  st_2 = digitalRead(ES2_CEN_LIM);
  st_3 = digitalRead(ES3_CEN_LIM);
  st_4 = digitalRead(ES4_CEN_LIM);

  loop_1 = false;
  loop_2 = false;
  loop_3 = false;
  loop_4 = false;
  
}

/*
 * modbusTCPServer.discreteInputRead
  modbusTCPServer.discreteInputWrite
  modbusTCPServer.coilRead
  modbusTCPServer.coilWrite
  modbusTCPServer.holdingRegisterRead
  modbusTCPServer.holdingRegisterWrite
  modbusTCPServer.inputRegisterRead
  modbusTCPServer.inputRegisterWrite
 */

uint8_t coils[8] = {0,0,0,0,0,0,0,0};
uint8_t vels[8] = {0,0,0,0,0,0,0,0};

uint8_t manVel = 0;
uint8_t state = 0;



uint8_t stp1 = 0;
uint8_t stp2 = 0;
uint8_t stp3 = 0;
uint8_t stp4 = 0;

bool prima = true;

void loop() {

  EthernetClient client = server.available();
  modbusTCPServer.accept(client);
  modbusTCPServer.poll();

  checkES();

  if (digitalRead(EMERG) or !digitalRead(NO_ANOM)) {
    
    state = 11;
    stopMot();
    digitalWrite(LAMP_ANOM, HIGH);
    digitalWrite(LAMP_LISTO, LOW);
    loop_1 = false;
    loop_2 = false;
    loop_3 = false;
    loop_4 = false;
    
  }
  
  else  {
    digitalWrite(LAMP_ANOM, LOW);
    digitalWrite(LAMP_LISTO, HIGH);
    
    // SW0_STATE
    if (digitalRead(SW0_AUTO)) {
      state = 1;
    }
    else if (digitalRead(SW0_MAN)) {
      state = 2;
      if (prima) {
        loop_1 = false;
        loop_2 = false;
        loop_3 = false;
        loop_4 = false;
        st_1 = digitalRead(ES1_CEN_LIM);
        st_2 = digitalRead(ES2_CEN_LIM);
        st_3 = digitalRead(ES3_CEN_LIM);
        st_4 = digitalRead(ES4_CEN_LIM);
        prima = false;
      }
      
    }
    else {
      state = 0;
      loop_1 = false;
      loop_2 = false;
      loop_3 = false;
      loop_4 = false;
      st_1 = digitalRead(ES1_CEN_LIM);
      st_2 = digitalRead(ES2_CEN_LIM);
      st_3 = digitalRead(ES3_CEN_LIM);
      st_4 = digitalRead(ES4_CEN_LIM);
    }

    //////////////////////////////////////////////////////////// MAIN
        
    if (state == 1) { //SW0_AUTO
      
      for (uint8_t i=0; i<4; i++) {
        vels[i] = modbusTCPServer.holdingRegisterRead(i);
        updatePWM(i,vels[i]);
      }
      
      for (uint8_t i=0; i<8; i++) {
        coils[i] = modbusTCPServer.coilRead(i);
        if ((i%2) && (coils[i]) && (coils[i-1])) {
          coils[i] = 0;
          modbusTCPServer.coilWrite(i,0);
        }
        updateDIR(i,coils[i]);
      }
      
    }

    
    ////////////////////////////////////////////////////////////////
  
    else if (state == 2) { //SW0_MAN
        
        
        manVel = 255;

        if (digitalRead(SW2_BWD)) {
          loop_1 = false;
          loop_2 = false;
          loop_3 = false;
          loop_4 = false;
          
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
          loop_1 = false;
          loop_2 = false;
          loop_3 = false;
          loop_4 = false;
          
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
          if (!loop_1 && !loop_2 && !loop_3 && !loop_4) {
            stopMot();
          }

          if (digitalRead(ES1_CEN_LIM) != st_1) {
            loop_1 = !loop_1;
            st_1 = !st_1;
          }
          if (digitalRead(ES2_CEN_LIM) != st_2) {
            loop_2 = !loop_2;
            st_2 = !st_2;
          }
          if (digitalRead(ES3_CEN_LIM) != st_3) {
            loop_3 = !loop_3;
            st_3 = !st_3;
          }
          if (digitalRead(ES4_CEN_LIM) != st_4) {
            loop_4 = !loop_4;
            st_4 = !st_4;
          }
          
          if (loop_1) {
            if (stp1==0) {
              actMot(1,true,100);
              if (!digitalRead(ES1_FWD_LIM)) {
                stp1=1;
              }
            }
            else {
              actMot(1,false,100);
              if (!digitalRead(ES1_BWD_LIM)) {
                stp1=0;
              }
            }
          }

          if (loop_2) {
            if (stp2==0) {
              actMot(2,true,100);
              if (!digitalRead(ES2_FWD_LIM)) {
                stp2=1;
              }
            }
            else {
              actMot(2,false,100);
              if (!digitalRead(ES2_BWD_LIM)) {
                stp2=0;
              }
            }
          }

          if (loop_3) {
            if (stp3==0) {
              actMot(3,true,100);
              if (!digitalRead(ES3_FWD_LIM)) {
                stp3=1;
              }
            }
            else {
              actMot(3,false,100);
              if (!digitalRead(ES3_BWD_LIM)) {
                stp3=0;
              }
            }
          }

          if (loop_4) {
            if (stp4==0) {
              actMot(4,true,100);
              if (!digitalRead(ES4_FWD_LIM)) {
                stp4=1;
              }
            }
            else {
              actMot(4,false,100);
              if (!digitalRead(ES4_BWD_LIM)) {
                stp4=0;
              }
            }
          }
          
        }
      }
      else { //SW0_0FF or ANOMALIES
        stopMot(); 
      }
    }
  updateBaseVAR();
  updateVAR();
}

void checkES() {

  if (!digitalRead(ES1_FWD_LIM)) {
    digitalWrite(MOT1_FWD,LOW);
    digitalWrite(MOT1_PWM,LOW);
  }
  if (!digitalRead(ES1_BWD_LIM)) {
    digitalWrite(MOT1_BWD,LOW);
    digitalWrite(MOT1_PWM,LOW);
  }

  if (!digitalRead(ES2_FWD_LIM)) {
    digitalWrite(MOT2_FWD,LOW);
    digitalWrite(MOT2_PWM,LOW);
  }
  if (!digitalRead(ES2_BWD_LIM)) {
    digitalWrite(MOT2_BWD,LOW);
    digitalWrite(MOT2_PWM,LOW);
  }

  if (!digitalRead(ES3_FWD_LIM)) {
    digitalWrite(MOT3_FWD,LOW);
    digitalWrite(MOT3_PWM,LOW);
  }
  if (!digitalRead(ES3_BWD_LIM)) {
    digitalWrite(MOT3_BWD,LOW);
    digitalWrite(MOT3_PWM,LOW);
  }

  if (!digitalRead(ES4_FWD_LIM)) {
    digitalWrite(MOT4_FWD,LOW);
    digitalWrite(MOT4_PWM,LOW);
  }
  if (!digitalRead(ES4_BWD_LIM)) {
    digitalWrite(MOT4_BWD,LOW);
    digitalWrite(MOT4_PWM,LOW);
  }
  
}

void updateBaseVAR() {
  modbusTCPServer.inputRegisterWrite(0, state);

  // BUTTON STATE
  modbusTCPServer.discreteInputWrite(12,digitalRead(SW0_AUTO));
  modbusTCPServer.discreteInputWrite(13,digitalRead(SW0_MAN));

  // SAFETY STATE
  modbusTCPServer.discreteInputWrite(20,digitalRead(EMERG));
  modbusTCPServer.discreteInputWrite(21,digitalRead(NO_ANOM));

  modbusTCPServer.discreteInputWrite(22,digitalRead(LAMP_ANOM));
  modbusTCPServer.discreteInputWrite(23,digitalRead(LAMP_LISTO));

}

void updateVAR() {
  
  
  // END_SWITCH STATE
  modbusTCPServer.discreteInputWrite(0,digitalRead(ES1_FWD_LIM));
  modbusTCPServer.discreteInputWrite(1,digitalRead(ES1_CEN_LIM));
  modbusTCPServer.discreteInputWrite(2,digitalRead(ES1_BWD_LIM));

  modbusTCPServer.discreteInputWrite(3,digitalRead(ES2_FWD_LIM));
  modbusTCPServer.discreteInputWrite(4,digitalRead(ES2_CEN_LIM));
  modbusTCPServer.discreteInputWrite(5,digitalRead(ES2_BWD_LIM));

  modbusTCPServer.discreteInputWrite(6,digitalRead(ES3_FWD_LIM));
  modbusTCPServer.discreteInputWrite(7,digitalRead(ES3_CEN_LIM));
  modbusTCPServer.discreteInputWrite(8,digitalRead(ES3_BWD_LIM));

  modbusTCPServer.discreteInputWrite(9,digitalRead(ES4_FWD_LIM));
  modbusTCPServer.discreteInputWrite(10,digitalRead(ES4_CEN_LIM));
  modbusTCPServer.discreteInputWrite(11,digitalRead(ES4_BWD_LIM));
  
  modbusTCPServer.discreteInputWrite(14,digitalRead(SW1_MOT1));
  modbusTCPServer.discreteInputWrite(15,digitalRead(SW1_MOT2));
  modbusTCPServer.discreteInputWrite(16,digitalRead(SW1_MOT3));
  if (!digitalRead(SW1_MOT1) && !digitalRead(SW1_MOT2) && !digitalRead(SW1_MOT3)) {
    modbusTCPServer.discreteInputWrite(17,1);
  }
  else {
    modbusTCPServer.discreteInputWrite(17,0);
  }
  
  modbusTCPServer.discreteInputWrite(18,digitalRead(SW2_FWD));
  modbusTCPServer.discreteInputWrite(19,digitalRead(SW2_BWD));
   
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
  switch (index) {
    case 0:
      digitalRead(ES1_FWD_LIM) ? digitalWrite(MOT1_FWD,value) : digitalWrite(MOT1_FWD,LOW); 
      break;
    case 1:
      digitalRead(ES1_BWD_LIM) ? digitalWrite(MOT1_BWD,value) : digitalWrite(MOT1_BWD,LOW);
      break;
    case 2:
      digitalRead(ES2_FWD_LIM) ? digitalWrite(MOT2_FWD,value) : digitalWrite(MOT2_FWD,LOW);
      break;
    case 3:
      digitalRead(ES2_BWD_LIM) ? digitalWrite(MOT2_BWD,value) : digitalWrite(MOT2_BWD,LOW);
      break;
    case 4:
      digitalRead(ES3_FWD_LIM) ? digitalWrite(MOT3_FWD,value) : digitalWrite(MOT3_FWD,LOW);
      break;
    case 5:
      digitalRead(ES3_BWD_LIM) ? digitalWrite(MOT3_BWD,value) : digitalWrite(MOT3_BWD,LOW);
      break;
    case 6:
      digitalRead(ES4_FWD_LIM) ? digitalWrite(MOT4_FWD,value) : digitalWrite(MOT4_FWD,LOW);
      break;
    case 7:
      digitalRead(ES4_BWD_LIM) ? digitalWrite(MOT4_BWD,value) : digitalWrite(MOT4_BWD,LOW);
      break;
  }
}

void stopMot() {

  digitalWrite(MOT1_FWD,LOW);
  digitalWrite(MOT1_BWD,LOW);
  modbusTCPServer.coilWrite(0,0);
  modbusTCPServer.coilWrite(1,0);

  digitalWrite(MOT2_FWD,LOW);
  digitalWrite(MOT2_BWD,LOW);
  modbusTCPServer.coilWrite(2,0);
  modbusTCPServer.coilWrite(3,0);

  digitalWrite(MOT3_FWD,LOW);
  digitalWrite(MOT3_BWD,LOW);
  modbusTCPServer.coilWrite(4,0);
  modbusTCPServer.coilWrite(5,0);

  digitalWrite(MOT4_FWD,LOW);
  digitalWrite(MOT4_BWD,LOW);
  modbusTCPServer.coilWrite(6,0);
  modbusTCPServer.coilWrite(7,0);

  digitalWrite(MOT1_PWM,LOW);
  modbusTCPServer.holdingRegisterWrite(0,0);
  digitalWrite(MOT2_PWM,LOW);
  modbusTCPServer.holdingRegisterWrite(1,0);
  digitalWrite(MOT3_PWM,LOW);
  modbusTCPServer.holdingRegisterWrite(2,0);
  digitalWrite(MOT4_PWM,LOW);
  modbusTCPServer.holdingRegisterWrite(3,0);
 
}

void actMot(uint8_t motNum, bool dir, uint8_t vel) {

    if (motNum == 1) {
      if (dir) {
        if (digitalRead(ES1_FWD_LIM)) {
          digitalWrite(MOT1_BWD,LOW);
          digitalWrite(MOT1_FWD,HIGH);
          analogWrite(MOT1_PWM,vel);
        }
        else {
          digitalWrite(MOT1_FWD,LOW);
          digitalWrite(MOT1_PWM,LOW);
        }
      }
      else {
        if (digitalRead(ES1_BWD_LIM)) {
          digitalWrite(MOT1_FWD,LOW);
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
        if (digitalRead(ES2_FWD_LIM)) {
          digitalWrite(MOT2_BWD,LOW);
          digitalWrite(MOT2_FWD,HIGH);
          analogWrite(MOT2_PWM,vel);
        }
        else {
          digitalWrite(MOT2_FWD,LOW);
          digitalWrite(MOT2_PWM,LOW);
        }
      }
      else {
        if (digitalRead(ES2_BWD_LIM)) {
          digitalWrite(MOT2_FWD,LOW);
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
        if (digitalRead(ES3_FWD_LIM)) {
          digitalWrite(MOT3_BWD,LOW);
          digitalWrite(MOT3_FWD,HIGH);
          analogWrite(MOT3_PWM,vel);
        }
        else {
          digitalWrite(MOT3_FWD,LOW);
          digitalWrite(MOT3_PWM,LOW);
        }
      }
      else {
        if (digitalRead(ES3_BWD_LIM)) {
          digitalWrite(MOT3_FWD,LOW);
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
        if (digitalRead(ES4_FWD_LIM)) {
          digitalWrite(MOT4_BWD,LOW);
          digitalWrite(MOT4_FWD,HIGH);
          analogWrite(MOT4_PWM,vel);
        }
        else {
          digitalWrite(MOT4_FWD,LOW);
          digitalWrite(MOT4_PWM,LOW);
        }
      }
      else {
        if (digitalRead(ES4_BWD_LIM)) {
          digitalWrite(MOT4_FWD,LOW);
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
