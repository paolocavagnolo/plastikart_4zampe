/*
  Modbus RTU Client Kitchen Sink

  This sketch creates a Modbus RTU Client and demostrates
  how to use various Modbus Client APIs.

  Circuit:
   - MKR board
   - MKR 485 shield
     - ISO GND connected to GND of the Modbus RTU server
     - Y connected to A/Y of the Modbus RTU server
     - Z connected to B/Z of the Modbus RTU server
     - Jumper positions
       - FULL set to OFF
       - Z \/\/ Y set to ON

  created 18 July 2018
  by Sandeep Mistry
*/

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <SoftwareSerial.h>

int counter = 0;

bool arancione();
bool viola();
bool rosso();
void rset();
void stp(uint8_t mot);
void fwd(uint8_t mot, uint8_t vel);
void bwd(uint8_t mot, uint8_t vel);

SoftwareSerial ss(6,7); //RX, TX

uint8_t state[] = {0,0,0,0};
uint8_t old_state[] = {0,0,0,0};
bool on_change[] = {0,0,0,0};

void setup() {
  
  ModbusRTUClient.begin(19200);
  pinMode(10,INPUT_PULLUP);
  pinMode(11,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);

  ss.begin(9600);

  state[0] = digitalRead(10);
  state[1] = digitalRead(11);
  state[2] = digitalRead(12);
  state[3] = digitalRead(13);
}

uint8_t sA = 0;
uint8_t sV = 0;
uint8_t sR = 0;
uint8_t sN = 0;

bool iA = 0;
bool iV = 0;
bool iR = 0;
bool iN = 0;

bool onExitA = 0;
bool onExitV = 0;
bool onExitR = 0;
bool onExitN = 0;

void loop() {

  if (ModbusRTUClient.inputRegisterRead(43,0) != 1) {
    state[0] = digitalRead(10);
    state[1] = digitalRead(11);
    state[2] = digitalRead(12);
    state[3] = digitalRead(13);
  }
  else {
    old_state[0] = state[0];
    old_state[1] = state[1];
    old_state[2] = state[2];
    old_state[3] = state[3];
    
    state[0] = digitalRead(10);
    state[1] = digitalRead(11);
    state[2] = digitalRead(12);
    state[3] = digitalRead(13);
    
    if (state[0] != old_state[0]) {
      on_change[0] = 1;
    }
    if (state[1] != old_state[1]) {
      on_change[1] = 1;
    }
    if (state[2] != old_state[2]) {
      on_change[2] = 1;
    }
    if (state[3] != old_state[3]) {
      on_change[3] = 1;
    }
  
    if (on_change[0]) {
      iA = !iA;
      on_change[0] = 0;
    }
    
    if (on_change[1]) {
      iV = !iV;
      on_change[1] = 0;
    }
    
    if (on_change[2]) {
      iR = !iR;
      on_change[2] = 0;
    }
  
    if (on_change[3]) {
      iN = !iN;
      on_change[3] = 0;
    }
  
    if (iA) {
      arancione();
      onExitA = 1;
    }
    else {
      if (onExitA) {
        onExitA = 0;
        rset();
      }
      sA = 0;
    }
    
    if (iV) {
      viola();
      onExitV = 1;
    }
    else {
      if (onExitV) {
        onExitV = 0;
        stp(0);
        stp(1);
      }
      sV = 0;
    }
  
    if (iR) {
      rosso();
      onExitR = 1;
    }
    else {
      if (onExitR) {
        onExitR = 0;
        stp(2);
        stp(3);
      }
      sR = 0;
    }
  
    if (iN) {
      nero();
      onExitN = 1;
    }
    else {
      if (onExitN) {
        onExitN = 0;
        rset();
      }
      sN = 0;
    }
  }

}


bool ssA1 = 0;
bool ssA2 = 0;
bool ssA3 = 0;
bool ssA4 = 0;

bool ssB1 = 0;
bool ssB2 = 0;
bool ssB3 = 0;
bool ssB4 = 0;

bool arancione() {
  switch(sA) {
    case 0:
      fwd(0,100);
      fwd(1,100);
      fwd(2,100);
      fwd(3,100);
      sA++;
      break;
      
    case 1:
      if (!ssA1) {
        if ((ModbusRTUClient.discreteInputRead(43,0) || ModbusRTUClient.discreteInputRead(43,1))) {
          ssA1 = 1;
          stp(0);
          if (ModbusRTUClient.discreteInputRead(43,1)) {
            ssB1 = 1;
          }
        }
      }
      if (!ssA2) {
        if ((ModbusRTUClient.discreteInputRead(43,3) || ModbusRTUClient.discreteInputRead(43,4))) {
          ssA2 = 1;
          stp(1);
          if (ModbusRTUClient.discreteInputRead(43,4)) {
            ssB2 = 1;
          }
        }
      }
      if (!ssA3) {
        if ((ModbusRTUClient.discreteInputRead(43,6) || ModbusRTUClient.discreteInputRead(43,7))) {
          ssA3 = 1;
          stp(2);
          if (ModbusRTUClient.discreteInputRead(43,7)) {
            ssB3 = 1;
          }
        }
      }
      if (!ssA4) {
        if ((ModbusRTUClient.discreteInputRead(43,9) || ModbusRTUClient.discreteInputRead(43,10))) {
          ssA4 = 1;
          stp(3);
          if (ModbusRTUClient.discreteInputRead(43,10)) {
            ssB4 = 1;
          }
        }
      }

      if ((ssA1) && (ssA2) && (ssA3) && (ssA4)) {
        sA++;
        ssA1 = 0;
        ssA2 = 0;
        ssA3 = 0;
        ssA4 = 0;
      }
      break;
      
    case 2:
      if (!ssB1) {
        bwd(0,100);
      }
      if (!ssB2) {
        bwd(1,100);
      }
      if (!ssB3) {
        bwd(2,100);
      }
      if (!ssB4) {
        bwd(3,100);
      }
      sA++;
      break;

    case 3:
      if (!ssB1) {
        if (ModbusRTUClient.discreteInputRead(43,1)) {
          stp(0);
          ssB1 = 1;
        }
      }
      if (!ssB2) {
        if (ModbusRTUClient.discreteInputRead(43,4)) {
          stp(1);
          ssB2 = 1;
        }
      }
      if (!ssB3) {
        if (ModbusRTUClient.discreteInputRead(43,7)) {
          stp(2);
          ssB3 = 1;
        }
      }
      if (!ssB4) {
        if (ModbusRTUClient.discreteInputRead(43,10)) {
          stp(3);
          ssB4 = 1;
        }
      }
      if ((ssB1) && (ssB2) && (ssB3) && (ssB4)) {
        iA = 0;
        sA = 0;
        return 0;
      }
      break;
  }
}



bool viola() {
  switch (sV) {
    case 0:
      fwd(0,255);
      bwd(1,255);
      sV++;
      break;
    case 1:
      if (ModbusRTUClient.discreteInputRead(43,0) && ModbusRTUClient.discreteInputRead(43,5)) {
        sV++;
        stp(0);
        stp(1);
      }
      break;
    case 2:
      bwd(0,50);
      fwd(1,50);
      sV++;
      break;
    case 3:
      if (ModbusRTUClient.discreteInputRead(43,1) && ModbusRTUClient.discreteInputRead(43,4)) {
        sV++;
        stp(0);
        stp(1);
      }
      break;
    case 4:
      bwd(0,255);
      fwd(1,255);
      sV++;
      break;
    case 5:
      if (ModbusRTUClient.discreteInputRead(43,2) && ModbusRTUClient.discreteInputRead(43,3)) {
        sV++;
        stp(0);
        stp(1);
      }
      break;
    case 6:
      fwd(0,50);
      bwd(1,50);
      sV++;
      break;
    case 7:
      if (ModbusRTUClient.discreteInputRead(43,1) && ModbusRTUClient.discreteInputRead(43,4)) {
        sV = 0;
        //iV = 0;
        stp(0);
        stp(1);
        return 0;
      }
      break;
  }
  return 0;
}

bool rosso() {
  switch (sR) {
    case 0:
      fwd(2,255);
      bwd(3,255);
      sR++;
      break;
    case 1:
      if (ModbusRTUClient.discreteInputRead(43,6) && ModbusRTUClient.discreteInputRead(43,11)) {
        sR++;
        stp(2);
        stp(3);
      }
      break;
    case 2:
      bwd(2,50);
      fwd(3,50);
      sR++;
      break;
    case 3:
      if (ModbusRTUClient.discreteInputRead(43,7) && ModbusRTUClient.discreteInputRead(43,10)) {
        sR++;
        stp(2);
        stp(3);
      }
      break;
    case 4:
      bwd(2,255);
      fwd(3,255);
      sR++;
      break;
    case 5:
      if (ModbusRTUClient.discreteInputRead(43,8) && ModbusRTUClient.discreteInputRead(43,9)) {
        sR++;
        stp(2);
        stp(3);
      }
      break;
    case 6:
      fwd(2,50);
      bwd(3,50);
      sR++;
      break;
    case 7:
      if (ModbusRTUClient.discreteInputRead(43,7) && ModbusRTUClient.discreteInputRead(43,10)) {
        sR = 0;
        //iR = 0;
        stp(2);
        stp(3);
        return 0;
      }
      break;
  }
  return 0;
}

bool ssN1 = 0;
bool ssN2 = 0;
bool ssN3 = 0;
bool ssN4 = 0;

bool nero() {
  switch(sN) {
    case 0:
      fwd(0,50);
      fwd(1,50);
      fwd(2,50);
      fwd(3,50);
      sN++;
      break;
      
    case 1:
      if (!ssN1) {
        if (ModbusRTUClient.discreteInputRead(43,0)) {
          ssN1 = 1;
          stp(0);
        }
      }
      if (!ssN2) {
        if (ModbusRTUClient.discreteInputRead(43,3)) {
          ssN2 = 1;
          stp(1);
        }
      }
      if (!ssN3) {
        if (ModbusRTUClient.discreteInputRead(43,6)) {
          ssN3 = 1;
          stp(2);
        }
      }
      if (!ssN4) {
        if (ModbusRTUClient.discreteInputRead(43,9)) {
          ssN4 = 1;
          stp(3);
        }
      }

      if ((ssN1) && (ssN2) && (ssN3) && (ssN4)) {
        sN++;
        ssN1 = 0;
        ssN2 = 0;
        ssN3 = 0;
        ssN4 = 0;
      }
      break;
      
    case 2:
      bwd(0,100);
      bwd(1,100);
      bwd(2,100);
      bwd(3,100);
      sN++;
      break;
      
    case 3:
      if (!ssN1) {
        if (ModbusRTUClient.discreteInputRead(43,2)) {
          ssN1 = 1;
          stp(0);
        }
      }
      if (!ssN2) {
        if (ModbusRTUClient.discreteInputRead(43,5)) {
          ssN2 = 1;
          stp(1);
        }
      }
      if (!ssN3) {
        if (ModbusRTUClient.discreteInputRead(43,8)) {
          ssN3 = 1;
          stp(2);
        }
      }
      if (!ssN4) {
        if (ModbusRTUClient.discreteInputRead(43,11)) {
          ssN4 = 1;
          stp(3);
        }
      }

      if ((ssN1) && (ssN2) && (ssN3) && (ssN4)) {
        sN++;
        ssN1 = 0;
        ssN2 = 0;
        ssN3 = 0;
        ssN4 = 0;
      }
      break;  
      
    case 4:
      fwd(0,150);
      fwd(1,150);
      fwd(2,150);
      fwd(3,150);
      sN++;
      break;
      
    case 5:
      if (!ssN1) {
        if (ModbusRTUClient.discreteInputRead(43,0)) {
          ssN1 = 1;
          stp(0);
        }
      }
      if (!ssN2) {
        if (ModbusRTUClient.discreteInputRead(43,3)) {
          ssN2 = 1;
          stp(1);
        }
      }
      if (!ssN3) {
        if (ModbusRTUClient.discreteInputRead(43,6)) {
          ssN3 = 1;
          stp(2);
        }
      }
      if (!ssN4) {
        if (ModbusRTUClient.discreteInputRead(43,9)) {
          ssN4 = 1;
          stp(3);
        }
      }

      if ((ssN1) && (ssN2) && (ssN3) && (ssN4)) {
        sN++;
        ssN1 = 0;
        ssN2 = 0;
        ssN3 = 0;
        ssN4 = 0;
      }
      break;
      
    case 6:
      bwd(0,200);
      bwd(1,200);
      bwd(2,200);
      bwd(3,200);
      sN++;
      break;
      
    case 7:
      if (!ssN1) {
        if (ModbusRTUClient.discreteInputRead(43,2)) {
          ssN1 = 1;
          stp(0);
        }
      }
      if (!ssN2) {
        if (ModbusRTUClient.discreteInputRead(43,5)) {
          ssN2 = 1;
          stp(1);
        }
      }
      if (!ssN3) {
        if (ModbusRTUClient.discreteInputRead(43,8)) {
          ssN3 = 1;
          stp(2);
        }
      }
      if (!ssN4) {
        if (ModbusRTUClient.discreteInputRead(43,11)) {
          ssN4 = 1;
          stp(3);
        }
      }

      if ((ssN1) && (ssN2) && (ssN3) && (ssN4)) {
        sN=0;
        ssN1 = 0;
        ssN2 = 0;
        ssN3 = 0;
        ssN4 = 0;
        iN = 0;
      }
      break; 
    
  }
}


void fwd(uint8_t mot, uint8_t vel) {
  ModbusRTUClient.holdingRegisterWrite(43,mot,vel);
  ModbusRTUClient.coilWrite(43,mot*2,1);
}
void bwd(uint8_t mot,  uint8_t vel) {
  ModbusRTUClient.holdingRegisterWrite(43,mot,vel);
  ModbusRTUClient.coilWrite(43,mot*2+1,1);
}
void stp(uint8_t mot) {
  ModbusRTUClient.holdingRegisterWrite(43,mot,0);
  ModbusRTUClient.coilWrite(43,mot*2,0);
  ModbusRTUClient.coilWrite(43,mot*2+1,0);  
}
void rset() {
  ModbusRTUClient.holdingRegisterWrite(43,0,0);
  ModbusRTUClient.holdingRegisterWrite(43,1,0);
  ModbusRTUClient.holdingRegisterWrite(43,2,0);
  ModbusRTUClient.holdingRegisterWrite(43,3,0);
  ModbusRTUClient.coilWrite(43,0,0);
  ModbusRTUClient.coilWrite(43,1,0);
  ModbusRTUClient.coilWrite(43,2,0);
  ModbusRTUClient.coilWrite(43,3,0);
  ModbusRTUClient.coilWrite(43,4,0);
  ModbusRTUClient.coilWrite(43,5,0);
  ModbusRTUClient.coilWrite(43,6,0);
  ModbusRTUClient.coilWrite(43,7,0);
}

