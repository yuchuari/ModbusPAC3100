/*
 * DESARROLLO DE SISTEMA DE COMUNIICACIÓN ENTRE UN MICROCONTROLADOR Y UN PAC3100
 * Autor: Yandri Uchuari Gallardo
 * Fecha: 12/08/2023
 * Versión: 01
 * Lugar: BZA
 * 
 * Este programa fue elaboraado por Yandri Uchuari Gallardo, quien trabaja como Mecatrónico de Campo en el Grupo Minero Bonanza
 */

#include <ModbusMaster.h>

#define MAX485_DE      24
#define MAX485_RE_NEG  22

ModbusMaster node;

uint8_t resultMain;

int endereco_Rede_Slaves[] {
  1,
  2,
  
  3,
  4,
  5,
  6,
  
};

int vetor_Endereco_Regs[] {
  0x01,   //Tensión F1-N DEC(01)
  0x03,   //Tensión F2-N DEC(03)
  0x05,   //Tensión F3-N DEC(05)
  0x0D,   //Corriente L1 DEC(13)
  0x0F,   //Corriente L2 DEC(15))
  0x11,   //Corriente L3 DEC(17)
  0x27,   //Frequencia DEC(39)
};

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8O1);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}


void loop() {
  for ( int i = 0; i < 1; i ++) {
    read_Multimedidor(endereco_Rede_Slaves[i]);
    delay(5000);
  }
}
/*************************************************************
  Realiza lectura de datos via RS485 al PAC3100
*************************************************************/
float read_Multimedidor(uint8_t deviceID) {

  uint8_t j;
  uint16_t _buffer[2], memAux;
  float  dataOut;

  node.begin(deviceID, Serial2);

  Serial.println("************INICIANDO COMUNICACIÓN*************");
  Serial.println("--------------------------------------------");
  Serial.print("Slave to Communication: ");
  Serial.println(deviceID);
  Serial.println("--------------------------------------------");

  for (int i = 0; i < 8; i ++) {
    resultMain = node.readHoldingRegisters(vetor_Endereco_Regs[i], 2);

    delay(1500);

    if (resultMain == node.ku8MBSuccess) {
      Serial.println("Sucesso! Processando ...");

      for (j = 0; j < 8; j++ ) {
        _buffer[j] = node.getResponseBuffer(j);
      }

      memAux = _buffer[1];
      _buffer[1] = _buffer[0];
      _buffer[0] = memAux;
      
      memcpy(&dataOut, &_buffer, sizeof(float));

      switch (i) {
        case 0:
          Serial.println("Tensión de fase a Neutro - L1-N");
          Serial.print(dataOut, 2);   // 2 Decimales
          Serial.println(" Volts");
          Serial.println("+++++++++++++++++++++++++++++++++++++++++++++");
          Serial.println(" ");
          break;
        case 1:
          Serial.println("Tensión de fase a Neutro - L2-N");
          Serial.print(dataOut, 2);   
          Serial.println(" Volts");
          Serial.println("+++++++++++++++++++++++++++++++++++++++++++++");
          Serial.println(" ");
          break;
        case 2:
          Serial.println("Tensión de fase a Neutro - L3-N");
          Serial.print(dataOut, 2);   
          Serial.println(" Volts");
          Serial.println("+++++++++++++++++++++++++++++++++++++++++++++");
          Serial.println(" ");
          break;
        case 3:
          Serial.println("Corriente L1");
          Serial.print(dataOut, 2);  
          Serial.println(" Ampere");
          Serial.println("+++++++++++++++++++++++++++++++++++++++++++++");
          Serial.println(" ");
          break;
        case 4:
          Serial.println("Corriente L2");
          Serial.print(dataOut, 2);   
          Serial.println(" Ampere");
          Serial.println("+++++++++++++++++++++++++++++++++++++++++++++");
          Serial.println(" ");
          break;
        case 5:
          Serial.println("Corriente L3");
          Serial.print(dataOut, 2);   
          Serial.println(" Ampere");
          Serial.println("+++++++++++++++++++++++++++++++++++++++++++++");
          Serial.println(" ");
          break;
        case 6:
          Serial.println("Frequencia - HZ");
          Serial.print(dataOut, 2);   
          Serial.println(" Hertz");
          Serial.println("+++++++++++++++++++++++++++++++++++++++++++++");
          Serial.println(" ");
          break;
      }
    }
  }
}
