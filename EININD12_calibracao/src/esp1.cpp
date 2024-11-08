#include "IiKit.h"
#include <Arduino.h>

#define TRMNivel def_pin_R4a20_1
#define MOTBomba def_pin_RELE
#define CHNivel def_pin_D1
#define IdCanalValvula 0

bool tripValue = false;

AsyncDelay_c delayPOT(50); // time mili second
void monitoraPOT(void)
{
  if (delayPOT.isExpired())
  {
    delayPOT.repeat();

    const uint16_t vlPOT1 = analogRead(def_pin_POT1);
    ledcWrite(IdCanalValvula, vlPOT1);
    IIKit.disp.setText(2, ("P1:" + String(100.0 * vlPOT1 / 4096.0)).c_str());
    IIKit.WSerial.plot("vlPOT1", vlPOT1);
  }
}

// https://portal.vidadesilicio.com.br/controle-de-potencia-via-pwm-esp32/#:~:text=Esta%20função%20configura%20um%20canal,Resolução%3A%201%20–%2016%20bits.
AsyncDelay_c delay4A20(100); // time mili second
uint8_t count = 0;
double vlR4a20_1 = 0.0;
int counter = 0;
int pwm = 0;
void monitora4A20(void){
  if (delay4A20.isExpired())
  {
    delay4A20.repeat();
    counter++;
    vlR4a20_1 += (double)analogRead(def_pin_R4a20_1);
    if (++count >= 20)
    {
      vlR4a20_1 = vlR4a20_1 / count;
      //vlR4a20_1 = (vlR4a20_1 + 309.17) / 220.10 ;
      IIKit.disp.setText(3, ("4a20B:" + String(vlR4a20_1/204.8)).c_str());
      IIKit.WSerial.plot("vlR4a20_1", vlR4a20_1);
      ledcWrite(IdCanalValvula, pwm);
      count = 0;
      vlR4a20_1 = 0;
    }
  }
}

void IRAM_ATTR trip_func(){
  if (digitalRead(CHNivel) == false)
  {
    digitalWrite(MOTBomba, LOW);
    tripValue = true;
  }
  else
    tripValue = false;
}

void setup(){
  IIKit.setup();

  pinMode(CHNivel, INPUT_PULLDOWN);
  pinMode(def_pin_D4, OUTPUT);
  attachInterrupt(CHNivel, trip_func, CHANGE);

  ledcAttachPin(def_pin_W4a20_1, IdCanalValvula); // Atribuimos o pino def_pin_W4a20_1 ao canal POSValvula.
  ledcSetup(IdCanalValvula, 19000, 8);           // Atribuimos ao canal 0 a frequencia de 78kHz com resolucao de 10bits.
  ledcWrite(IdCanalValvula, 0);                   // Escrevemos um duty cycle de 0% no canal 0.

  IIKit.rtn_1.onValueChanged([](uint8_t status)
                             {
      if(!tripValue) {
        digitalWrite(MOTBomba,status);
        digitalWrite(def_pin_D4,status);
        IIKit.WSerial.println(status? "MOTBomba ON" :"MOTBomba OFF"); 
      } });
}

void loop(){
  IIKit.loop();
  //monitoraPOT();
  monitora4A20();
}