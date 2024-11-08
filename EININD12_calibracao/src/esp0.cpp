#include "IIKit.h"
#include <Arduino.h>

#define TRMNivel def_pin_R4a20_1
#define MOTBomba def_pin_RELE
#define CHNivel def_pin_D1
#define IdCanalValvula 0

bool tripValue = false;

float time_counter_sa = 0;

AsyncDelay_c delayPOT(50); // time in milliseconds
void monitoraPOT(void) {
    if (delayPOT.isExpired()) {
        delayPOT.repeat();
        const uint16_t vlPOT1 = analogRead(def_pin_POT1);
        ledcWrite(IdCanalValvula, vlPOT1);
        IIKit.disp.setText(2, ("P1:" + String(100.0 * vlPOT1 / 4095.0)).c_str());
        IIKit.WSerial.plot("vlPOT1", vlPOT1);
    }
}

AsyncDelay_c delay4A20(100); // time in milliseconds
uint8_t count = 0;
double vlR4a20_1 = 0.0;
int Josue = 100;
int pwm = 200; // Inicializa PWM como 0

//VARIAVEIS DE CONTROLE
float Kp = /* valor de Kp */7.82;
float Ki = /* valor de Ki */0.0074;
float Kd = /* valor de Kd */0;
float Ts = 0.0001; // Tempo de amostragem em segundos

float erro = 0;
float erro_anterior = 0;
float erro_integral = 0;
float saida_controlador = 0;
float setpoint = 12.5;
int aux = 0;

int controle_funcao(){
     // monitoraPOT();
    erro = setpoint - vlR4a20_1; // Calcule o erro
    erro_integral += erro * Ts;
    float erro_derivativo = (erro - erro_anterior) / Ts;

    // PID Discreto
    saida_controlador = Kp * erro + Ki * erro_integral + Kd * erro_derivativo;
    return saida_controlador;
}


void monitora4A20(void) {
    if (delay4A20.isExpired()) {
        delay4A20.repeat();
        vlR4a20_1 += (double)analogRead(def_pin_R4a20_1);

        if (++count >= 20) {
            vlR4a20_1 /= count; // média dos 20 valores
            vlR4a20_1 = (vlR4a20_1 + 309.17) / 220.10;
            // Atualize o erro anterior
            erro_anterior = erro;
            aux = controle_funcao();
            if(aux > 255){
                pwm = 255;
            }
            else if(aux < 100){
                aux = aux*100;
                if(aux > 255){
                    aux = 100;
                }
                pwm = aux; 
            }
            else{
                pwm = aux; 
            }
            //pwm = 200;
            // Lógica para calcular o PWM
            /*if (time_counter_sa < Josue) {
                pwm = 80;
            } else if (time_counter_sa >= Josue && time_counter_sa < 2 * Josue) {
                pwm = 200;
            } else if (time_counter_sa >= 2 * Josue && time_counter_sa < Josue * 3) {
                pwm = 150;
            } else if (time_counter_sa >= Josue * 3 && time_counter_sa < Josue * 4) {
                pwm = 100;
            } else {
                pwm = 0;
            
            }*/

            IIKit.disp.setText(3, ("T1:" + String(vlR4a20_1)).c_str());
            time_counter_sa = float(millis() / 1000.00);


            // Enviar dados via serial (Tempo, Pressão em %, PWM)

           IIKit.WSerial.print(time_counter_sa);
           IIKit.WSerial.print(",");
           IIKit.WSerial.print(String(vlR4a20_1));  // Pressão em %
           IIKit.WSerial.print(",");
           IIKit.WSerial.println(pwm);  // Valor PWM (fixo)
            ledcWrite(IdCanalValvula, pwm); // Atualiza o PWM
            count = 0;
            vlR4a20_1 = 0;
        }
    }
}

void IRAM_ATTR trip_func() {
    if (digitalRead(CHNivel) == LOW) { // Considerando LOW como sinal de trip
        digitalWrite(MOTBomba, LOW);
        tripValue = true;
    } else {
        tripValue = false;
    }
}

void setup() {
    IIKit.setup();
    Serial.begin(115200); // Inicializa a comunicação serial
    pinMode(CHNivel, INPUT_PULLDOWN);
    pinMode(def_pin_D4, OUTPUT);
    attachInterrupt(CHNivel, trip_func, CHANGE);

    ledcAttachPin(def_pin_W4a20_1, IdCanalValvula); // Atribui o pino ao canal de PWM.
    ledcSetup(IdCanalValvula, 19000, 8);           // Configura o canal de PWM.
    ledcWrite(IdCanalValvula, 0);                  // Inicializa o PWM com duty cycle de 0%.

    IIKit.rtn_1.onValueChanged([](uint8_t status) {
        if (!tripValue) {
            digitalWrite(MOTBomba, status);
            digitalWrite(def_pin_D4, status);
            IIKit.WSerial.println(status ? "MOTBomba ON" : "MOTBomba OFF");
        }
    });
}

void loop() {
    
    IIKit.loop();
    // Descomente se necessário

    // Envie o valor da saída para o atuador, ajustando conforme necessário
    monitora4A20();
}

