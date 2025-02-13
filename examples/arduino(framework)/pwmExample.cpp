#include <Arduino.h>

/* Código de teste da funcionalidade de Pwm utilizando arduino (framework)
* Autor: Vitor Alexandre
* Especificações:
* -Código utilizável tanto na platform io, incluindo os seguintes pacotes em platform.ini
*
*     platform_packages = 
*      framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.0.2
*      framework-arduinoespressif32-libs @ https://github.com/espressif/arduino-esp32/releases/download/3.0.2/esp32-arduino-libs-3.0.2.zip
* 
* -A partir desse comentário, o código pode ser utilizado em um arquivo .ino, sem a
*necessidade de inclusão de pacotes.
*
* Funcionamento: Esse código aumenta linearmente a intensidade da luz de um led conectado a PIN_LED
*usando a modulação de um sinal analógico através de PWM.
*/

#include "platform.h"

#define PIN_LED 2
#define CHN 0 // canal PWM
#define FRQ 1000 // frequência definida no PWM
#define PWM_BIT 8 //precisão em bits 2^8 -> 0% a 100% <-> 0 a 255

pwm_t Pwm;

void setup() {
  // Variável de erro
  error_t error;

  // Iniciação do monitor serial com taxa de 115200
  Serial.begin(115200);

  // Setagem de atributos do PWM
  Pwm.bits = PWM_BIT;
  Pwm.channel = CHN;
  Pwm.handle.frequency = FRQ;
  Pwm.handle.target = PIN_LED;

  // Configuração do pwm
  error = pwm_start(Pwm);

  // Tratamento de erro de configuração
  if(error){ 
    Serial.println("=====> Setup with errors");
    return;
  }
  
  // Sem erros de configuração
  Serial.println("=====> Setup without errors");
}

void loop() {

  // Aumento linear da luminosidade
  for(int i=0; i<255; i++){
    pwm_write(Pwm, i);  // modulação mde sinal com (100*i)/255 % da voltagem máxima
    delay(5);           // delay necessário entre as modulações de sinal
  }

  // Diminuição linear da luminosidade
  for(int i=0255; i>=0; i--){
    pwm_write(Pwm, i);
    delay(5);
  }
}