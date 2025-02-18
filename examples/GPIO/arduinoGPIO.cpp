#include "Arduino.h"

/* Código de teste da funcionalidade de GPIO utilizando arduino (framework)
* Autor: Vitor Alexandre
* Especificações:
* -Código utilizável tanto na Arduino IDE quanto na PlatformIO, incluindo os seguintes 
*pacotes em platform.ini:
*
*     platform_packages = 
*      framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.0.2  
*      framework-arduinoespressif32-libs @ https://github.com/espressif/arduino-esp32/releases/download/3.0.2/esp32-arduino-libs-3.0.2.zip
* 
* -A partir desse comentário, o código pode ser utilizado em um arquivo .ino, sem a
*necessidade de inclusão de pacotes.
*
* Funcionamento: esse código permite o desligamento de um led quando um botão, associado
*a um dado pino de entrada, é presionado
*/

#include "platform.h"

#define PIN_LED 2
#define PIN_BUTTON 13

gpio_pin_t Led;
gpio_pin_t Button;

// Atribuição das GPIOs com os pinos associados ao led e ao botão
void setup() {
  Led.pin = PIN_LED;
  Button.pin = PIN_BUTTON;
}

// Funcionamento do programa
void loop() {
  // Caso o pino ligado ao botão esteja em nível de tensão 0, o LED desliga
  if (gpio_read(Button) == gpio_low_level) {
    gpio_low(Led);
  }

  // Caso o contrário, o LED permanece ligado
  else{
    gpio_high(Led);
  }
}