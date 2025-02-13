#include <Arduino.h>

/* Código de teste da funcionalidade de ADC utilizando arduino (framework)
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
* Funcionamento: Esse código obtem o valor ADC da saída de um potenciômetro, exebindo
*valor ADC e a tensão correspondente no monitor serial.
*/

#include "platform.h"

// Defines
#define PIN_ANALOG_IN 4   // pino de entrada analógica (pino 3 do potenciômetro) - ADC2-CH0
#define ADC_BITS 12       // bits de precisão do ADC da esp32
#define MCU_VOLTAGE 3.3   // tensão de trabalho da esp32 (alguns arduinos possuem tensão de 5V)

// Variáveis globais  
adc_t adc;

void setup() {
  // Configuraçãom do monitor serial
  Serial.begin(115200); // taxa da transmissão do monitor serial

  // Setagem de atributos do ADC
  adc.bits = ADC_BITS; 
  adc.voltage_reference = MCU_VOLTAGE;
  adc.handle = PIN_ANALOG_IN;
}

void loop() {

  result_uint16_t adcResult;  // resultado da conversão adc
  float voltage;              // voltagem correspondente ao nível ADC lido

  // Conversão ADC (entrada)
  /* Leitura do nível digital do pino de entrada de valor analógico de tensão
  *  ADC2 é desativado quando utilizamos o wifi da ESP32
  */
  adcResult = adc_read(&adc);

  // Tratamento de possíveis erros de leitura
  if(adcResult.hasError){ 
    Serial.println("Erro de leitura do ADC");
  }

  // Obtenção da voltagem do potenciômetro
  /* tendo que adcValue = analogicVoltage / (MCU_VOLTAGE / (2^ADC_BITS - 1)) */
  voltage = adc_raw_to_voltage(&adc, adcResult.value);
                          
  // Exposição dos valores no monitor serial
  Serial.printf("ADC = %d V, Voltage = %.2f V\n", adcResult.value, voltage);

  // Delay de medida
  delay(500);
}

