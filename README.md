<h1 align="center" style="color:white; background-color:black">Finish Platform</h1>
<h4 align="center">The project was created to refactor and finish the lib platform, which was begun for the old members of Zenith to enable some drivers (utilizing the lib) for embedded systems with different microcontrollers.</h4>

<p align="center">
	<a href="http://zenith.eesc.usp.br/">
    <img src="https://img.shields.io/badge/Zenith-Embarcados-black?style=for-the-badge"/>
    </a>
    <a href="https://eesc.usp.br/">
    <img src="https://img.shields.io/badge/Linked%20to-EESC--USP-black?style=for-the-badge"/>
    </a>
    <a href="https://github.com/${{ env.REPOSITORY_FULL_NAME }}/blob/main/LICENSE">
    <img src="https://img.shields.io/github/license/${{ env.REPOSITORY_FULL_NAME }}?style=for-the-badge"/>
    </a>
    <a href="https://github.com/${{ env.REPOSITORY_FULL_NAME }}/issues">
    <img src="https://img.shields.io/github/issues/${{ env.REPOSITORY_FULL_NAME }}?style=for-the-badge"/>
    </a>
    <a href="https://github.com/${{ env.REPOSITORY_FULL_NAME }}/commits/main">
    <img src="https://img.shields.io/github/commit-activity/m/${{ env.REPOSITORY_FULL_NAME }}?style=for-the-badge">
    </a>
    <a href="https://github.com/${{ env.REPOSITORY_FULL_NAME }}/graphs/contributors">
    <img src="https://img.shields.io/github/contributors/${{ env.REPOSITORY_FULL_NAME }}?style=for-the-badge"/>
    </a>
    <a href="https://github.com/${{ env.REPOSITORY_FULL_NAME }}/commits/main">
    <img src="https://img.shields.io/github/last-commit/${{ env.REPOSITORY_FULL_NAME }}?style=for-the-badge"/>
    </a>
    <a href="https://github.com/${{ env.REPOSITORY_FULL_NAME }}/issues">
    <img src="https://img.shields.io/github/issues-raw/${{ env.REPOSITORY_FULL_NAME }}?style=for-the-badge" />
    </a>
    <a href="https://github.com/${{ env.REPOSITORY_FULL_NAME }}/pulls">
    <img src = "https://img.shields.io/github/issues-pr-raw/${{ env.REPOSITORY_FULL_NAME }}?style=for-the-badge">
    </a>
</p>

#  Platform.h
## Objective

Platform headers is an abstraction layer to help application drivers to be
platform agnostic. That means that including the header will provide a
common interface between our embedded platforms such as STM32 (HAL) or 
Arduino. 

## Scope

Each platform implementation provides access to common (internal) peripherals,
such as I2C, SPI, UART and ADCs. As it's expected to be implemented in various 
circumstances, the scope is limited to transmitting and receiving raw bytes.

## Usage

Just add the `platform/` folder to the project, then
 `#include "platform.h"` in the driver. It then checks for 
preprocessor macros to detect which platform is being compiled.
For STM32, it will also attempt to find the proper chip family HAL. 
If a platform is not found it is assumed to be running on a PC.
This means calls will be logged to `stdout` with no side-effects.

## Notes
If you're using an arduino (or esp-idf) implementation, put the `#include "platform/platform.h" after
includes to other arduino (or esp-idf) libraries. Check issue #43.

## Specification

<p align="center">
    <a href="###Common">Common</a> •
    <a href="###GPIO">GPIO</a> •
    <a href="###I2C">I2C</a> •
    <a href="####SPI">SPI</a> •
    <a href="###UART">UART</a> •
    <a href="###ADC">ADC</a> •
</p>

### Common

#### Types
 - `error_t`
 ```c
  typedef int error_t;
 ```
 This is a common error code type, it is recommended to be an underlying `int` but can also be a platform specific. The requirement is that 
 evaluation of no error is false. So essentially, that the following pattern
 still applies:
 ```c
  if(error){
    // error handling...
  }
  // ... code assuming no errors
 ``` 
This pattern was chosen to prevent [Indent Hadouken](https://i.pinimg.com/originals/98/bd/38/98bd38321fad0e8b14ebf3a5a9ec70c9.jpg)
 
 - `buffer_view_t`
```c
typedef struct {
	uint8_t *data;
	int size;
} buffer_view_t;
```

It's used to pass pointers to `data` buffers of `size`. It is used throughout 
this header to pass a bunch of bytes. This was chosen to keep things simple 
and prevent mixing up sizes and arrays.
 
 - `result{8,16,float}_t`
```c
typedef struct {
	error_t hasError;
	uint8_t value;
} result8_t;

```

The `results` are a complement to the error handling strategy. Because 
there are no parameters in C, the platform header only creates result types
for common types: `uint8_t`, `uint16_t` and `float`. Other result types
should be declared on a driver-by-driver basis.


#### Functions
 - `delay_ms`
 ```c
  void delay_ms(uint32_t time);
 ```
 A blocking millisecond delay. It is recommended that RTOS use be 
 detected via preprocessor macros, so that drivers can take advantage.

### GPIO

#### Types

- `gpio_pin_t`
 ```c
typedef struct {
	<PORT> *port;
	uint16_t pin;
} gpio_pin_t;
 ```
 GPIO Pin Type

 #### Functions
 - `gpio_low` 
 
 ```c
 void gpio_low(gpio_pin_t pin);
 ```
 Reset GPIO

 - `gpio_high` 
 
 ```c
 void gpio_high(gpio_pin_t pin);
 ```
 Set GPIO

 - `gpio_toggle` 
 
 ```c
 void gpio_toggle(gpio_pin_t pin);
 ```
 Toggle GPIO

### I2C

#### Types

- `i2c_t`
 ```c
  typedef <I2C_Type> i2c_dev_t;
 ```

 I2C Interface Type

- `i2c_dev_t`
 ```c
typedef struct {
	i2c_t *i2c;
	uint8_t address;
} i2c_device_t;
 ```

 I2C Device Type
 
 #### Functions

 - `i2c_transmit` 
 
 ```c
 error_t i2c_transmit(i2c_device_t device, buffer_view_t buffer);
 ```
 Transmit bytes via I2C


 - `i2c_receive` 
 
 ```c
 error_t i2c_receive(i2c_device_t device, buffer_view_t buffer);
 ```
 Receive bytes via I2C
### SPI

#### Types
- `spi_t`
 ```c
  typedef <SPI_Type> spi_t;
 ```
 SPI Interface Type

- `spi_dev_t`
 ```c
typedef struct {
	spi_t *spi;
	gpio_pin_t pin;
} spi_device_t;
 ```
 SPI Device Type
 
#### Functions

 - `spi_transceive`
```c
error_t spi_transceive(spi_device_t device, buffer_view_t rx_buffer, 
  buffer_view_t tx_buffer);
```

Transceive (Transmit and Receive) via SPI

 - `spi_transmit`
```c
error_t spi_transmit(spi_device_t device, buffer_view_t tx_buffer);
```

Transmit via SPI

 - `spi_transceive`
```c
error_t spi_receive(spi_device_t device, buffer_view_t rx_buffer);
```

Receive via SPI
### UART

#### Types

 - `uart_t`
 ```c
  typedef <UART_Type> uart_t;
 ```
 UART Interface Type

- `uart_connection_t`
```c
typedef struct {
	uart_t *uart;
} uart_connection_t;
```
UART Connection Type
 #### Functions

 - `uart_transmit`
```c
error_t uart_transmit(uart_connection_t conn, buffer_view_t buffer);
```
Transmit bytes via UART

 - `uart_receive`
```c
error_t uart_receive(uart_connection_t conn, buffer_view_t buffer);
```
Transmit bytes via UART

### ADC

#### Types
 - `adc_handle_t`
```c
typedef <ADC_Type> adc_handle_t;
```
ADC Channel Type
- `adc_t`
```c
typedef struct {
  adc_handle_t handle;
  uint8_t bits;
  float voltage_reference;
} adc_t;
```
ADC Type

#### Functions

 - `adc_init`
```c
error_t adc_init(adc_t* adc);
```
ADC initialization 

 - `adc_read`
```c
result_uint16_t adc_read(adc_t* adc);
```
ADC Read raw value

 - `adc_raw_to_voltage`
```c
float adc_raw_to_voltage(adc_t adc, const uint16_t value);
```

ADC conversion to volts.

<p align="center">
    <a href="http://zenith.eesc.usp.br">
    <img src="https://img.shields.io/badge/Check%20out-Zenith's Oficial Website-black?style=for-the-badge" />
    </a> 
    <a href="https://www.facebook.com/zenitheesc">
    <img src="https://img.shields.io/badge/Like%20us%20on-facebook-blue?style=for-the-badge"/>
    </a> 
    <a href="https://www.instagram.com/zenith_eesc/">
    <img src="https://img.shields.io/badge/Follow%20us%20on-Instagram-red?style=for-the-badge"/>
    </a>

</p>
<p align = "center">
<a href="zenith.eesc@gmail.com">zenith.eesc@gmail.com</a>
</p>
