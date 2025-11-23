### Targets

`linux/generic`

`avr/avr-*`
- The mega-series boards can be compiled with most versions of avr-gcc. They newer dx-series, however, requires a more recent version. We can use either avr-gcc or Microchips proprietary xc8 compiler but we opt for the former as it is both open-source and easier to install. However, the device-specs for each board still need to be downloaded. They can be obtained here: TODO.

`linux/rpi`

`arm/stm32`

### Configuration

For each version, we specify a full list of connections to be used in the configuration. Each board needs to have the following:

MinCopter v0.1 netlist
- LED A (GPIO)
- LED B (GPIO)
- LED C (GPIO)
- External (Battery) Current Measure (ADC)
- External (Battery) Voltage Measure (ADC)
- RSSI Pin (ADC)
- Telemetry RX (UART)
- Telemetry TX (UART)
- GPS RX (UART)
- GPS TX (UART)
- SCK (I2C)
- SDA (I2C)
- MOSI (SPI)
- MISO (SPI)
- SCLK (SPI)
- PWM0 (GPIO)
- PWM1 (GPIO)
- PWM2 (GPIO)
- PWM3 (GPIO)


```c++
#define NL_LEDA 0
#define NL_LEDB 1
#define NL_LEDC 2
#define NL_EXT_CURR_MEAS 3
#define NL_EXT_VOLT_MEAS 4
#define NL_RSSI 5
#define NL_TELEM_RX 6
#define NL_TELEM_TX 7
#define NL_GPS_RX 8
#define NL_GPS_TX 9
#define NL_I2C_SCK 10
#define NL_I2C_SDA 11
#define NL_SPI_MOSI 12
#define NL_SPI_MISO 13
#define NL_SPI_SCLK 14
#define NL_PWM0 15
#define NL_PWM1 16
#define NL_PWM2 17
#define NL_PWM3 18

#define PL_PORT_A 0
#define PL_PORT_B 1
#define PL_PORT_C 2
#define PL_PORT_D 3
#define PL_PORT_E 4
#define PL_PORT_F 5
#define PL_PORT_G 6
#define PL_PORT_H 7

typedef struct {
    uint8_t port;
    uint8_t pin;
} netlist_t;

typedef struct {
    // TODO Does the uint16_t type actually match that used by the enum??
    uint16_t type;
    uint8_t port;
    uint8_t pin;
} netlist_gpio_t;

// We list all possible GPIO inputs across all sensors (just CS pins for now but will add external interrupts in a later version)
enum {
    GPIO_BME280_CS;
    GPIO_MS5611_CS;
    GPIO_ICM20948_CS;
    GPIO_MPU6000_CS;
    GPIO_L3G4200D_CS;
    GPIO_AT45DB321D_CS; // External Storage - ATMEL DataFlash
    GPIO_ADS7844_CS;    // External ADC - Texas Instruments
};

// This is where we supply our full netlist
netlist_t nl[18] = {
    { PL_PORT_A, 0 }, // NL_LEDA
    { PL_PORT_A, 1 }, // NL_LEDB
    { PL_PORT_A, 4 }, // NL_LEDC
    { PL_PORT_F, 6 }, // NL_EXT_CURR_MEAS
    { PL_PORT_F, 7 }, // NL_EXT_VOLT_MEAS
    // ...
    // ...
    // ...
};

// Our standard defines 16 GPIO (including 4 PWM and 3 LED), leaving us 9 remaining GPIO for CS and external interrupts
netlist_gpio_t nl_gpio[] = {
    { GPIO_MPU6050_CS, PL_PORT_B, 4 }, // MPU6050_CS
    { GPIO_MPU6050_DR, PL_PORT_B, 5 }, // MPU6050_DR
    // ...
    // ...
};
```

During configuration of the HAL, we use this netlist to initialise peripherals.

For UART:

```c++
UARTDriver::init(void)
{
    // Here, we use the nl array to retrieve the port used
    if (uarttype==AP_HAL::UARTTypeGPS) {
        HAL_UART_Init(nl[NL_GPS_RX].port);
    } else if (uarttype==AP_HAL::UARTTypeTelemetry) {
        HAL_UART_Init(nl[NL_TELEM_RX].port);
    }

}
```

For GPIO, we support 16 potential channels, so we create an DigitalSource for each entry in nl_gpio and initialise it
according to the correct port/pin. We also store the type and when each device retrieves it's GPIO pin via the ::channel
function, we return the correct DigitalSource by iterating through our list and finding the match. This should only be done
once during initialisation.

```c++
// From within the
AP_HAL::DigitalSource* ds = hal.gpio->channel(GPIO_MPU6050_CS);
```

For ADC, we support 8 potential channels. These are created in a similar to the GPIO
For I2C, ..
For SPI, ..

Then, for each HAL backend, we can provide a mapping between the above PL_PORT_xx macros and the underlying PORT/PIN object

We need to map these to each of the abstracted HAL modules. We have the following list of HAL modules that each HAL backend needs to support

- 16 GPIO Channels (including 4 PWM)
- 8 ADC channels
- GPS-UART
- Telemetry-UART
- I2C
- SPI

Our HAL class should now contain something like the following:

- hal.gpio (to access each of the 16 GPIO channels)
- hal.rcout (to access the PWM outputs)
- hal.i2cA (to access I2C)
- hal.spiA (to access SPI)
- hal.analogin (to access each of the 8 ADC channels)
- hal.uartGPS  (to access the GPS UART)
- hal.uartTelem (to acess the Telemetry UART)

- hal.util
- hal.storage
- hal.scheduler

No hal.rcin anymore either

