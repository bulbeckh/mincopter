
## HAL Configuration

`CONFIG_HAL_BOARD`

- Removed HAL_CPU_CLASS (was only used by baro)
- Removed HAL_OS_POSIX_IO (used in DataFlash_File)

Then defined by CMake:
- TARGET_ARCH_LINUX
- TARGET_ARCH_AVR
- ...

HAL interface has

- x5 UARTs
- x1 I2C
- x1 SPI manager
- x1 AnalogIn (ADC converter??)
- x1 Storage (EEPROM/DataFlash etc.)
- x1 GPIO

- x1 RCInput
- x1 RCOutput
- x1 Scheduler
- x1 Util

### GPIO
- Collection of `AP_HAL::DigitalSource` objects
- Each has standard read, write, pinMode and accessed via ::channel method

### AnalogIn
- Collection `AP_HAL::AnalogSource` objects representing ADCs
- Accesed via `voltage_latest` and `voltage_average`

### I2CDriver
- Methods for write, writeRegister, and writeRegisters (and read equivalents) over I2C

### SPIDeviceManager
- Collection of `AP_HAL::SPIDeviceDriver` objects which are each accessible via ::device
- Each device driver has methods for chip select, transfer, and transaction

### Storage
- High-level interface for reading and write of byte/word/dword/block

### UARTDriver
- No centralised UART manager, but rather individual UARTDriver objects
- Contains intialisation methods (which sets buffer sizes and baudrate) and interfaces for printf methods
- Inherits from `AP_HAL::BetterStream` which contains write functions
- UARTDriver -> BetterStream -> Stream -> Print - Where is ::write defined??
- ::write method is not part of the interface but defined in derived classes??

## Non-Hardware
The following don't represent hardware abstractions but higher level interfaces

### RCInput and RCOutput
- Interfaces with read/write methods for changing radio channels
- Uses GPIO pins as input/output (intitialised during init)

## Scheduler
- Methods for obtaining current micros/millis and registering timer processes

## Util
- Conversion functions (float to string, unsigned to string, etc.)
- Printf implementations and variations




## Implementations
We have implementations for AVR and Linux. Need to understand how they are built in order to build implementations for other AVR boards as well as ARM chips etc.

### `AP_HAL_AVR`


## Scheduler
One of the key elements is the ability to schedule callbacks and other functions based on timer interrupts. Note, there is the `AP_Scheduler` class which is a different scheduler that ensures
loop functions run at the correct frequency.

`AP_HAL::Scheduler` is the interface between the hardware timers and the corresponding callback functions. These functions may also be member functions which require the fast_delegate interface.

We currently have 5 member functions that make use of the `register_timer_process` function (1 for baro, 2 for ADC and 2 for IMU).

We have 2 member functions that make use of the `register_io_process` function (1 for LEDs via AP_Notify, and 1 for Dataflash backend).

We don't use `register_delay_callback` except in the lib/menu which may soon be deprecated

We don't use `register_timer_failsafe'



## Methods to implement

GPIO (AP_HAL::DigitalSource)
- virtual void mode(uint8_t output) = 0;
- virtual uint8_t read() = 0;
- virtual void write(uint8_t value) = 0;
- virtual void toggle() = 0;

GPIO
- virtual void init() = 0;
- virtual void pinMode(uint8_t pin, uint8_t output) = 0;
- virtual uint8_t read(uint8_t pin) = 0;
- virtual void write(uint8_t pin, uint8_t value) = 0;
- virtual void toggle(uint8_t pin) = 0;
- virtual int8_t analogPinToDigitalPin(uint8_t pin) = 0;
- virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;
- virtual bool attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) = 0;
- virtual bool usb_connected(void) = 0;

AnalogIn (AP_HAL::AnalogSource) 
- virtual float read_average() = 0;
- virtual float read_latest() = 0;
- virtual void set_pin(uint8_t p) = 0;
- virtual void set_stop_pin(uint8_t p) = 0;
- virtual void set_settle_time(uint16_t settle_time_ms) = 0;
- virtual float voltage_average() = 0;
- virtual float voltage_latest() = 0;
- virtual float voltage_average_ratiometric() = 0;

AnalogIn (AP_HAL::AnalogIn)
- virtual void init(void* implspecific) = 0;
- virtual AP_HAL::AnalogSource* channel(int16_t n) = 0;

I2C (AP_HAL::I2CDriver)
- virtual void begin() = 0;
- virtual void end() = 0;
- virtual void setTimeout(uint16_t ms) = 0;
- virtual void setHighSpeed(bool active) = 0;
- virtual uint8_t write(uint8_t addr, uint8_t len, uint8_t* data) = 0;
- virtual uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val) = 0;
- virtual uint8_t writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) = 0;
- virtual uint8_t read(uint8_t addr, uint8_t len, uint8_t* data) = 0;
- virtual uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data) = 0;
- virtual uint8_t readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) = 0;
- virtual uint8_t lockup_count() = 0;
- virtual AP_HAL::Semaphore* get_semaphore() = 0;

SPI (AP_HAL::SPIDeviceManager)
- virtual void init(void *) = 0;
- virtual AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice) = 0;

SPI (AP_HAL::SPIDeviceDriver)
- virtual void init() = 0;
- virtual AP_HAL::Semaphore* get_semaphore() = 0;
- virtual void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) = 0;
- virtual void cs_assert() = 0;
- virtual void cs_release() = 0;
- virtual uint8_t transfer (uint8_t data) = 0;
- virtual void transfer (const uint8_t *data, uint16_t len) = 0;
- virtual void set_bus_speed(enum bus_speed speed) {}

UART (AP_HAL::UARTDriver)
- virtual void begin(uint32_t baud) = 0;
- virtual void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) = 0;
- virtual void end() = 0;
- virtual void flush() = 0;
- virtual bool is_initialized() = 0;
- virtual void set_blocking_writes(bool blocking) = 0;
- virtual bool tx_pending() = 0;

Storage (AP_HAL::Storage)
- virtual void init(void *) = 0;
- virtual uint8_t  read_byte(uint16_t loc) = 0;
- virtual uint16_t read_word(uint16_t loc) = 0;
- virtual uint32_t read_dword(uint16_t loc) = 0;
- virtual void     read_block(void *dst, uint16_t src, size_t n) = 0;
- virtual void write_byte(uint16_t loc, uint8_t value) = 0;
- virtual void write_word(uint16_t loc, uint16_t value) = 0;
- virtual void write_dword(uint16_t loc, uint32_t value) = 0;
- virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;


### NOTE TODO Things to potentially remove
- FastDelegate (need to replace with something simpler and smaller)
- Print/Stream/BetterStream (replace with standard printf)




