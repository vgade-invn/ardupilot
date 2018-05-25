#include "UARTDriver.h"


#if HAL_CPU_CLASS >= HAL_CPU_CLASS_1000
    #define AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_TX            32768
    #define AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_RX            8192
#else
    #define AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_TX            1024
    #define AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_RX            1024
#endif

/*
  open virtual port
 */
void UARTDriver::begin(uint32_t baud)
{
    begin(baud, 0, 0);
}

void UARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS)
{
    (void)baud; // unused in virtual ports

    _initialised = false;

    // enforce minimum sizes
    if (rxS < AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_RX) {
        rxS = AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_RX;
    }
    if (txS < AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_TX) {
        txS = AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_TX;
    }

    if (_writebuf.set_size(txS) && _readbuf.set_size(rxS)) {
        _initialised = true;
    }
}


/*
  shutdown a Virtual UART
 */
void UARTDriver::end()
{
    _initialised = false;
    _readbuf.set_size(0);
    _writebuf.set_size(0);
}


/*
  do we have any bytes pending transmission?
 */
bool UARTDriver::tx_pending()
{
    // any data in SerialManager -> UAVCAN
    return (_writebuf.available() > 0);
}

/*
  return the number of bytes available to be read
 */
uint32_t UARTDriver::available()
{
    if (!_initialised) {
        return 0;
    }
    // any data in UAVCAN -> SerialManager
    return _readbuf.available();
}

/*
  how many bytes are available in the output buffer?
 */
uint32_t UARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }
    // how much free space available in SerialManager -> UAVCAN
    return _writebuf.space();
}

uint32_t UARTDriver::handle_inbound(const uint8_t *buffer, uint32_t size)
{
    // load data into UAVCAN -> SerialManager
    return _readbuf.write(buffer, size);
}

int16_t UARTDriver::fetch_for_outbound(void)
{
    if (!_initialised) {
        return -1;
    }

    uint8_t byte;
    if (!_writebuf.read_byte(&byte)) {
        return -1;
    }

    return byte;
}


int16_t UARTDriver::read()
{
    if (!_initialised) {
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }

    return byte;
}

/* implementation of write virtual methods */
size_t UARTDriver::write(uint8_t c)
{
    if (!_initialised) {
        return 0;
    }
    return _writebuf.write(&c, 1);
}

/*
  write size bytes to the write buffer
 */
size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return 0;
    }
    return _writebuf.write(buffer, size);
}

