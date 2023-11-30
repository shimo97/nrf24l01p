# NRF24L01 LIBRARY

This library is based on mokhwasomssi's work (https://github.com/mokhwasomssi/stm32_hal_nrf24l01p).

The library has been almost completely rewritten (only some low level defines and
functions has been preserved) to be simpler to use and be more compatible to an
half duplex rather than simplex usage as it was intended before.

Other than that, now the library can support multiple transceivers on the same microcontroller.

Now the library will only pass through the recommended operating modes from the datasheet
(Power Down, Standby-I, Rx and TX) rather than resorting to Standby-II mode like typically
done for simplex-oriented libraries (Standby-II mode is easier to use for simplex transmission
because it doesn't require a specific timing of CE pin but it doesn't give the possibility to pass
directly on RX mode without a dummy transmission or a power down)

INSTRUCTIONS:
To use the library, the user has to:
-define the get_tick_us() function (i.e. with an hardware timer)
-place the nrf24l01p_tx_irq() inside the EXTI interrupt handler of the IRQ pin
-define the _weak functions nrf24l01p_tx_callback(), nrf24l01p_rx_callback() and nrf24l01p_max_callback()
with the code to execute after correct transmission, reception and failed transmission, respectively

The transceiver can be set up to idle on RX mode or Standby(-I) mode and it will go on TX mode
only when a transmission is requested, Standby(-I) idle mode is recommended for a simplex setup
because have a much lower power consumption w.r.t. RX mode
This way it can be used for both simplex or half-duplex communication.

TRANSCEIVER SETUP:
All the user has to do to set up the transceiver is to pass an initialization struct nrf24l01p_init_struct
to the nrf24l01p_add() function, this function will return an integer (the transceiver ID) thet identifies
the transceiver and must be pssed to the other library functions
