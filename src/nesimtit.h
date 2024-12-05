#ifdef ESTI_NESIMTIT

// Wrappers around MSDK functions for peripherals
// Why? In our case (adi_hal, max32662) zephyr does not properly support SPI and CAN
// SPI: somewhere in the zephyr abstraction layers, it pulses the serial clock line right when deasserting chip select. This confuses the receiving device and breaks SPI comms.
// CAN: no wrapper at all

/* Initialize bodged SPI and CAN */
void nesimtit_init();

/* Transmit / receive an 8 byte SPI datagram. tx, rx can be NULL. */
int nesimtit_spi_transceive(char *tx, char *rx);

/* TODO: CAN functions */

#endif // ESTI_NESIMTIT
