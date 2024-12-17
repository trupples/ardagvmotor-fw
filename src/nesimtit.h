#ifdef ESTI_NESIMTIT

// Wrappers around MSDK functions for peripherals

/* Initialize bodged SPI */
void nesimtit_init();

/* Transmit / receive an 8 byte SPI datagram. tx, rx can be NULL. */
int nesimtit_spi_transceive(char *tx, char *rx);

#endif // ESTI_NESIMTIT
