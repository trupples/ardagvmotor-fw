#include <stdint.h>
#include <zephyr/kernel.h>
#include <spi.h>

void nesimtit_busy()
{
   // while(*SPI0_STAT & 1) k_yield();
}

void nesimtit_init()
{
    mxc_spi_pins_t tmp;
    MXC_SPI_Init(MXC_SPI0, 1, 0, 1, 0, 1000000, tmp);
    MXC_SPI_SetDataSize(MXC_SPI0, 8);
    MXC_SPI_SetMode(MXC_SPI0, 3);
}

int nesimtit_transceive2(char *tx, char *rx)
{
    struct _mxc_spi_req_t req = {
        .spi = MXC_SPI0,
        .ssIdx = 0,
        .ssDeassert = 1,
        .txData = tx,
        .rxData = rx,
        .txLen = tx ? 8 : 0,
        .rxLen = rx ? 8 : 0,
        .txCnt = 0,
        .rxCnt = 0,
        .completeCB = NULL
    };
    return MXC_SPI_MasterTransaction(&req);
}
