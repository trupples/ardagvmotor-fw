#include <stdint.h>
#include <zephyr/kernel.h>
#include <spi.h>

void nesimtit_init()
{
    mxc_spi_pins_t tmp;
    MXC_SPI_Init(MXC_SPI0, 1, 0, 1, 0, 5000000, tmp);
    MXC_SPI_SetDataSize(MXC_SPI0, 8);
    MXC_SPI_SetMode(MXC_SPI0, 3);
    MXC_SPI0->sstime = 0x001919; // max inactive (256), a bit of post, a bit of pre (0x19 = 25 ~ 1us)
    // cca 50-65us between CS seems to be the fastest we can go
}

int nesimtit_spi_transceive(char *tx, char *rx)
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
    k_busy_wait(100); // @5MHz: 10 unstable
    return MXC_SPI_MasterTransaction(&req);
}
