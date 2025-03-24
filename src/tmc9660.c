#include <errno.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "tmc9660.h"

LOG_MODULE_REGISTER(tmc9660, LOG_LEVEL_INF);

enum tmc9660_spi_status {
    SPI_STATUS_OK = 0xff,
    SPI_STATUS_CHECKSUM_ERROR = 0x00,
    SPI_STATUS_FIRST_CMD = 0x0c,
    SPI_STATUS_NOT_READY = 0xf0
};

enum tmc9660_tmcl_status {
    REPLY_OK = 100,
    REPLY_CMD_LOADED = 101,
    REPLY_CHKERR = 1,
    REPLY_INVALID_CMD = 2,
    REPLY_WRONG_TYPE = 3,
    REPLY_INVALID_VALUE = 4,
    REPLY_CMD_NOT_AVAILABLE = 6,
    REPLY_CMD_LOAD_ERROR = 7,
    REPLY_MAX_EXCEEDED = 9,
    REPLY_DOWNLOAD_NOT_POSSIBLE = 10
};

// Parameter mode checksum
// NOT SUITABLE FOR BOOTLOADER MODE, THAT EXPECTS CRC
uint8_t tmc9660_checksum(uint8_t buffer[8])
{
    uint8_t res = 0;
    for(int i = 0; i < 7; i++)
        res += buffer[i];
    return res;
}

int tmc9660_tmcl_command(
    struct tmc9660_dev *dev,
    enum tmc9660_operation_id operation,
    uint16_t type,
    uint8_t motorbank,
    uint32_t value_send,
    uint32_t *value_recv
)
{
    k_mutex_lock(&dev->mutex, K_FOREVER);

    uint8_t noop[8] = {
        GetInfo, 0, 0, 0, 0, 0, 0, GetInfo
    };
    uint8_t send[8] = {
        operation,
        (type & 0xff),
        ((type&0xf00)>>4) | motorbank,
        (value_send >> 24) & 0xff,
        (value_send >> 16) & 0xff,
        (value_send >> 8) & 0xff,
        (value_send >> 0) & 0xff,
        0
    };
    uint8_t recv[8] = { 0 };

    struct spi_buf tx_buf[2] = {
        { .buf = send, .len = 8 }
    };
    struct spi_buf_set tx_bufs = {
        .buffers = tx_buf, .count = 1
    };
    struct spi_buf rx_buf[2] = {
        { .buf = recv, .len = 8 },
    };
    struct spi_buf_set rx_bufs = {
        .buffers = rx_buf, .count = 1
    };
    struct spi_buf noop_buf[2] = {
        { .buf = noop, .len = 8 },
    };
    struct spi_buf_set noop_bufs = {
        .buffers = noop_buf, .count = 1
    };

    send[7] = tmc9660_checksum(send);
    
    uint8_t spi_status, tmcl_status, reply_operation;
    uint32_t data;

    k_usleep(20);

    int retries = 5;
    do {
        spi_transceive_dt(dev->spi, &tx_bufs, &rx_bufs);

        if(recv[0] == SPI_STATUS_CHECKSUM_ERROR) {
            LOG_DBG("Previously sent bad checksum...");
            LOG_HEXDUMP_DBG(send, 8, "SPI send");
            LOG_HEXDUMP_DBG(recv, 8, "SPI recv");
        }
        if(recv[0] == SPI_STATUS_NOT_READY) {
            k_usleep(20);
        }
    }
    while((recv[0] == SPI_STATUS_NOT_READY || recv[0] == SPI_STATUS_CHECKSUM_ERROR) && retries-- > 0);
    
    k_usleep(10);

    if(retries <= 0) {
        LOG_WRN("TMC9660 took 5 retries for submitting a command");
        k_mutex_unlock(&dev->mutex);
        return -1;
    }

    retries = 5;
    do {
        spi_transceive_dt(dev->spi, &noop_bufs, &rx_bufs);
        if(recv[0] == SPI_STATUS_NOT_READY) {
            k_usleep(10);
        }
    } while(recv[0] == SPI_STATUS_NOT_READY && retries-- > 0);
    
    if(retries <= 0) {
        LOG_WRN("TMC9660 took 5 retries for answering a command");
        k_mutex_unlock(&dev->mutex);
        return -1;
    }

    spi_status = recv[0];
    tmcl_status = recv[1];
    reply_operation = recv[2];
    data = (recv[3] << 24) | (recv[4] << 16) | (recv[5] << 8) | recv[6];

    if(recv[7] != tmc9660_checksum(recv))
    {
        // bad checksum - rest of datagram will be malformed, incl the checksul
        LOG_HEXDUMP_ERR(recv, 8, "Checksum error in received");
        k_mutex_unlock(&dev->mutex);
        return -EIO;
    }
    
    if(spi_status != SPI_STATUS_OK || retries < 4) {
        LOG_HEXDUMP_DBG(recv, 8, "SPI recv");
        LOG_WRN("tmcl(%d, %d, %d, %d) -> spi %d (want 255), tmcl %d (want 100), reply %d, data %08x, retries %d (want 5)", 
            operation, type, motorbank, value_send,
            spi_status, tmcl_status, reply_operation, data, retries);
    }
    /*if(tmcl_status != 100) {
        LOG_ERR("tmcl(%d, %d, %d, %d) -> spi %d (want 255), tmcl %d (want 100), reply %d, data %08x, retries %d (want 5)", 
            operation, type, motorbank, value_send,
            spi_status, tmcl_status, reply_operation, data, retries);
    }*/

    if(value_recv) *value_recv = data;

    k_mutex_unlock(&dev->mutex);

    // TODO: sanity check for returned values...
    switch(spi_status)
    {
    case SPI_STATUS_OK: break;
    case SPI_STATUS_CHECKSUM_ERROR: return -EIO;
    case SPI_STATUS_FIRST_CMD: return -EIO; // Should never happen outside of init
    case SPI_STATUS_NOT_READY: return -EAGAIN;
    }

    switch(tmcl_status)
    {
    case REPLY_OK: return 0;
    case REPLY_CMD_LOADED: return 0;
    case REPLY_CHKERR: return -EINVAL;
    case REPLY_INVALID_CMD: return -EOPNOTSUPP;
    case REPLY_WRONG_TYPE: return -EINVAL;
    case REPLY_INVALID_VALUE: return -EINVAL;
    case REPLY_CMD_NOT_AVAILABLE: return -EOPNOTSUPP;
    case REPLY_CMD_LOAD_ERROR: return -EPERM;
    case REPLY_MAX_EXCEEDED: return -EPERM;
    case REPLY_DOWNLOAD_NOT_POSSIBLE: return -EPERM;
    }

    CODE_UNREACHABLE;
}

int tmc9660_init(
    struct tmc9660_dev *dev,
    const struct spi_dt_spec *spi
)
{
    dev->spi = spi;
    k_mutex_init(&dev->mutex);

    k_mutex_lock(&dev->mutex, K_FOREVER);

    char tx[8] = { 3, 0, 0, 0, 0, 0, 0, 0 };
    char rx[8];
    tx[7] = tmc9660_checksum(tx);

    struct spi_buf tx_buf[1] = {
        { .buf = tx, .len = 8 }
    };
    struct spi_buf_set tx_bufs = { .buffers = tx_buf, .count = 1 };
    struct spi_buf rx_buf[1] = {
        { .buf = rx, .len = 8 }
    };
    struct spi_buf_set rx_bufs = { .buffers = rx_buf, .count = 1 };

    int retries = 5;
    do {
        LOG_HEXDUMP_DBG(tx, 8, "Init SPI send");

        spi_transceive_dt(dev->spi, &tx_bufs, &rx_bufs);
        k_busy_wait(100);

        LOG_HEXDUMP_DBG(rx, 8, "Init SPI recv");
    } while(!(rx[0] == SPI_STATUS_FIRST_CMD || rx[0] == SPI_STATUS_OK) && retries-- > 0);

    k_mutex_unlock(&dev->mutex);
    
    if(rx[0] != SPI_STATUS_FIRST_CMD && rx[0] != SPI_STATUS_OK) {
        // Invalid wakeup
        LOG_ERR("Invalid TMC wakeup");
        return -1;
    }

    return 0;
}

// TODO if performance is an issue: transaction function that allows for
// multiple operations wihout requring NOOPs for reading back data

int tmc9660_set_param(
    struct tmc9660_dev *dev,
    enum tmc9660_param_id param,
    int value
)
{
    return tmc9660_tmcl_command(dev, SAP, param, 0, value, NULL);
}

int tmc9660_set_param2(
    struct tmc9660_dev *dev,
    enum tmc9660_param_id param,
    int value,
    int *value_out
)
{
    return tmc9660_tmcl_command(dev, SAP, param, 0, value, value_out);
}

int tmc9660_set_param_retry(
    struct tmc9660_dev *dev,
    enum tmc9660_param_id param,
    int value,
    int retries
)
{
    int ret = 0;
    for(int i = 0; i < retries; i++) {
        ret = tmc9660_tmcl_command(dev, SAP, param, 0, value, NULL);
        if(ret == 0)
            return 0;
    }
    return ret;
}

int tmc9660_get_param(
    struct tmc9660_dev *dev,
    enum tmc9660_param_id param,
    int *value
)
{
    return tmc9660_tmcl_command(dev, GAP, param, 0, 0, value);
}

int tmc9660_get_param_retry(
    struct tmc9660_dev *dev,
    enum tmc9660_param_id param,
    int *value,
    int retries
)
{
    int ret = 0;
    for(int i = 0; i < retries; i++) {
        ret = tmc9660_tmcl_command(dev, GAP, param, 0, 0, value);
        if(ret == 0)
            return 0;
    }
    return ret;
}

int tmc9660_set_gpio(
    struct tmc9660_dev *dev,
    int port,
    int value
)
{
    return tmc9660_tmcl_command(dev, SIO, port, 0, value, NULL);
}

int tmc9660_get_gpio_analog(
    struct tmc9660_dev *dev,
    int port,
    int *value
)
{
    return tmc9660_tmcl_command(dev, GIO, port, 1, 0, value);
}

int tmc9660_get_gpio_digital(
    struct tmc9660_dev *dev,
    int port,
    int *value
)
{
    return tmc9660_tmcl_command(dev, GIO, port, 0, 0, value);
}

int tmc9660_motor_stop(
    struct tmc9660_dev *dev
)
{
    return tmc9660_tmcl_command(dev, MST, 0, 0, 0, 0);
}
