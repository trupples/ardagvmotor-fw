#include <stdint.h>
#include <zephyr/kernel.h>
#include <spi.h>
#include <can.h>
#include <gpio.h>
#include <string.h>
#include <nvic_table.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nesimtit, LOG_LEVEL_DBG);

int g_node_id;

void CAN_IRQHandler(void*)
{
    MXC_CAN_Handler(0); // Moves message from RX FIFO to request structure
}

// Handles "object" events = relating to messages (transmit done, receive done, receive overrun)
void nesimtit_can_obj_handler(uint32_t can_idx, uint32_t event);

// Handles "unit" events = relating to state (too many errors)
void nesimtit_can_unit_handler(uint32_t can_idx, uint32_t event) ;

struct nesimtit_can_msg {
    int node_id;
    int data_size;
    char data[8];
};

// CAN receive stuff
K_MSGQ_DEFINE(canq, sizeof(struct nesimtit_can_msg), 16, 1); // Receive queue
char can_recv_data[8];
mxc_can_msg_info_t can_recv_info = { 0 };

mxc_can_req_t can_recv_req = {
    .msg_info = &can_recv_info,
    .data = can_recv_data,
    .data_sz = 8
};

char can_tx_data[8];
mxc_can_msg_info_t can_tx_info;
mxc_can_req_t can_tx_req = {
    .msg_info = &can_tx_info,
    .data = can_tx_data,
};

void nesimtit_init(int can_id)
{
    // SPI to TMC9660
    mxc_spi_pins_t tmp;
    MXC_SPI_Init(MXC_SPI0, 1, 0, 1, 0, 5000000, tmp);
    MXC_SPI_SetDataSize(MXC_SPI0, 8);
    MXC_SPI_SetMode(MXC_SPI0, 3);
    MXC_SPI0->sstime = 0x001919; // max inactive (256), a bit of post, a bit of pre (0x19 = 25 ~ 1us)
    // cca 50-65us between CS seems to be the fastest we can go

    g_node_id = can_id;

    // MSDK_NO_GPIO_CLK_INIT is set, likely due to other zephyr internals
    // (auto config based on devicetree?) so we need to manually do the things
    // that are deselected by MSDK_NO_GPIO_CLK_INIT in can_me12.c
    // Otherwise the entire can system is off
    MXC_GPIO_Config(&gpio_cfg_can);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CAN);
    
    MXC_CAN_Init(0, MXC_CAN_OBJ_CFG_TXRX, nesimtit_can_unit_handler, nesimtit_can_obj_handler, 0);
    
    MXC_CAN_ObjectSetFilter(0, (MXC_CAN_FILT_CFG_MASK_ADD | MXC_CAN_FILT_CFG_SINGLE_EXT_ID), 0x1FFFFFFF, 0); // Set receive filter to accept all extended IDs
    MXC_CAN_ObjectSetFilter(0, (MXC_CAN_FILT_CFG_MASK_ADD | MXC_CAN_FILT_CFG_SINGLE_STD_ID), 0x1FFFFFFF, 0); // Set receive filter to accept all standard IDs
    int err = MXC_CAN_SetBitRate(0, MXC_CAN_BITRATE_SEL_NOMINAL, 500000, MXC_CAN_BIT_SEGMENTS(7, 2, 2)); // Nominal bitrate 500kHz, TSEG1 - 7, TSEG2 - 2
    if ((MXC_CAN_GetBitRate(0, MXC_CAN_BITRATE_SEL_NOMINAL)) != 500000) {
        LOG_ERR("Could not initialize 500kbaud CAN. Have %d baud", MXC_CAN_GetBitRate(0, MXC_CAN_BITRATE_SEL_NOMINAL));
        return;
    }

   IRQ_CONNECT(CAN_IRQn, 1, CAN_IRQHandler, 0, 0);
   irq_enable(CAN_IRQn);

    MXC_CAN_MessageReadAsync(0, &can_recv_req);

    LOG_INF("Bodged CAN & SPI initialized!");
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

void nesimtit_can_transmit(char *data, int len)
{
    can_tx_info.msg_id = MXC_CAN_STANDARD_ID(g_node_id);
    can_tx_info.rtr = 0;
    can_tx_info.fdf = 0;
    can_tx_info.brs = 0;
    can_tx_info.esi = 0;
    can_tx_info.dlc = len;
    can_tx_info.rsv = 0;

    memcpy(can_tx_data, data, len);
    can_tx_req.data_sz = len;

    LOG_DBG("CAN sending...");
    MXC_CAN_MessageSend(0, &can_tx_req);
    LOG_DBG("CAN sent!");
}

int nesimtit_can_receive_noblock(char *message, int *out_len, int *src_node)
{
    struct nesimtit_can_msg msg = { 0 };

    if(k_msgq_get(&canq, &msg, K_NO_WAIT) != 0)
        return -EAGAIN;
    
    *out_len = msg.data_size;
    memcpy(message, msg.data, msg.data_size);
    if(src_node) *src_node = msg.node_id;

    return 0;
}

void nesimtit_can_obj_handler(uint32_t can_idx, uint32_t event)
{
    LOG_DBG("obj %x %x", can_idx, event);
    if(can_idx != 0) return;

    if(event == MXC_CAN_OBJ_EVT_TX_COMPLETE) // Transmit done
    {
        LOG_DBG("CAN transmit OK");
        return;
    }

    if(event == MXC_CAN_OBJ_EVT_RX) // Receive done
    {
        struct nesimtit_can_msg msg = {
            .node_id = can_recv_info.msg_id,
            .data_size = can_recv_info.dlc,
            .data = { 0 } // uninitialized might be a stack leak!
        };
        memcpy(msg.data, can_recv_data, msg.data_size);

        // Add received message to queue
        k_msgq_put(&canq, &msg, K_NO_WAIT); // FIXME: no error handling!

        LOG_HEXDUMP_DBG(msg.data, msg.data_size, "CAN recv");
        return;
    }
}

void nesimtit_can_unit_handler(uint32_t can_idx, uint32_t event) 
{

    LOG_DBG("unit %x %x", can_idx, event);
    if(can_idx != 0) return;

    switch(event) {
    case MXC_CAN_UNIT_EVT_INACTIVE:
        LOG_INF("CAN state: MXC_CAN_UNIT_EVT_INACTIVE");
        break;
    case MXC_CAN_UNIT_EVT_ACTIVE:
        LOG_INF("CAN state: MXC_CAN_UNIT_EVT_ACTIVE");
        break;
    case MXC_CAN_UNIT_EVT_WARNING:
        LOG_INF("CAN state: MXC_CAN_UNIT_EVT_WARNING");
        break;
    case MXC_CAN_UNIT_EVT_PASSIVE:
        LOG_INF("CAN state: MXC_CAN_UNIT_EVT_PASSIVE");
        break;
    case MXC_CAN_UNIT_EVT_BUS_OFF:
        LOG_INF("CAN state: MXC_CAN_UNIT_EVT_BUS_OFF");
        break;
    }
}
