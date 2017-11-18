#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include "dw1000.h"
#include "chprintf.h"
#include <stdio.h>
#include <string.h>
#include "hw_conf.h"
#include <inttypes.h>

#define ANSWER_DELAY_US             5000
#define ANSWER_DELAY_TIMEUNITS      ANSWER_DELAY_US * (128*499.2)
#define TIMEUNITS_TO_US       (1/(128*499.2))

#define TX
static DW1000 *dw1000;
static dw1000_tx_conf_t tx_conf;
static uint64_t sentTimes[4];
static uint64_t receivedTimes[3];
static uint8_t state;

void rxcb(uint64_t status)
{
    if (!(status & DW_RXDFR_MASK))
    {
        chprintf((BaseSequentialStream*)&SDU1, "receive error %d\n", status);
        dw1000->ResetRX();
#ifndef TX
        dw1000->Receive( DW_TRANCEIVE_ASYNC );
#endif
        return;
    }

    uint32_t rxlen;
    uint8_t* rxbuf = dw1000->GetRXBuffer(&rxlen);

    //chprintf((BaseSequentialStream*)&SDU1, "recv: %d %d\n", state, rxbuf[0]);
    switch(rxbuf[0])
    {
        uint8_t data[17];
        uint64_t treply1;
        uint64_t tround2;
        case 0:
        case 1:
            receivedTimes[rxbuf[0]] = dw1000->GetRXTimestamp();
            state = rxbuf[0] + 1;
            data[0] = state;
            tx_conf.is_delayed = 1;
            tx_conf.dx_timestamp = receivedTimes[rxbuf[0]] + ANSWER_DELAY_TIMEUNITS;
            dw1000->ConfigureTX( &tx_conf );
            dw1000->Transmit(data, tx_conf.data_len, DW_TRANCEIVE_ASYNC);
            break;
        case 2:
            receivedTimes[rxbuf[0]] = dw1000->GetRXTimestamp();
            state = rxbuf[0] + 1;
            data[0] = state;
            treply1 = sentTimes[1] - receivedTimes[0];
            tround2 = receivedTimes[2] - sentTimes[1];
            memcpy(&data[1], &treply1, 8);
            memcpy(&data[9], &tround2, 8);
            tx_conf.is_delayed = 1;
            tx_conf.dx_timestamp = receivedTimes[rxbuf[0]] + ANSWER_DELAY_TIMEUNITS;
            dw1000->ConfigureTX( &tx_conf );
            dw1000->Transmit(data, tx_conf.data_len, DW_TRANCEIVE_ASYNC);
            break;
        case 3:
            memcpy(&treply1, &rxbuf[1], 8);
            memcpy(&tround2, &rxbuf[9], 8);
            uint64_t tround1 = receivedTimes[1] - sentTimes[0];
            uint64_t treply2 = sentTimes[2] - receivedTimes[1];
            float tround1_us = tround1 * TIMEUNITS_TO_US;
            float tround2_us = tround2 * TIMEUNITS_TO_US;
            float treply1_us = treply1 * TIMEUNITS_TO_US;
            float treply2_us = treply2 * TIMEUNITS_TO_US;
            float tprop = (tround1_us * tround2_us - treply1_us * treply2_us) / (tround1_us + tround2_us + treply1_us + treply2_us);
            float dist = tprop * 300 / 4.0;

            chprintf((BaseSequentialStream*)&SDU1, "tprop: %f %f %f %f %f %f\n", tround1_us, tround2_us, treply1_us, treply2_us, tprop, dist);
            break;
    }
}

void txcb(uint64_t status)
{
    sentTimes[state] = dw1000->GetTXTimestamp();
    //chprintf((BaseSequentialStream*)&SDU1, "sent: %d\n", sentTimes[state]);
    dw1000->Receive( DW_TRANCEIVE_ASYNC );
}

int main(void) {
    halInit();
    chSysInit();

    dw1000 = new DW1000();
    dw1000->SetCallbacks(&txcb, &rxcb);

    tx_conf.data_len = 17;
    tx_conf.is_delayed = 0;
    dw1000->ConfigureTX( &tx_conf );

    dw1000_rx_conf_t rx_conf;
    rx_conf.is_delayed = 0;
    rx_conf.timeout = 0;//0xFFFF; // ~65 ms
    dw1000->ConfigureRX( &rx_conf );

#ifndef TX
    dw1000->Receive( DW_TRANCEIVE_ASYNC );
#endif

    for (;;)
    {
#ifdef TX
        state = 0;
        uint8_t data[17];
        data[0] = state;
        tx_conf.is_delayed = 0;
        dw1000->ConfigureTX( &tx_conf );
        dw1000->Transmit(&state, tx_conf.data_len, DW_TRANCEIVE_ASYNC);
#endif
        chThdSleepMilliseconds(100);
    }
}
