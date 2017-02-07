#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include "dw1000.h"
#include "chprintf.h"
#include <stdio.h>
#include <string.h>
#include "hw_conf.h"

int main(void) {
    halInit();
    chSysInit();

    DW1000 *dw1000 = new DW1000();
    comm_usb_serial_init();

    for (;;)
    {
        // Configure transmission
        dw1000_tx_conf_t tx_conf;
        tx_conf.data_len = 10;
        tx_conf.is_delayed = 0;
        dw1000->ConfigureTX( &tx_conf );

        // Transmit
        static uint8_t counter = 0;
        uint8_t  p_data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        p_data[0] = counter++;
        dw1000->Transmit( p_data, tx_conf.data_len, DW_TRANCEIVE_SYNC );

        // Configure reception
        dw1000_rx_conf_t rx_conf;
        rx_conf.is_delayed = 0;
        rx_conf.timeout = 0;//0xFFFF; // ~65 ms

        // Receive
        dw1000->ConfigureRX( &rx_conf );
        dw1000->Receive( DW_TRANCEIVE_SYNC );
        uint32_t rxlen;
        uint8_t* rxbuf = dw1000->GetRXBuffer(&rxlen);

        chprintf((BaseSequentialStream*)&SDU1, "dw: ");
        for (uint8_t i = 0; i < rxlen; i++)
        {
            chprintf((BaseSequentialStream*)&SDU1, "%d ", rxbuf[i]);
        }
        chprintf((BaseSequentialStream*)&SDU1, "\n");
        chThdSleepMilliseconds(5);
    }
}
