#include "dw1000.h"
#include "hw_conf.h"

extern "C"
{
    static void dw_interrupt_handler(EXTDriver *extp, expchannel_t channel)
    {
        (void)extp;
        (void)channel;

        chSysLockFromISR();

        chSysUnlockFromISR();
    }

    static const EXTConfig extcfg = {
        {
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOC, dw_interrupt_handler},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL},
            {EXT_CH_MODE_DISABLED, NULL}
        }
    };
}

const SPIConfig DW1000::spicfg =
{
    NULL,
    CS_GPIO,
    CS_PIN,
    SPI_CR1_BR_2 | SPI_CR1_BR_1
};

DW1000::DW1000()
{
    m_state = DW_STATE_INITIALIZING;

    palSetPadMode(SCK_GPIO, SCK_PIN, PAL_MODE_ALTERNATE(5) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(MISO_GPIO, MISO_PIN, PAL_MODE_ALTERNATE(5) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(MOSI_GPIO, MOSI_PIN, PAL_MODE_ALTERNATE(5) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(CS_GPIO, CS_PIN, PAL_MODE_OUTPUT_PUSHPULL |
            PAL_STM32_OSPEED_HIGHEST);
    spiAcquireBus(&SPI_DEV);
    spiStart(&SPI_DEV, &spicfg);
    spiReleaseBus(&SPI_DEV);

    extStart(&EXTD1, &extcfg);

    trxOff();
    ClearPendingInterrupt(0x00000007FFFFFFFFULL);
    const uint32_t mask = DW_MTXFRS_MASK
        | DW_MRXDFR_MASK
        | DW_MRXPHE_MASK
        | DW_MRXRFTO_MASK
        | DW_MRXPTO_MASK
        | DW_MRXSFDTO_MASK
        | DW_MRXRFSL_MASK;
    EnableInterrupt(mask);

    const uint32_t lde1  = 0x0301;
    const uint32_t lde2  = 0x8000;
    const uint32_t lde3  = 0x0200;
    writeSubregister(0x36, 0x00, 2, (uint8_t *)&lde1);
    writeSubregister(0x2D, 0x06, 2, (uint8_t *)&lde2);
    chThdSleepMicroseconds(250);
    writeSubregister(0x36, 0x00, 2, (uint8_t *)&lde3);

    m_conf.prf             = DW_PRF_16_MHZ; 
    m_conf.channel         = DW_CHANNEL_5;
    m_conf.preamble_length = DW_PREAMBLE_LENGTH_128;
    m_conf.preamble_code   = DW_PREAMBLE_CODE_3;
    m_conf.pac_size        = DW_PAC_SIZE_8;
    m_conf.sfd_type        = DW_SFD_STANDARD;
    m_conf.data_rate       = DW_DATA_RATE_850_KBPS;
    Configure(&m_conf);

    m_state = DW_STATE_IDLE;
}

DW1000::~DW1000()
{
}

void DW1000::Configure(dw1000_base_conf_t *dw_conf)
{
    uint32_t sys_cfg_val   = readRegister32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
    uint32_t tx_fctrl_val  = readRegister32(DW_REG_TX_FCTRL, 4);
    uint32_t chan_ctrl_val = readRegister32(DW_REG_CHAN_CTRL, DW_LEN_CHAN_CTRL);
    uint32_t agc_tune1_val;
    const uint32_t agc_tune2_val = 0x2502A907;    /* Always use this */;
    const uint32_t agc_tune3_val = 0x0055;        /* Always use this */;
    uint32_t drx_tune0b_val;
    uint32_t drx_tune1a_val;
    uint32_t drx_tune1b_val;
    uint32_t drx_tune2_val;
    uint32_t drx_tune4h_val;
    uint32_t rf_rxctrl_val;
    uint32_t rf_txctrl_val;
    uint32_t tc_pgdelay_val;
    uint32_t fs_pllcfg_val;
    uint32_t fs_plltune_val;

    // === Configure PRF
    tx_fctrl_val  &= ~DW_TXPRF_MASK;
    chan_ctrl_val &= ~DW_RXPRF_MASK;
    switch (dw_conf->prf)
    {
        case DW_PRF_16_MHZ:
            agc_tune1_val  = 0x8870;
            drx_tune1a_val = 0x0087;
            tx_fctrl_val  |= (0x01 << DW_TXPRF) & DW_TXPRF_MASK;
            chan_ctrl_val |= (0x01 << DW_RXPRF) & DW_RXPRF_MASK;
            break;

        case DW_PRF_64_MHZ:
            agc_tune1_val  = 0x889B;
            drx_tune1a_val = 0x008D;
            tx_fctrl_val  |= (0x02 << DW_TXPRF) & DW_TXPRF_MASK;
            chan_ctrl_val |= (0x02 << DW_RXPRF) & DW_RXPRF_MASK;
            break;
    }

    // === Configure rx/tx channel
    chan_ctrl_val &= ~DW_TXCHAN_MASK;
    chan_ctrl_val &= ~DW_RXCHAN_MASK;

    uint8_t channel = (uint8_t)dw_conf->channel  & 0x1F;
    chan_ctrl_val |= channel;      // tx chan
    chan_ctrl_val |= channel << 5; // rx chan

    switch (dw_conf->channel)
    {
        case DW_CHANNEL_1:
            rf_rxctrl_val  = 0xD8;
            rf_txctrl_val  = 0x00005C40;
            tc_pgdelay_val = 0xC9;
            fs_pllcfg_val  = 0x09000407;
            fs_plltune_val = 0x1E;
            break;
        case DW_CHANNEL_2:
            rf_rxctrl_val  = 0xD8;
            rf_txctrl_val  = 0x00045CA0;
            tc_pgdelay_val = 0xC2;
            fs_pllcfg_val  = 0x08400508;
            fs_plltune_val = 0x26;
            break;
        case DW_CHANNEL_3:
            rf_rxctrl_val  = 0xD8;
            rf_txctrl_val  = 0x00086CC0;
            tc_pgdelay_val = 0xC5;
            fs_pllcfg_val  = 0x08401009;
            fs_plltune_val = 0x5E;
            break;
        case DW_CHANNEL_4:
            rf_rxctrl_val  = 0xBC;
            rf_txctrl_val  = 0x00045C80;
            tc_pgdelay_val = 0x95;
            fs_pllcfg_val  = 0x08400508;
            fs_plltune_val = 0x26;
            break;
        case DW_CHANNEL_5:
            rf_rxctrl_val  = 0xD8;
            rf_txctrl_val  = 0x001E3FE0;
            tc_pgdelay_val = 0xC0;
            fs_pllcfg_val  = 0x0800041D;
            fs_plltune_val = 0xA6;
            break;
        case DW_CHANNEL_7:
            rf_rxctrl_val  = 0xBC;
            rf_txctrl_val  = 0x001E7DE0;
            tc_pgdelay_val = 0x93;
            fs_pllcfg_val  = 0x0800041D;
            fs_plltune_val = 0xA6;
            break;
    }

    // === Configure Preamble length
    tx_fctrl_val  &= ~DW_TXPSR_MASK;
    tx_fctrl_val  &= ~DW_PE_MASK;
    if (dw_conf->preamble_length == DW_PREAMBLE_LENGTH_64)
    {
        drx_tune1b_val = 0x0010;
        drx_tune4h_val = 0x0010;
    }
    else if (dw_conf->preamble_length <= DW_PREAMBLE_LENGTH_1024)
    {
        drx_tune1b_val = 0x0020;
        drx_tune4h_val = 0x0028;
    }
    else if (dw_conf->preamble_length >  DW_PREAMBLE_LENGTH_1024)
    {
        drx_tune1b_val = 0x0064;
        drx_tune4h_val = 0x0028;
    }
    switch (dw_conf->preamble_length)
    {
        case DW_PREAMBLE_LENGTH_64:
            tx_fctrl_val   |= (0x01 << DW_TXPSR) & DW_TXPSR_MASK;
            tx_fctrl_val   |= (0x00 << DW_PE)    & DW_PE_MASK;
            break;
        case DW_PREAMBLE_LENGTH_128:
            tx_fctrl_val   |= (0x01 << DW_TXPSR) & DW_TXPSR_MASK;
            tx_fctrl_val   |= (0x01 << DW_PE)    & DW_PE_MASK;
            break;
        case DW_PREAMBLE_LENGTH_256:
            tx_fctrl_val   |= (0x01 << DW_TXPSR) & DW_TXPSR_MASK;
            tx_fctrl_val   |= (0x02 << DW_PE)    & DW_PE_MASK;
            break;
        case DW_PREAMBLE_LENGTH_512:
            tx_fctrl_val   |= (0x01 << DW_TXPSR) & DW_TXPSR_MASK;
            tx_fctrl_val   |= (0x03 << DW_PE)    & DW_PE_MASK;
            break;
        case DW_PREAMBLE_LENGTH_1024:
            tx_fctrl_val   |= (0x02 << DW_TXPSR) & DW_TXPSR_MASK;
            tx_fctrl_val   |= (0x00 << DW_PE)    & DW_PE_MASK;
            break;
        case DW_PREAMBLE_LENGTH_1536:
            tx_fctrl_val   |= (0x02 << DW_TXPSR) & DW_TXPSR_MASK;
            tx_fctrl_val   |= (0x01 << DW_PE)    & DW_PE_MASK;
            break;
        case DW_PREAMBLE_LENGTH_2048:
            tx_fctrl_val   |= (0x02 << DW_TXPSR) & DW_TXPSR_MASK;
            tx_fctrl_val   |= (0x02 << DW_PE)    & DW_PE_MASK;
            break;
        case DW_PREAMBLE_LENGTH_4096:
            tx_fctrl_val   |= (0x03 << DW_TXPSR) & DW_TXPSR_MASK;
            tx_fctrl_val   |= (0x00 << DW_PE)    & DW_PE_MASK;
            break;
    }

    // === Configure Preamble code
    chan_ctrl_val &= ~DW_TX_PCODE_MASK;
    chan_ctrl_val &= ~DW_RX_PCODE_MASK;

    uint8_t preamble_code = (uint8_t)dw_conf->preamble_code;
    chan_ctrl_val |= (preamble_code << DW_TX_PCODE) & DW_TX_PCODE_MASK;
    chan_ctrl_val |= (preamble_code << DW_RX_PCODE) & DW_RX_PCODE_MASK;

    // === Configure PAC size
    switch (dw_conf->pac_size)
    {
        case DW_PAC_SIZE_8:
            if      (dw_conf->prf == DW_PRF_16_MHZ) {drx_tune2_val = 0x311A002D;}
            else if (dw_conf->prf == DW_PRF_64_MHZ) {drx_tune2_val = 0x313B006B;}
            break;
        case DW_PAC_SIZE_16:
            if      (dw_conf->prf == DW_PRF_16_MHZ) {drx_tune2_val = 0x331A0052;}
            else if (dw_conf->prf == DW_PRF_64_MHZ) {drx_tune2_val = 0x333B00BE;}
            break;
        case DW_PAC_SIZE_32:
            if      (dw_conf->prf == DW_PRF_16_MHZ) {drx_tune2_val = 0x351A009A;}
            else if (dw_conf->prf == DW_PRF_64_MHZ) {drx_tune2_val = 0x353B015E;}
            break;
        case DW_PAC_SIZE_64:
            if      (dw_conf->prf == DW_PRF_16_MHZ) {drx_tune2_val = 0x371A011D;}
            else if (dw_conf->prf == DW_PRF_64_MHZ) {drx_tune2_val = 0x373B0296;}
            break;
    }

    // === Configure SFD
    // TODO: Implement user specified 
    chan_ctrl_val &= ~DW_DWSFD_MASK;

    if (dw_conf->sfd_type == DW_SFD_USER_SPECIFIED)
        DW_ERROR("dw_conf - SFD: User specified SFD not implemented");
    switch (dw_conf->sfd_type)
    {
        case DW_SFD_STANDARD:
            chan_ctrl_val &= ~((1 << DW_DWSFD) & DW_DWSFD_MASK);
            break;
        case DW_SFD_NON_STANDARD:
            chan_ctrl_val |= (1 << DW_DWSFD) & DW_DWSFD_MASK;
            break;
        case DW_SFD_USER_SPECIFIED:
            // Not implemented yet!
            break;
    }
    switch (dw_conf->data_rate)
    {
        case DW_DATA_RATE_110_KBPS:
            if      (dw_conf->sfd_type == DW_SFD_STANDARD  ) {drx_tune0b_val = 0x000A;}
            else if (dw_conf->sfd_type == DW_SFD_NON_STANDARD) {drx_tune0b_val = 0x0016;}
            break;
        case DW_DATA_RATE_850_KBPS:
            if      (dw_conf->sfd_type == DW_SFD_STANDARD  ) {drx_tune0b_val = 0x0001;}
            else if (dw_conf->sfd_type == DW_SFD_NON_STANDARD) {drx_tune0b_val = 0x0006;}
            break;
        case DW_DATA_RATE_6800_KBPS:
            if      (dw_conf->sfd_type == DW_SFD_STANDARD  ) {drx_tune0b_val = 0x0001;}
            else if (dw_conf->sfd_type == DW_SFD_NON_STANDARD) {drx_tune0b_val = 0x0002;}
            break;
    }

    // === Configure Data rate
    sys_cfg_val  &= ~DW_RXM110K_MASK;
    tx_fctrl_val &= ~DW_TXBR_MASK;
    switch (dw_conf->data_rate)
    {
        case DW_DATA_RATE_110_KBPS:
            sys_cfg_val  |= (1<<DW_RXM110K) & DW_RXM110K_MASK;
            tx_fctrl_val |= (0x00 << DW_TXBR) & DW_TXBR_MASK;
            break;
        case DW_DATA_RATE_850_KBPS:
            sys_cfg_val  &= ~((1<<DW_RXM110K) & DW_RXM110K_MASK);
            tx_fctrl_val |= (0x01 << DW_TXBR) & DW_TXBR_MASK;
            break;
        case DW_DATA_RATE_6800_KBPS:
            sys_cfg_val  &= ~((1<<DW_RXM110K) & DW_RXM110K_MASK);
            tx_fctrl_val |= (0x02 << DW_TXBR) & DW_TXBR_MASK;
            break;
    }

    // Commit configuration to device
    writeRegister(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&sys_cfg_val);
    writeRegister(DW_REG_TX_FCTRL, 4, (uint8_t *)&tx_fctrl_val);
    writeRegister(DW_REG_CHAN_CTRL, DW_LEN_CHAN_CTRL, (uint8_t *)&chan_ctrl_val);
    writeSubregister(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE1, DW_SUBLEN_AGC_TUNE1, (uint8_t *)&agc_tune1_val);
    writeSubregister(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE2, DW_SUBLEN_AGC_TUNE2, (uint8_t *)&agc_tune2_val);
    writeSubregister(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE3, DW_SUBLEN_AGC_TUNE3, (uint8_t *)&agc_tune3_val);
    writeSubregister(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE0b, DW_SUBLEN_DRX_TUNE0b, (uint8_t *)&drx_tune0b_val);
    writeSubregister(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1a, DW_SUBLEN_DRX_TUNE1a, (uint8_t *)&drx_tune1a_val);
    writeSubregister(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1b, DW_SUBLEN_DRX_TUNE1b, (uint8_t *)&drx_tune1b_val);
    writeSubregister(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE2 , DW_SUBLEN_DRX_TUNE2 , (uint8_t *)&drx_tune2_val);
    writeSubregister(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE4h, DW_SUBLEN_DRX_TUNE4h, (uint8_t *)&drx_tune4h_val);
    writeSubregister(DW_REG_RF_CONF , DW_SUBREG_RF_RXCTRLH, DW_SUBLEN_RF_RXCTRLH, (uint8_t *)&rf_rxctrl_val);
    writeSubregister(DW_REG_RF_CONF , DW_SUBREG_RF_TXCTRL , DW_SUBLEN_RF_TXCTRL , (uint8_t *)&rf_txctrl_val);
    writeSubregister(DW_REG_TX_CAL  , DW_SUBREG_TC_PGDELAY, DW_SUBLEN_TC_PGDELAY, (uint8_t *)&tc_pgdelay_val);
    writeSubregister(DW_REG_FS_CTRL , DW_SUBREG_FS_PLLCFG , DW_SUBLEN_FS_PLLCFG , (uint8_t *)&fs_pllcfg_val);
    writeSubregister(DW_REG_FS_CTRL , DW_SUBREG_FS_PLLTUNE, DW_SUBLEN_FS_PLLTUNE, (uint8_t *)&fs_plltune_val);
}

void DW1000::ConfigureRX(dw1000_rx_conf_t * rx_conf)
{
    // Timeout
    SetRXTimeout(rx_conf->timeout);
    if (rx_conf->timeout)
    {
        EnableRXTimeout();
    }
    else
    {
        DisableRXTimeout();
    }

    // Delayed reception
    if (rx_conf->is_delayed)
    {
        SetDXTimestamp(rx_conf->dx_timestamp);

        uint32_t sys_ctrl_val;
        sys_ctrl_val  = readRegister32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
        sys_ctrl_val |= DW_RXDLYE_MASK;
        writeRegister(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl_val);
    }
}

void DW1000::ConfigureTX(dw1000_tx_conf_t * tx_conf)
{
    // TODO: Handling of long data frames (length > 128 or whatever.)
    // TODO: Cache data..?
    // TODO: Should check m_configuration for FCS enable and add the 2 conditionally.
    uint32_t data_len = tx_conf->data_len;
    data_len += 2; // The +2 is for fcs
    uint32_t tx_frame_control_val = readRegister32(DW_REG_TX_FCTRL, 4);
    tx_frame_control_val |= (data_len << DW_TXLEN) & DW_TXLEN_MASK;
    writeRegister(DW_REG_TX_FCTRL, 4, (uint8_t *)&tx_frame_control_val);

    // Delayed transmission
    if (tx_conf->is_delayed)
    {
	SetDXTimestamp(tx_conf->dx_timestamp);

	uint32_t ctrl_reg_val;
	ctrl_reg_val  = readRegister32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
	ctrl_reg_val |= DW_TXDLYS_MASK;
	writeRegister(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&ctrl_reg_val);
    }
}

void DW1000::Receive(dw1000_tranceive_t receive_type)
{
    // TODO: Fast receive / transmit

    if ( m_state == DW_STATE_RECEIVING
	    || m_state == DW_STATE_TRANSMITTING)
    {
	//printf("dw1000 error: already using antenna.\n");
	return;
    }

    //  Start reception
    initRX();

    const uint32_t wait_mask_lo = DW_RXDFR_MASK
	| DW_RXPHE_MASK
	| DW_RXRFTO_MASK
	| DW_RXPTO_MASK
	| DW_RXSFDTO_MASK
	| DW_RXRFSL_MASK;
    uint64_t status_reg;
    uint64_t has_received;
    switch (receive_type)
    {
	case DW_TRANCEIVE_ASYNC:
            extChannelEnable(&EXTD1, 4);
            break;

	case DW_TRANCEIVE_SYNC:
	    do
	    {
		// Wait until data received
		status_reg   = readRegister64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
		has_received  = status_reg & wait_mask_lo;
		has_received |= (status_reg>>31>>1) & DW_RXPREJ_MASK;
	    }
	    while(!has_received);

	    ProcessRXBuffer();
	    break;
    }
    return;
}

void DW1000::Transmit(uint8_t * p_data, uint32_t data_len, dw1000_tranceive_t transmit_type)
{
    // TODO: Functionality can be separated so that this function only triggers 
    // a transmission, akin to how reception works. You would then have a 
    // singluar dw_transmit and two upload functions, dw_upload_data and 
    // dw_upload_multiple_data.
    if ( m_state == DW_STATE_RECEIVING
        || m_state == DW_STATE_TRANSMITTING)
    {
        //printf("dw1000 error: already using antenna.\n");
        return;
    }

    // Place data on DW1000
    if (data_len > 0 && data_len < 1024)
    {
        // Copy data to dw1000
        writeRegister(DW_REG_TX_BUFFER, data_len, p_data);
    }

    // Initiate transmission
    initTX();

    // Handle transmission complete
    uint64_t status_reg;
    uint64_t is_sending;
    switch(transmit_type)
    {
        case DW_TRANCEIVE_ASYNC:
            extChannelEnable(&EXTD1, 4);
            break;

        case DW_TRANCEIVE_SYNC:
            do
            {
                // Wait until data sent
                status_reg = readRegister64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
                is_sending = !(status_reg & (DW_TXFRS_MASK));
            }
            while(is_sending);
            ClearPendingInterrupt(DW_TXFRS_MASK);

            m_state = DW_STATE_IDLE;
            break;
    }
    return;
}

void DW1000::TransmitMultipleData(uint8_t  ** pp_data,
                            uint32_t *  p_data_len,
                            uint32_t    length,
                            dw1000_tranceive_t transmit_type)
{
    if (m_state == DW_STATE_RECEIVING
        || m_state == DW_STATE_TRANSMITTING)
    {
        //printf("dw1000 error: already using antenna.\n");
        return;
    }

    // Copy data to dw1000
    writeRegisterMultipleData(DW_REG_TX_BUFFER, DW_LEN_TX_BUFFER, pp_data, p_data_len, length);

    Transmit(NULL, 0, transmit_type);
}


void DW1000::EnableADC()
{
    uint32_t pmsc_val = readSubregister32(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0);
    pmsc_val |= DW_ADCCE_MASK;
    writeSubregister(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, (uint8_t *)&pmsc_val);
}

void DW1000::DisableADC()
{
    uint32_t pmsc_val = readSubregister32(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0);
    pmsc_val &= ~DW_ADCCE_MASK;
    writeSubregister(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, (uint8_t *)&pmsc_val);
}

void DW1000::adcSample()
{
    // Make sure adc clock is enabled
    EnableADC();

    // Undocumented procedure to take a sample
    uint8_t val;
    val = 0x80;
    writeSubregister(0x28, 0x11, 1, &val);
    val = 0x0A;
    writeSubregister(0x28, 0x12, 1, &val);
    val = 0x0F;
    writeSubregister(0x28, 0x12, 1, &val);

    // Take sample.
    // Wait for reading to complete.
    // Disable sampling
    uint8_t tc_sarc_val;
    tc_sarc_val = DW_SAR_CTRL_MASK;
    writeSubregister(DW_REG_TX_CAL, DW_SUBREG_TC_SARC, 1, &tc_sarc_val);
    chThdSleepMicroseconds(200);
    tc_sarc_val = 0;
    writeSubregister(DW_REG_TX_CAL, DW_SUBREG_TC_SARC, 1, &tc_sarc_val);
}

float DW1000::GetTemperature(dw_adc_src_t temp_source)
{
    // Get calibration data from otp. Tmeas @ 23 degrees resides in addr 0x9.
    uint32_t otp_temp = readOTP32(0x009) & 0xFF;
    uint32_t read_temp;

    // Load to CPU sample
    switch (temp_source)
    {
        case DW_ADC_SRC_LATEST:
            adcSample();
            read_temp   = readSubregister32(DW_REG_TX_CAL, DW_SUBREG_TC_SARL, DW_SUBLEN_TC_SARL);
            read_temp  &= DW_SAR_LTEMP_MASK;
            read_temp >>= DW_SAR_LTEMP;
            break;

        case DW_ADC_SRC_WAKEUP:
            read_temp   = readSubregister32(DW_REG_TX_CAL, DW_SUBREG_TC_SARW, DW_SUBLEN_TC_SARW);
            read_temp  &= DW_SAR_WTEMP_MASK;
            read_temp >>= DW_SAR_WTEMP;
            break;
    }

    return ((float)read_temp - (float)otp_temp)*1.14f + 23.f;
}

float DW1000::GetVoltage(dw_adc_src_t voltage_source)
{
    // Get calibration data from otp. Vmeas @ 3.3V residies in addr 0x8.
    uint32_t otp_voltage = readOTP32(0x008) & 0xFF;
    uint32_t read_voltage;
    
    switch (voltage_source)
    {
        case DW_ADC_SRC_LATEST:
            adcSample();
            read_voltage   = readSubregister32(DW_REG_TX_CAL, DW_SUBREG_TC_SARL, DW_SUBLEN_TC_SARL);
            read_voltage  &= DW_SAR_LVBAT_MASK;
            read_voltage >>= DW_SAR_LVBAT;
            break;

        case DW_ADC_SRC_WAKEUP:
            read_voltage   = readSubregister32(DW_REG_TX_CAL, DW_SUBREG_TC_SARW, DW_SUBLEN_TC_SARW);
            read_voltage  &= DW_SAR_WVBAT_MASK;
            read_voltage >>= DW_SAR_WVBAT;
            break;
    }

    return ((float)read_voltage - (float)otp_voltage)/173.f + 3.3f;
}

float DW1000::GetNoiseLevel()
{
    return (float)((readRegister64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL) & (DW_STD_NOISE_MASK)) >> DW_STD_NOISE);
}

float DW1000::GetFPAmplitude()
{
    return (float)((readRegister64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL) & (DW_FP_AMPL2_MASK)) >> DW_FP_AMPL2);
}

float DW1000::GetRXPower()
{
    uint64_t rx_fqual_val = readRegister64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL);
    uint32_t rx_finfo_val = readRegister32(DW_REG_RX_FINFO, DW_LEN_RX_FINFO);
    float c = (rx_fqual_val & (DW_CIR_PWR_MASK)) >> DW_CIR_PWR;
    float n = (rx_finfo_val & (DW_RXPACC_MASK)) >> DW_RXPACC;
    float a;
    float rx_power;

    switch (m_conf.prf)
    {
        case DW_PRF_16_MHZ: a = 115.72; break;
        case DW_PRF_64_MHZ: a = 121.74; break;
    }

    // If you have access to logarithm...
    //rx_power = 10.f * log10((float)(c * powf(2,17)) / (float)(n*n)) - a;
    // This value needs external processing 
    rx_power = (float)(c * powf(2,17)) / (float)(n*n);
    return rx_power;
}

float DW1000::GetFPPower()
{
    uint64_t rx_fqual_val = readRegister64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL);
    uint32_t rx_finfo_val = readRegister32(DW_REG_RX_FINFO, DW_LEN_RX_FINFO);
    // Special way to read fp_ampl1, not following ordinary definitions.
    uint32_t fp_ampl1_val = readSubregister32(DW_REG_RX_TIME, 0x7, 0x2);

    float fp_ampl1 = (float)fp_ampl1_val;
    float fp_ampl2 = (float)((rx_fqual_val & (DW_FP_AMPL2_MASK)) >> DW_FP_AMPL2);
    float fp_ampl3 = (float)((rx_fqual_val & (DW_FP_AMPL3_MASK)) >> DW_FP_AMPL3);
    float n = (float)((rx_finfo_val & (DW_RXPACC_MASK)) >> DW_RXPACC);
    float a;
    float fp_power;

    switch (m_conf.prf)
    {
        case DW_PRF_16_MHZ: a = 115.72; break;
        case DW_PRF_64_MHZ: a = 121.74; break;
    }

    float fp_ampl1_2 = (float) (fp_ampl1 * fp_ampl1);
    float fp_ampl2_2 = (float) (fp_ampl2 * fp_ampl2);
    float fp_ampl3_2 = (float) (fp_ampl3 * fp_ampl3);
    float n_2 = (float) (n * n);

    // Use this if you have math lib.
    //fp_power = 10 * log10((fp_ampl1_2+fp_ampl2_2+fp_ampl3_2)/(n_2)) - a;
    //Else, we compute logarithm externally.
    fp_power = (fp_ampl1_2+fp_ampl2_2+fp_ampl3_2)/(n_2);
    return fp_power;
}

void DW1000::ProcessRXBuffer()
{
    uint32_t * status_reg;
    uint64_t status_reg_64;
    uint32_t isError;
    uint32_t rx_frame_info_reg;
    uint32_t rx_len;

    // Get error status
    // TODO: Break out into separate function.
    const uint32_t error_mask_lo = DW_RXPHE_MASK | DW_RXRFTO_MASK | DW_RXPTO_MASK | DW_RXSFDTO_MASK | DW_RXRFSL_MASK;
    const uint32_t error_mask_hi = DW_RXPREJ_MASK;
    status_reg_64 = readRegister64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
    status_reg = (uint32_t *)&status_reg_64;
    isError  = *(status_reg+0) & error_mask_lo;
    isError |= *(status_reg+1) & error_mask_hi;

    if (isError) m_state = DW_STATE_ERROR;
    else         m_state = DW_STATE_IDLE;

    // Get length of received frame
    rx_frame_info_reg = readRegister32(DW_REG_RX_FINFO, DW_LEN_RX_FINFO);
    rx_len = rx_frame_info_reg & (DW_RXFLEN_MASK|DW_RXFLE_MASK);
    rx_len = (rx_len < DW_RX_BUFFER_MAX_LEN) ? (rx_len) : (DW_RX_BUFFER_MAX_LEN);

    // Store rx data in global variable rxBuffer
    m_receive_buffer_len = rx_len;
    
    if (rx_len > 0)
    {
        // Store rx data in global variable rxBuffer
        readRegister(DW_REG_RX_BUFFER, rx_len, m_p_receive_buffer);
    }

    // Cleanup
    ClearPendingInterrupt(DW_RXDFR_MASK | DW_RXRFTO_MASK);
}

uint8_t* DW1000::GetRXBuffer(uint32_t *len)
{
    *len = m_receive_buffer_len;
    return m_p_receive_buffer;
}

uint8_t DW1000::GetSequenceNumber()
{
    static uint8_t seq_no = 0;
    return seq_no++;
}

void DW1000::SetRXTimeout(uint16_t us)
{
    writeRegister(DW_REG_RX_FWTO, DW_LEN_RX_FWTO, (uint8_t *)&us);
}

uint16_t DW1000::GetRXTimeout()
{
    return readRegister32(DW_REG_RX_FWTO, DW_LEN_RX_FWTO);
}

void DW1000::EnableRXTimeout()
{
    uint32_t cfgReg;
    cfgReg  = readRegister32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
    cfgReg |= DW_RXWTOE_MASK;
    writeRegister(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&cfgReg);
}

void DW1000::DisableRXTimeout()
{
    uint32_t cfgReg;
    cfgReg  = readRegister32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
    cfgReg &= ~DW_RXWTOE_MASK;
    writeRegister(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&cfgReg);
    //printf("CFG: %x\n", cfgReg);
}

uint64_t DW1000::GetRXTimestamp()
{
    return readRegister64(DW_REG_RX_TIME, 8) & 0x000000FFFFFFFFFFULL;
}

uint64_t DW1000::GetTXTimestamp()
{
    return readRegister64(DW_REG_TX_TSTAMP, 8) & 0x000000FFFFFFFFFFULL;
}

void DW1000::SetAntennaDelay(uint16_t delay)
{
    writeSubregister(DW_REG_LDE_IF, DW_SUBREG_LDE_RXANTD, DW_SUBLEN_LDE_RXANTD, (uint8_t *)&delay);
    writeRegister(DW_REG_TX_ANTD, DW_LEN_TX_ANTD, (uint8_t *)&delay);
}

uint16_t DW1000::GetAntennaDelay()
{
    return readRegister32(DW_REG_TX_ANTD, DW_LEN_TX_ANTD);
}

void DW1000::SetDXTimestamp(uint64_t timestamp)
{
    writeRegister(DW_REG_DX_TIME, DW_LEN_DX_TIME, (uint8_t *)&timestamp);
}

uint64_t DW1000::GetDXTimestamp()
{
    return readRegister64(DW_REG_DX_TIME, DW_LEN_DX_TIME) & 0x000000FFFFFFFFFFULL;
}

dw1000_state_t DW1000::GetState()
{
    return m_state;
}

uint32_t DW1000::GetDeviceID()
{
    static uint32_t device_id = 0x00;
    if (device_id == 0x00)
    {
        device_id = readRegister32(DW_REG_DEV_ID, DW_LEN_DEV_ID);
    }
    return device_id;
}

uint64_t DW1000::GetDeviceTime()
{
    return readRegister64(DW_REG_SYS_TIME, DW_LEN_SYS_TIME);
}

void DW1000::ClearPendingInterrupt(uint64_t mask)
{
    writeRegister(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *)&mask);
}

void DW1000::EnableInterrupt(uint32_t mask)
{
    writeRegister(DW_REG_SYS_MASK, DW_LEN_SYS_MASK, (uint8_t *)&mask);
}

void DW1000::trxOff()
{
    uint32_t sys_ctrl_val = readRegister32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
    sys_ctrl_val |= (1<<DW_TRXOFF) & DW_TRXOFF_MASK;
    writeRegister(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl_val);
}


void DW1000::initRX()
{
    m_state = DW_STATE_RECEIVING;
    // Enable antenna
    uint32_t sys_ctrl_val = readRegister32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
    sys_ctrl_val |= (1<<DW_RXENAB) & DW_RXENAB_MASK;
    writeRegister(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl_val);
}

void DW1000::initTX()
{
    m_state = DW_STATE_TRANSMITTING;

    // Start transmission
    uint32_t ctrl_reg_val;
    ctrl_reg_val  = readRegister32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
    ctrl_reg_val |= DW_TXSTRT_MASK;
    writeRegister(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&ctrl_reg_val);
}

void DW1000::writeRegister(uint32_t  reg_addr, uint32_t  reg_len, uint8_t * p_data)
{
    uint8_t txbuf[1];
    uint8_t rxbuf[reg_len];
    txbuf[0] = 0x80 | (reg_addr & 0x3F);
    spiAcquireBus(&SPI_DEV);
    spiSelect(&SPI_DEV);
    spiExchange(&SPI_DEV, 1, txbuf, rxbuf);
    spiExchange(&SPI_DEV, reg_len, p_data, rxbuf);
    spiUnselect(&SPI_DEV);
    spiReleaseBus(&SPI_DEV); 
}

void DW1000::writeSubregister(uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len, uint8_t *p_data)
{
    // Check if 3-octet header is requried or if 2 will do
    uint32_t isThreeOctet = (subreg_addr > 0x7F);
    
    // Prepare instruction
    uint32_t instruction = 0x0;
    instruction = (0xC0 | (reg_addr & 0x3F));
    instruction = (instruction << 8) | (subreg_addr&0x7F) | (isThreeOctet<<7);
    instruction = (instruction << 8) | (subreg_addr&0x7F80 >> 7);
    
    // Write instruction
    
    uint8_t rxbuf[subreg_len];
    uint8_t * pInstr = (uint8_t *)(&instruction) + 2;
    spiAcquireBus(&SPI_DEV);
    spiSelect(&SPI_DEV);
    spiExchange(&SPI_DEV, 1, pInstr--, rxbuf);
    spiExchange(&SPI_DEV, 1, pInstr--, rxbuf);
    if (isThreeOctet) {
        spiExchange(&SPI_DEV, 1, pInstr--, rxbuf);
    }
    
    // Write data
    spiExchange(&SPI_DEV, subreg_len, p_data, rxbuf);
    spiUnselect(&SPI_DEV);
    spiReleaseBus(&SPI_DEV); 
}

void DW1000::writeRegisterMultipleData( uint32_t    reg_addr, 
                            uint32_t    reg_len, 
                            uint8_t  ** pp_data, 
                            uint32_t *  p_data_len, 
                            uint32_t    len_pp_data )
{
    // Get total length of data.
    uint32_t data_len = 0;
    uint32_t length = len_pp_data;
    while (length-- ) {data_len += *p_data_len++;}
    p_data_len-=len_pp_data;

    // Bounds check
    if (data_len > reg_len) {return;}

    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    // Transfer data to dw1000
    txbuf[0] = 0x80 | (reg_addr & 0x3F);
    spiAcquireBus(&SPI_DEV);
    spiSelect(&SPI_DEV);
    spiExchange(&SPI_DEV, 1, txbuf, rxbuf);

    uint32_t i_transaction;
    for (i_transaction = 0; i_transaction < len_pp_data-1; ++i_transaction)
    {
        uint8_t rxbuf_temp[*p_data_len];
        spiExchange(&SPI_DEV, *p_data_len++, *pp_data++, rxbuf_temp);
    }
    uint8_t rxbuf_temp[*p_data_len];
    spiExchange(&SPI_DEV, *p_data_len, *pp_data, rxbuf_temp);
    spiUnselect(&SPI_DEV);
    spiReleaseBus(&SPI_DEV); 
}

void DW1000::readRegister(uint32_t reg_addr, uint32_t reg_len, uint8_t * p_data)
{
    uint8_t txbuf[reg_len];
    uint8_t rxbuf[1];
    txbuf[0] = 0x00 | (reg_addr & 0x3F);
    spiAcquireBus(&SPI_DEV);
    spiSelect(&SPI_DEV);
    spiExchange(&SPI_DEV, 1, txbuf, rxbuf);
    txbuf[0] = 0;
    spiExchange(&SPI_DEV, reg_len, txbuf, p_data);
    spiUnselect(&SPI_DEV);
    spiReleaseBus(&SPI_DEV); 
}

uint32_t DW1000::readRegister32(uint32_t regAddr, uint32_t regLen)
{
    uint32_t result = 0;
    readRegister(regAddr, regLen, (uint8_t *)&result);
    return result;
}

uint64_t DW1000::readRegister64(uint32_t regAddr, uint32_t regLen)
{
    uint64_t result = 0;
    readRegister(regAddr, regLen, (uint8_t *)&result);
    return result;
}

void DW1000::readSubregister(uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len, uint8_t * p_data)
{
    uint32_t is_three_octet = (subreg_addr > 0x7F);

    // Compose instruction
    uint32_t instruction = 0x40 | (reg_addr & 0x3F);
    instruction = (instruction << 8) | ((subreg_addr & 0x7F )    ) | (is_three_octet<<7);
    instruction = (instruction << 8) | ((subreg_addr & 0x7F80) >> 7);

    // Write instruction
    // Data is written as --001122 
    uint8_t rxbuf[1];
    uint8_t txbuf[subreg_len];
    uint8_t * pInstr = (uint8_t *)(&instruction) + 2;
    spiAcquireBus(&SPI_DEV);
    spiSelect(&SPI_DEV);
    spiExchange(&SPI_DEV, 1, pInstr--, rxbuf);
    spiExchange(&SPI_DEV, 1, pInstr--, rxbuf);
    if (is_three_octet)
    {
        spiExchange(&SPI_DEV, 1, pInstr, rxbuf);
    }

    // Read data
    spiExchange(&SPI_DEV, subreg_len, txbuf, p_data);
    spiUnselect(&SPI_DEV);
    spiReleaseBus(&SPI_DEV); 
}

uint32_t DW1000::readSubregister32(uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len)
{
    uint32_t result = 0U;
    readSubregister(reg_addr, subreg_addr, subreg_len, (uint8_t *)&result);
    return result;
}

uint64_t DW1000::readSubregister64(uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len)
{
    uint64_t result = 0ULL;
    readSubregister(reg_addr, subreg_addr, subreg_len, (uint8_t *)&result);
    return result;
}

uint32_t DW1000::readOTP32(uint16_t otp_addr)
{
    static const uint8_t cmd[] = {DW_OTPRDEN_MASK||DW_OTPREAD_MASK, // Enable manual read
                                  DW_OTPREAD_MASK,                  // Do the acutal read
                                  0x00                              // Reset otp_ctrl
                                 };

    uint32_t read_data = 0;
    writeSubregister(DW_REG_OTP_IF  , DW_SUBREG_OTP_ADDR, DW_SUBLEN_OTP_ADDR, (uint8_t *)&otp_addr);
    writeSubregister(DW_REG_OTP_IF  , DW_SUBREG_OTP_CTRL, 1, (uint8_t *)&cmd[0]);
    writeSubregister(DW_REG_OTP_IF  , DW_SUBREG_OTP_CTRL, 1, (uint8_t *)&cmd[1]);
    read_data = readSubregister32(DW_REG_OTP_IF, DW_SUBREG_OTP_RDAT, DW_SUBLEN_OTP_RDAT);
    writeSubregister(DW_REG_OTP_IF  , DW_SUBREG_OTP_CTRL, 1, (uint8_t *)&cmd[2]);

    return read_data;
}
