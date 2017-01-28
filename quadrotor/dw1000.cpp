#include "dw1000.h"
#include "hw_conf.h"

#define DW_MS_TO_DEVICE_TIME_SCALE 62.6566416e6f

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

    palSetPadMode(SCK_GPIO, SCK_PIN, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(MISO_GPIO, MISO_PIN, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(MOSI_GPIO, MOSI_PIN, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(CS_GPIO, CS_PIN, PAL_MODE_OUTPUT_PUSHPULL |
            PAL_STM32_OSPEED_HIGHEST);
    spiAcquireBus(&SPI_DEV);
    spiStart(&SPI_DEV, &spicfg);
    spiReleaseBus(&SPI_DEV);

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
            if      (dw_conf->sfd_type == DW_SFD_STANDARD    ) {drx_tune0b_val = 0x000A;}
            else if (dw_conf->sfd_type == DW_SFD_NON_STANDARD) {drx_tune0b_val = 0x0016;}
            break;
        case DW_DATA_RATE_850_KBPS:
            if      (dw_conf->sfd_type == DW_SFD_STANDARD    ) {drx_tune0b_val = 0x0001;}
            else if (dw_conf->sfd_type == DW_SFD_NON_STANDARD) {drx_tune0b_val = 0x0006;}
            break;
        case DW_DATA_RATE_6800_KBPS:
            if      (dw_conf->sfd_type == DW_SFD_STANDARD    ) {drx_tune0b_val = 0x0001;}
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
    writeSubregister(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE2 , DW_SUBLEN_DRX_TUNE2 , (uint8_t *)&drx_tune2_val );
    writeSubregister(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE4h, DW_SUBLEN_DRX_TUNE4h, (uint8_t *)&drx_tune4h_val);
    writeSubregister(DW_REG_RF_CONF , DW_SUBREG_RF_RXCTRLH, DW_SUBLEN_RF_RXCTRLH, (uint8_t *)&rf_rxctrl_val );
    writeSubregister(DW_REG_RF_CONF , DW_SUBREG_RF_TXCTRL , DW_SUBLEN_RF_TXCTRL , (uint8_t *)&rf_txctrl_val );
    writeSubregister(DW_REG_TX_CAL  , DW_SUBREG_TC_PGDELAY, DW_SUBLEN_TC_PGDELAY, (uint8_t *)&tc_pgdelay_val);
    writeSubregister(DW_REG_FS_CTRL , DW_SUBREG_FS_PLLCFG , DW_SUBLEN_FS_PLLCFG , (uint8_t *)&fs_pllcfg_val );
    writeSubregister(DW_REG_FS_CTRL , DW_SUBREG_FS_PLLTUNE, DW_SUBLEN_FS_PLLTUNE, (uint8_t *)&fs_plltune_val);
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
    return readRegister64( DW_REG_SYS_TIME, DW_LEN_SYS_TIME );
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
    instruction = (instruction << 8) | (subreg_addr&0x7F       ) | (isThreeOctet<<7);
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
