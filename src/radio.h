/*!
 * \file      radio.h
 *
 * \brief     Radio driver API definition
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __RADIO_H__
#define __RADIO_H__

#include <stdint.h>
//#include "sx1276.h"
//#include <stdbool.h>

#define RF_MID_BAND_THRESH                          525000000

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

#define RX_BUFFER_SIZE                              256

// RTC-Board.h
#ifdef __cplusplus
extern "C"{
#endif

extern uint32_t TimeSwitch;

#ifdef __cplusplus
} // extern "C"
#endif

/*!
 * ============================================================================
 * SX1276 Internal registers Address
 * ============================================================================
 */
#define REG_PADAC                0x4D

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

/*!
 * ============================================================================
 * SX1276 LoRa bits control definition
 * ============================================================================
 */
/*!
 * RegOpMode
 */
#define RFLR_OPMODE_LONGRANGEMODE_MASK              0x7F
#define RFLR_OPMODE_LONGRANGEMODE_OFF               0x00 // Default
#define RFLR_OPMODE_LONGRANGEMODE_ON                0x80

#define RFLR_OPMODE_ACCESSSHAREDREG_MASK            0xBF
#define RFLR_OPMODE_ACCESSSHAREDREG_ENABLE          0x40
#define RFLR_OPMODE_ACCESSSHAREDREG_DISABLE         0x00 // Default

#define RFLR_OPMODE_FREQMODE_ACCESS_MASK            0xF7
#define RFLR_OPMODE_FREQMODE_ACCESS_LF              0x08 // Default
#define RFLR_OPMODE_FREQMODE_ACCESS_HF              0x00

#define RFLR_OPMODE_MASK                            0xF8
#define RFLR_OPMODE_SLEEP                           0x00
#define RFLR_OPMODE_STANDBY                         0x01 // Default
#define RFLR_OPMODE_SYNTHESIZER_TX                  0x02
#define RFLR_OPMODE_TRANSMITTER                     0x03
#define RFLR_OPMODE_SYNTHESIZER_RX                  0x04
#define RFLR_OPMODE_RECEIVER                        0x05
// LoRa specific modes
#define RFLR_OPMODE_RECEIVER_SINGLE                 0x06
#define RFLR_OPMODE_CAD                             0x07

/*!
 * RegFrf (MHz)
 */
#define RFLR_FRFMSB_434_MHZ                         0x6C // Default
#define RFLR_FRFMID_434_MHZ                         0x80 // Default
#define RFLR_FRFLSB_434_MHZ                         0x00 // Default

/*!
 * RegPaConfig
 */
#define RFLR_PACONFIG_PASELECT_MASK                 0x7F
#define RFLR_PACONFIG_PASELECT_PABOOST              0x80
#define RFLR_PACONFIG_PASELECT_RFO                  0x00 // Default

// PA config
//#define PA_BOOST                 0x80
//#define RFO                      0x70

// IRQ masks
#if defined ( WIFI_LoRa_32_V2 )
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#endif

/*!
 * RegInvertIQ
 */
#define RFLR_INVERTIQ_RX_MASK                       0xBF
#define RFLR_INVERTIQ_RX_OFF                        0x00
#define RFLR_INVERTIQ_RX_ON                         0x40
#define RFLR_INVERTIQ_TX_MASK                       0xFE
#define RFLR_INVERTIQ_TX_OFF                        0x01
#define RFLR_INVERTIQ_TX_ON                         0x00


/*!
 * RegInvertIQ2
 */
#define RFLR_INVERTIQ2_ON                           0x19
#define RFLR_INVERTIQ2_OFF                          0x1D



#define MAX_PKT_LENGTH           255


// ####################

#define REG_LR_FIFO                                 0x00
// Common settings
#define REG_LR_OPMODE                               0x01
#define REG_LR_FRFMSB                               0x06
#define REG_LR_FRFMID                               0x07
#define REG_LR_FRFLSB                               0x08
// Tx settings
#define REG_LR_PACONFIG                             0x09
#define REG_LR_PARAMP                               0x0A
#define REG_LR_OCP                                  0x0B
// Rx settings
#define REG_LR_LNA                                  0x0C
// LoRa registers
#define REG_LR_FIFOADDRPTR                          0x0D
#define REG_LR_FIFOTXBASEADDR                       0x0E
#define REG_LR_FIFORXBASEADDR                       0x0F
#define REG_LR_FIFORXCURRENTADDR                    0x10
#define REG_LR_IRQFLAGSMASK                         0x11
#define REG_LR_IRQFLAGS                             0x12
#define REG_LR_RXNBBYTES                            0x13
#define REG_LR_RXHEADERCNTVALUEMSB                  0x14
#define REG_LR_RXHEADERCNTVALUELSB                  0x15
#define REG_LR_RXPACKETCNTVALUEMSB                  0x16
#define REG_LR_RXPACKETCNTVALUELSB                  0x17
#define REG_LR_MODEMSTAT                            0x18
#define REG_LR_PKTSNRVALUE                          0x19
#define REG_LR_PKTRSSIVALUE                         0x1A
#define REG_LR_RSSIVALUE                            0x1B
#define REG_LR_HOPCHANNEL                           0x1C
#define REG_LR_MODEMCONFIG1                         0x1D
#define REG_LR_MODEMCONFIG2                         0x1E
#define REG_LR_SYMBTIMEOUTLSB                       0x1F
#define REG_LR_PREAMBLEMSB                          0x20
#define REG_LR_PREAMBLELSB                          0x21
#define REG_LR_PAYLOADLENGTH                        0x22
#define REG_LR_PAYLOADMAXLENGTH                     0x23
#define REG_LR_HOPPERIOD                            0x24
#define REG_LR_FIFORXBYTEADDR                       0x25
#define REG_LR_MODEMCONFIG3                         0x26
#define REG_LR_FEIMSB                               0x28
#define REG_LR_FEIMID                               0x29
#define REG_LR_FEILSB                               0x2A
#define REG_LR_RSSIWIDEBAND                         0x2C
#define REG_LR_TEST2F                               0x2F
#define REG_LR_TEST30                               0x30
#define REG_LR_DETECTOPTIMIZE                       0x31
#define REG_LR_INVERTIQ                             0x33
#define REG_LR_TEST36                               0x36
#define REG_LR_DETECTIONTHRESHOLD                   0x37
#define REG_LR_SYNCWORD                             0x39
#define REG_LR_TEST3A                               0x3A
#define REG_LR_INVERTIQ2                            0x3B

// end of documented register in datasheet
// I/O settings
#define REG_LR_DIOMAPPING1                          0x40
#define REG_LR_DIOMAPPING2                          0x41
// Version
#define REG_LR_VERSION                              0x42
// Additional settings
#define REG_LR_PLLHOP                               0x44
#define REG_LR_TCXO                                 0x4B
#define REG_LR_PADAC                                0x4D
#define REG_LR_FORMERTEMP                           0x5B
#define REG_LR_BITRATEFRAC                          0x5D
#define REG_LR_AGCREF                               0x61
#define REG_LR_AGCTHRESH1                           0x62
#define REG_LR_AGCTHRESH2                           0x63
#define REG_LR_AGCTHRESH3                           0x64
#define REG_LR_PLL                                  0x70


/*!
 * RegImageCal
 */
#define RF_IMAGECAL_AUTOIMAGECAL_MASK               0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON                 0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF                0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK                   0xBF
#define RF_IMAGECAL_IMAGECAL_START                  0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING                0x20
#define RF_IMAGECAL_IMAGECAL_DONE                   0x00  // Default

#define RF_IMAGECAL_TEMPCHANGE_HIGHER               0x08
#define RF_IMAGECAL_TEMPCHANGE_LOWER                0x00

#define RF_IMAGECAL_TEMPTHRESHOLD_MASK              0xF9
#define RF_IMAGECAL_TEMPTHRESHOLD_05                0x00
#define RF_IMAGECAL_TEMPTHRESHOLD_10                0x02  // Default
#define RF_IMAGECAL_TEMPTHRESHOLD_15                0x04
#define RF_IMAGECAL_TEMPTHRESHOLD_20                0x06

#define RF_IMAGECAL_TEMPMONITOR_MASK                0xFE
#define RF_IMAGECAL_TEMPMONITOR_ON                  0x00 // Default
#define RF_IMAGECAL_TEMPMONITOR_OFF                 0x01

// Status
#define REG_IRQFLAGS1                               0x3E
#define REG_IRQFLAGS2                               0x3F
#define RF_IRQFLAGS2_CRCOK                          0x02

/*!
 * RegIrqFlagsMask
 */
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK                0x80
#define RFLR_IRQFLAGS_RXDONE_MASK                   0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK          0x20
#define RFLR_IRQFLAGS_VALIDHEADER_MASK              0x10
#define RFLR_IRQFLAGS_TXDONE_MASK                   0x08
#define RFLR_IRQFLAGS_CADDONE_MASK                  0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK       0x02
#define RFLR_IRQFLAGS_CADDETECTED_MASK              0x01


/*!
 * RegIrqFlags
 */
#define RFLR_IRQFLAGS_RXTIMEOUT                     0x80
#define RFLR_IRQFLAGS_RXDONE                        0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR               0x20
#define RFLR_IRQFLAGS_VALIDHEADER                   0x10
#define RFLR_IRQFLAGS_TXDONE                        0x08
#define RFLR_IRQFLAGS_CADDONE                       0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL            0x02
#define RFLR_IRQFLAGS_CADDETECTED                   0x01


/*!
 * RegIrqFlags1
 */
#define RF_IRQFLAGS1_MODEREADY                      0x80
#define RF_IRQFLAGS1_RXREADY                        0x40
#define RF_IRQFLAGS1_TXREADY                        0x20
#define RF_IRQFLAGS1_PLLLOCK                        0x10
#define RF_IRQFLAGS1_RSSI                           0x08
#define RF_IRQFLAGS1_TIMEOUT                        0x04
#define RF_IRQFLAGS1_PREAMBLEDETECT                 0x02
#define RF_IRQFLAGS1_SYNCADDRESSMATCH               0x01

/*!
 * RegIrqFlags2
 */
#define RF_IRQFLAGS2_FIFOFULL                       0x80
#define RF_IRQFLAGS2_FIFOEMPTY                      0x40
#define RF_IRQFLAGS2_FIFOLEVEL                      0x20
#define RF_IRQFLAGS2_FIFOOVERRUN                    0x10
#define RF_IRQFLAGS2_PACKETSENT                     0x08
#define RF_IRQFLAGS2_PAYLOADREADY                   0x04
#define RF_IRQFLAGS2_CRCOK                          0x02
#define RF_IRQFLAGS2_LOWBAT                         0x01

// Tx settings
#define REG_PACONFIG                                0x09
#define REG_PARAMP                                  0x0A
#define REG_OCP                                     0x0B
// Rx settings
#define REG_LNA                                     0x0C
#define REG_RXCONFIG                                0x0D
#define REG_RSSICONFIG                              0x0E
#define REG_RSSICOLLISION                           0x0F
#define REG_RSSITHRESH                              0x10
#define REG_RSSIVALUE                               0x11
#define REG_RXBW                                    0x12
#define REG_AFCBW                                   0x13
#define REG_OOKPEAK                                 0x14
#define REG_OOKFIX                                  0x15
#define REG_OOKAVG                                  0x16
#define REG_RES17                                   0x17
#define REG_RES18                                   0x18
#define REG_RES19                                   0x19
#define REG_AFCFEI                                  0x1A
#define REG_AFCMSB                                  0x1B
#define REG_AFCLSB                                  0x1C
#define REG_FEIMSB                                  0x1D
#define REG_FEILSB                                  0x1E
#define REG_PREAMBLEDETECT                          0x1F
#define REG_RXTIMEOUT1                              0x20
#define REG_RXTIMEOUT2                              0x21
#define REG_RXTIMEOUT3                              0x22
#define REG_RXDELAY                                 0x23

// REG_HOP_CHANNEL
#define CRC_ON_PAYLOAD                              0x40  // (bit 6 - 1 Header indicates CRC on)

// Oscillator settings
#define REG_OSC                                     0x24
// Packet handler settings
#define REG_PREAMBLEMSB                             0x25
#define REG_PREAMBLELSB                             0x26
#define REG_SYNCCONFIG                              0x27
#define REG_SYNCVALUE1                              0x28
#define REG_SYNCVALUE2                              0x29
#define REG_SYNCVALUE3                              0x2A
#define REG_SYNCVALUE4                              0x2B
#define REG_SYNCVALUE5                              0x2C
#define REG_SYNCVALUE6                              0x2D
#define REG_SYNCVALUE7                              0x2E
#define REG_SYNCVALUE8                              0x2F
#define REG_PACKETCONFIG1                           0x30
#define REG_PACKETCONFIG2                           0x31
#define REG_PAYLOADLENGTH                           0x32
#define REG_NODEADRS                                0x33
#define REG_BROADCASTADRS                           0x34
#define REG_FIFOTHRESH                              0x35
// SM settings
#define REG_SEQCONFIG1                              0x36
#define REG_SEQCONFIG2                              0x37
#define REG_TIMERRESOL                              0x38
#define REG_TIMER1COEF                              0x39
#define REG_TIMER2COEF                              0x3A
// Service settings
#define REG_IMAGECAL                                0x3B
#define REG_TEMP                                    0x3C
#define REG_LOWBAT                                  0x3D
// Status
#define REG_IRQFLAGS1                               0x3E
#define REG_IRQFLAGS2                               0x3F
// I/O settings
#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41
// Version
#define REG_VERSION                                 0x42
// Additional settings
#define REG_PLLHOP                                  0x44
#define REG_TCXO                                    0x4B
#define REG_PADAC                                   0x4D
#define REG_FORMERTEMP                              0x5B
#define REG_BITRATEFRAC                             0x5D
#define REG_AGCREF                                  0x61
#define REG_AGCTHRESH1                              0x62
#define REG_AGCTHRESH2                              0x63
#define REG_AGCTHRESH3                              0x64
#define REG_PLL                                     0x70

/*!
 * RegRxConfig
 */
#define RF_RXCONFIG_RESTARTRXONCOLLISION_MASK       0x7F
#define RF_RXCONFIG_RESTARTRXONCOLLISION_ON         0x80
#define RF_RXCONFIG_RESTARTRXONCOLLISION_OFF        0x00 // Default
#define RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK         0x40 // Write only
#define RF_RXCONFIG_RESTARTRXWITHPLLLOCK            0x20 // Write only

#define RF_RXCONFIG_AFCAUTO_MASK                    0xEF
#define RF_RXCONFIG_AFCAUTO_ON                      0x10
#define RF_RXCONFIG_AFCAUTO_OFF                     0x00 // Default

#define RF_RXCONFIG_AGCAUTO_MASK                    0xF7
#define RF_RXCONFIG_AGCAUTO_ON                      0x08 // Default
#define RF_RXCONFIG_AGCAUTO_OFF                     0x00

#define RF_RXCONFIG_RXTRIGER_MASK                   0xF8
#define RF_RXCONFIG_RXTRIGER_OFF                    0x00
#define RF_RXCONFIG_RXTRIGER_RSSI                   0x01
#define RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT         0x06 // Default
#define RF_RXCONFIG_RXTRIGER_RSSI_PREAMBLEDETECT    0x07


/*!
 * RegRssiConfig
 */
#define RF_RSSICONFIG_OFFSET_MASK                   0x07
#define RF_RSSICONFIG_OFFSET_P_00_DB                0x00  // Default
#define RF_RSSICONFIG_OFFSET_P_01_DB                0x08
#define RF_RSSICONFIG_OFFSET_P_02_DB                0x10
#define RF_RSSICONFIG_OFFSET_P_03_DB                0x18
#define RF_RSSICONFIG_OFFSET_P_04_DB                0x20
#define RF_RSSICONFIG_OFFSET_P_05_DB                0x28
#define RF_RSSICONFIG_OFFSET_P_06_DB                0x30
#define RF_RSSICONFIG_OFFSET_P_07_DB                0x38
#define RF_RSSICONFIG_OFFSET_P_08_DB                0x40
#define RF_RSSICONFIG_OFFSET_P_09_DB                0x48
#define RF_RSSICONFIG_OFFSET_P_10_DB                0x50
#define RF_RSSICONFIG_OFFSET_P_11_DB                0x58
#define RF_RSSICONFIG_OFFSET_P_12_DB                0x60
#define RF_RSSICONFIG_OFFSET_P_13_DB                0x68
#define RF_RSSICONFIG_OFFSET_P_14_DB                0x70
#define RF_RSSICONFIG_OFFSET_P_15_DB                0x78
#define RF_RSSICONFIG_OFFSET_M_16_DB                0x80
#define RF_RSSICONFIG_OFFSET_M_15_DB                0x88
#define RF_RSSICONFIG_OFFSET_M_14_DB                0x90
#define RF_RSSICONFIG_OFFSET_M_13_DB                0x98
#define RF_RSSICONFIG_OFFSET_M_12_DB                0xA0
#define RF_RSSICONFIG_OFFSET_M_11_DB                0xA8
#define RF_RSSICONFIG_OFFSET_M_10_DB                0xB0
#define RF_RSSICONFIG_OFFSET_M_09_DB                0xB8
#define RF_RSSICONFIG_OFFSET_M_08_DB                0xC0
#define RF_RSSICONFIG_OFFSET_M_07_DB                0xC8
#define RF_RSSICONFIG_OFFSET_M_06_DB                0xD0
#define RF_RSSICONFIG_OFFSET_M_05_DB                0xD8
#define RF_RSSICONFIG_OFFSET_M_04_DB                0xE0
#define RF_RSSICONFIG_OFFSET_M_03_DB                0xE8
#define RF_RSSICONFIG_OFFSET_M_02_DB                0xF0
#define RF_RSSICONFIG_OFFSET_M_01_DB                0xF8

#define RF_RSSICONFIG_SMOOTHING_MASK                0xF8
#define RF_RSSICONFIG_SMOOTHING_2                   0x00
#define RF_RSSICONFIG_SMOOTHING_4                   0x01
#define RF_RSSICONFIG_SMOOTHING_8                   0x02  // Default
#define RF_RSSICONFIG_SMOOTHING_16                  0x03
#define RF_RSSICONFIG_SMOOTHING_32                  0x04
#define RF_RSSICONFIG_SMOOTHING_64                  0x05
#define RF_RSSICONFIG_SMOOTHING_128                 0x06
#define RF_RSSICONFIG_SMOOTHING_256                 0x07

/*!
 * RegRssiCollision
 */
#define RF_RSSICOLISION_THRESHOLD                   0x0A  // Default

/*!
 * RegRssiThresh
 */
#define RF_RSSITHRESH_THRESHOLD                     0xFF  // Default

/*!
 * RegPacketConfig1
 */
#define RF_PACKETCONFIG1_PACKETFORMAT_MASK          0x7F
#define RF_PACKETCONFIG1_PACKETFORMAT_FIXED         0x00
#define RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE      0x80  // Default

#define RF_PACKETCONFIG1_DCFREE_MASK                0x9F
#define RF_PACKETCONFIG1_DCFREE_OFF                 0x00  // Default
#define RF_PACKETCONFIG1_DCFREE_MANCHESTER          0x20
#define RF_PACKETCONFIG1_DCFREE_WHITENING           0x40

#define RF_PACKETCONFIG1_CRC_MASK                   0xEF
#define RF_PACKETCONFIG1_CRC_ON                     0x10  // Default
#define RF_PACKETCONFIG1_CRC_OFF                    0x00

#define RF_PACKETCONFIG1_CRCAUTOCLEAR_MASK          0xF7
#define RF_PACKETCONFIG1_CRCAUTOCLEAR_ON            0x00  // Default
#define RF_PACKETCONFIG1_CRCAUTOCLEAR_OFF           0x08

#define RF_PACKETCONFIG1_ADDRSFILTERING_MASK         0xF9
#define RF_PACKETCONFIG1_ADDRSFILTERING_OFF          0x00  // Default
#define RF_PACKETCONFIG1_ADDRSFILTERING_NODE         0x02
#define RF_PACKETCONFIG1_ADDRSFILTERING_NODEBROADCAST 0x04

#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_MASK      0xFE
#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT     0x00  // Default
#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_IBM       0x01

/*!
 * RegPacketConfig2
 */

#define RF_PACKETCONFIG2_WMBUS_CRC_ENABLE_MASK      0x7F
#define RF_PACKETCONFIG2_WMBUS_CRC_ENABLE           0x80
#define RF_PACKETCONFIG2_WMBUS_CRC_DISABLE          0x00  // Default

#define RF_PACKETCONFIG2_DATAMODE_MASK              0xBF
#define RF_PACKETCONFIG2_DATAMODE_CONTINUOUS        0x00
#define RF_PACKETCONFIG2_DATAMODE_PACKET            0x40  // Default

#define RF_PACKETCONFIG2_IOHOME_MASK                0xDF
#define RF_PACKETCONFIG2_IOHOME_ON                  0x20
#define RF_PACKETCONFIG2_IOHOME_OFF                 0x00  // Default

#define RF_PACKETCONFIG2_BEACON_MASK                0xF7
#define RF_PACKETCONFIG2_BEACON_ON                  0x08
#define RF_PACKETCONFIG2_BEACON_OFF                 0x00  // Default

#define RF_PACKETCONFIG2_PAYLOADLENGTH_MSB_MASK     0xF8

/*!
 * RegModemConfig1
 */
#define RFLR_MODEMCONFIG1_BW_MASK                   0x0F
#define RFLR_MODEMCONFIG1_BW_7_81_KHZ               0x00
#define RFLR_MODEMCONFIG1_BW_10_41_KHZ              0x10
#define RFLR_MODEMCONFIG1_BW_15_62_KHZ              0x20
#define RFLR_MODEMCONFIG1_BW_20_83_KHZ              0x30
#define RFLR_MODEMCONFIG1_BW_31_25_KHZ              0x40
#define RFLR_MODEMCONFIG1_BW_41_66_KHZ              0x50
#define RFLR_MODEMCONFIG1_BW_62_50_KHZ              0x60
#define RFLR_MODEMCONFIG1_BW_125_KHZ                0x70 // Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ                0x80
#define RFLR_MODEMCONFIG1_BW_500_KHZ                0x90

#define RFLR_MODEMCONFIG1_CODINGRATE_MASK           0xF1
#define RFLR_MODEMCONFIG1_CODINGRATE_4_5            0x02
#define RFLR_MODEMCONFIG1_CODINGRATE_4_6            0x04 // Default
#define RFLR_MODEMCONFIG1_CODINGRATE_4_7            0x06
#define RFLR_MODEMCONFIG1_CODINGRATE_4_8            0x08

#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK       0xFE
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_ON         0x01
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF        0x00 // Default

/*!
 * RegModemConfig2
 */
#define RFLR_MODEMCONFIG2_SF_MASK                   0x0F
#define RFLR_MODEMCONFIG2_SF_6                      0x60
#define RFLR_MODEMCONFIG2_SF_7                      0x70 // Default
#define RFLR_MODEMCONFIG2_SF_8                      0x80
#define RFLR_MODEMCONFIG2_SF_9                      0x90
#define RFLR_MODEMCONFIG2_SF_10                     0xA0
#define RFLR_MODEMCONFIG2_SF_11                     0xB0
#define RFLR_MODEMCONFIG2_SF_12                     0xC0

#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK     0xF7
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON       0x08
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF      0x00

#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK         0xFB
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON           0x04
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF          0x00 // Default

#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK       0xFC
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB            0x00 // Default


/*!
 * RegModemConfig3
 */
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK  0xF7
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON    0x08
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF   0x00 // Default

#define RFLR_MODEMCONFIG3_AGCAUTO_MASK              0xFB
#define RFLR_MODEMCONFIG3_AGCAUTO_ON                0x04 // Default
#define RFLR_MODEMCONFIG3_AGCAUTO_OFF               0x00

/*!
 * RegPllHop
 */
#define RFLR_PLLHOP_FASTHOP_MASK                    0x7F
#define RFLR_PLLHOP_FASTHOP_ON                      0x80
#define RFLR_PLLHOP_FASTHOP_OFF                     0x00 // Default


/*!
 * RegDetectOptimize
 */
#define RFLR_DETECTIONOPTIMIZE_MASK                 0xF8
#define RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12          0x03 // Default
#define RFLR_DETECTIONOPTIMIZE_SF6                  0x05

/*!
 * RegDetectionThreshold
 */
#define RFLR_DETECTIONTHRESH_SF7_TO_SF12            0x0A // Default
#define RFLR_DETECTIONTHRESH_SF6                    0x0C

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 14

#define RADIO_MOSI                                  27
#define RADIO_MISO                                  19
#define RADIO_SCLK                                  5
#define RADIO_NSS                                   18
#define RADIO_DIO_0                                 26

/*!
 * RegOpMode
 */
#define RF_OPMODE_LONGRANGEMODE_MASK                0x7F
#define RF_OPMODE_LONGRANGEMODE_OFF                 0x00
#define RF_OPMODE_LONGRANGEMODE_ON                  0x80

#define RF_OPMODE_MODULATIONTYPE_MASK               0x9F
#define RF_OPMODE_MODULATIONTYPE_FSK                0x00  // Default
#define RF_OPMODE_MODULATIONTYPE_OOK                0x20

#define RF_OPMODE_MODULATIONSHAPING_MASK            0xE7
#define RF_OPMODE_MODULATIONSHAPING_00              0x00  // Default
#define RF_OPMODE_MODULATIONSHAPING_01              0x08
#define RF_OPMODE_MODULATIONSHAPING_10              0x10
#define RF_OPMODE_MODULATIONSHAPING_11              0x18

#define RF_OPMODE_MASK                              0xF8
#define RF_OPMODE_SLEEP                             0x00
#define RF_OPMODE_STANDBY                           0x01  // Default
#define RF_OPMODE_SYNTHESIZER_TX                    0x02
#define RF_OPMODE_TRANSMITTER                       0x03
#define RF_OPMODE_SYNTHESIZER_RX                    0x04
#define RF_OPMODE_RECEIVER                          0x05


/*!
 * RegSyncConfig
 */
#define RF_SYNCCONFIG_AUTORESTARTRXMODE_MASK        0x3F
#define RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON  0x80  // Default
#define RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_OFF 0x40
#define RF_SYNCCONFIG_AUTORESTARTRXMODE_OFF         0x00


#define RF_SYNCCONFIG_PREAMBLEPOLARITY_MASK         0xDF
#define RF_SYNCCONFIG_PREAMBLEPOLARITY_55           0x20
#define RF_SYNCCONFIG_PREAMBLEPOLARITY_AA           0x00  // Default

#define RF_SYNCCONFIG_SYNC_MASK                     0xEF
#define RF_SYNCCONFIG_SYNC_ON                       0x10  // Default
#define RF_SYNCCONFIG_SYNC_OFF                      0x00


#define RF_SYNCCONFIG_SYNCSIZE_MASK                 0xF8
#define RF_SYNCCONFIG_SYNCSIZE_1                    0x00
#define RF_SYNCCONFIG_SYNCSIZE_2                    0x01
#define RF_SYNCCONFIG_SYNCSIZE_3                    0x02
#define RF_SYNCCONFIG_SYNCSIZE_4                    0x03  // Default
#define RF_SYNCCONFIG_SYNCSIZE_5                    0x04
#define RF_SYNCCONFIG_SYNCSIZE_6                    0x05
#define RF_SYNCCONFIG_SYNCSIZE_7                    0x06
#define RF_SYNCCONFIG_SYNCSIZE_8                    0x07


/*!
 * RegDioMapping1
 */
        
#define RF_DIOMAPPING1_DIO0_MASK                    0x3F
#define RF_DIOMAPPING1_DIO0_00                      0x00  // Default
#define RF_DIOMAPPING1_DIO0_01                      0x40
#define RF_DIOMAPPING1_DIO0_10                      0x80
#define RF_DIOMAPPING1_DIO0_11                      0xC0

#define RF_DIOMAPPING1_DIO1_MASK                    0xCF
#define RF_DIOMAPPING1_DIO1_00                      0x00  // Default
#define RF_DIOMAPPING1_DIO1_01                      0x10
#define RF_DIOMAPPING1_DIO1_10                      0x20
#define RF_DIOMAPPING1_DIO1_11                      0x30

#define RF_DIOMAPPING1_DIO2_MASK                    0xF3
#define RF_DIOMAPPING1_DIO2_00                      0x00  // Default
#define RF_DIOMAPPING1_DIO2_01                      0x04
#define RF_DIOMAPPING1_DIO2_10                      0x08
#define RF_DIOMAPPING1_DIO2_11                      0x0C

#define RF_DIOMAPPING1_DIO3_MASK                    0xFC
#define RF_DIOMAPPING1_DIO3_00                      0x00  // Default
#define RF_DIOMAPPING1_DIO3_01                      0x01
#define RF_DIOMAPPING1_DIO3_10                      0x02
#define RF_DIOMAPPING1_DIO3_11                      0x03

/*!
 * RegDioMapping2
 */
#define RF_DIOMAPPING2_DIO4_MASK                    0x3F
#define RF_DIOMAPPING2_DIO4_00                      0x00  // Default
#define RF_DIOMAPPING2_DIO4_01                      0x40
#define RF_DIOMAPPING2_DIO4_10                      0x80
#define RF_DIOMAPPING2_DIO4_11                      0xC0

#define RF_DIOMAPPING2_DIO5_MASK                    0xCF
#define RF_DIOMAPPING2_DIO5_00                      0x00  // Default
#define RF_DIOMAPPING2_DIO5_01                      0x10
#define RF_DIOMAPPING2_DIO5_10                      0x20
#define RF_DIOMAPPING2_DIO5_11                      0x30

#define RF_DIOMAPPING2_MAP_MASK                     0xFE
#define RF_DIOMAPPING2_MAP_PREAMBLEDETECT           0x01
#define RF_DIOMAPPING2_MAP_RSSI                     0x00  // Default


/*!
 * RegDioMapping1
 */
#define RFLR_DIOMAPPING1_DIO0_MASK                  0x3F
#define RFLR_DIOMAPPING1_DIO0_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO0_01                    0x40
#define RFLR_DIOMAPPING1_DIO0_10                    0x80
#define RFLR_DIOMAPPING1_DIO0_11                    0xC0

#define RFLR_DIOMAPPING1_DIO1_MASK                  0xCF
#define RFLR_DIOMAPPING1_DIO1_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO1_01                    0x10
#define RFLR_DIOMAPPING1_DIO1_10                    0x20
#define RFLR_DIOMAPPING1_DIO1_11                    0x30

#define RFLR_DIOMAPPING1_DIO2_MASK                  0xF3
#define RFLR_DIOMAPPING1_DIO2_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO2_01                    0x04
#define RFLR_DIOMAPPING1_DIO2_10                    0x08
#define RFLR_DIOMAPPING1_DIO2_11                    0x0C

#define RFLR_DIOMAPPING1_DIO3_MASK                  0xFC
#define RFLR_DIOMAPPING1_DIO3_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO3_01                    0x01
#define RFLR_DIOMAPPING1_DIO3_10                    0x02
#define RFLR_DIOMAPPING1_DIO3_11                    0x03

/*!
 * RegDioMapping2
 */
#define RFLR_DIOMAPPING2_DIO4_MASK                  0x3F
#define RFLR_DIOMAPPING2_DIO4_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO4_01                    0x40
#define RFLR_DIOMAPPING2_DIO4_10                    0x80
#define RFLR_DIOMAPPING2_DIO4_11                    0xC0

#define RFLR_DIOMAPPING2_DIO5_MASK                  0xCF
#define RFLR_DIOMAPPING2_DIO5_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO5_01                    0x10
#define RFLR_DIOMAPPING2_DIO5_10                    0x20
#define RFLR_DIOMAPPING2_DIO5_11                    0x30

#define RFLR_DIOMAPPING2_MAP_MASK                   0xFE
#define RFLR_DIOMAPPING2_MAP_PREAMBLEDETECT         0x01
#define RFLR_DIOMAPPING2_MAP_RSSI                   0x00  // Default

/*!
 * Syncword for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12

/*!
 * Syncword for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34

/*!
 * RegHopChannel (Read Only)
 */
#define RFLR_HOPCHANNEL_PLL_LOCK_TIMEOUT_MASK       0x7F
#define RFLR_HOPCHANNEL_PLL_LOCK_FAIL               0x80
#define RFLR_HOPCHANNEL_PLL_LOCK_SUCCEED            0x00 // Default

#define RFLR_HOPCHANNEL_CRCONPAYLOAD_MASK           0xBF
#define RFLR_HOPCHANNEL_CRCONPAYLOAD_ON             0x40
#define RFLR_HOPCHANNEL_CRCONPAYLOAD_OFF            0x00 // Default

#define RFLR_HOPCHANNEL_CHANNEL_MASK                0x3F

/*!
 * RegPaConfig
 */
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00 // Default

#define RF_PACONFIG_MAX_POWER_MASK                  0x8F

#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

/*!
 * RegPaDac
 */
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04  // Default

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */

#define BOARD_TCXO_WAKEUP_TIME                      0

/*!
 * \brief Radio hardware registers initialization definition
 *
 * \remark Can be automatically generated by the SX1276 GUI (not yet implemented)
 */
#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    { MODEM_FSK , REG_AFCFEI             , 0x01 },\
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0xFF },\
}                                                 \


/*!
 * Radio driver supported modems
 */
typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_t;

/*!
 * Radio driver internal state machine states definition
 */
typedef enum
{
    RF_IDLE = 0,   //!< The radio is idle
    RF_RX_RUNNING, //!< The radio is in reception state
    RF_TX_RUNNING, //!< The radio is in transmission state
    RF_CAD,        //!< The radio is doing channel activity detection
}RadioState_t;


typedef enum
{
    STATUS_LOWPOWER,
    STATUS_RX,
    STATUS_TX
}States_t;

/*!
 * \brief Radio driver callback functions
 */
typedef struct
{
    /*!
     * \brief  Tx Done callback prototype.
     */
    void    ( *TxDone )( void );
    /*!
     * \brief  Tx Timeout callback prototype.
     */
    void    ( *TxTimeout )( void );
    /*!
     * \brief Rx Done callback prototype.
     *
     * \param [IN] payload Received buffer pointer
     * \param [IN] size    Received buffer size
     * \param [IN] rssi    RSSI value computed while receiving the frame [dBm]
     * \param [IN] snr     Raw SNR value given by the radio hardware
     *                     FSK : N/A ( set to 0 )
     *                     LoRa: SNR value in dB
     */
    void    ( *RxDone )( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
    /*!
     * \brief  Rx Timeout callback prototype.
     */
    void    ( *RxTimeout )( void );
    /*!
     * \brief Rx Error callback prototype.
     */
    void    ( *RxError )( void );
    /*!
     * \brief  FHSS Change Channel callback prototype.
     *
     * \param [IN] currentChannel   Index number of the current channel
     */
    void ( *FhssChangeChannel )( uint8_t currentChannel );

    /*!
     * \brief CAD Done callback prototype.
     *
     * \param [IN] channelDetected    Channel Activity detected during the CAD
     */
    void ( *CadDone ) ( bool channelActivityDetected );
}RadioEvents_t;

/*!
 * Add a pull-up, a pull-down or nothing on the GPIO line
 */
typedef enum
{
    PIN_NO_PULL = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN
}PinTypes;

/*!
 * GPIO IRQ handler function prototype
 */
typedef void( GpioIrqHandler )( void );

/*!
 * Structure for the GPIO
 */
typedef struct
{
    uint8_t  pin;
    uint16_t pinIndex;
    void *port;
    uint16_t portIndex;
    PinTypes pull;
}Gpio_t;

/*!
 * SPI object type definition
 */
typedef struct Spi_s
{
   // SpiId_t SpiId;
    Gpio_t Mosi;
    Gpio_t Miso;
    Gpio_t Sclk;
    Gpio_t Nss;
}Spi_t;


/*!
 * Define the GPIO IRQ on a rising, falling or both edges
 */
typedef enum
{
    NO_IRQ = 0,
    IRQ_RISING_EDGE,
    IRQ_FALLING_EDGE,
    IRQ_RISING_FALLING_EDGE
}IrqModes;

/*!
 * Define the IRQ priority on the GPIO
 */
typedef enum
{
    IRQ_VERY_LOW_PRIORITY = 0,
    IRQ_LOW_PRIORITY,
    IRQ_MEDIUM_PRIORITY,
    IRQ_HIGH_PRIORITY,
    IRQ_VERY_HIGH_PRIORITY
}IrqPriorities;

/*!
 * Hardware IO IRQ callback function definition
 */
typedef void ( DioIrqHandler )( void );

/*!
 * \brief Radio driver
 *
 * \remark This variable is defined and initialized in the specific radio
 *         board implementation
 */
extern const struct Radio_s Radio;


extern RadioEvents_t RadioEvents1;


//static void RxChainCalibration( void );
//void SX1276SetOpMode( uint8_t opMode );
//void SX1276ReadFifo( uint8_t *buffer, uint8_t size );
//extern void write0(uint16_t address, uint8_t value);
//extern uint8_t read0(uint16_t address);
//extern void writefifo(uint16_t address, uint8_t *buffer, uint8_t size);
//extern void readfifo(uint16_t address, uint8_t *buffer, uint8_t size);

//void SX1276OnDio0Irq( void );
//void SX1276OnDio1Irq( void );
//void SX1276OnDio2Irq( void );
//void SX1276OnDio3Irq( void );
//void SX1276OnDio4Irq( void );

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] channel Channel frequency in Hz
 * \retval PaSelect RegPaConfig PaSelect value
 */
uint8_t SX1276GetPaSelect( uint32_t channel );


#ifdef __cplusplus
extern "C"{
#endif

/*!
 * \brief Radio driver definition
 */
struct Radio_s
{
    /*!
     * \brief Initializes the radio
     *
     * \param [IN] events Structure containing the driver callback functions
     */
    void    ( *Init )( RadioEvents_t *events );
    /*!
     * Return current radio status
     *
     * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
     */
    RadioState_t ( *GetStatus )( void );
    /*!
     * \brief Configures the radio with the given modem
     *
     * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
     */
    void    ( *SetModem )( RadioModems_t modem );
    /*!
     * \brief Sets the channel frequency
     *
     * \param [IN] freq         Channel RF frequency
     */
    void    ( *SetChannel )( uint32_t freq );
    /*!
     * \brief Checks if the channel is free for the given time
     *
     * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
     * \param [IN] freq       Channel RF frequency
     * \param [IN] rssiThresh RSSI threshold
     * \param [IN] maxCarrierSenseTime Max time while the RSSI is measured
     *
     * \retval isFree         [true: Channel is free, false: Channel is not free]
     */
    bool    ( *IsChannelFree )( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime );
    /*!
     * \brief Generates a 32 bits random value based on the RSSI readings
     *
     * \remark This function sets the radio in LoRa modem mode and disables
     *         all interrupts.
     *         After calling this function either Radio.SetRxConfig or
     *         Radio.SetTxConfig functions must be called.
     *
     * \retval randomValue    32 bits random value
     */
    uint32_t ( *Random )( void );
    /*!
     * \brief Sets the reception parameters
     *
     * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
     * \param [IN] bandwidth    Sets the bandwidth
     *                          FSK : >= 2600 and <= 250000 Hz
     *                          LoRa: [0: 125 kHz, 1: 250 kHz,
     *                                 2: 500 kHz, 3: Reserved]
     * \param [IN] datarate     Sets the Datarate
     *                          FSK : 600..300000 bits/s
     *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
     *                                10: 1024, 11: 2048, 12: 4096  chips]
     * \param [IN] coderate     Sets the coding rate (LoRa only)
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
     * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
     *                          FSK : >= 2600 and <= 250000 Hz
     *                          LoRa: N/A ( set to 0 )
     * \param [IN] preambleLen  Sets the Preamble length
     *                          FSK : Number of bytes
     *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
     * \param [IN] symbTimeout  Sets the RxSingle timeout value
     *                          FSK : timeout in number of bytes
     *                          LoRa: timeout in symbols
     * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
     * \param [IN] payloadLen   Sets payload length when fixed length is used
     * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
     * \param [IN] freqHopOn    Enables disables the intra-packet frequency hopping
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: OFF, 1: ON]
     * \param [IN] hopPeriod    Number of symbols between each hop
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: Number of symbols
     * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: not inverted, 1: inverted]
     * \param [IN] rxContinuous Sets the reception in continuous mode
     *                          [false: single mode, true: continuous mode]
     */
    void    ( *SetRxConfig )( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint32_t bandwidthAfc, uint16_t preambleLen,
                              uint16_t symbTimeout, bool fixLen,
                              uint8_t payloadLen,
                              bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                              bool iqInverted, bool rxContinuous );
    /*!
     * \brief Sets the transmission parameters
     *
     * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
     * \param [IN] power        Sets the output power [dBm]
     * \param [IN] fdev         Sets the frequency deviation (FSK only)
     *                          FSK : [Hz]
     *                          LoRa: 0
     * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
     *                          FSK : 0
     *                          LoRa: [0: 125 kHz, 1: 250 kHz,
     *                                 2: 500 kHz, 3: Reserved]
     * \param [IN] datarate     Sets the Datarate
     *                          FSK : 600..300000 bits/s
     *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
     *                                10: 1024, 11: 2048, 12: 4096  chips]
     * \param [IN] coderate     Sets the coding rate (LoRa only)
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
     * \param [IN] preambleLen  Sets the preamble length
     *                          FSK : Number of bytes
     *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
     * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
     * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
     * \param [IN] freqHopOn    Enables disables the intra-packet frequency hopping
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: OFF, 1: ON]
     * \param [IN] hopPeriod    Number of symbols between each hop
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: Number of symbols
     * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: not inverted, 1: inverted]
     * \param [IN] timeout      Transmission timeout [ms]
     */
    void    ( *SetTxConfig )( RadioModems_t modem, int8_t power, uint32_t fdev,
                              uint32_t bandwidth, uint32_t datarate,
                              uint8_t coderate, uint16_t preambleLen,
                              bool fixLen, bool crcOn, bool freqHopOn,
                              uint8_t hopPeriod, bool iqInverted, uint32_t timeout );
    /*!
     * \brief Checks if the given RF frequency is supported by the hardware
     *
     * \param [IN] frequency RF frequency to be checked
     * \retval isSupported [true: supported, false: unsupported]
     */
    bool    ( *CheckRfFrequency )( uint32_t frequency );
    /*!
     * \brief Computes the packet time on air in ms for the given payload
     *
     * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
     *
     * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
     * \param [IN] pktLen     Packet payload length
     *
     * \retval airTime        Computed airTime (ms) for the given packet payload length
     */
    uint32_t  ( *TimeOnAir )( RadioModems_t modem, uint8_t pktLen );
    /*!
     * \brief Sends the buffer of size. Prepares the packet to be sent and sets
     *        the radio in transmission
     *
     * \param [IN]: buffer     Buffer pointer
     * \param [IN]: size       Buffer size
     */
    void    ( *Send )( uint8_t *buffer, uint8_t size );
    /*!
     * \brief Sets the radio in sleep mode
     */
    void    ( *Sleep )( void );
    /*!
     * \brief Sets the radio in standby mode
     */
    void    ( *Standby )( void );
    /*!
     * \brief Sets the radio in reception mode for the given time
     * \param [IN] timeout Reception timeout [ms]
     *                     [0: continuous, others timeout]
     */
    void    ( *Rx )( uint32_t timeout );
    /*!
     * \brief Start a Channel Activity Detection
     */
    void    ( *StartCad )( void );
    /*!
     * \brief Sets the radio in continuous wave transmission mode
     *
     * \param [IN]: freq       Channel RF frequency
     * \param [IN]: power      Sets the output power [dBm]
     * \param [IN]: time       Transmission mode timeout [s]
     */
    void    ( *SetTxContinuousWave )( uint32_t freq, int8_t power, uint16_t time );
    /*!
     * \brief Reads the current RSSI value
     *
     * \retval rssiValue Current RSSI value in [dBm]
     */
    int16_t ( *Rssi )( RadioModems_t modem );
    /*!
     * \brief Writes the radio register at the specified address
     *
     * \param [IN]: addr Register address
     * \param [IN]: data New register value
     */
    void    ( *Write )( uint16_t addr, uint8_t data );
    /*!
     * \brief Reads the radio register at the specified address
     *
     * \param [IN]: addr Register address
     * \retval data Register value
     */
    uint8_t ( *Read )( uint16_t addr );
    /*!
     * \brief Writes multiple radio registers starting at address
     *
     * \param [IN] addr   First Radio register address
     * \param [IN] buffer Buffer containing the new register's values
     * \param [IN] size   Number of registers to be written
     */
    void    ( *WriteBuffer )( uint16_t addr, uint8_t *buffer, uint8_t size );
    /*!
     * \brief Reads multiple radio registers starting at address
     *
     * \param [IN] addr First Radio register address
     * \param [OUT] buffer Buffer where to copy the registers data
     * \param [IN] size Number of registers to be read
     */
    void    ( *ReadBuffer )( uint16_t addr, uint8_t *buffer, uint8_t size );
    /*!
     * \brief Sets the maximum payload length.
     *
     * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
     * \param [IN] max        Maximum payload length in bytes
     */
    void    ( *SetMaxPayloadLength )( RadioModems_t modem, uint8_t max );
    /*!
     * \brief Sets the network to public or private. Updates the sync byte.
     *
     * \remark Applies to LoRa modem only
     *
     * \param [IN] enable if true, it enables a public network
     */
    void    ( *SetPublicNetwork )( bool enable );
    /*!
     * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
     *
     * \retval time Radio plus board wakeup time in ms.
     */
    uint32_t  ( *GetWakeupTime )( void );
    /*!
     * \brief Process radio irq
     */
    void ( *IrqProcess )( void );
    /*
     * The next functions are available only on SX126x radios.
     */
    /*!
     * \brief Sets the radio in reception mode with Max LNA gain for the given time
     *
     * \remark Available on SX126x radios only.
     *
     * \param [IN] timeout Reception timeout [ms]
     *                     [0: continuous, others timeout]
     */
    void    ( *RxBoosted )( uint32_t timeout );
    /*!
     * \brief Sets the Rx duty cycle management parameters
     *
     * \remark Available on SX126x radios only.
     *
     * \param [in]  rxTime        Structure describing reception timeout value
     * \param [in]  sleepTime     Structure describing sleep timeout value
     */
    void ( *SetRxDutyCycle ) ( uint32_t rxTime, uint32_t sleepTime );
};

/*!
 * \brief Copies size elements of src array to dst array
 *
 * \remark STM32 Standard memcpy function only works on pointers that are aligned
 *
 * \param [OUT] dst  Destination array
 * \param [IN]  src  Source array
 * \param [IN]  size Number of bytes to be copied
 */
void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size );

/*!
 * \brief Set the RF Switch I/Os pins in low power mode
 *
 * \param [IN] status enable or disable
 */
void SX1276SetAntSwLowPower( bool status );

/*!
 * \brief Controls the antenna switch if necessary.
 *
 * \remark see errata note
 *
 * \param [IN] opMode Current radio operating mode
 */
void SX1276SetAntSw( uint8_t opMode );

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void SX1276IoIrqInit( DioIrqHandler **irqHandlers );


/*!
 * \brief Resets the radio
 */
void SX1276Reset( void );

/*!
 * \brief Sets the channel configuration
 *
 * \param [IN] freq         Channel RF frequency
 */
void SX1276SetChannel( uint32_t freq );

void DelayMs( uint32_t ms );

/*!
 * \brief Sets the radio output power.
 *
 * \param [IN] power Sets the RF output power
 */
void SX1276SetRfTxPower( int8_t power );

/*!
 * \brief Gets the Defines the time required for the TCXO to wakeup [ms].
 *
 * \retval time Board TCXO wakeup time in ms.
 */
uint32_t SX1276GetBoardTcxoWakeupTime( void );

#ifdef __cplusplus
}
#endif


#endif // __RADIO_H__
