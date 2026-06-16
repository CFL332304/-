#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* Remote controller uses a fixed 12-byte payload inside the NRF24L01 frame. */
#define NRF24L01_MAX_PAYLOAD_WIDTH     32U
#define NRF24L01_REMOTE_PAYLOAD_WIDTH  12U

#define NRF24L01_R_REGISTER            0x00U
#define NRF24L01_W_REGISTER            0x20U
#define NRF24L01_R_RX_PAYLOAD          0x61U
#define NRF24L01_W_TX_PAYLOAD          0xA0U
#define NRF24L01_FLUSH_TX              0xE1U
#define NRF24L01_FLUSH_RX              0xE2U
#define NRF24L01_NOP                   0xFFU

#define NRF24L01_CONFIG                0x00U
#define NRF24L01_EN_AA                 0x01U
#define NRF24L01_EN_RXADDR             0x02U
#define NRF24L01_SETUP_AW              0x03U
#define NRF24L01_SETUP_RETR            0x04U
#define NRF24L01_RF_CH                 0x05U
#define NRF24L01_RF_SETUP              0x06U
#define NRF24L01_STATUS                0x07U
#define NRF24L01_RX_ADDR_P0            0x0AU
#define NRF24L01_TX_ADDR               0x10U
#define NRF24L01_RX_PW_P0              0x11U
#define NRF24L01_FIFO_STATUS           0x17U
#define NRF24L01_DYNPD                 0x1CU
#define NRF24L01_FEATURE               0x1DU

#define NRF24L01_STATUS_RX_DR          0x40U
#define NRF24L01_STATUS_TX_DS          0x20U
#define NRF24L01_STATUS_MAX_RT         0x10U
#define NRF24L01_STATUS_IRQ_MASK       0x70U

#define NRF24L01_CONFIG_EN_CRC         0x08U
#define NRF24L01_CONFIG_CRCO           0x04U
#define NRF24L01_CONFIG_PWR_UP         0x02U
#define NRF24L01_CONFIG_PRIM_RX        0x01U

#define NRF24L01_FIFO_RX_EMPTY         0x01U

typedef enum
{
  /* Transmit result codes make wireless send failures easier to diagnose. */
  NRF24L01_TX_IDLE = 0U,
  NRF24L01_TX_OK = 1U,
  NRF24L01_TX_MAX_RT = 2U,
  NRF24L01_TX_BAD_STATUS = 3U,
  NRF24L01_TX_TIMEOUT = 4U,
  NRF24L01_TX_NOT_PRESENT = 5U,
  NRF24L01_TX_BAD_LENGTH = 6U
} NRF24L01_TxResult_t;

uint8_t NRF24L01_Init(void);
uint8_t NRF24L01_IsPresent(void);

uint8_t NRF24L01_SPI_SwapByte(uint8_t byte);
uint8_t NRF24L01_ReadReg(uint8_t reg);
void NRF24L01_ReadRegs(uint8_t reg, uint8_t *data, uint8_t count);
void NRF24L01_WriteReg(uint8_t reg, uint8_t data);
void NRF24L01_WriteRegs(uint8_t reg, const uint8_t *data, uint8_t count);
void NRF24L01_ReadRxPayload(uint8_t *data, uint8_t count);
void NRF24L01_WriteTxPayload(const uint8_t *data, uint8_t count);
void NRF24L01_FlushTx(void);
void NRF24L01_FlushRx(void);
uint8_t NRF24L01_ReadStatus(void);

void NRF24L01_PowerDown(void);
void NRF24L01_StandbyI(void);
void NRF24L01_RxMode(void);
void NRF24L01_TxMode(void);
uint8_t NRF24L01_SendPacket(const uint8_t *data, uint8_t count, uint32_t timeout_ms);
uint8_t NRF24L01_ReceivePacket(uint8_t *data, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif
