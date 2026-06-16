#include "nrf24l01.h"
#include <string.h>

/* NRF24L01 uses a software SPI bus on GPIO pins, not the STM32 hardware SPI. */
#define NRF24L01_RF_CHANNEL        40U
#define NRF24L01_ADDRESS_WIDTH     5U
#define NRF24L01_CE_PULSE_LOOPS    240U
#define NRF24L01_SPI_DELAY_LOOPS   4U

static const uint8_t nrf24l01Address[NRF24L01_ADDRESS_WIDTH] =
{
  0x34U, 0x43U, 0x10U, 0x10U, 0x01U
};

static void NRF24L01_DelayLoops(uint32_t loops)
{
  while (loops-- != 0U)
  {
    __NOP();
  }
}

static void NRF24L01_WriteCE(GPIO_PinState state)
{
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, state);
}

static void NRF24L01_WriteCSN(GPIO_PinState state)
{
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, state);
}

static void NRF24L01_WriteSCK(GPIO_PinState state)
{
  HAL_GPIO_WritePin(NRF_SCK_GPIO_Port, NRF_SCK_Pin, state);
}

static void NRF24L01_WriteMOSI(GPIO_PinState state)
{
  HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port, NRF_MOSI_Pin, state);
}

static uint8_t NRF24L01_ReadMISO(void)
{
  return (HAL_GPIO_ReadPin(NRF_MISO_GPIO_Port, NRF_MISO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t NRF24L01_WriteCommand(uint8_t command)
{
  uint8_t status;

  NRF24L01_WriteCSN(GPIO_PIN_RESET);
  status = NRF24L01_SPI_SwapByte(command);
  NRF24L01_WriteCSN(GPIO_PIN_SET);

  return status;
}

static void NRF24L01_ClearIrqFlags(void)
{
  NRF24L01_WriteReg(NRF24L01_STATUS, NRF24L01_STATUS_IRQ_MASK);
}

uint8_t NRF24L01_SPI_SwapByte(uint8_t byte)
{
  uint8_t i;

  /* Mode-compatible bit banging: write MOSI, clock high, sample MISO, clock low. */
  for (i = 0U; i < 8U; i++)
  {
    NRF24L01_WriteMOSI(((byte & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    byte <<= 1U;

    NRF24L01_DelayLoops(NRF24L01_SPI_DELAY_LOOPS);
    NRF24L01_WriteSCK(GPIO_PIN_SET);
    NRF24L01_DelayLoops(NRF24L01_SPI_DELAY_LOOPS);

    if (NRF24L01_ReadMISO() != 0U)
    {
      byte |= 0x01U;
    }

    NRF24L01_WriteSCK(GPIO_PIN_RESET);
    NRF24L01_DelayLoops(NRF24L01_SPI_DELAY_LOOPS);
  }

  return byte;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
  uint8_t data;

  NRF24L01_WriteCSN(GPIO_PIN_RESET);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | (reg & 0x1FU));
  data = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
  NRF24L01_WriteCSN(GPIO_PIN_SET);

  return data;
}

void NRF24L01_ReadRegs(uint8_t reg, uint8_t *data, uint8_t count)
{
  uint8_t i;

  if (data == NULL)
  {
    return;
  }

  NRF24L01_WriteCSN(GPIO_PIN_RESET);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | (reg & 0x1FU));
  for (i = 0U; i < count; i++)
  {
    data[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
  }
  NRF24L01_WriteCSN(GPIO_PIN_SET);
}

void NRF24L01_WriteReg(uint8_t reg, uint8_t data)
{
  NRF24L01_WriteCSN(GPIO_PIN_RESET);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | (reg & 0x1FU));
  (void)NRF24L01_SPI_SwapByte(data);
  NRF24L01_WriteCSN(GPIO_PIN_SET);
}

void NRF24L01_WriteRegs(uint8_t reg, const uint8_t *data, uint8_t count)
{
  uint8_t i;

  if (data == NULL)
  {
    return;
  }

  NRF24L01_WriteCSN(GPIO_PIN_RESET);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | (reg & 0x1FU));
  for (i = 0U; i < count; i++)
  {
    (void)NRF24L01_SPI_SwapByte(data[i]);
  }
  NRF24L01_WriteCSN(GPIO_PIN_SET);
}

void NRF24L01_ReadRxPayload(uint8_t *data, uint8_t count)
{
  uint8_t i;

  if (data == NULL)
  {
    return;
  }

  NRF24L01_WriteCSN(GPIO_PIN_RESET);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_R_RX_PAYLOAD);
  for (i = 0U; i < count; i++)
  {
    data[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
  }
  NRF24L01_WriteCSN(GPIO_PIN_SET);
}

void NRF24L01_WriteTxPayload(const uint8_t *data, uint8_t count)
{
  uint8_t i;

  if (data == NULL)
  {
    return;
  }

  NRF24L01_WriteCSN(GPIO_PIN_RESET);
  (void)NRF24L01_SPI_SwapByte(NRF24L01_W_TX_PAYLOAD);
  for (i = 0U; i < count; i++)
  {
    (void)NRF24L01_SPI_SwapByte(data[i]);
  }
  NRF24L01_WriteCSN(GPIO_PIN_SET);
}

void NRF24L01_FlushTx(void)
{
  (void)NRF24L01_WriteCommand(NRF24L01_FLUSH_TX);
}

void NRF24L01_FlushRx(void)
{
  (void)NRF24L01_WriteCommand(NRF24L01_FLUSH_RX);
}

uint8_t NRF24L01_ReadStatus(void)
{
  return NRF24L01_WriteCommand(NRF24L01_NOP);
}

uint8_t NRF24L01_IsPresent(void)
{
  uint8_t readback[NRF24L01_ADDRESS_WIDTH];

  /* Verify the module by writing and reading back a multi-byte address register. */
  NRF24L01_WriteRegs(NRF24L01_TX_ADDR, nrf24l01Address, NRF24L01_ADDRESS_WIDTH);
  memset(readback, 0, sizeof(readback));
  NRF24L01_ReadRegs(NRF24L01_TX_ADDR, readback, NRF24L01_ADDRESS_WIDTH);

  return (memcmp(readback, nrf24l01Address, NRF24L01_ADDRESS_WIDTH) == 0) ? 1U : 0U;
}

void NRF24L01_PowerDown(void)
{
  uint8_t config;

  NRF24L01_WriteCE(GPIO_PIN_RESET);
  config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  config &= (uint8_t)~NRF24L01_CONFIG_PWR_UP;
  NRF24L01_WriteReg(NRF24L01_CONFIG, config);
}

void NRF24L01_StandbyI(void)
{
  uint8_t config;

  NRF24L01_WriteCE(GPIO_PIN_RESET);
  config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  config |= NRF24L01_CONFIG_PWR_UP;
  config &= (uint8_t)~NRF24L01_CONFIG_PRIM_RX;
  NRF24L01_WriteReg(NRF24L01_CONFIG, config);
}

void NRF24L01_RxMode(void)
{
  uint8_t config;

  NRF24L01_WriteCE(GPIO_PIN_RESET);
  config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  config |= (NRF24L01_CONFIG_PWR_UP | NRF24L01_CONFIG_PRIM_RX);
  NRF24L01_WriteReg(NRF24L01_CONFIG, config);
  NRF24L01_ClearIrqFlags();
  NRF24L01_FlushRx();
  NRF24L01_WriteCE(GPIO_PIN_SET);
}

void NRF24L01_TxMode(void)
{
  uint8_t config;

  NRF24L01_WriteCE(GPIO_PIN_RESET);
  config = NRF24L01_ReadReg(NRF24L01_CONFIG);
  config |= NRF24L01_CONFIG_PWR_UP;
  config &= (uint8_t)~NRF24L01_CONFIG_PRIM_RX;
  NRF24L01_WriteReg(NRF24L01_CONFIG, config);
}

uint8_t NRF24L01_Init(void)
{
  /* Put pins in a known idle state before register configuration. */
  NRF24L01_WriteCE(GPIO_PIN_RESET);
  NRF24L01_WriteCSN(GPIO_PIN_SET);
  NRF24L01_WriteSCK(GPIO_PIN_RESET);
  NRF24L01_WriteMOSI(GPIO_PIN_RESET);
  HAL_Delay(5U);

  if (NRF24L01_IsPresent() == 0U)
  {
    return 0U;
  }

  /* Fixed 12-byte payload, auto-ack enabled on pipe 0, shared TX/RX address. */
  NRF24L01_WriteReg(NRF24L01_CONFIG, NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO);
  NRF24L01_WriteReg(NRF24L01_EN_AA, 0x01U);
  NRF24L01_WriteReg(NRF24L01_EN_RXADDR, 0x01U);
  NRF24L01_WriteReg(NRF24L01_SETUP_AW, 0x03U);
  NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0x2FU);
  NRF24L01_WriteReg(NRF24L01_RF_CH, NRF24L01_RF_CHANNEL);
  NRF24L01_WriteReg(NRF24L01_RF_SETUP, 0x06U);
  NRF24L01_WriteReg(NRF24L01_RX_PW_P0, NRF24L01_REMOTE_PAYLOAD_WIDTH);
  NRF24L01_WriteReg(NRF24L01_DYNPD, 0x00U);
  NRF24L01_WriteReg(NRF24L01_FEATURE, 0x00U);
  NRF24L01_WriteRegs(NRF24L01_TX_ADDR, nrf24l01Address, NRF24L01_ADDRESS_WIDTH);
  NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, nrf24l01Address, NRF24L01_ADDRESS_WIDTH);
  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  NRF24L01_ClearIrqFlags();
  NRF24L01_StandbyI();
  HAL_Delay(2U);

  return 1U;
}

uint8_t NRF24L01_SendPacket(const uint8_t *data, uint8_t count, uint32_t timeout_ms)
{
  uint32_t start_tick;
  uint8_t status;

  if ((data == NULL) || (count == 0U) || (count > NRF24L01_MAX_PAYLOAD_WIDTH))
  {
    return NRF24L01_TX_BAD_LENGTH;
  }

  if (NRF24L01_IsPresent() == 0U)
  {
    return NRF24L01_TX_NOT_PRESENT;
  }

  /* Blocking transmit waits until TX success, max retry, or timeout. */
  NRF24L01_TxMode();
  NRF24L01_ClearIrqFlags();
  NRF24L01_FlushTx();
  NRF24L01_WriteTxPayload(data, count);

  NRF24L01_WriteCE(GPIO_PIN_SET);
  NRF24L01_DelayLoops(NRF24L01_CE_PULSE_LOOPS);
  NRF24L01_WriteCE(GPIO_PIN_RESET);

  start_tick = HAL_GetTick();
  do
  {
    status = NRF24L01_ReadStatus();

    if ((status & (NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT)) ==
        (NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT))
    {
      NRF24L01_ClearIrqFlags();
      NRF24L01_FlushTx();
      return NRF24L01_TX_BAD_STATUS;
    }

    if ((status & NRF24L01_STATUS_TX_DS) != 0U)
    {
      NRF24L01_WriteReg(NRF24L01_STATUS, NRF24L01_STATUS_TX_DS);
      return NRF24L01_TX_OK;
    }

    if ((status & NRF24L01_STATUS_MAX_RT) != 0U)
    {
      NRF24L01_WriteReg(NRF24L01_STATUS, NRF24L01_STATUS_MAX_RT);
      NRF24L01_FlushTx();
      return NRF24L01_TX_MAX_RT;
    }
  } while ((HAL_GetTick() - start_tick) <= timeout_ms);

  NRF24L01_ClearIrqFlags();
  NRF24L01_FlushTx();
  return NRF24L01_TX_TIMEOUT;
}

uint8_t NRF24L01_ReceivePacket(uint8_t *data, uint8_t count)
{
  uint8_t status;
  uint8_t fifo_status;

  if ((data == NULL) || (count == 0U) || (count > NRF24L01_MAX_PAYLOAD_WIDTH))
  {
    return 0U;
  }

  /* Data is available when RX_DR is set or RX FIFO is not empty. */
  status = NRF24L01_ReadStatus();
  fifo_status = NRF24L01_ReadReg(NRF24L01_FIFO_STATUS);
  if (((status & NRF24L01_STATUS_RX_DR) == 0U) &&
      ((fifo_status & NRF24L01_FIFO_RX_EMPTY) != 0U))
  {
    return 0U;
  }

  NRF24L01_ReadRxPayload(data, count);
  NRF24L01_WriteReg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR);

  return 1U;
}
