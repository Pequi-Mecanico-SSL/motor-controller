// Minimal polling FDCAN1 driver for the B-G431B-ESC1 (classic CAN 2.0A only).
// RX=PA11, TX=PB9 (AF9), kernel clock PCLK1 = 170 MHz.
// Timing per bittiming.can-wiki.info: prescaler/17tq, seg1=14 seg2=2 (SP 88%).
// Hardware filters accept only the IDs passed to canInit; everything else
// (incl. remote frames) is rejected before it costs CPU.
#pragma once
#include <Arduino.h>

struct CanFrame {
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];
};

static FDCAN_HandleTypeDef hfdcan1 = {};

// speed: 500 kbit -> prescaler 20, 1 Mbit -> 10 (170 MHz / presc / 17)
inline bool canInit(uint32_t prescaler, const uint16_t* rx_ids, uint8_t n_ids) {
  RCC_PeriphCLKInitTypeDef pclk = {};
  HAL_RCCEx_GetPeriphCLKConfig(&pclk);
  pclk.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;
  pclk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) return false;
  __HAL_RCC_FDCAN_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF9_FDCAN1;
  gpio.Pin = GPIO_PIN_11;                 // PA11 RX
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_9;                  // PB9 TX
  HAL_GPIO_Init(GPIOB, &gpio);

  hfdcan1.Instance = FDCAN1;
  FDCAN_InitTypeDef* init = &hfdcan1.Init;
  init->ClockDivider = FDCAN_CLOCK_DIV1;
  init->FrameFormat = FDCAN_FRAME_CLASSIC;
  init->Mode = FDCAN_MODE_NORMAL;
  init->AutoRetransmission = ENABLE;
  init->TransmitPause = ENABLE;
  init->ProtocolException = DISABLE;
  init->NominalPrescaler = prescaler;
  init->NominalSyncJumpWidth = 1;
  init->NominalTimeSeg1 = 14;
  init->NominalTimeSeg2 = 2;
  init->DataPrescaler = 1;
  init->DataSyncJumpWidth = 1;
  init->DataTimeSeg1 = 1;
  init->DataTimeSeg2 = 1;
  init->StdFiltersNbr = n_ids;
  init->ExtFiltersNbr = 0;
  init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) return false;

  for (uint8_t i = 0; i < n_ids; i++) {
    FDCAN_FilterTypeDef f = {};
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = i;
    f.FilterType = FDCAN_FILTER_DUAL;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = rx_ids[i];
    f.FilterID2 = rx_ids[i];
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK) return false;
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    return false;

  return HAL_FDCAN_Start(&hfdcan1) == HAL_OK;
}

inline bool canSend(uint32_t id, const uint8_t* data, uint8_t len) {
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) return false;
  FDCAN_TxHeaderTypeDef h = {};
  h.Identifier = id;
  h.IdType = FDCAN_STANDARD_ID;
  h.TxFrameType = FDCAN_DATA_FRAME;
  h.DataLength = len;
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch = FDCAN_BRS_OFF;
  h.FDFormat = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, (uint8_t*)data) == HAL_OK;
}

inline bool canRead(CanFrame& frame) {
  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) return false;
  FDCAN_RxHeaderTypeDef h;
  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &h, frame.buf) != HAL_OK)
    return false;
  if (h.IdType != FDCAN_STANDARD_ID || h.RxFrameType != FDCAN_DATA_FRAME)
    return false;
  frame.id = h.Identifier;
  frame.len = (h.DataLength > 8) ? 8 : h.DataLength;
  return true;
}
