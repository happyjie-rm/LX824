#include "lx824.h"

#include <string.h>

#include "comp_utils.h"

typedef enum {
  LX824_RX_HEADER1,
  LX824_RX_HEADER2,
  LX824_RX_ID,
  LX824_RX_LENGTH,
  LX824_RX_DATA
} lx824_rx_state_t;

static uint8_t lx824_tx_buff_[2][LX824_TX_LEN];
static uint8_t lx824_rx_dma_buff_[LX824_RX_DMA_BUF_LEN];
static uint8_t lx824_rx_fifo_[LX824_RX_FIFO_LEN];
static volatile uint16_t rx_head_ = 0U;
static volatile uint16_t rx_tail_ = 0U;

static lx824_rx_state_t rx_state_ = LX824_RX_HEADER1;
static uint8_t resp_id_ = 0U;
static uint8_t resp_length_ = 0U;
static uint8_t resp_data_[1U + LX824_MAX_PARAM + 1U];
static uint8_t resp_data_len_ = 0U;

static volatile bool resp_ready_ = false;
static uint8_t resp_cmd_ = 0U;
static uint8_t resp_params_[LX824_MAX_PARAM] = {0};
static uint8_t resp_param_len_ = 0U;

static LX824_t *instance_ = NULL;

// ==================== FIFO 索引与底层构帧 ====================

//! 计算环形 FIFO 下一个索引；LX824_RX_FIFO_LEN 必须为 2 的幂。
static uint16_t LX824_NextRxIndex(uint16_t index)
{
  return (uint16_t)((index + 1U) & (LX824_RX_FIFO_LEN - 1U));
}

//! 校验和 = ~(ID + Length + Cmd + Prm1 + ... + PrmN)，溢出取最低字节。
//! frame 指向帧头之后的 ID 字节，len 为参与校验的字节数。
static uint8_t LX824_CalcChecksum(const uint8_t *frame, size_t len)
{
  uint16_t sum = 0U;
  for (size_t i = 0U; i < len; ++i) {
    sum += frame[i];
  }
  return (uint8_t)(~sum);
}

//! 组帧：0x55 0x55 | ID | Length | Cmd | 参数... | Checksum。
//! 返回整包长度，参数非法时返回 0。
static size_t LX824_BuildFrame(uint8_t *frame, uint8_t id, uint8_t cmd,
                               const uint8_t *params, uint8_t param_len)
{
  ASSERT(frame != NULL);
  if (frame == NULL) {
    return 0U;
  }

  ASSERT(param_len <= LX824_MAX_PARAM);
  if (param_len > LX824_MAX_PARAM) {
    return 0U;
  }

  ASSERT((params != NULL) || (param_len == 0U));
  if ((params == NULL) && (param_len != 0U)) {
    return 0U;
  }

  // Length = 参数个数 + 3（含 Length、Cmd、Checksum 自身）
  const uint8_t length = (uint8_t)(param_len + 3U);
  frame[0] = LX824_HEADER;
  frame[1] = LX824_HEADER;
  frame[2] = id;
  frame[3] = length;
  frame[4] = cmd;
  for (uint8_t i = 0U; i < param_len; ++i) {
    frame[5U + i] = params[i];
  }

  // 校验范围从 ID（索引 2）到最后一个参数
  const size_t checksum_index = 5U + param_len;
  frame[checksum_index] = LX824_CalcChecksum(&frame[2], checksum_index - 2U);
  return checksum_index + 1U;
}

// ==================== 字节拆分工具 ====================

//! 取低八位。
static uint8_t LowByte(uint16_t value)
{
  return (uint8_t)(value & 0xFFU);
}

//! 取高八位。
static uint8_t HighByte(uint16_t value)
{
  return (uint8_t)((value >> 8U) & 0xFFU);
}

//! 由低/高八位还原 16 位无符号值。
static uint16_t MakeWord(uint8_t low, uint8_t high)
{
  return (uint16_t)((uint16_t)low | ((uint16_t)high << 8U));
}

// ==================== ISR 到任务环形 FIFO ====================

//! 从 ISR 写入接收 FIFO，满时丢弃当前字节，不覆盖旧数据。
static void LX824_PushRxByteFromISR(uint8_t byte)
{
  const uint16_t head = rx_head_;
  const uint16_t next_head = LX824_NextRxIndex(head);
  if (next_head == rx_tail_) {
    return;  // FIFO 满，丢弃
  }
  lx824_rx_fifo_[head] = byte;
  rx_head_ = next_head;
}

//! 在任务上下文批量取出 FIFO 数据，返回是否取到数据。
static bool LX824_PopRxBatch(uint8_t *buf, uint16_t *count)
{
  *count = 0U;
  taskENTER_CRITICAL();
  while (rx_tail_ != rx_head_) {
    buf[*count] = lx824_rx_fifo_[rx_tail_];
    rx_tail_ = LX824_NextRxIndex(rx_tail_);
    ++(*count);
  }
  taskEXIT_CRITICAL();
  return (*count > 0U);
}

// ==================== 应答帧解析状态机 ====================

//! 逐字节解析舵机应答帧：0x55 0x55 | ID | Length | Cmd | 参数... | Checksum。
//! 收齐一帧后校验和通过，则填充 resp_* 并置位 resp_ready_。
static void LX824_ProcessRxByte(uint8_t byte)
{
  switch (rx_state_) {
    case LX824_RX_HEADER1:
      if (byte == LX824_HEADER) {
        rx_state_ = LX824_RX_HEADER2;
      }
      break;

    case LX824_RX_HEADER2:
      // 连续两个 0x55 才进入帧体，否则重新寻找帧头。
      rx_state_ = (byte == LX824_HEADER) ? LX824_RX_ID : LX824_RX_HEADER1;
      break;

    case LX824_RX_ID:
      resp_id_ = byte;
      rx_state_ = LX824_RX_LENGTH;
      break;

    case LX824_RX_LENGTH:
      // Length = 参数个数 + 3；kData 段需收 Length - 1 字节（Cmd + 参数 + Checksum）。
      if ((byte < 3U) || (((size_t)byte - 1U) > sizeof(resp_data_))) {
        rx_state_ = LX824_RX_HEADER1;  // Length 非法，丢弃当前帧并重新寻找帧头。
        break;
      }
      resp_length_ = byte;
      resp_data_len_ = 0U;
      rx_state_ = LX824_RX_DATA;
      break;

    case LX824_RX_DATA: {
      resp_data_[resp_data_len_] = byte;
      ++resp_data_len_;

      // kData 段总字节数 = Cmd(1) + 参数 + Checksum(1) = Length - 1。
      const uint8_t need = (uint8_t)(resp_length_ - 1U);
      if (resp_data_len_ < need) {
        break;
      }

      // 收齐一帧，重建校验范围：ID + Length + Cmd + 参数（不含 Checksum）。
      uint8_t buf[2U + sizeof(resp_data_)];
      buf[0] = resp_id_;
      buf[1] = resp_length_;
      const uint8_t body_len = (uint8_t)(need - 1U);
      memcpy(&buf[2], resp_data_, body_len);
      const uint8_t expect = LX824_CalcChecksum(buf, 2U + body_len);
      const uint8_t recv_checksum = resp_data_[need - 1U];

      if (expect == recv_checksum) {
        taskENTER_CRITICAL();
        resp_cmd_ = resp_data_[0];
        resp_param_len_ = (uint8_t)(body_len - 1U);
        memcpy(resp_params_, &resp_data_[1], resp_param_len_);
        resp_ready_ = true;
        taskEXIT_CRITICAL();
      }

      rx_state_ = LX824_RX_HEADER1;  // 无论校验是否通过，重新开始下一帧。
      break;
    }

    default:
      rx_state_ = LX824_RX_HEADER1;  // 异常状态恢复，重新寻找帧头。
      break;
  }
}

// ==================== ISR 接收入口 ====================

//! RX DMA 接收回调（中断上下文）：逐字节写入 FIFO，并通知处理任务。
static void LX824_RxCallback(uint8_t *data, size_t size)
{
  if ((data == NULL) || (size == 0U)) {
    return;
  }

  for (size_t i = 0U; i < size; ++i) {
    LX824_PushRxByteFromISR(data[i]);
  }

  if ((instance_ != NULL) && (instance_->thread_alert != NULL)) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(instance_->thread_alert, SIGNAL_LX824_RX_READY, eSetBits,
                       &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// ==================== 请求发送与应答等待 ====================

//! 组帧并通过 TX DMA 发送。
static err_t LX824_SendFrame(LX824_t *self, uint8_t id, uint8_t cmd,
                             const uint8_t *params, uint8_t param_len)
{
  if (self == NULL) {
    return PTR_NULL;
  }

  uint8_t frame[LX824_TX_LEN];
  const size_t frame_len = LX824_BuildFrame(frame, id, cmd, params, param_len);
  if (frame_len == 0U) {
    return ARG_ERR;
  }

  taskENTER_CRITICAL();
  const err_t err = STM32UARTDoubleBufTx_Write(&self->uart_send_, frame, frame_len);
  taskEXIT_CRITICAL();

  return err;
}

//! 读请求统一流程：清空旧应答→发读指令→等待应答信号→校验 Cmd 与参数长度。
//! out_params 接收应答参数，长度必须等于 expect_param_len。
static err_t LX824_Request(LX824_t *self, uint8_t id, uint8_t cmd,
                           uint8_t *out_params, uint8_t expect_param_len,
                           uint32_t timeout_ms)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  ASSERT((out_params != NULL) || (expect_param_len == 0U));
  if ((out_params == NULL) && (expect_param_len != 0U)) {
    return PTR_NULL;
  }

  // 清除可能残留的旧应答与未消费通知，避免误读上一次结果。
  resp_ready_ = false;
  xTaskNotifyStateClear(NULL);

  const err_t send_err = LX824_SendFrame(self, id, cmd, NULL, 0U);
  if (send_err != OK) {
    return send_err;
  }

  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  for (;;) {
    TickType_t now = xTaskGetTickCount();
    if (now >= deadline) {
      return TIMEOUT;
    }

    xTaskNotifyWait(0U, SIGNAL_LX824_RX_READY, NULL, deadline - now);

    uint8_t local_buf[LX824_RX_FIFO_LEN];
    uint16_t count = 0U;
    LX824_PopRxBatch(local_buf, &count);
    for (uint16_t i = 0U; i < count; ++i) {
      LX824_ProcessRxByte(local_buf[i]);  // 逐字节推进应答解析状态机。
    }

    if (resp_ready_) {
      taskENTER_CRITICAL();
      const uint8_t got_cmd = resp_cmd_;
      const uint8_t got_len = resp_param_len_;
      uint8_t snapshot[LX824_MAX_PARAM];
      memcpy(snapshot, resp_params_, sizeof(snapshot));
      resp_ready_ = false;
      taskEXIT_CRITICAL();

      if (got_cmd != cmd) {
        continue;  // 非本次请求的应答，继续等待。
      }
      if (got_len != expect_param_len) {
        return CHECK_ERR;
      }
      if (expect_param_len != 0U) {
        memcpy(out_params, snapshot, expect_param_len);
      }
      return OK;
    }
  }
}

// ==================== 初始化与任务周期处理 ====================

//! 初始化 LX824 对象，绑定 UART 收发通道和静态 DMA 缓冲。
err_t LX824_Init(LX824_t *self, UART_HandleTypeDef *uart_handle)
{
  if (self == NULL) {
    return PTR_NULL;
  }

  memset(self, 0, sizeof(*self));
  self->tx_init_error_ = STM32UARTDoubleBufTx_Init(
      &self->uart_send_, uart_handle,
      (BSP_UART_RawData_t){lx824_tx_buff_[0], sizeof(lx824_tx_buff_[0])},
      (BSP_UART_RawData_t){lx824_tx_buff_[1], sizeof(lx824_tx_buff_[1])},
      NULL);
  self->rx_init_error_ = STM32UART_Init(
      &self->uart_receive_, uart_handle,
      (BSP_UART_RawData_t){lx824_rx_dma_buff_, sizeof(lx824_rx_dma_buff_)},
      LX824_RxCallback);
  instance_ = self;

  return (self->tx_init_error_ != OK) ? self->tx_init_error_ : self->rx_init_error_;
}

//! 启动 TX 双缓冲 DMA 和 RX 循环 DMA。
err_t LX824_Start(LX824_t *self)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->tx_init_error_ != OK) {
    return self->tx_init_error_;
  }
  if (self->rx_init_error_ != OK) {
    return self->rx_init_error_;
  }

  err_t err = STM32UARTDoubleBufTx_SetTxDMA(&self->uart_send_);
  if (err != OK) {
    return err;
  }
  return STM32UART_SetRxDMA(&self->uart_receive_);
}

//! 任务周期入口：等待接收通知，并将 FIFO 中的字节喂入应答解析状态机。
void LX824_Update(LX824_t *self, uint32_t timeout_ms)
{
  if (self == NULL) {
    return;
  }

  xTaskNotifyWait(0U, SIGNAL_LX824_RX_READY, NULL, pdMS_TO_TICKS(timeout_ms));

  uint8_t local_buf[LX824_RX_FIFO_LEN];
  uint16_t count = 0U;
  LX824_PopRxBatch(local_buf, &count);

  for (uint16_t i = 0U; i < count; ++i) {
    LX824_ProcessRxByte(local_buf[i]);  // 逐字节推进应答解析状态机。
  }
}

// ==================== 写指令 API ====================

//! Cmd 1：写角度+时间，舵机立即匀速转动到目标角度。
err_t LX824_MoveTimeWrite(LX824_t *self, uint8_t id, uint16_t angle, uint16_t time_ms)
{
  if ((angle > LX824_ANGLE_MAX) || (time_ms > LX824_TIME_MAX)) {
    return OUT_OF_RANGE;
  }
  const uint8_t params[4] = {LowByte(angle), HighByte(angle),
                             LowByte(time_ms), HighByte(time_ms)};
  return LX824_SendFrame(self, id, LX824_CMD_MOVE_TIME_WRITE, params, 4U);
}

//! Cmd 7：写预设角度+时间，等待 MoveStart 才转动。
err_t LX824_MoveTimeWaitWrite(LX824_t *self, uint8_t id, uint16_t angle, uint16_t time_ms)
{
  if ((angle > LX824_ANGLE_MAX) || (time_ms > LX824_TIME_MAX)) {
    return OUT_OF_RANGE;
  }
  const uint8_t params[4] = {LowByte(angle), HighByte(angle),
                             LowByte(time_ms), HighByte(time_ms)};
  return LX824_SendFrame(self, id, LX824_CMD_MOVE_TIME_WAIT_WRITE, params, 4U);
}

//! Cmd 11：启动预设动作。
err_t LX824_MoveStart(LX824_t *self, uint8_t id)
{
  return LX824_SendFrame(self, id, LX824_CMD_MOVE_START, NULL, 0U);
}

//! Cmd 12：立即停止，停在当前角度。
err_t LX824_MoveStop(LX824_t *self, uint8_t id)
{
  return LX824_SendFrame(self, id, LX824_CMD_MOVE_STOP, NULL, 0U);
}

//! Cmd 13：写舵机 ID（掉电保存）；总线仅挂一个舵机时可用广播 ID 设置。
err_t LX824_IdWrite(LX824_t *self, uint8_t id, uint8_t new_id)
{
  if (new_id > LX824_ID_MAX) {
    return OUT_OF_RANGE;
  }
  const uint8_t params[1] = {new_id};
  return LX824_SendFrame(self, id, LX824_CMD_ID_WRITE, params, 1U);
}

//! Cmd 17：调整偏差（不保存），参数为 signed char，需强制转换为 uint8_t 发送。
err_t LX824_AngleOffsetAdjust(LX824_t *self, uint8_t id, int8_t offset)
{
  if ((offset < LX824_OFFSET_MIN) || (offset > LX824_OFFSET_MAX)) {
    return OUT_OF_RANGE;
  }
  const uint8_t params[1] = {(uint8_t)offset};
  return LX824_SendFrame(self, id, LX824_CMD_ANGLE_OFFSET_ADJUST, params, 1U);
}

//! Cmd 18：保存偏差（掉电保存），无参数。
err_t LX824_AngleOffsetWrite(LX824_t *self, uint8_t id)
{
  return LX824_SendFrame(self, id, LX824_CMD_ANGLE_OFFSET_WRITE, NULL, 0U);
}

//! Cmd 20：写角度限制，要求 min_angle < max_angle。
err_t LX824_AngleLimitWrite(LX824_t *self, uint8_t id, uint16_t min_angle, uint16_t max_angle)
{
  if ((min_angle > LX824_ANGLE_MAX) || (max_angle > LX824_ANGLE_MAX) ||
      (min_angle >= max_angle)) {
    return OUT_OF_RANGE;
  }
  const uint8_t params[4] = {LowByte(min_angle), HighByte(min_angle),
                             LowByte(max_angle), HighByte(max_angle)};
  return LX824_SendFrame(self, id, LX824_CMD_ANGLE_LIMIT_WRITE, params, 4U);
}

//! Cmd 22：写输入电压限制，要求 min_mv < max_mv。
err_t LX824_VinLimitWrite(LX824_t *self, uint8_t id, uint16_t min_mv, uint16_t max_mv)
{
  if ((min_mv < LX824_VIN_MIN) || (max_mv > LX824_VIN_MAX) ||
      (min_mv >= max_mv)) {
    return OUT_OF_RANGE;
  }
  const uint8_t params[4] = {LowByte(min_mv), HighByte(min_mv),
                             LowByte(max_mv), HighByte(max_mv)};
  return LX824_SendFrame(self, id, LX824_CMD_VIN_LIMIT_WRITE, params, 4U);
}

//! Cmd 24：写最高温度限制，范围 50~100 摄氏度。
err_t LX824_TempMaxLimitWrite(LX824_t *self, uint8_t id, uint8_t temp_c)
{
  if ((temp_c < LX824_TEMP_MIN) || (temp_c > LX824_TEMP_MAX_LIMIT)) {
    return OUT_OF_RANGE;
  }
  const uint8_t params[1] = {temp_c};
  return LX824_SendFrame(self, id, LX824_CMD_TEMP_MAX_LIMIT_WRITE, params, 1U);
}

//! Cmd 29：写舵机/电机模式。mode=0 位置控制，mode=1 电机控制。
//! speed 为 signed short，需强制转换为 uint16_t 后拆分发送。
err_t LX824_OrMotorModeWrite(LX824_t *self, uint8_t id, uint8_t mode, int16_t speed)
{
  if (mode > 1U) {
    return ARG_ERR;
  }
  if ((speed < LX824_SPEED_MIN) || (speed > LX824_SPEED_MAX)) {
    return OUT_OF_RANGE;
  }
  const uint16_t speed_u = (uint16_t)speed;
  // 参数 1：模式；参数 2：空值；参数 3/4：速度低/高八位。
  const uint8_t params[4] = {mode, 0U, LowByte(speed_u), HighByte(speed_u)};
  return LX824_SendFrame(self, id, LX824_CMD_OR_MOTOR_MODE_WRITE, params, 4U);
}

//! Cmd 31：装载/卸载电机。0 卸载掉电，1 装载输出力矩。
err_t LX824_LoadOrUnloadWrite(LX824_t *self, uint8_t id, uint8_t load)
{
  if (load > 1U) {
    return ARG_ERR;
  }
  const uint8_t params[1] = {load};
  return LX824_SendFrame(self, id, LX824_CMD_LOAD_OR_UNLOAD_WRITE, params, 1U);
}

//! Cmd 33：LED 亮灭。0 常亮，1 常灭。
err_t LX824_LedCtrlWrite(LX824_t *self, uint8_t id, uint8_t off)
{
  if (off > 1U) {
    return ARG_ERR;
  }
  const uint8_t params[1] = {off};
  return LX824_SendFrame(self, id, LX824_CMD_LED_CTRL_WRITE, params, 1U);
}

//! Cmd 35：写 LED 故障报警值，范围 0~7。
err_t LX824_LedErrorWrite(LX824_t *self, uint8_t id, uint8_t fault)
{
  if (fault > LX824_LED_ERROR_MAX) {
    return OUT_OF_RANGE;
  }
  const uint8_t params[1] = {fault};
  return LX824_SendFrame(self, id, LX824_CMD_LED_ERROR_WRITE, params, 1U);
}

// ==================== 读指令 API（阻塞式请求-应答） ====================

//! Cmd 2：读角度+时间。
err_t LX824_MoveTimeRead(LX824_t *self, uint8_t id, uint16_t *angle, uint16_t *time_ms)
{
  if ((angle == NULL) || (time_ms == NULL)) {
    return PTR_NULL;
  }
  uint8_t params[4];
  err_t err = LX824_Request(self, id, LX824_CMD_MOVE_TIME_READ, params, 4U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *angle = MakeWord(params[0], params[1]);
  *time_ms = MakeWord(params[2], params[3]);
  return OK;
}

//! Cmd 8：读预设角度+时间。
err_t LX824_MoveTimeWaitRead(LX824_t *self, uint8_t id, uint16_t *angle, uint16_t *time_ms)
{
  if ((angle == NULL) || (time_ms == NULL)) {
    return PTR_NULL;
  }
  uint8_t params[4];
  err_t err = LX824_Request(self, id, LX824_CMD_MOVE_TIME_WAIT_READ, params,
                            4U, LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *angle = MakeWord(params[0], params[1]);
  *time_ms = MakeWord(params[2], params[3]);
  return OK;
}

//! Cmd 14：读舵机 ID。广播 ID(0xFE) 时舵机仍会应答，限总线仅挂一个舵机。
err_t LX824_IdRead(LX824_t *self, uint8_t id, uint8_t *servo_id)
{
  if (servo_id == NULL) {
    return PTR_NULL;
  }
  uint8_t params[1];
  err_t err = LX824_Request(self, id, LX824_CMD_ID_READ, params, 1U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *servo_id = params[0];
  return OK;
}

//! Cmd 19：读偏差，uint8_t 还原为 signed char。
err_t LX824_AngleOffsetRead(LX824_t *self, uint8_t id, int8_t *offset)
{
  if (offset == NULL) {
    return PTR_NULL;
  }
  uint8_t params[1];
  err_t err = LX824_Request(self, id, LX824_CMD_ANGLE_OFFSET_READ, params, 1U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *offset = (int8_t)params[0];
  return OK;
}

//! Cmd 21：读角度限制。
err_t LX824_AngleLimitRead(LX824_t *self, uint8_t id, uint16_t *min_angle, uint16_t *max_angle)
{
  if ((min_angle == NULL) || (max_angle == NULL)) {
    return PTR_NULL;
  }
  uint8_t params[4];
  err_t err = LX824_Request(self, id, LX824_CMD_ANGLE_LIMIT_READ, params, 4U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *min_angle = MakeWord(params[0], params[1]);
  *max_angle = MakeWord(params[2], params[3]);
  return OK;
}

//! Cmd 23：读输入电压限制。
err_t LX824_VinLimitRead(LX824_t *self, uint8_t id, uint16_t *min_mv, uint16_t *max_mv)
{
  if ((min_mv == NULL) || (max_mv == NULL)) {
    return PTR_NULL;
  }
  uint8_t params[4];
  err_t err = LX824_Request(self, id, LX824_CMD_VIN_LIMIT_READ, params, 4U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *min_mv = MakeWord(params[0], params[1]);
  *max_mv = MakeWord(params[2], params[3]);
  return OK;
}

//! Cmd 25：读最高温度限制。
err_t LX824_TempMaxLimitRead(LX824_t *self, uint8_t id, uint8_t *temp_c)
{
  if (temp_c == NULL) {
    return PTR_NULL;
  }
  uint8_t params[1];
  err_t err = LX824_Request(self, id, LX824_CMD_TEMP_MAX_LIMIT_READ, params,
                            1U, LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *temp_c = params[0];
  return OK;
}

//! Cmd 26：读实时温度。
err_t LX824_TempRead(LX824_t *self, uint8_t id, uint8_t *temp_c)
{
  if (temp_c == NULL) {
    return PTR_NULL;
  }
  uint8_t params[1];
  err_t err = LX824_Request(self, id, LX824_CMD_TEMP_READ, params, 1U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *temp_c = params[0];
  return OK;
}

//! Cmd 27：读实时输入电压，单位 mV。
err_t LX824_VinRead(LX824_t *self, uint8_t id, uint16_t *mv)
{
  if (mv == NULL) {
    return PTR_NULL;
  }
  uint8_t params[2];
  err_t err = LX824_Request(self, id, LX824_CMD_VIN_READ, params, 2U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *mv = MakeWord(params[0], params[1]);
  return OK;
}

//! Cmd 28：读实时角度位置，uint16_t 还原为 signed short，可能为负。
err_t LX824_PosRead(LX824_t *self, uint8_t id, int16_t *pos)
{
  if (pos == NULL) {
    return PTR_NULL;
  }
  uint8_t params[2];
  err_t err = LX824_Request(self, id, LX824_CMD_POS_READ, params, 2U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *pos = (int16_t)MakeWord(params[0], params[1]);
  return OK;
}

//! Cmd 30：读舵机/电机模式与速度。
err_t LX824_OrMotorModeRead(LX824_t *self, uint8_t id, uint8_t *mode, int16_t *speed)
{
  if ((mode == NULL) || (speed == NULL)) {
    return PTR_NULL;
  }
  uint8_t params[4];
  err_t err = LX824_Request(self, id, LX824_CMD_OR_MOTOR_MODE_READ, params,
                            4U, LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *mode = params[0];  // 参数 2 为空值，忽略。
  *speed = (int16_t)MakeWord(params[2], params[3]);
  return OK;
}

//! Cmd 32：读电机装载/卸载状态。
err_t LX824_LoadOrUnloadRead(LX824_t *self, uint8_t id, uint8_t *load)
{
  if (load == NULL) {
    return PTR_NULL;
  }
  uint8_t params[1];
  err_t err = LX824_Request(self, id, LX824_CMD_LOAD_OR_UNLOAD_READ, params,
                            1U, LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *load = params[0];
  return OK;
}

//! Cmd 34：读 LED 亮灭状态。
err_t LX824_LedCtrlRead(LX824_t *self, uint8_t id, uint8_t *off)
{
  if (off == NULL) {
    return PTR_NULL;
  }
  uint8_t params[1];
  err_t err = LX824_Request(self, id, LX824_CMD_LED_CTRL_READ, params, 1U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *off = params[0];
  return OK;
}

//! Cmd 36：读 LED 故障报警值。
err_t LX824_LedErrorRead(LX824_t *self, uint8_t id, uint8_t *fault)
{
  if (fault == NULL) {
    return PTR_NULL;
  }
  uint8_t params[1];
  err_t err = LX824_Request(self, id, LX824_CMD_LED_ERROR_READ, params, 1U,
                            LX824_READ_TIMEOUT_MS);
  if (err != OK) {
    return err;
  }
  *fault = params[0];
  return OK;
}
