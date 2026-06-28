#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_uart.h"
#include "comp_cmd.h"
#include "task.h"

// ==================== 协议常量 ====================

#define LX824_HEADER (0x55u)        //! 帧头字节，连续两个 0x55 表示数据包到达
#define LX824_BROADCAST_ID (0xFEu)  //! 广播 ID，所有舵机接收；读 ID 时要求总线仅挂一个舵机
#define LX824_ID_MAX (253u)         //! 普通舵机 ID 上限，0xFE 保留为广播 ID

#define LX824_HEADER_LEN (2u)       //! 帧头长度
#define LX824_MAX_PARAM (4u)        //! 协议单包最多 4 个参数
//! 整包 = 帧头2 + ID1 + Length1 + Cmd1 + 参数N + Checksum1
#define LX824_MAX_FRAME_LEN (LX824_HEADER_LEN + 3u + LX824_MAX_PARAM + 1u)

#define LX824_ANGLE_MAX (1000u)     //! 角度参数范围 0~1000，对应 0~240 度
#define LX824_TIME_MAX (30000u)     //! 运动时间范围 0~30000ms
#define LX824_OFFSET_MIN (-125)     //! 偏差范围 -125~125，对应约 -30~30 度
#define LX824_OFFSET_MAX (125)
#define LX824_SPEED_MIN (-1000)     //! 电机模式速度范围 -1000~1000
#define LX824_SPEED_MAX (1000)
#define LX824_VIN_MIN (4500u)       //! 输入电压限制范围 4500~12000mV
#define LX824_VIN_MAX (12000u)
#define LX824_TEMP_MIN (50u)        //! 最高温度限制范围 50~100 摄氏度
#define LX824_TEMP_MAX_LIMIT (100u)
#define LX824_LED_ERROR_MAX (7u)    //! LED 故障报警值范围 0~7

#define LX824_TX_LEN LX824_MAX_FRAME_LEN       //! TX 单帧缓冲长度
#define LX824_RX_DMA_BUF_LEN (64u)             //! RX DMA 原始接收缓冲长度
#define LX824_RX_FIFO_LEN (128u)               //! ISR 到任务的环形 FIFO 长度，必须为 2 的幂
#define LX824_READ_TIMEOUT_MS (20u)            //! 读指令默认等待应答超时
#define SIGNAL_LX824_RX_READY (1u << 8)        //! ISR 收到字节后通知任务处理的信号位

#ifdef __cplusplus
extern "C" {
#endif

//! 指令值，详见 LX-824 协议表：奇数多为写指令，偶数多为对应读指令。
typedef enum {
  LX824_CMD_MOVE_TIME_WRITE = 1,        //! 写角度+时间，立即转动
  LX824_CMD_MOVE_TIME_READ = 2,         //! 读角度+时间
  LX824_CMD_MOVE_TIME_WAIT_WRITE = 7,   //! 写预设角度+时间，等待 MOVE_START
  LX824_CMD_MOVE_TIME_WAIT_READ = 8,    //! 读预设角度+时间
  LX824_CMD_MOVE_START = 11,            //! 启动等待中的预设动作
  LX824_CMD_MOVE_STOP = 12,             //! 立即停止
  LX824_CMD_ID_WRITE = 13,              //! 写 ID，掉电保存
  LX824_CMD_ID_READ = 14,               //! 读 ID
  LX824_CMD_ANGLE_OFFSET_ADJUST = 17,   //! 调整偏差，不保存
  LX824_CMD_ANGLE_OFFSET_WRITE = 18,    //! 保存偏差，掉电保存
  LX824_CMD_ANGLE_OFFSET_READ = 19,     //! 读偏差
  LX824_CMD_ANGLE_LIMIT_WRITE = 20,     //! 写角度限制
  LX824_CMD_ANGLE_LIMIT_READ = 21,      //! 读角度限制
  LX824_CMD_VIN_LIMIT_WRITE = 22,       //! 写输入电压限制
  LX824_CMD_VIN_LIMIT_READ = 23,        //! 读输入电压限制
  LX824_CMD_TEMP_MAX_LIMIT_WRITE = 24,  //! 写最高温度限制
  LX824_CMD_TEMP_MAX_LIMIT_READ = 25,   //! 读最高温度限制
  LX824_CMD_TEMP_READ = 26,             //! 读实时温度
  LX824_CMD_VIN_READ = 27,              //! 读实时输入电压
  LX824_CMD_POS_READ = 28,              //! 读实时角度位置
  LX824_CMD_OR_MOTOR_MODE_WRITE = 29,   //! 写舵机/电机模式
  LX824_CMD_OR_MOTOR_MODE_READ = 30,    //! 读舵机/电机模式
  LX824_CMD_LOAD_OR_UNLOAD_WRITE = 31,  //! 写电机装载/卸载
  LX824_CMD_LOAD_OR_UNLOAD_READ = 32,   //! 读电机装载/卸载
  LX824_CMD_LED_CTRL_WRITE = 33,        //! 写 LED 亮灭
  LX824_CMD_LED_CTRL_READ = 34,         //! 读 LED 亮灭
  LX824_CMD_LED_ERROR_WRITE = 35,       //! 写 LED 故障报警
  LX824_CMD_LED_ERROR_READ = 36         //! 读 LED 故障报警
} lx824_cmd_t;

//! LX824 总线舵机对象。
//! TX 使用双缓冲 DMA，RX 使用循环 DMA + 软件 FIFO；thread_alert 用于 ISR 通知任务解析应答。
typedef struct {
  TaskHandle_t thread_alert;          //! 接收事件通知的 FreeRTOS 任务句柄
  STM32UARTDoubleBufTx_t uart_send_;  //! 舵机指令发送通道
  STM32UART_t uart_receive_;          //! 舵机应答接收通道
  err_t tx_init_error_;               //! TX BSP 初始化结果，Start 前保留
  err_t rx_init_error_;               //! RX BSP 初始化结果，Start 前保留
} LX824_t;

//! 初始化 LX824 对象，绑定 UART 句柄和收发 DMA 缓冲。
err_t LX824_Init(LX824_t *self, UART_HandleTypeDef *uart_handle);
//! 启动 TX/RX DMA，必须在任务循环前调用。
err_t LX824_Start(LX824_t *self);
//! 任务周期入口：等待 RX 通知、排空 FIFO、推进应答解析状态机。
void LX824_Update(LX824_t *self, uint32_t timeout_ms);

// ==================== 写指令 API ====================

err_t LX824_MoveTimeWrite(LX824_t *self, uint8_t id, uint16_t angle, uint16_t time_ms);
err_t LX824_MoveTimeWaitWrite(LX824_t *self, uint8_t id, uint16_t angle, uint16_t time_ms);
err_t LX824_MoveStart(LX824_t *self, uint8_t id);
err_t LX824_MoveStop(LX824_t *self, uint8_t id);
err_t LX824_IdWrite(LX824_t *self, uint8_t id, uint8_t new_id);
err_t LX824_AngleOffsetAdjust(LX824_t *self, uint8_t id, int8_t offset);
err_t LX824_AngleOffsetWrite(LX824_t *self, uint8_t id);
err_t LX824_AngleLimitWrite(LX824_t *self, uint8_t id, uint16_t min_angle, uint16_t max_angle);
err_t LX824_VinLimitWrite(LX824_t *self, uint8_t id, uint16_t min_mv, uint16_t max_mv);
err_t LX824_TempMaxLimitWrite(LX824_t *self, uint8_t id, uint8_t temp_c);
err_t LX824_OrMotorModeWrite(LX824_t *self, uint8_t id, uint8_t mode, int16_t speed);
err_t LX824_LoadOrUnloadWrite(LX824_t *self, uint8_t id, uint8_t load);
err_t LX824_LedCtrlWrite(LX824_t *self, uint8_t id, uint8_t off);
err_t LX824_LedErrorWrite(LX824_t *self, uint8_t id, uint8_t fault);

// ==================== 读指令 API（阻塞式请求-应答） ====================
//! 读接口须在 FreeRTOS 任务上下文调用，会等待匹配 Cmd 的合法应答或超时。

err_t LX824_MoveTimeRead(LX824_t *self, uint8_t id, uint16_t *angle, uint16_t *time_ms);
err_t LX824_MoveTimeWaitRead(LX824_t *self, uint8_t id, uint16_t *angle, uint16_t *time_ms);
err_t LX824_IdRead(LX824_t *self, uint8_t id, uint8_t *servo_id);
err_t LX824_AngleOffsetRead(LX824_t *self, uint8_t id, int8_t *offset);
err_t LX824_AngleLimitRead(LX824_t *self, uint8_t id, uint16_t *min_angle, uint16_t *max_angle);
err_t LX824_VinLimitRead(LX824_t *self, uint8_t id, uint16_t *min_mv, uint16_t *max_mv);
err_t LX824_TempMaxLimitRead(LX824_t *self, uint8_t id, uint8_t *temp_c);
err_t LX824_TempRead(LX824_t *self, uint8_t id, uint8_t *temp_c);
err_t LX824_VinRead(LX824_t *self, uint8_t id, uint16_t *mv);
err_t LX824_PosRead(LX824_t *self, uint8_t id, int16_t *pos);
err_t LX824_OrMotorModeRead(LX824_t *self, uint8_t id, uint8_t *mode, int16_t *speed);
err_t LX824_LoadOrUnloadRead(LX824_t *self, uint8_t id, uint8_t *load);
err_t LX824_LedCtrlRead(LX824_t *self, uint8_t id, uint8_t *off);
err_t LX824_LedErrorRead(LX824_t *self, uint8_t id, uint8_t *fault);

#ifdef __cplusplus
}
#endif
