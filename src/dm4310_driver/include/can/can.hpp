#pragma once

#include "dmcan.h"
#include <cstdint>
#include <cstdio>
#include <iostream>

// 适配motor模块的can_frame结构（兼容原有代码逻辑）
struct can_frame {
    uint32_t can_id;  // CAN ID (包含扩展帧标志CAN_EFF_FLAG)
    uint8_t dlc;      // 数据长度(0-8)
    uint8_t data[8];  // CAN数据域
};

// CAN总线封装类（适配motor模块的CanBus接口）
namespace CAN {
class CanBus {
public:
    dmcan_device_handle* dev_handle;  // dmcan设备句柄
    uint8_t channel;                  // CAN通道号
    // ⭐FD 新增：帧格式标志（注意：这是【帧级】，和通道级是两回事）
    //   false = 这一帧按【经典 CAN 2.0】格式发（DM 系列电机只认这个）
    //   true  = 这一帧按【CAN FD】格式发（FDF=1，需要从站也支持 FD）
    //
    //   ⚠️ 区分两个 canfd：
    //     ① 通道级 dmcan_channel_can_info_t.canfd  —— 通道控制器是否工作在 FD 模式
    //        开 FD 模式的控制器【向下兼容经典帧】，所以开了也不影响经典帧收发
    //     ② 帧级  dmcan_device_send_can(..., canfd, ...) —— 这一帧用哪种格式
    //        这个才决定从站能不能解析
    //
    //   典型正确组合：通道 FD 模式 = true，帧格式 = false（发给 DM 电机）
    bool    canfd_frame;
    // ⭐FD 新增：BRS（Bit Rate Switch，速率切换开关）
    //   true  = 数据段切到 canfd_baudrate（真正提速，比如 5 Mbps）
    //   false = 数据段仍跑仲裁段速率（只是"格式是 FD"，速度没变）
    bool    canfd_brs;
    can_frame send_frame;             // 待发送的CAN帧

    // ⭐FD 修改：构造函数新增 fd_frame / fd_brs 参数
    CanBus(dmcan_device_handle* handle, uint8_t ch,
           bool fd_frame = false, bool fd_brs = false)
    : dev_handle(handle), channel(ch), canfd_frame(fd_frame),
      canfd_brs(fd_brs), send_frame{} {}

    // 设置要发送的CAN帧参数
    void SetCanSendFrame(can_frame& frame, uint8_t dlen, uint32_t can_id, uint8_t* data);
    
    // 执行CAN帧发送
    void Transmit();
};
}

// 通用CAN发送函数（封装dmcan底层接口，适配电机控制场景）
// 参数说明：
//   dev_handle: dmcan设备句柄
//   channel: CAN通道号(0-3)
//   data: 8字节发送数据
//   id: CAN ID（若is_extended=true，需传入纯ID，扩展标志由参数控制）
//   is_extended: 是否为扩展帧(29bit ID)
//   canfd: 是否启用CANFD
//   rtr: 是否为远程帧
//   brs: CANFD波特率切换标志
void CanSendMsg(dmcan_device_handle* dev_handle, 
                uint8_t channel, 
                uint8_t* data, 
                uint32_t id = 0, 
                bool is_extended = false, 
                bool canfd = false, 
                bool rtr = false, 
                bool brs = false);

// ============================================================
//  设备打开 / 关闭
//
//  队里是在 real_robot_hardware.cpp 的 LinkInit() 里做的，
//  我们这里是单电机，单独抽出来方便复用。
// ============================================================

// 打开 DM-LinkX-4C
//   流程：建上下文 → 找 LINKX4C 设备 → 打开 → 使能通道 → 配波特率
//
// 参数：
//   ctx        - 输出，SDK 上下文句柄（关闭时要传给 CloseCanDevice）
//   dev        - 输出，设备句柄
//   ch         - 通道号 0~3
// ⭐FD 新增参数（注意区分"通道级"和"帧级"）：
//   canfd_channel - 通道控制器是否工作在 CAN FD 模式（对应 info.canfd）
//                   ✅ 建议 true：FD 模式的控制器【向下兼容经典帧】，
//                      开了之后仍然能收发经典帧，不影响 DM 电机
//                   false = 纯经典模式
//   data_baud     - FD 帧的【数据段】波特率（canfd_channel=false 时不用）
//                   常用 2000000 / 5000000；LinkX-4C 最高支持 8 Mbps
//   fd_sp         - FD 帧的【数据段】采样点（之前漏了，必须和 data_baud 配套）
//                   不设的话默认 0，会算出非法位时序
//   baudrate      - 【仲裁段】波特率，两种模式都用，达妙固定 1 Mbps
//   sp            - 【仲裁段】采样点，达妙固定 0.75
//
// ⚠️ 帧级 canfd 不在这里配 —— 那是每帧的事，见 CanBus::canfd_frame
// 返回：true 成功；false 失败（已打印原因）
bool OpenCanDevice(dmcan_context** ctx,
                   dmcan_device_handle** dev,
                   uint8_t ch = 0,
                   bool canfd_channel = true,     // ⭐FD 新增：通道开 FD 模式
                   uint32_t baudrate = 1000000,   // 仲裁段
                   uint32_t data_baud = 5000000,  // ⭐FD 新增：数据段
                   float sp = 0.75f,              // 仲裁段采样点
                   float fd_sp = 0.75f);          // ⭐FD 新增：数据段采样点

// 关闭：关设备 + 销毁上下文
void CloseCanDevice(dmcan_context* ctx, dmcan_device_handle* dev);
