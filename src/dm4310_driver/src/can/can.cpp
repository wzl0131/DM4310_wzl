#include "can/can.hpp"
#include <cstring>      // memset / memcpy
#define CAN_EFF_FLAG 0x80000000U  // 扩展帧标志位

// CAN::CanBus - 设置发送帧参数实现
namespace CAN {
void CanBus::SetCanSendFrame(can_frame& frame, uint8_t dlen, uint32_t can_id, uint8_t* data) {
    frame.can_id = can_id;
    frame.dlc = (dlen > 8) ? 8 : dlen;  // 限制最大8字节
    memset(frame.data, 0, 8);           // 初始化数据域
    if (data != nullptr) {
        memcpy(frame.data, data, frame.dlc);
    }
    this->send_frame = frame;  // 缓存到总线对象
}

// CAN::CanBus - 发送帧实现
void CanBus::Transmit() {
    // 提取纯CAN ID（移除扩展帧标志）
    uint32_t pure_id = this->send_frame.can_id & ~CAN_EFF_FLAG;
    // 调用通用发送函数
    CanSendMsg(this->dev_handle,
               this->channel,
               this->send_frame.data,
               pure_id,
               (this->send_frame.can_id & CAN_EFF_FLAG) != 0);
}
}

// 通用CAN发送函数实现（复用dmcan_device_send_can）
void CanSendMsg(dmcan_device_handle* dev_handle, 
                uint8_t channel, 
                uint8_t* data, 
                uint32_t id, 
                bool is_extended, 
                bool canfd, 
                bool rtr, 
                bool brs) {
    // 入参校验
    if (dev_handle == nullptr || data == nullptr || channel > 3) {
        std::cerr << "[ERROR] CanSendMsg: invalid parameters!" << std::endl;
        return;
    }

    // 调用dmcan底层发送接口
    bool send_ok = dmcan_device_send_can(
        dev_handle,
        channel,
        id,
        canfd,
        is_extended,
        rtr,
        brs,
        8,  // 电机通信固定8字节数据长度
        data
    );

    if (!send_ok) {
        std::cerr << "[ERROR] CanSendMsg failed: ch=" << (int)channel 
                  << ", id=0x" << std::hex << id << std::dec << std::endl;
    }
}

// ============================================================
//  设备打开 / 关闭
// ============================================================
bool OpenCanDevice(dmcan_context** ctx,
                   dmcan_device_handle** dev,
                   uint8_t ch,
                   bool canfd_channel,   // ⭐FD 修改：改名为"通道级"
                   uint32_t baudrate,
                   uint32_t data_baud,
                   float sp,
                   float fd_sp) {        // ⭐FD 新增：数据段采样点
    if (ctx == nullptr || dev == nullptr) {
        std::cerr << "[ERROR] OpenCanDevice: null pointer" << std::endl;
        return false;
    }
    *ctx = nullptr;
    *dev = nullptr;

    // ---- ① 建上下文（必须第一个调） ----
    dmcan_context_create(ctx);
    if (*ctx == nullptr) {
        std::cerr << "[ERROR] dmcan_context_create failed" << std::endl;
        return false;
    }
    dmcan_print_version(*ctx);

    // ---- ② 只找 LINKX4C 这一种设备 ----
    const int dev_cnt = dmcan_find_devices_with_type(*ctx, LINKX4C);
    std::cout << "[INFO] LINKX4C 设备数 = " << dev_cnt << std::endl;
    if (dev_cnt <= 0) {
        std::cerr << "[ERROR] 没找到 LINKX4C。检查：USB 插好没 / 有没有 sudo / lsusb 能不能看到"
                  << std::endl;
        return false;
    }
    dmcan_show_all_devices(*ctx);

    // ---- ③ 取句柄并打开 ----
    if (!dmcan_device_get(*ctx, dev, 0) || *dev == nullptr) {
        std::cerr << "[ERROR] dmcan_device_get 失败" << std::endl;
        return false;
    }
    if (!dmcan_device_open(*dev)) {
        std::cerr << "[ERROR] dmcan_device_open 失败" << std::endl;
        return false;
    }
    dmcan_device_print_version(*dev);

    // ---- ④ 使能通道 + 配波特率 ----
    // ⚠️ 不要用本文件里的 dmcan_init_channels()：
    //    那个函数是 1~4 基的（全仓库无调用者，历史遗留），
    //    而实际通道号是 0~3。
    if (ch > 3) {
        std::cerr << "[ERROR] 通道号必须 0~3，现在是 " << static_cast<int>(ch) << std::endl;
        return false;
    }
    if (!dmcan_device_enable_channel(*dev, ch)) {
        std::cerr << "[ERROR] 通道 " << static_cast<int>(ch) << " 使能失败" << std::endl;
        return false;
    }

    dmcan_channel_can_info_t info = {};
    if (dmcan_device_get_channel_baudrate(*dev, ch, &info)) {
        // ⭐FD 修改：通道级 FD 模式 + 两段波特率/采样点都由调用方决定
        //    ⚠️ 这里的 canfd 是【通道级】：
        //       true  = 通道控制器工作在 FD 模式（向下兼容经典帧，DM 电机照样能收）
        //       false = 纯经典模式
        //    它【不决定】具体某一帧是不是 FD 帧 —— 那是 send_can() 的参数
        info.canfd          = canfd_channel;                  // ⭐FD 修改：通道级
        info.can_baudrate   = baudrate;                       // 仲裁段（两种模式都用）
        info.canfd_baudrate = data_baud;                      // ⭐FD 新增：数据段波特率
        info.can_sp         = sp;                             // 仲裁段采样点
        info.canfd_sp       = fd_sp;                          // ⭐FD 新增：数据段采样点
        if (!dmcan_device_set_channel_baudrate(*dev, ch, info)) {
            std::cerr << "[WARN] 设置波特率失败，沿用默认（可能就是 1 Mbps）" << std::endl;
        }
    }

    // ⭐FD 修改：日志区分通道模式
    if (canfd_channel) {
        std::cout << "[INFO] 通道 " << static_cast<int>(ch) << " 就绪，"
                  << "FD 模式  仲裁段 " << baudrate << " bps (sp " << sp << ")，"
                  << "数据段 " << data_baud << " bps (sp " << fd_sp << ")"
                  << std::endl;
    } else {
        std::cout << "[INFO] 通道 " << static_cast<int>(ch) << " 就绪，"
                  << "经典模式 " << baudrate << " bps，采样点 " << sp << std::endl;
    }
    return true;
}

void CloseCanDevice(dmcan_context* ctx, dmcan_device_handle* dev) {
    if (dev != nullptr) {
        dmcan_device_close(dev);
    }
    if (ctx != nullptr) {
        dmcan_context_destroy(ctx);
    }
}
