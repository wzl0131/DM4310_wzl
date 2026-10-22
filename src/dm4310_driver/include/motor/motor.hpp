#pragma once
#include "can/can.hpp"
#include <cstdint>
#include <cstring>
#include <iostream>
#include <chrono>
#include <atomic>
#include <map>
#include <array>
#include <mutex>

namespace MOTOR {

// 电机类型
enum class MotorType {
    DM8006,
    DM8009,
    DM4310,
    DM4340,
    RS06
};

// 控制模式
enum class MotorControlMode {
    MIT_Control,
    Position_Control,
    Velocity_Control,
    TorPosMix_Control
};

// 编码器基类
// position/velocity/torque 等字段由 CAN 接收回调写，由控制循环线程读，
// 必须用 mutex 保护，避免数据竞争。
//
// ⭐ 只保留电机反馈帧里真的有的量（DM4310 反馈帧共 8 字节：
//    ID|ERR / pos / vel / torque / MOS 温度 / 转子温度）。
//    原版里的 accelerate / current / power / consequence_position /
//    init_offset / round 在 DM 电机上拿不到，已删。
class Encoder {
public:
    double position;      // rad（输出侧角度，驱动板内部已按减速比换算）
    double velocity;      // rad/s
    double torque;        // N·m
    double tem_mos;       // MOS 温度 ℃
    double tem_rotor;     // 转子温度 ℃
    int    error_code;    // 反馈帧 D[0] 高 4 位的 ERR

    // 保护编码器字段的互斥锁（CAN 回调线程 vs 控制循环线程）
    mutable std::mutex mutex;

    Encoder() {
        position = velocity = torque = 0;
        tem_mos = tem_rotor = 0;
        error_code = 0;
    }
    virtual ~Encoder() = default;
};

// 指令基类
// ⭐ 只保留 MIT 模式真正用到的 5 个量；原版里的 kp_max/kd_max 和各种
//    min/max 限位在 DMCommand 里用构造参数 clamp 实现，所以删掉避免重复。
class Command {
public:
    double target_position;   // rad
    double target_velocity;   // rad/s
    double target_torque;     // N·m
    double kp;                // 刚度
    double kd;                // 阻尼

    Command() {
        target_position = target_velocity = target_torque = kp = kd = 0;
    }
    virtual ~Command() = default;
    virtual void CommandPack(uint8_t* data, uint32_t& ext_id) = 0;
};

// 电机基类（核心：完全复用CAN接口）
class Motor {
public:
    uint32_t rec_can_id;
    uint32_t send_can_id;
    CAN::CanBus* canbus;
    MotorType motor_type;
    MotorControlMode control_mode;
    Encoder encoder;
    Command& command;
    uint8_t send_data[8] = {0};

    // ⭐ 原版有 direction / offset 两个字段，但队里自己在 motorgroups.cpp 里写了：
    //    "offset/direction are no longer stored on the motor;
    //     the transform lives in RealRobotHardware::readMotors()/writeMotors()"
    //    所以这里删掉，方向/零位改由业务层（本项目的 dm4310_node）处理。

    Motor(MotorType type, MotorControlMode mode,
          uint32_t rec_id, uint32_t send_id,
          CAN::CanBus* bus, Command& cmd)
        // 初始化顺序必须与上面成员声明顺序一致，否则 -Wreorder 警告
        : rec_can_id(rec_id), send_can_id(send_id),
          canbus(bus), motor_type(type), control_mode(mode),
          command(cmd) {}

    virtual ~Motor() = default;

    // 纯虚接口
    virtual void CanMsgParse(const can_frame& frame) = 0;
    virtual void MotorControl() = 0;
    virtual void MotorEnable() = 0;
    virtual void MotorDisable() = 0;

    // 统一CAN发送：完全复用can.cpp通用接口
    virtual void SendCanMsg(uint8_t* data, uint32_t id = 0, bool is_extended = false) {
        if (!canbus || !data) return;
        uint32_t real_id = id ? id : send_can_id;
        // ⭐FD 修改：额外把 canbus->canfd_frame 传下去
        //    注意是【帧级】标志（这一帧用经典还是 FD 格式），
        //    不是通道级 —— 通道开不开 FD 模式由 OpenCanDevice 管
        // ⭐FD 修改：把 BRS 也带下去（决定数据段是否真的提速）
        CanSendMsg(canbus->dev_handle, canbus->channel, data, real_id,
                   is_extended, canbus->canfd_frame,
                   /*rtr=*/false, canbus->canfd_brs);
    }

    // ---- 超时安全机制 ----
    // CAN回调线程写，控制循环线程读，全 atomic 无锁
    // 注意：encoder.* 字段（位置/速度/力矩）不是 atomic，访问需加 encoder.mutex
    std::atomic<bool> feedback_timed_out{false};
    std::atomic<bool> feedback_received_once{false};
    std::atomic<int64_t> last_feedback_time_ns{0};   // 纳秒时间戳
    std::atomic<uint32_t> timeout_ms{1000};           // 可配置超时阈值，默认1s

    // 在 CanMsgParse 开头调用，原子记录当前时间
    void StampFeedbackTime() {
        auto now = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();
        last_feedback_time_ns.store(ns, std::memory_order_release);
        feedback_received_once.store(true, std::memory_order_release);
    }

    // 由控制循环调用，检查是否超时
    bool CheckTimeout() const {
        if (!feedback_received_once.load(std::memory_order_acquire)) {
            return false;  // 尚未收到反馈，不判定超时
        }
        auto now = std::chrono::high_resolution_clock::now();
        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();
        int64_t last_ns = last_feedback_time_ns.load(std::memory_order_acquire);
        uint32_t tmo = timeout_ms.load(std::memory_order_acquire);
        int64_t elapsed_ms = (now_ns - last_ns) / 1'000'000;
        return (elapsed_ms > static_cast<int64_t>(tmo));
    }

    // 重新使能时调用，清除超时状态并刷新时间戳
    void ResetTimeout() {
        feedback_timed_out.store(false, std::memory_order_release);
        auto now = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();
        last_feedback_time_ns.store(ns, std::memory_order_release);
        feedback_received_once.store(true, std::memory_order_release);
    }
};

// 数据转换工具函数
inline int16_t FloatToInt(float x, float min, float max, int bits) {
    return (int16_t)((x - min) * ((1 << bits) - 1) / (max - min));
}

inline float IntToFloat(uint16_t x, float min, float max, int bits) {
    return (float)x * (max - min) / ((1 << bits) - 1) + min;
}

inline int16_t FloatToUint(float x, float min, float max, int bits) {
    return FloatToInt(x, min, max, bits);
}

inline float UintToFloat(uint16_t x, float min, float max, int bits) {
    return IntToFloat(x, min, max, bits);
}



// 电机参数限位表 {P_MAX, V_MAX, T_MAX, KD_MAX}
extern const std::map<MotorType, std::array<double, 4>> motor_limit_params;

double GetMotorPMax(MotorType type);
double GetMotorVMax(MotorType type);

// MIT 力矩映射半幅 (T_MAX, Nm); 命令打包与反馈解析必须一致
double GetMotorTMax(MotorType type);
double GetMotorKdMax(MotorType type);

} // namespace MOTOR