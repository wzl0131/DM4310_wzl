#pragma once
#include "motor.hpp"

namespace MOTOR {

class DMCommand : public Command {
public:
    // ⚠️ 原版这里有个 static constexpr double REDUCTION = 9.0，
    //    它只是记录减速比、当前不参与任何运算（达妙驱动板内部已按减速比换算，
    //    反馈的 pos/vel 就是输出侧的值），所以删掉避免误解。
    DMCommand(double p_max, double v_max, double t_max, double kd_max)
        : p_max_(p_max), v_max_(v_max), t_max_(t_max), kd_max_(kd_max) {}
    void CommandPack(uint8_t* data, uint32_t& ext_id) override;

private:
    double p_max_;
    double v_max_;
    double t_max_;
    double kd_max_;
};

class DMMotor : public Motor {
public:
    // 使能 / 失能帧（8 字节固定内容）
    // 提成公开常量：① MotorEnable/Disable 用同一份 ② 测试可以直接校验
    static constexpr uint8_t FRAME_ENABLE[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    static constexpr uint8_t FRAME_DISABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    DMMotor(MotorType type, MotorControlMode mode,
            uint32_t rec_id, uint32_t send_id, CAN::CanBus* bus);
    ~DMMotor() override;

    void CanMsgParse(const can_frame& frame) override;
    void MotorControl() override;
    void MotorEnable() override;
    void MotorDisable() override;

private:
    double p_max_;
    double v_max_;
    double t_max_;
    DMCommand* dm_command_;
};

}  // namespace MOTOR
