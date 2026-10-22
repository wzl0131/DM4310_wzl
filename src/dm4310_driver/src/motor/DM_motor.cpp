#include "motor/DM_motor.hpp"

#include <algorithm>
#include <cstring>

namespace MOTOR {

void DMCommand::CommandPack(uint8_t* data, uint32_t& ext_id) {
    (void)ext_id;
    const double p = p_max_;
    const double v = v_max_;
    const double t = t_max_;
    const double kd_limit = kd_max_;
    const double pos_cmd = std::clamp(target_position, -p, p);
    const double vel_cmd = std::clamp(target_velocity, -v, v);
    const double kp_cmd = std::clamp(kp, 0.0, 500.0);
    const double kd_cmd = std::clamp(kd, 0.0, kd_limit);
    const double tor_cmd = std::clamp(target_torque, -t, t);
    int16_t pos = FloatToInt(pos_cmd, -p, p, 16);
    int16_t vel = FloatToInt(vel_cmd, -v, v, 12);
    int16_t kp_val = FloatToInt(kp_cmd, 0.0, 500.0, 12);
    int16_t kd_val = FloatToInt(kd_cmd, 0.0, kd_limit, 12);
    int16_t tor = FloatToInt(tor_cmd, -t, t, 12);

    // MIT mode CAN frame layout:
    //   D[0~1]: p_des  (16-bit)
    //   D[2~3]: v_des  (12-bit, upper)
    //   D[3~4]: Kp     (12-bit, lower)
    //   D[5~6]: Kd     (12-bit, upper)
    //   D[6~7]: t_ff   (12-bit, lower)
    data[0] = pos >> 8;
    data[1] = pos & 0xFF;
    data[2] = vel >> 4;
    data[3] = ((vel & 0x0F) << 4) | (kp_val >> 8);
    data[4] = kp_val & 0xFF;
    data[5] = kd_val >> 4;
    data[6] = ((kd_val & 0x0F) << 4) | (tor >> 8);
    data[7] = tor & 0xFF;
}

DMMotor::DMMotor(MotorType type, MotorControlMode mode,
                 uint32_t rec_id, uint32_t send_id, CAN::CanBus* bus)
    : Motor(type, mode, rec_id, send_id, bus,
            *(dm_command_ = new DMCommand(GetMotorPMax(type), GetMotorVMax(type),
                                          GetMotorTMax(type), GetMotorKdMax(type)))),
      p_max_(GetMotorPMax(type)),
      v_max_(GetMotorVMax(type)),
      t_max_(GetMotorTMax(type)) {}

DMMotor::~DMMotor() {
    delete dm_command_;
}

void DMMotor::CanMsgParse(const can_frame& frame) {
    StampFeedbackTime();

    uint16_t pos_int = (frame.data[1] << 8) | frame.data[2];
    uint16_t vel_int = ((frame.data[3] << 4) | (frame.data[4] >> 4));
    uint16_t tor_int = (((frame.data[4] & 0x0F) << 8) | frame.data[5]);

    // 原版只解析了位置/速度/力矩；
    // 这里补上 D[0] 的 ERR（高4位）和 D[6]/D[7] 的两个温度，
    // 节点要拿它们做状态显示。
    const uint8_t err     = static_cast<uint8_t>((frame.data[0] >> 4) & 0x0F);
    const double  t_mos   = static_cast<double>(frame.data[6]);
    const double  t_rotor = static_cast<double>(frame.data[7]);

    {
        std::lock_guard<std::mutex> lock(encoder.mutex);
        encoder.position   = IntToFloat(pos_int, -p_max_, p_max_, 16);
        encoder.velocity   = IntToFloat(vel_int, -v_max_, v_max_, 12);
        encoder.torque     = IntToFloat(tor_int, -t_max_, t_max_, 12);
        encoder.tem_mos    = t_mos;
        encoder.tem_rotor  = t_rotor;
        encoder.error_code = static_cast<int>(err);
    }
}

void DMMotor::MotorControl() {
    command.CommandPack(send_data, send_can_id);
    SendCanMsg(send_data);
}

void DMMotor::MotorEnable() {
    uint8_t data[8];
    std::memcpy(data, FRAME_ENABLE, 8);
    SendCanMsg(data);
}

void DMMotor::MotorDisable() {
    uint8_t data[8];
    std::memcpy(data, FRAME_DISABLE, 8);
    SendCanMsg(data);
}

}  // namespace MOTOR
