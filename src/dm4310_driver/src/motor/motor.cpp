#include "motor/motor.hpp"
#include <array>  
#include <cstddef>


namespace MOTOR {
// Per-model MIT mapping limits: {P_MAX (rad), V_MAX (rad/s), T_MAX (Nm), KD_MAX}.
// CommandPack and feedback parse must use the same limits for each motor instance.
const std::map<MotorType, std::array<double, 4>> motor_limit_params = {
    {MotorType::DM8006, {12.57, 50.0, 36.0, 5.0}},
    {MotorType::DM8009, {25.13, 30.0, 50.0, 5.0}},
    {MotorType::DM4310, {12.5, 50.0, 10.0, 4.0}},
    {MotorType::DM4340, {12.5, 20.0, 28.0, 4.0}},
    {MotorType::RS06, {12.57, 50.0, 12.0, 5.0}}
};

namespace {
double GetMotorLimit(MotorType type, size_t index, double fallback) {
    const auto it = motor_limit_params.find(type);
    if (it != motor_limit_params.end()) {
        return it->second[index];
    }
    return fallback;
}
}  // namespace

double GetMotorPMax(MotorType type) {
    return GetMotorLimit(type, 0, 12.5);
}

double GetMotorVMax(MotorType type) {
    return GetMotorLimit(type, 1, 50.0);
}

double GetMotorTMax(MotorType type) {
    return GetMotorLimit(type, 2, 10.0);
}

double GetMotorKdMax(MotorType type) {
    return GetMotorLimit(type, 3, 4.0);
}

}  // namespace MOTOR