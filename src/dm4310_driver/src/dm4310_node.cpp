// ============================================================
//  dm4310_node.cpp —— 达妙 DM4310 · MIT 模式 · ROS 2 驱动
//
//  ⭐ 类的三层结构（完全参照队里 dogdualarms 的格式）：
//
//     can/can.hpp        CAN::CanBus       —— CAN 总线封装
//     motor/motor.hpp    MOTOR::Motor      —— 电机基类
//     motor/DM_motor.hpp MOTOR::DMMotor    —— 达妙派生类
//     本文件             Dm4310Node        —— ROS 2 业务层
//
//  和队里唯一的区别：
//    队里用 ros2_control 插件（多关节 + MoveIt），
//    我们用单节点（单电机，够用）。
//
//  跑法：
//    ros2 launch dm4310_driver dm4310.launch.py
//
//  ⭐ 一个指令话题：/dm4310/mit_command
//
//    类型：dm4310_driver/msg/DmMitCommand
//    内容：MIT 五参数（position / velocity / kp / kd / torque）+ mode
//
//    为什么合成一个消息：
//      MIT 的本质是【五参数一起】决定力矩：
//          τ = kp·(p_des − p) + kd·(v_des − v) + t_ff
//      拆成三个话题发，等于把一组参数拆开发，还得自己保证时序一致。
//
//    为什么加 mode（切模式不用手动调 PD）：
//      MODE_POSITION  (0) → kp/kd 用消息里的（填 0 = 用参数默认值）
//      MODE_VELOCITY  (1) → 节点自动 kp = 0
//      MODE_TORQUE    (2) → 节点自动 kp = 0，kd 最小 0.3（防飞车）
//      MODE_IMPEDANCE (3) → 位置 + 力矩前馈
//
//  用法举例（每条是一整行，别加续行符）：
//
//    # 位置：走到 0.3 rad（kp/kd 填 0 → 用 yaml 里的 10 / 0.5）
//    ros2 topic pub --once /dm4310/mit_command dm4310_driver/msg/DmMitCommand "{mode: 0, position: 0.3, velocity: 0.0, kp: 0.0, kd: 0.0, torque: 0.0}"
//
//    # 速度：1.0 rad/s（kp 自动为 0，不用手调）
//    ros2 topic pub --once /dm4310/mit_command dm4310_driver/msg/DmMitCommand "{mode: 1, position: 0.0, velocity: 1.0, kp: 0.0, kd: 0.0, torque: 0.0}"
//
//    # 力矩：0.3 N·m（kd 自动补到 ≥0.3，不会飞车）
//    ros2 topic pub --once /dm4310/mit_command dm4310_driver/msg/DmMitCommand "{mode: 2, position: 0.0, velocity: 0.0, kp: 0.0, kd: 0.0, torque: 0.3}"
//
//  看反馈 / 调参：
//    ros2 topic echo /joint_states --once
//    ros2 param set /dm4310_node kp 20.0      # 改"默认 kp"（mode 里 kp 填 0 时用它）
// ============================================================

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "dm4310_driver/msg/dm_mit_command.hpp"   // ⭐ 五参数合一的消息

#include "can/can.hpp"          // CAN::CanBus, CanSendMsg, OpenCanDevice
#include "motor/motor.hpp"      // MOTOR::Motor, Encoder, Command
#include "motor/DM_motor.hpp"   // MOTOR::DMMotor

// ============================================================
//  SDK 回调是普通函数指针，用全局变量转交
//  （和队里 can.cpp 的 recv_frame_handle 思路一致）
// ============================================================
namespace
{
MOTOR::Motor* g_motor    = nullptr;   // 单电机场景：全局唯一
uint32_t      g_rx_id    = 0;         // 只认这个 MST_ID（0 = 不过滤）
bool          g_received = false;     // 是否收到过反馈

void On_Can_Recv(dmcan_device_handle* /*handle*/, usb_rx_frame_t* f)
{
    if (f == nullptr || f->head.dlc < 8)
    {
        return;
    }

    MOTOR::Motor* motor = g_motor;
    if (motor == nullptr)
    {
        return;
    }

    // 单电机场景：g_rx_id 为 0 时不过滤
    if (g_rx_id != 0 && f->head.can_id != g_rx_id)
    {
        return;
    }

    // usb_rx_frame_t → can_frame（motor 模块统一的帧结构）
    can_frame frame{};
    frame.can_id = f->head.can_id;
    frame.dlc    = 8;
    std::memcpy(frame.data, f->payload, 8);

    // ⭐ 解析在 Motor 子类里做，内部已加锁（encoder.mutex）
    motor->CanMsgParse(frame);
    g_received = true;
}

// ⭐ mode 数字 → 中文名（打日志用）
std::string Mode_Name(uint8_t mode)
{
    switch (mode)
    {
    case dm4310_driver::msg::DmMitCommand::MODE_POSITION:  return "位置";
    case dm4310_driver::msg::DmMitCommand::MODE_VELOCITY:  return "速度";
    case dm4310_driver::msg::DmMitCommand::MODE_TORQUE:    return "力矩";
    case dm4310_driver::msg::DmMitCommand::MODE_IMPEDANCE: return "阻抗";
    default:                                               return "未知";
    }
}
}  // namespace

// ============================================================
//  ROS 2 节点（业务层：话题 / 定时器 / 安全）
// ============================================================
class Dm4310Node : public rclcpp::Node
{
public:
    Dm4310Node() : Node("dm4310_node")
    {
        // ---------- 参数 ----------
        Can_Id_     = static_cast<int>(declare_parameter<int64_t>("can_id", 1));
        Mst_Id_     = static_cast<int>(declare_parameter<int64_t>("mst_id", 0));
        Channel_    = static_cast<int>(declare_parameter<int64_t>("channel", 0));
        Rate_       = static_cast<float>(declare_parameter<double>("control_rate_hz", 1000.0));
        Kp_         = static_cast<float>(declare_parameter<double>("kp", 20.0));
        Kd_         = static_cast<float>(declare_parameter<double>("kd", 1.0));
        Timeout_Ms_ = static_cast<int>(declare_parameter<int64_t>("timeout_ms", 100));

        // ⭐FD 新增：CAN FD 相关参数（一共 5 个，注意区分"通道级"和"帧级"）
        //
        //   ① canfd_channel —— 【通道级】通道控制器是否工作在 FD 模式
        //      true  = FD 模式（向下兼容经典帧，DM 电机照样能收发经典帧）
        //      false = 纯经典模式
        //
        //   ② canfd_frame  —— 【帧级】发出去的这一帧用哪种格式
        //      false = 经典 CAN 帧（DM 系列电机只认这个）
        //      true  = CAN FD 帧（FDF=1，从站也必须是 FD 节点）
        //      ⚠️ 这两个是【独立的】。学长那边能跑，很可能是
        //         "通道 FD 模式 = true，但帧仍然发经典帧"
        //
        //   ③④⑤ 波特率和采样点
        CanFdChannel_ = declare_parameter<bool>("canfd_channel", true);                            // ⭐FD 新增
        CanFdFrame_   = declare_parameter<bool>("canfd_frame", false);                             // ⭐FD 新增
        CanBaud_      = static_cast<int64_t>(declare_parameter<int64_t>("can_baudrate", 1000000));   // ⭐FD 新增
        CanFdBaud_    = static_cast<int64_t>(declare_parameter<int64_t>("canfd_baudrate", 5000000)); // ⭐FD 新增
        CanSp_        = static_cast<float>(declare_parameter<double>("can_sp", 0.75));               // ⭐FD 新增
        CanFdSp_      = static_cast<float>(declare_parameter<double>("canfd_sp", 0.75));             // ⭐FD 新增
        // ⭐FD 新增：BRS（数据段是否真的切换到 canfd_baudrate）
        CanFdBrs_     = declare_parameter<bool>("canfd_brs", true);

        if (Can_Id_ < 1 || Can_Id_ > 0x7F)
        {
            RCLCPP_ERROR(get_logger(), "can_id 非法：%d", Can_Id_);
            throw std::runtime_error("bad can_id");
        }
        if (Channel_ < 0 || Channel_ > 3)
        {
            RCLCPP_ERROR(get_logger(), "channel 非法：%d（必须 0~3）", Channel_);
            throw std::runtime_error("bad channel");
        }

        g_rx_id = static_cast<uint32_t>(Mst_Id_);

        // ---------- 参数回调：运行时改 kp/kd/timeout_ms ----------
        Param_Cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &Params) {
                rcl_interfaces::msg::SetParametersResult Result;
                Result.successful = true;

                for (const auto &P : Params)
                {
                    const std::string &Name = P.get_name();

                    if (Name == "kp")
                    {
                        Kp_ = static_cast<float>(P.as_double());
                        RCLCPP_INFO(get_logger(), "⭐ kp 改为 %.1f（立即生效）", Kp_);
                    }
                    else if (Name == "kd")
                    {
                        Kd_ = static_cast<float>(P.as_double());
                        RCLCPP_INFO(get_logger(), "⭐ kd 改为 %.2f（立即生效）", Kd_);
                    }
                    else if (Name == "timeout_ms")
                    {
                        Timeout_Ms_ = static_cast<int>(P.as_int());
                        if (Motor_ != nullptr)
                        {
                            Motor_->timeout_ms = static_cast<uint32_t>(Timeout_Ms_);
                        }
                        RCLCPP_INFO(get_logger(), "⭐ timeout_ms 改为 %d", Timeout_Ms_);
                    }
                    else if (Name == "can_id" || Name == "mst_id" ||
                             Name == "channel" || Name == "control_rate_hz")
                    {
                        Result.successful = false;
                        Result.reason = Name + " 改了要重启节点（要重配 SDK/定时器）";
                        RCLCPP_WARN(get_logger(), "%s", Result.reason.c_str());
                    }
                }
                return Result;
            });

        // ---------- 话题 ----------
        Joint_Pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // ⭐ MIT 三个指令：位置 / 速度 / 力矩前馈
        //    三个一起用 kp/kd 组合出不同控制模式，见文件头的说明
        // ⭐ 一个话题装 MIT 五参数 + mode（取代原来三个 Float64 话题）
        Mit_Sub_ = create_subscription<dm4310_driver::msg::DmMitCommand>(
            "dm4310/mit_command", 10,
            [this](const dm4310_driver::msg::DmMitCommand::SharedPtr m) {
                // 五个参数原样收下，mode 决定怎么用（见 Loop 里的 ④ mode 分发）
                Cmd_Pos_ = static_cast<float>(m->position);
                Cmd_Vel_ = static_cast<float>(m->velocity);
                Cmd_Kp_  = static_cast<float>(m->kp);
                Cmd_Kd_  = static_cast<float>(m->kd);
                Cmd_Tor_ = static_cast<float>(m->torque);
                Mode_    = m->mode;

                RCLCPP_INFO(get_logger(),
                            "[%s] pos=%.3f vel=%.3f kp=%.1f kd=%.2f tor=%.3f",
                            Mode_Name(Mode_).c_str(), Cmd_Pos_, Cmd_Vel_,
                            Cmd_Kp_, Cmd_Kd_, Cmd_Tor_);
            });

        // ---------- 硬件 ----------
        if (!Open_Device())          // CanBus_ + Motor_ 一起建
        {
            throw std::runtime_error("打开 DM-Link 失败");
        }

        Motor_->timeout_ms = static_cast<uint32_t>(Timeout_Ms_);
        Enable();

        // ---------- 控制循环 ----------
        const auto Period_Us = static_cast<int64_t>(1.0e6 / Rate_);
        Timer_ = create_wall_timer(std::chrono::microseconds(Period_Us),
                                   std::bind(&Dm4310Node::Loop, this));

        RCLCPP_INFO(get_logger(),
                    "dm4310_node 启动：CAN_ID=%d MST_ID=%d ch=%d %.0fHz kp=%.1f kd=%.2f",
                    Can_Id_, Mst_Id_, Channel_, Rate_, Kp_, Kd_);
        // ⭐FD 新增：启动时把【通道模式】和【帧格式】分开打印，一眼看全
        RCLCPP_INFO(get_logger(),
                    "CAN 通道：%s（仲裁段 %ld bps, sp %.2f）%s",
                    CanFdChannel_ ? "FD 模式" : "经典模式",
                    static_cast<long>(CanBaud_), static_cast<double>(CanSp_),
                    CanFdChannel_
                        ? (std::string("，数据段 ") + std::to_string(CanFdBaud_) +
                           " bps, sp " + std::to_string(CanFdSp_)).c_str()
                        : "");
        RCLCPP_INFO(get_logger(),
                    "CAN 帧格式：%s%s",
                    CanFdFrame_ ? "CAN FD 帧（FDF=1）" : "经典 CAN 帧（8 字节）",
                    (CanFdFrame_ && CanFdBrs_) ? "，BRS=1（数据段提速）"
                                              : (CanFdFrame_ ? "，BRS=0（数据段不提速）" : ""));
        RCLCPP_INFO(get_logger(),
                    "指令话题：/dm4310/mit_command（DmMitCommand：五参数 + mode）");
    }

    ~Dm4310Node() override
    {
        Timer_.reset();

        // ⭐ 先失能，再拆对象（顺序不能反）
        if (Motor_ != nullptr)
        {
            Motor_->MotorDisable();
        }

        g_motor = nullptr;   // 先断回调引用

        delete Motor_;       // Motor 引用 CanBus_，所以先删 Motor
        Motor_ = nullptr;

        delete CanBus_;
        CanBus_ = nullptr;

        CloseCanDevice(Ctx_, Dev_);
        Ctx_ = nullptr;
        Dev_ = nullptr;

        RCLCPP_INFO(get_logger(), "已发送失能帧，设备已关闭");
    }

private:
    // ========================================================
    //  打开设备 + 建类对象
    // ========================================================
    bool Open_Device()
    {
        // ① 打开 DM-LinkX-4C
        //    队里是在 real_robot_hardware 的 LinkInit() 里做的，
        //    我们抽成了 can.cpp 里的 OpenCanDevice()
        // ⭐FD 修改：多传 5 个参数
        //    通道级 canfd_channel + 仲裁段(波特率/采样点) + 数据段(波特率/采样点)
        if (!OpenCanDevice(&Ctx_, &Dev_, static_cast<uint8_t>(Channel_),
                           CanFdChannel_, static_cast<uint32_t>(CanBaud_),
                           static_cast<uint32_t>(CanFdBaud_), CanSp_, CanFdSp_))
        {
            return false;
        }

        // ② CAN 总线对象
        //    ⭐FD 修改：这里传的是【帧级】标志（这一帧用经典还是 FD 格式），
        //    和上面 OpenCanDevice 的【通道级】标志是两回事
        CanBus_ = new CAN::CanBus(Dev_, static_cast<uint8_t>(Channel_),
                                  CanFdFrame_, CanFdBrs_);

        // ③ 达妙电机对象（MIT 模式）
        //    DM4310 的 PMAX/VMAX/TMAX/KD_MAX 由 motor.cpp 的参数表取
        Motor_ = new MOTOR::DMMotor(MOTOR::MotorType::DM4310,
                                    MOTOR::MotorControlMode::MIT_Control,
                                    static_cast<uint32_t>(Mst_Id_),    // 收
                                    static_cast<uint32_t>(Can_Id_),    // 发
                                    CanBus_);

        // ④ 挂接收回调（全局变量转交，见文件顶部）
        g_motor = Motor_;
        dmcan_device_hook_recv_callback(Dev_, On_Can_Recv);

        return true;
    }

    // ========================================================
    //  使能（连续发 3 次，防丢帧）
    // ========================================================
    void Enable()
    {
        for (int i = 0; i < 3; ++i)
        {
            Motor_->MotorEnable();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // ========================================================
    //  ⭐ 控制循环（1 kHz）
    // ========================================================
    void Loop()
    {
        if (Motor_ == nullptr)
        {
            return;
        }

        // ---------- ① 读反馈（Encoder 内部有锁）----------
        MOTOR::Encoder &Enc = Motor_->encoder;
        double Pos = 0.0, Vel = 0.0, Tor = 0.0, T_Mos = 0.0, T_Rotor = 0.0;
        {
            std::lock_guard<std::mutex> Lock(Enc.mutex);
            Pos     = Enc.position;
            Vel     = Enc.velocity;
            Tor     = Enc.torque;
            T_Mos   = Enc.tem_mos;
            T_Rotor = Enc.tem_rotor;
        }

        // ---------- ② 首帧同步目标（防上电猛冲）----------
        //  电机在 3 rad、目标 0、kp=20 → τ = 60 N·m，被限到 10（满力矩）
        if (!Init_Once_ && g_received)
        {
            Cmd_Pos_   = static_cast<float>(Pos);
            Mode_      = dm4310_driver::msg::DmMitCommand::MODE_POSITION;
            Init_Once_ = true;
            RCLCPP_INFO(get_logger(),
                        "首帧反馈：pos=%.4f rad，目标已同步到当前位置", Pos);
        }

        // ---------- ④ 按 mode 算出这一拍真正发给电机的五参数 ----------
        //  ⭐ 切模式不用手动调 PD：mode 自动决定 kp/kd
        //     · POSITION  → kp/kd 用消息给的（0 = 用参数默认值）
        //     · VELOCITY  → 节点自动 kp = 0
        //     · TORQUE    → 节点自动 kp = 0，且 kd 不许为 0（防飞车）
        //     · IMPEDANCE → 位置 + 力矩前馈
        double Eff_Pos = Pos;
        double Eff_Vel = 0.0;
        double Eff_Tor = 0.0;
        double Eff_Kp  = static_cast<double>(Kp_);
        double Eff_Kd  = static_cast<double>(Kd_);

        switch (Mode_)
        {
        case dm4310_driver::msg::DmMitCommand::MODE_VELOCITY:
            Eff_Pos = Pos;                                   // 位置项不生效
            Eff_Vel = static_cast<double>(Cmd_Vel_);
            Eff_Tor = 0.0;
            Eff_Kp  = 0.0;                                   // ⭐ 自动
            Eff_Kd  = (Cmd_Kd_ > 0.0f) ? Cmd_Kd_ : Kd_;
            break;

        case dm4310_driver::msg::DmMitCommand::MODE_TORQUE:
            Eff_Pos = Pos;
            Eff_Vel = 0.0;
            Eff_Tor = static_cast<double>(Cmd_Tor_);
            Eff_Kp  = 0.0;                                   // ⭐ 自动
            // ⭐ kd 不许为 0：空载恒力矩会一直加速（飞车）
            Eff_Kd  = (Cmd_Kd_ > 0.0f) ? static_cast<double>(Cmd_Kd_)
                                       : static_cast<double>(KD_TORQUE_MIN);
            break;

        case dm4310_driver::msg::DmMitCommand::MODE_IMPEDANCE:
            Eff_Pos = static_cast<double>(Cmd_Pos_);
            Eff_Vel = 0.0;
            Eff_Tor = static_cast<double>(Cmd_Tor_);         // 前馈叠加
            Eff_Kp  = (Cmd_Kp_ > 0.0f) ? Cmd_Kp_ : Kp_;
            Eff_Kd  = (Cmd_Kd_ > 0.0f) ? Cmd_Kd_ : Kd_;
            break;

        case dm4310_driver::msg::DmMitCommand::MODE_POSITION:
        default:
            Eff_Pos = static_cast<double>(Cmd_Pos_);
            Eff_Vel = 0.0;
            Eff_Tor = 0.0;
            Eff_Kp  = (Cmd_Kp_ > 0.0f) ? Cmd_Kp_ : Kp_;
            Eff_Kd  = (Cmd_Kd_ > 0.0f) ? Cmd_Kd_ : Kd_;
            break;
        }

        // ---------- ④a 模式切换时打印"实际生效"的 PD ⭐ ----------
        //  为什么要打印：
        //    消息里 kp/kd 填 0 时，节点会用 yaml 里的默认值替换，
        //    订阅回调打印的是【消息里的值】，看不到真正用了多少。
        //    这里在模式变化时打印【实际生效】的一组，避免"到底用了啥"的困惑。
        if (Mode_ != Last_Logged_Mode_)
        {
            Last_Logged_Mode_ = Mode_;
            RCLCPP_INFO(get_logger(),
                        "⭐ 模式=%s  实际生效：pos=%.3f vel=%.3f kp=%.1f kd=%.2f tor=%.3f",
                        Mode_Name(Mode_).c_str(), Eff_Pos, Eff_Vel, Eff_Kp, Eff_Kd, Eff_Tor);
        }

        // ---------- ④b 刚度恢复同步（防"狂转"）⭐ 修 2026-10-21 ----------
        //  场景：kp=0 跑速度/力矩模式 → 电机自由转出去很远，但 Cmd_Pos_ 还是老值
        //        → 切回位置模式（kp 突然变非 0）的瞬间，误差可能是十几 rad
        //        → τ = kp × 误差 = 150 N·m，被限到 TMAX 10 N·m → 满力矩往回冲
        //
        //  所以：只要检测到【有效 kp】从 ≤0 变成 >0，先把目标对齐当前位置。
        if (Eff_Kp > 0.0 && Last_Eff_Kp_ <= 0.0)
        {
            Cmd_Pos_ = static_cast<float>(Pos);
            Eff_Pos  = Pos;
            RCLCPP_WARN(get_logger(),
                        "⭐ 刚度恢复（有效 kp %.1f → %.1f）：目标已对齐当前位置 %.4f rad，"
                        "避免以满力矩冲向旧目标",
                        Last_Eff_Kp_, Eff_Kp, Pos);
        }
        Last_Eff_Kp_ = Eff_Kp;

        // ---------- ③ 超时判断 ----------
        const bool Timed_Out = Motor_->CheckTimeout();
        if (Timed_Out && !Timeout_Warned_)
        {
            Timeout_Warned_ = true;
            RCLCPP_WARN(get_logger(),
                        "反馈超时 %d ms → 切阻尼模式（kp=0, kd=0.5）", Timeout_Ms_);
        }
        else if (!Timed_Out && Timeout_Warned_)
        {
            Timeout_Warned_ = false;
            RCLCPP_INFO(get_logger(), "反馈恢复，回到正常控制");
        }

        // ---------- ⑤ 填指令（超时则强制阻尼）----------
        Motor_->command.target_position = Timed_Out ? Pos  : Eff_Pos;
        Motor_->command.target_velocity = Timed_Out ? 0.0  : Eff_Vel;
        Motor_->command.target_torque   = Timed_Out ? 0.0  : Eff_Tor;
        Motor_->command.kp              = Timed_Out ? 0.0  : Eff_Kp;
        Motor_->command.kd              = Timed_Out ? 0.5  : Eff_Kd;

        // ---------- ⑤ 打包 + 发送 ----------
        Motor_->MotorControl();

        // ---------- ⑥ 发布 /joint_states ----------
        if (g_received)
        {
            sensor_msgs::msg::JointState Msg;
            Msg.header.stamp = this->now();
            Msg.name         = {"dm4310_joint"};
            Msg.position     = {Pos};
            Msg.velocity     = {Vel};
            Msg.effort       = {Tor};
            Joint_Pub_->publish(Msg);

            const int64_t Now = Now_Ms();
            if ((Now - Last_Log_Ms_) >= 1000)
            {
                Last_Log_Ms_ = Now;
                RCLCPP_INFO(get_logger(),
                            "目标 %.3f | 当前 %.3f rad | 速度 %.2f | 力矩 %.2f | 温度 %.0f/%.0f",
                            Cmd_Pos_, Pos, Vel, Tor, T_Mos, T_Rotor);
            }//1ms打印一次防止刷屏
        }
        else if (!NoFb_Warned_)
        {
            NoFb_Warned_ = true;
            RCLCPP_WARN(get_logger(),
                        "还没收到反馈。检查 mst_id（0 = 不过滤）/ can_id / 电机是否上电");
        }
    }

    static int64_t Now_Ms()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }

    // ---------- 参数 ----------
    int   Can_Id_     = 1;
    int   Mst_Id_     = 0;
    int   Channel_    = 0;
    float Rate_       = 1000.0f;
    float Kp_         = 20.0f;
    float Kd_         = 1.0f;
    int   Timeout_Ms_ = 100;

    // ---------- CAN FD 相关（5 个）----------  ⭐FD 新增
    bool    CanFdChannel_ = true;     // ⭐FD 【通道级】通道是否 FD 模式（兼容经典帧）
    bool    CanFdFrame_   = false;    // ⭐FD 【帧级】发出的帧是否用 FD 格式
    int64_t CanBaud_      = 1000000;  // ⭐FD 仲裁段波特率（两种模式都用）
    int64_t CanFdBaud_    = 5000000;  // ⭐FD 数据段波特率（仅通道 FD 模式时有效）
    float   CanSp_        = 0.75f;    // ⭐FD 仲裁段采样点
    float   CanFdSp_      = 0.75f;    // ⭐FD 数据段采样点
    bool    CanFdBrs_     = true;     // ⭐FD 数据段是否提速（BRS）

    // ---------- 运行状态 ----------
    // ⭐ 一个消息装 MIT 五参数 + mode（取代原来三个 Float64 话题）
    float   Cmd_Pos_     = 0.0f;    // 消息里的 position
    float   Cmd_Vel_     = 0.0f;    // 消息里的 velocity
    float   Cmd_Kp_      = 0.0f;    // 消息里的 kp（0 = 用参数默认值）
    float   Cmd_Kd_      = 0.0f;    // 消息里的 kd（0 = 用参数默认值）
    float   Cmd_Tor_     = 0.0f;    // 消息里的 torque
    uint8_t Mode_        = dm4310_driver::msg::DmMitCommand::MODE_POSITION;

    // ⭐ 上一拍的【有效 kp】，用来检测"刚度恢复"（0 → 非0）
    double  Last_Eff_Kp_ = 10.0;
    // ⭐ 上次打印过的模式（模式变了才打印一次，避免 1kHz 刷屏）
    uint8_t Last_Logged_Mode_ = 255;

    // ⭐ 力矩模式下的最低阻尼（防止 kd=0 飞车）
    static constexpr float KD_TORQUE_MIN = 0.3f;

    bool    Init_Once_   = false;
    // ⚠️ 两个标志必须分开！以前共用一个 Warned_，导致：
    //    第 N 帧   "没收到反馈" 把 Warned_ 置 true
    //    第 N+1 帧 超时逻辑看到 Warned_=true 又打印"反馈恢复"并清 false
    //    第 N+2 帧 又打印"没收到反馈" …
    //    → 1 kHz 无限刷屏
    bool    Timeout_Warned_ = false;   // 只给"反馈超时"用
    bool    NoFb_Warned_    = false;   // 只给"还没收到反馈"用
    int64_t Last_Log_Ms_ = 0;

    // ---------- 硬件对象（⭐ 照队里的三层）----------
    dmcan_context*       Ctx_    = nullptr;
    dmcan_device_handle* Dev_    = nullptr;
    CAN::CanBus*         CanBus_ = nullptr;
    MOTOR::DMMotor*      Motor_  = nullptr;

    // ---------- ROS ----------
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Joint_Pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    Sub_;       // 位置
    // ⭐ 一个话题装五参数
    rclcpp::Subscription<dm4310_driver::msg::DmMitCommand>::SharedPtr Mit_Sub_;
    rclcpp::TimerBase::SharedPtr                               Timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr Param_Cb_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<Dm4310Node>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("dm4310_node"), "异常退出：%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
