# ═══════════════════════════════════════════════
#  DM4310 环境脚本
#  用法：source ~/dm_env.sh
#  作用：一条命令搞定两条 source + 进工作区
# ═══════════════════════════════════════════════
source /opt/ros/jazzy/setup.bash
if [ -f ~/dm4310_ws2/install/setup.bash ]; then
    source ~/dm4310_ws2/install/setup.bash
else
    echo "⚠️  ~/dm4310_ws2/install 不存在，先编译："
    echo "    cd ~/dm4310_ws2 && colcon build --symlink-install"
fi
cd ~/dm4310_ws2 2>/dev/null
echo "✅ DM4310 环境就绪（$(pwd)）"
echo "   CAN_ID=12  MST_ID=44  ch=0   kp=10  kd=0.5"

# ═══════════════════════════════════════════════
#  ⭐ 简写命令（source 本文件后可用）
#
#    dm 0 0.3     位置 0.3 rad
#    dm 1 1.0     速度 1.0 rad/s   （kp 自动为 0）
#    dm 2 0.3     力矩 0.3 N·m     （kd 自动补 0.3）
#    dm 3 0.2 0.5 阻抗：位置 0.2 + 前馈 0.5
#    dm stop      全部归零，回位置模式（安全）
#    dmst         看当前状态
#
#  也支持英文：dm pos 0.3 / dm vel 1.0 / dm tor 0.3
# ═══════════════════════════════════════════════

dm() {
    local MSG="dm4310_driver/msg/DmMitCommand"
    local TOPIC="/dm4310/mit_command"

    # ── 各模式的默认 PD（和 yaml 里的 kp/kd 保持一致）──
    #    这样消息里就能直接看到"这次用的什么 PD"，
    #    不用去猜"0 到底被节点换成了多少"
    local KP_POS=10.0  KD_POS=0.5     # 位置：硬一点，跟得紧
    local KD_VEL=0.5                  # 速度：kp 必须 0，只留阻尼
    local KD_TOR=0.3                  # 力矩：kp=0，kd 不能太小（防飞车）

    if [ $# -eq 0 ]; then
        echo "用法:"
        echo "  dm 0 <位置rad> [kp] [kd]   位置（默认 kp=10 kd=0.5）"
        echo "  dm 1 <速度rad/s> [kd]      速度（kp 固定 0，默认 kd=0.5）"
        echo "  dm 2 <力矩N·m> [kd]        力矩（kp 固定 0，默认 kd=0.3）"
        echo "  dm 3 <位置> <力矩>         阻抗（kp=10 kd=0.5）"
        echo "  dm stop             归零回安全状态"
        echo "  dmst                看当前状态"
        return 0
    fi

    case "$1" in
        0|pos*)
            # 位置：kp/kd 显式给（${3}/${4} 可覆盖默认值）
            ros2 topic pub --once -w 0 "$TOPIC" "$MSG" \
                "{mode: 0, position: ${2:-0}, kp: ${3:-$KP_POS}, kd: ${4:-$KD_POS}}" ;;
        1|vel*)
            # 速度：kp 固定 0，kd 可覆盖
            ros2 topic pub --once -w 0 "$TOPIC" "$MSG" \
                "{mode: 1, velocity: ${2:-0}, kp: 0.0, kd: ${3:-$KD_VEL}}" ;;
        2|tor*)
            # 力矩：kp 固定 0，kd 可覆盖（但不建议小于 0.3）
            ros2 topic pub --once -w 0 "$TOPIC" "$MSG" \
                "{mode: 2, torque: ${2:-0}, kp: 0.0, kd: ${3:-$KD_TOR}}" ;;
        3|imp*)
            # 阻抗：位置 + 前馈，kp/kd 用位置模式的默认
            ros2 topic pub --once -w 0 "$TOPIC" "$MSG" \
                "{mode: 3, position: ${2:-0}, torque: ${3:-0}, kp: $KP_POS, kd: $KD_POS}" ;;
        stop)
            ros2 topic pub --once -w 0 "$TOPIC" "$MSG" \
                "{mode: 0, position: 0.0, kp: $KP_POS, kd: $KD_POS}" ;;
        *)
            echo "❌ 不认识的模式: $1（用 0/1/2/3 或 pos/vel/tor/imp/stop）"
            return 1 ;;
    esac
}

dmst() {
    echo "位置: $(ros2 topic echo /joint_states --field position --once 2>/dev/null)"
    echo "速度: $(ros2 topic echo /joint_states --field velocity --once 2>/dev/null)"
    echo "力矩: $(ros2 topic echo /joint_states --field effort   --once 2>/dev/null)"
}
