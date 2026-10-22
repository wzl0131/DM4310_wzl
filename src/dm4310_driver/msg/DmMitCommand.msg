# ═══════════════════════════════════════════════════════════════
#  DmMitCommand.msg —— 达妙 MIT 模式的完整指令（五参数合一）
#
#  【为什么合成一个消息】
#    达妙 MIT 模式的核心就是【五个参数一起】决定输出力矩：
#
#        τ = kp·(p_des − p) + kd·(v_des − v) + t_ff
#
#    分三个话题发，等于把"一组参数"拆成三份发出去，
#    还得自己保证它们时序一致（万一只到了 position 没到 kp，
#    这一毫秒的力矩就是错的）。
#    合成一个消息 → 一次原子地把整组参数送下去。
#
#  【为什么加 mode】
#    切模式时不想手动 ros2 param set kp 0.0 那样调 PD。
#    mode 让节点自动决定 kp/kd：
#
#      MODE_POSITION   → kp/kd 用消息里给的（填 0 则用参数默认值）
#      MODE_VELOCITY   → 节点自动 kp = 0            （不用手调）
#      MODE_TORQUE     → 节点自动 kp = 0，kd 不许为 0（防飞车）
#      MODE_IMPEDANCE  → 位置 + 力矩前馈，kp/kd 用消息里的
# ═══════════════════════════════════════════════════════════════

# ── 模式常量 ──
uint8 MODE_POSITION  = 0
uint8 MODE_VELOCITY  = 1
uint8 MODE_TORQUE    = 2
uint8 MODE_IMPEDANCE = 3

# ── 当前模式 ──
uint8 mode

# ── MIT 五参数 ──
# 位置：目标位置，单位 rad          （MODE_POSITION / MODE_IMPEDANCE 时生效）
float64 position
# 速度：目标速度，单位 rad/s        （MODE_VELOCITY 时生效）
float64 velocity
# 刚度：kp，无量纲                  （填 0 表示"用参数默认值"，MODE_VELOCITY/TORQUE 时被强制为 0）
float64 kp
# 阻尼：kd，无量纲                  （填 0 表示"用参数默认值"，MODE_TORQUE 时最小取安全值）
float64 kd
# 力矩前馈：t_ff，单位 N·m          （MODE_TORQUE 时就是纯力矩指令）
float64 torque
