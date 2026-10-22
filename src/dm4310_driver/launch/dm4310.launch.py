"""dm4310.launch.py —— 启动 DM4310 驱动节点

⭐ 三种改参数的方式：

1) 改配置文件（永久，推荐）：
     nano ~/dm4310_ws2/src/dm4310_driver/config/dm4310_params.yaml
     改完重启节点即可（--symlink-install 会立即生效）

2) launch 时命令行覆盖（临时，只覆盖你写的那几个）：
     ros2 launch dm4310_driver dm4310.launch.py kp:=50.0 kd:=2.0
     ros2 launch dm4310_driver dm4310.launch.py canfd_frame:=true canfd_baudrate:=2000000

3) ⭐ 运行中改（不用重启，只能改 kp / kd / timeout_ms）：
     ros2 param set /dm4310_node kp 50.0
     ros2 param set /dm4310_node kd 2.0
     ros2 param get /dm4310_node kp

⚠️ 不要用 sudo 跑 ros2（会丢 ROS 环境变量，报"命令未找到"）。
   USB 权限靠 udev 规则解决，见 /etc/udev/rules.d/99-dm-linkx.rules
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ============================================================
#  ⭐ 参数覆盖规则（很重要，别改回去）
#
#    所有可覆盖参数的默认值都是【空字符串】= 不覆盖，用 yaml。
#    只有命令行显式写了 xxx:=值，才覆盖 yaml 里对应的那一项。
#
#    为什么这么做（两次踩坑的教训）：
#
#      坑①：以前用 DeclareLaunchArgument 的默认值（'1'、'0'、'20.0'、'1.0'）
#           直接塞进 parameters 字典。ROS 2 中【后面的参数覆盖前面的】，
#           于是 yaml 里的值全被这些默认值盖掉
#           → 现象：改了 yaml 完全不生效。
#
#      坑②：后来把声明全删了，只留 parameters=[params_file]。
#           结果命令行传 canfd_frame:=true 没人接，被【静默忽略】
#           → 现象：传了参数但程序行为没变，还以为"三组实验都通了"。
#
#    现在这个写法两个坑都没有：
#      · 不传 → overrides 为空 → 只用 yaml
#      · 传了 → 只覆盖你写的那几个 → 启动日志会把覆盖内容打出来
# ============================================================

# 可覆盖的参数表：名字 → 类型转换函数
OVERRIDABLE = {
    'can_id':          int,
    'mst_id':          int,
    'channel':         int,
    'control_rate_hz': float,
    'kp':              float,
    'kd':              float,
    'timeout_ms':      int,
    'canfd_channel':   lambda s: s.strip().lower() in ('true', '1', 'yes'),
    'canfd_frame':     lambda s: s.strip().lower() in ('true', '1', 'yes'),
    'can_baudrate':    int,
    'canfd_baudrate':  int,
    'can_sp':          float,
    'canfd_sp':        float,
    'canfd_brs':       lambda s: s.strip().lower() in ('true', '1', 'yes'),
}


def _make_node(context):
    params_file = LaunchConfiguration('params_file').perform(context)

    overrides = {}
    for name, cast in OVERRIDABLE.items():
        raw = LaunchConfiguration(name).perform(context)
        if raw != '':
            try:
                overrides[name] = cast(raw)
            except ValueError:
                raise RuntimeError(
                    f'参数 {name}:={raw} 转换失败（类型不对）')

    params = [params_file]
    if overrides:
        params.append(overrides)

    actions = []
    if overrides:
        actions.append(LogInfo(msg=f'⭐ 命令行覆盖 yaml：{overrides}'))
    else:
        actions.append(LogInfo(msg='⭐ 全部使用 yaml 参数（无命令行覆盖）'))

    actions.append(Node(
        package='dm4310_driver',
        executable='dm4310_node',
        name='dm4310_node',
        output='screen',
        emulate_tty=True,
        parameters=params,
    ))
    return actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('dm4310_driver'),
        'config',
        'dm4310_params.yaml')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='参数文件路径'),
    ]

    for name in OVERRIDABLE:
        args.append(DeclareLaunchArgument(
            name,
            default_value='',          # ⭐ 空 = 不覆盖
            description=f'临时覆盖 yaml 里的 {name}（留空 = 用 yaml）'))

    return LaunchDescription(args + [OpaqueFunction(function=_make_node)])
