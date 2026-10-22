// sdk_probe.cpp —— DM-LinkX-4C 探针
//
// 目的：只验证「SDK 能不能找到设备 / 打开 / 配好 4 个通道」
// 不涉及电机，不发任何 CAN 帧，不挂接收回调。
//
// 跑法：sudo ros2 run dm4310_driver sdk_probe

#include <cstdio>
#include <cstdint>

#include "dmcan.h"

int main()
{
    // ① 创建上下文：SDK 所有操作的入口，必须第一个调用
    dmcan_context* ctx = nullptr;
    dmcan_context_create(&ctx);
    if (ctx == nullptr) {
        std::printf("[FAIL] dmcan_context_create 返回空\n");
        return 1;
    }
    dmcan_print_version(ctx);

    // ② 只找 LINKX4C 这一种设备（别用 dmcan_find_devices，会混进别的型号）
    int n = dmcan_find_devices_with_type(ctx, LINKX4C);
    std::printf("[INFO] 找到 LINKX4C 设备数 = %d\n", n);
    if (n == 0) {
        std::printf("[FAIL] 没找到设备。检查：USB 插好没 / 有没有 sudo\n");
        dmcan_context_destroy(ctx);
        return 2;
    }
    dmcan_show_all_devices(ctx);

    // ③ 取第 0 个设备句柄并打开
    dmcan_device_handle* dev = nullptr;
    if (!dmcan_device_get(ctx, &dev, 0)) {
        std::printf("[FAIL] dmcan_device_get 失败\n");
        dmcan_context_destroy(ctx);
        return 3;
    }
    if (!dmcan_device_open(dev)) {
        std::printf("[FAIL] dmcan_device_open 失败\n");
        dmcan_context_destroy(ctx);
        return 4;
    }
    dmcan_device_print_version(dev);

    // ④ 配 4 个通道：CAN 2.0 / 1 Mbps / 采样点 0.75（达妙固定参数）
    int ok_cnt = 0;
    for (uint8_t ch = 0; ch < 4; ++ch) {
        if (!dmcan_device_enable_channel(dev, ch)) {
            std::printf("[FAIL] ch%d enable_channel 失败\n", static_cast<int>(ch));
            continue;
        }

        dmcan_channel_can_info_t info = {};
        if (!dmcan_device_get_channel_baudrate(dev, ch, &info)) {
            std::printf("[FAIL] ch%d get_channel_baudrate 失败\n", static_cast<int>(ch));
            continue;
        }

        info.canfd        = false;       // 普通 CAN，不是 CAN FD
        info.can_baudrate = 1000000;     // 1 Mbps
        info.can_sp       = 0.75f;       // 采样点

        if (!dmcan_device_set_channel_baudrate(dev, ch, info)) {
            std::printf("[FAIL] ch%d set_channel_baudrate 失败\n", static_cast<int>(ch));
            continue;
        }

        std::printf("[ OK ] ch%d 已使能，1 Mbps，采样点 0.75\n", static_cast<int>(ch));
        ++ok_cnt;
    }

    std::printf("\n[RESULT] %d/4 通道配置成功\n", ok_cnt);

    // ⑤ 收尾
    dmcan_device_close(dev);
    dmcan_context_destroy(ctx);
    return (ok_cnt == 4) ? 0 : 5;
}
