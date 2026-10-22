// ============================================================
//  dm_scan.cpp —— 达妙电机 CAN_ID / MST_ID 扫描器
//
//  ⭐ 硬件到手第一件事跑这个！
//     刚拿到电机，你不知道它的 CAN_ID（发指令用）和 MST_ID（反馈用）。
//     用 DMTool 可能认不到 DM-LinkX-4C，所以自己扫。
//
//  原理（不依赖读寄存器，最可靠）：
//    ① 电机上电后会【主动】以 1kHz 发反馈帧，ID = MST_ID
//       → 什么都不发，监听 2 秒，收到的 ID 就是 MST_ID
//
//    ② 反馈帧 D[0] 的高 4 位 = ERR 状态
//       0 = 失能，1 = 使能
//       → 对 CAN_ID = 1~8 逐个发【使能帧】
//       → 哪个 CAN_ID 发出去后 ERR 变成 1，那个就是 CAN_ID
//
//  跑法：
//    sudo ros2 run dm4310_driver dm_scan
//    或
//    sudo ./install/dm4310_driver/lib/dm4310_driver/dm_scan
// ============================================================

#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "dmcan.h"
#include "can/can.hpp"

// 达妙使能/失能帧（8 字节固定内容）
// 本文件是独立工具，不依赖 dm_protocol.cpp，所以常量就地定义
static const uint8_t kFrameEnable[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static const uint8_t kFrameDisable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

// ---------------- 接收记录 ----------------
struct IdStat
{
    int      count = 0;          // 收到多少帧
    uint8_t  last_err = 0xFF;    // 最近一次 ERR
    uint8_t  id_field = 0;       // 帧里的 ID 字段
    uint8_t  last_data[8] = {0};
};

static std::mutex          g_mutex;
static std::map<uint32_t, IdStat> g_stats;
static bool                g_recv_cb_fired = false;

static void On_Recv(dmcan_device_handle* /*h*/, usb_rx_frame_t* f)
{
    if (f == nullptr || f->head.dlc < 8)
    {
        return;
    }

    std::lock_guard<std::mutex> lk(g_mutex);
    IdStat& s = g_stats[f->head.can_id];
    s.count++;
    s.id_field  = static_cast<uint8_t>(f->payload[0] & 0x0F);
    s.last_err  = static_cast<uint8_t>((f->payload[0] >> 4) & 0x0F);
    std::memcpy(s.last_data, f->payload, 8);
    g_recv_cb_fired = true;
}

static void Sleep_Ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static const char* Err_Name(uint8_t err)
{
    switch (err)
    {
        case 0x0: return "失能";
        case 0x1: return "使能";
        case 0x8: return "超压";
        case 0x9: return "欠压";
        case 0xA: return "过流";
        case 0xB: return "MOS过温";
        case 0xC: return "线圈过温";
        case 0xD: return "通讯丢失";
        case 0xE: return "过载";
        default:  return "未知";
    }
}

int main()
{
    std::printf("=====================================================\n");
    std::printf("  达妙电机 CAN_ID / MST_ID 扫描器\n");
    std::printf("  ⚠️ 跑之前请：\n");
    std::printf("     1. DM-LinkX-4C 插好，CAN 线接好\n");
    std::printf("     2. 电机【24V 已上电】\n");
    std::printf("     3. 电机【悬空固定】，周围别放东西\n");
    std::printf("=====================================================\n\n");

    // ---------------- 打开设备 ----------------
    // ⭐ 走 can 层统一入口，不重复写 SDK 打开流程
    const uint8_t CH = 0;

    dmcan_context*       ctx = nullptr;
    dmcan_device_handle* dev = nullptr;
    if (!OpenCanDevice(&ctx, &dev, CH))
    {
        return 2;
    }

    // 本工具自己的接收回调
    dmcan_device_hook_recv_callback(dev, On_Recv);

    // ============================================================
    //  第 1 步：静默监听，找 MST_ID
    // ============================================================
    std::printf("-----------------------------------------------------\n");
    std::printf("  第 1 步：静默监听 2 秒，找 MST_ID（电机主动上报）\n");
    std::printf("-----------------------------------------------------\n");

    Sleep_Ms(2000);

    std::vector<uint32_t> seen_ids;
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        if (g_stats.empty())
        {
            std::printf("[WARN] 2 秒内没收到任何 CAN 帧\n");
            std::printf("       可能原因：\n");
            std::printf("         · 电机没上电（24V 电源没开）\n");
            std::printf("         · CAN_H / CAN_L 接反\n");
            std::printf("         · 没有终端电阻（长线才需要）\n");
            std::printf("         · 波特率不是 1 Mbps\n");
            std::printf("         · 电机坏了\n\n");
        }
        else
        {
            std::printf("[ OK ] 收到 %zu 种 CAN ID 的帧：\n\n", g_stats.size());
            for (auto& kv : g_stats)
            {
                seen_ids.push_back(kv.first);
                std::printf("   CAN ID = 0x%03X   %d 帧   ERR=%X(%s)   内部ID字段=%d\n",
                            kv.first, kv.second.count, kv.second.last_err,
                            Err_Name(kv.second.last_err), kv.second.id_field);
            }
            std::printf("\n");
            std::printf("   ⭐ 这些 ID 就是【MST_ID】（反馈 ID）\n");
            std::printf("   ⭐ 最频繁的那个（1000 帧/秒）就是你的电机\n\n");
        }
    }

    if (seen_ids.empty())
    {
        std::printf("[STOP] 收不到反馈，无法继续扫描 CAN_ID。\n");
        std::printf("       请先解决上面列出的问题。\n");
        CloseCanDevice(ctx, dev);
        return 5;
    }

    // 取收到最多的那个 ID 作为参考
    uint32_t primary_id = 0;
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        int best = -1;
        for (auto& kv : g_stats)
        {
            if (kv.second.count > best) { best = kv.second.count; primary_id = kv.first; }
        }
    }

    // ============================================================
    //  第 2 步：逐个试 CAN_ID，看 ERR 是否从「失能」变「使能」
    // ============================================================
    std::printf("-----------------------------------------------------\n");
    std::printf("  第 2 步：对 CAN_ID 1~8 逐个发使能帧，看 ERR 是否变 1\n");
    std::printf("-----------------------------------------------------\n");
    std::printf("  说明：反馈帧 D[0] 高 4 位 = ERR，0=失能 1=使能\n");
    std::printf("       使能帧发对了 CAN_ID，ERR 会从 0 变成 1\n\n");

    int found_can_id = -1;

    for (uint8_t try_id = 1; try_id <= 8; ++try_id)
    {
        // 先把当前 ERR 记下来
        uint8_t err_before = 0xFF;
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            auto it = g_stats.find(primary_id);
            if (it != g_stats.end()) { err_before = it->second.last_err; }
        }

        // 发 3 次使能帧
        uint8_t en[8];
        std::memcpy(en, kFrameEnable, 8);
        for (int k = 0; k < 3; ++k)
        {
            CanSendMsg(dev, CH, en, try_id);
            Sleep_Ms(10);
        }

        // 等 200ms 看 ERR 变没变
        Sleep_Ms(200);

        uint8_t err_after = 0xFF;
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            auto it = g_stats.find(primary_id);
            if (it != g_stats.end()) { err_after = it->second.last_err; }
        }

        std::printf("  试 CAN_ID = %d : ERR %X(%s) → %X(%s)  ",
                    try_id, err_before, Err_Name(err_before),
                    err_after, Err_Name(err_after));

        if (err_after == 0x1 && err_before != 0x1)
        {
            std::printf("⭐ 命中！\n");
            found_can_id = try_id;
            break;
        }
        else if (err_after == 0x1)
        {
            std::printf("（本来就是使能）\n");
        }
        else
        {
            std::printf("\n");
        }
    }

    // ============================================================
    //  结果
    // ============================================================
    std::printf("\n=====================================================\n");
    std::printf("  扫描结果\n");
    std::printf("=====================================================\n");
    std::printf("  MST_ID  (反馈 ID) : %u   (0x%03X)\n", primary_id, primary_id);

    if (found_can_id > 0)
    {
        std::printf("  CAN_ID  (发送 ID) : %d\n", found_can_id);
        std::printf("\n  ⭐ 把这两个填进 config/dm4310_params.yaml：\n");
        std::printf("       can_id: %d\n", found_can_id);
        std::printf("       mst_id: %u\n", primary_id);
    }
    else
    {
        std::printf("  CAN_ID  (发送 ID) : ⚠️ 没扫到\n");
        std::printf("\n  可能原因：\n");
        std::printf("    · 电机已经被使能了（ERR 一直是 1）\n");
        std::printf("    · 使能需要先发别的命令\n");
        std::printf("    · 电机 CAN_ID 不是 1~8\n");
        std::printf("    · 波特率不对\n");
        std::printf("\n  下一步：先用 dm_probe 看反馈帧内容，或试 DMTool\n");
    }

    // 收尾：失能，避免电机一直带电
    std::printf("\n  发送失能帧…\n");
    uint8_t dis[8];
    std::memcpy(dis, kFrameDisable, 8);
    const uint32_t off_id = (found_can_id > 0) ? static_cast<uint32_t>(found_can_id) : primary_id;
    for (int k = 0; k < 3; ++k)
    {
        CanSendMsg(dev, CH, dis, off_id);
        Sleep_Ms(10);
    }

    CloseCanDevice(ctx, dev);
    std::printf("\n完成。\n");
    return 0;
}
