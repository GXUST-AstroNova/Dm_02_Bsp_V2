//
// 遥控接收机基类（协议无关）
//
// 设计约定：
//   - 基类持有通道缓存与全部访问接口，派生类只负责“把一帧数据解出来并提交”。
//   - 原始量程由派生类在构造时用 setRawRange() 设定（SBUS: 172/992/1811；
//     将来接 DBUS 就是 364/1024/1684），基类不对具体协议做任何假设。
//   - 返回值：Init() 0 = 成功、-1 = 失败。
//
// 用法：
//   static Sbus rc(&huart5);
//   rc.Init();
//   // 任意位置：
//   if (rc.Is_Online()) { float yaw = rc.Get_Channel(0); }
//
#ifndef DM_02_BSP_BASE_RC_RECEIVER_H
#define DM_02_BSP_BASE_RC_RECEIVER_H

#include <cstdint>

#include "RcSnapshot.h"

/**
 * 遥控接收机基类
 */
class RcReceiver {
public:
    static constexpr uint8_t kMaxChannels = 16U;

    virtual ~RcReceiver() = default;

    // -------------------- 生命周期 --------------------

    /**
     * 启动接收。
     * @return 0 = 成功；-1 = 失败（此时 Is_Ready() 保持 false）
     */
    virtual int8_t Init() = 0;

    // -------------------- 访问接口（getter） --------------------

    /** 帧里实际有效的通道槽位数（SBUS 恒为 16） */
    uint8_t Get_ChannelCount() const noexcept { return count_; }

    /** 11 bit 原始值；通道号非法时返回量程中位 */
    uint16_t Get_Raw(uint8_t ch) const noexcept;

    /** 归一化到 -1.0 ~ +1.0，中位为 0.0，带死区归零；通道号非法时返回 0.0f */
    float Get_Channel(uint8_t ch) const noexcept;

    /** 换算成 1000 ~ 2000 us；通道号非法时返回 1500.0f */
    float Get_Microseconds(uint8_t ch) const noexcept;

    bool Is_ChannelValid(uint8_t ch) const noexcept { return ch < count_; }

    // -------------------- 状态 --------------------

    /** 是否已完成初始化（Init() 成功过） */
    bool Is_Ready() const noexcept { return ready_; }

    /**
     * 是否在线：距最近一次成功解帧的时长是否在窗口内。
     * 从未解出过任何一帧时恒为 false。
     */
    bool Is_Online(uint32_t timeout_ms = 100U) const noexcept;

    /** 接收机上报的失控（failsafe）标志 */
    bool Is_Failsafe() const noexcept { return failsafe_; }

    /** 接收机上报的丢帧（frame lost）标志 */
    bool Is_FrameLost() const noexcept { return frame_lost_; }

    /** 最近一次成功解帧的 HAL tick */
    uint32_t Get_LastUpdateTick() const noexcept { return last_update_tick_; }

    /** 累计成功解帧次数（0 表示从未解出） */
    uint32_t Get_UpdateCount() const noexcept { return update_count_; }

    /** 累计 UART 错误次数 */
    uint32_t Get_ErrorCount() const noexcept { return error_count_; }

    /**
     * 整体快照（临界区），供控制节拍/纯逻辑遥控策略使用。
     * 与逐个 getter 相比，保证通道、量程、时间戳、标志来自同一帧。
     */
    void Get_Snapshot(RcSnapshot &out) const noexcept;

protected:
    RcReceiver() = default;

    /** 由派生类在构造时设定原始量程 */
    void setRawRange(uint16_t lo, uint16_t mid, uint16_t hi) noexcept;

    void setReady(bool r) noexcept { ready_ = r; }

    /**
     * 提交一帧成功解出的数据：写通道 + 打时间戳 + 计数 + 更新标志。
     * @param channels 原始通道值数组（至少 count 个）
     * @param count    本次有效的通道数（超过 kMaxChannels 的部分被截断）
     */
    void markUpdated(const uint16_t *channels, uint8_t count,
                     bool failsafe, bool frame_lost) noexcept;

    /** 累计一次 UART 错误 */
    void countError() noexcept { ++error_count_; }

    uint16_t channels_[kMaxChannels] {};   ///< 原始值快照（未解出过时全 0）
    uint16_t raw_lo_  = 0U;                ///< 量程下限
    uint16_t raw_mid_ = 0U;                ///< 量程中位
    uint16_t raw_hi_  = 0U;                ///< 量程上限
    uint8_t  count_   = 0U;                ///< 有效通道数
    uint32_t last_update_tick_ = 0U;       ///< 最近一次成功解帧的 tick
    uint32_t update_count_     = 0U;       ///< 累计成功解帧次数
    uint32_t error_count_      = 0U;       ///< 累计 UART 错误次数
    bool     ready_      = false;          ///< Init() 是否成功
    bool     failsafe_   = false;          ///< 失控标志
    bool     frame_lost_ = false;          ///< 丢帧标志
};

#endif // DM_02_BSP_BASE_RC_RECEIVER_H
