//
// 蜂鸣器抽象基类：非阻塞音乐段播放
//
// 设计约定（与参考工程“基类持有逻辑/状态，派生类实现硬件”一致）：
//   - Init()：调用派生类 Hardware_Init() 并静音；只用一次。
//   - Play()：只做参数校验 + 记录播放计划，立即返回（不等待、不延时）。
//   - Update(now_ms)：非阻塞序列推进；只有换音/结束时才调用 Apply_Tone()（写寄存器）。
//   - 音调由硬件 PWM 持续产生，软件不需要在播放期间持续“发送”。
//   - 所有时间用 uint32_t 毫秒差，支持 HAL_GetTick 回绕。
//
// 非阻塞边界：
//   - Play/Update/Stop 不做 HAL_Delay、不轮询标志、不阻塞；
//   - Update 建议在主循环调用；若希望节奏不受主循环长阻塞影响，也可以放在
//     1 kHz 定时节拍里（每拍只做几次比较，必要时一次寄存器写）。
//
// 序列语义：
//   - 第一拍从 Play(now_ms) 的 now_ms 起算，之后按 note_start += duration 递推，
//     不会因 Update 调用抖动产生累积漂移；
//   - 休止符（freq=0）占时长但不发声；duration_ms=0 的音符在推进时被跳过；
//   - loop=true 时末音后回到第 0 个音符；
//   - 主循环长时间卡顿时，单次 Update 最多追赶 count+1 个音符并重新对齐当前时刻，
//     不会无限循环、也不会把中间音符完整“补播”；
//   - 非法输入（空指针/0 个/频率越界/总时长为 0）被拒绝，且不改变当前播放。
//

#ifndef DM_02_BSP_BASE_BUZZER_H
#define DM_02_BSP_BASE_BUZZER_H

#include <cstdint>

#include "buzzer_note.h"

class Buzzer {
  public:
    Buzzer(uint16_t min_hz, uint16_t max_hz) noexcept;
    virtual ~Buzzer() = default;

    Buzzer(const Buzzer &) = delete;
    Buzzer &operator=(const Buzzer &) = delete;

    /** 硬件初始化并静音；0=成功，-1=失败。 */
    int8_t Init() noexcept;

    /**
     * 播放一段乐谱（可传任意 BuzzerNote 数组/子区间）。
     * @param now_ms 第一拍起始时刻（通常 HAL_GetTick()）
     * @return false = 未初始化 / 参数非法（此时当前播放保持不变）
     */
    bool Play(const BuzzerNote *notes, uint16_t count, uint32_t now_ms,
              bool loop = false) noexcept;

    /** 立即静音并停止当前播放（只写寄存器，非阻塞）。 */
    void Stop(uint32_t now_ms) noexcept;

    /** 非阻塞推进序列；换音/结束时写一次硬件。 */
    void Update(uint32_t now_ms) noexcept;

    bool Is_Ready() const noexcept { return ready_; }
    bool Is_Playing() const noexcept { return playing_; }
    uint16_t Position() const noexcept { return index_; } ///< 当前音符下标
    uint16_t Note_Count() const noexcept { return count_; }
    uint16_t Min_Hz() const noexcept { return min_hz_; }
    uint16_t Max_Hz() const noexcept { return max_hz_; }

  protected:
    /** 派生类硬件初始化（配置定时器/启动 PWM），0=成功。 */
    virtual int8_t Hardware_Init() noexcept = 0;
    /** 派生类输出目标音调；freq_hz=0 表示静音。只写寄存器，不阻塞。 */
    virtual void Apply_Tone(uint16_t freq_hz) noexcept = 0;

  private:
    const BuzzerNote *notes_ = nullptr;
    uint16_t count_ = 0U;
    uint16_t index_ = 0U;
    uint32_t note_start_ms_ = 0U;
    uint16_t min_hz_ = 0U;
    uint16_t max_hz_ = 0U;
    bool loop_ = false;
    bool playing_ = false;
    bool applied_ = false;
    bool ready_ = false;
};

#endif // DM_02_BSP_BASE_BUZZER_H
