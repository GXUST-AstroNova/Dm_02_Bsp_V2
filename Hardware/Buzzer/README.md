# 无源蜂鸣器（TIM12_CH2 / PB15）

板载无源蜂鸣器（典型谐振 4 kHz）：NPN 低边驱动，TIM12_CH2（PB15）经 R20 接 Q2 基极，
**高电平导通**，50% 占空比方波驱动。TIM12 已由 CubeMX 配成 PWM1：
定时器时钟 275 MHz，PSC=274 → 计数 1 MHz，ARR=249 → 4 kHz。

## 文件

| 文件 | 内容 |
| --- | --- |
| `buzzer_note.h` | `BuzzerNote{freq_hz, duration_ms}` + 十二平均律音符常量（C4..B6，0=休止） |
| `buzzer_tone.h` | 纯逻辑：频率→ARR、50% 占空比（constexpr，可主机测试） |
| `base_buzzer.h/.cpp` | 抽象基类：**非阻塞**乐谱播放（Play/Update/Stop），不依赖 HAL |
| `pwm_buzzer.h/.cpp` | TIM12_CH2 硬件实现：自动推导计数频率、启动 PWM、写 ARR/CCR |
| `README.md` | 本文件 |

## 非阻塞原理

- 声音由 TIM12 硬件 PWM 持续产生，软件**不需要**逐周期输出；
- `Play()` 只做参数校验 + 记录播放计划，立即返回；
- `Update(now_ms)` 只比较时间，换音时写 `ARR/CCR` 两个寄存器（微秒级），
  不做 `HAL_Delay`、不轮询标志、不阻塞；
- `Stop()`/播完只把 CCR 清 0（晶体管关断），立即静音；
- 建议在主循环调用 `Update`；若主循环可能长时间阻塞，也可以放到现有 1 kHz 节拍里
  （每拍几次整数比较）。

## 用法

```cpp
#include "pwm_buzzer.h"

static PwmBuzzer buzzer(&htim12, PwmBuzzer::Config{});   // 自动推导 1 MHz 计数

void Board_Status_Init(void) { (void)buzzer.Init(); }

// 任意音乐段：数组/子区间（指针+个数）均可
static constexpr BuzzerNote kTwinkle[] = {
    {buzzer_note::C5, 400}, {buzzer_note::C5, 400},
    {buzzer_note::G5, 400}, {buzzer_note::G5, 400},
    {buzzer_note::A5, 400}, {buzzer_note::A5, 400},
    {buzzer_note::G5, 600}, {buzzer_note::kRest, 120},
};

void App_Play(void) {
    buzzer.Play(kTwinkle, sizeof(kTwinkle) / sizeof(kTwinkle[0]), HAL_GetTick());
}

void App_Loop(void) {
    buzzer.Update(HAL_GetTick());   // 非阻塞：仅比较/换音
}
```

序列语义：

- 第一拍从 `Play(now_ms)` 起算，之后按每音时长递推，不因 `Update` 抖动累积漂移；
- 休止符 `{0, ms}` 占时长但不发声；`duration_ms=0` 的音符被跳过；
- `Play(..., loop=true)` 循环播放；非法输入（空指针/0 个/频率越界）被拒绝且不改变当前播放；
- 主循环长时间卡顿时单次 `Update` 最多追赶 `count+1` 个音符并重新对齐当前时刻，
  不会无限循环；
- 时间全部用 `uint32_t` 毫秒差，支持 `HAL_GetTick()` 回绕。

`PwmBuzzer::Config`：

| 字段 | 默认 | 说明 |
| --- | --- | --- |
| `timer_clock_hz` | 0 | 0=从 `HAL_RCC_GetPCLK1Freq/HCLK` 自动推导（当前 275 MHz） |
| `channel` | `TIM_CHANNEL_2` | PB15 对应 TIM12_CH2 |
| `min_hz / max_hz` | 20 / 10000 | `Play` 校验的音符范围；超出拒绝 |

`buzzer_tone` 换算：`ARR = round(tick_hz/f)-1`，`CCR = (ARR+1)/2`。当前 tick=1 MHz：
440 Hz→ARR 2272、4000 Hz→249。

## 乐谱

`scores/` 目录存放从实际乐谱转换的 `BuzzerNote` 数组，例如
`ni.h`（《你》RoboMaster 主题曲全曲主旋律，550 音，约 216 s，自动识谱草稿）：

```cpp
#include "scores/ni.h"
buzzer.Play(buzzer_score::kNiSong, buzzer_score::kNiSongCount, HAL_GetTick());
```

详见 `scores/README.md`（来源、生成方式与限制）。

## 注意

- 无源蜂鸣器无音量控制；4 kHz 附近最响，低音明显偏小属正常。
- 驱动在 `Hardware_Init()` 里置 `TIM_CR1_ARPE`（ARR 预装载）以减少换音毛刺，同时关闭
  CCR 预装载（`OC2PE`），使 `Stop`/静音立即生效（最多一次 PWM 周期内的边沿抖动，可忽略）；
- 只支持 `TIM_CHANNEL_2`（PB15）；`Config::channel` 传其它值会让 `Init()` 返回 -1；
- 不改 TIM12 中断/DMA，不影响 1 kHz 底盘控制节拍。
