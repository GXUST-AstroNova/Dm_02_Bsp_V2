# 蜂鸣器乐谱：《你》（RoboMaster 主题曲）

本目录放从乐谱转换出的 `BuzzerNote` 数组。

| 文件 | 内容 |
| --- | --- |
| `ni.h` | 《你》全曲主旋律（右手最高声部，单音），550 个音符，约 216 秒 |

## 来源与生成方式

- 来源：`你.pdf`（MuseScore 3.6.2 导出，5 页；原作 GALA，钢琴改编 麻治昊 / RoboWalker）
- 音高：解析 PDF 内嵌的 Leland 乐谱字体字形（notehead 字形 + 谱号位置）得到，包含
  调号（降 E 大调）与升/降/还原记号；
- 时值：按 PDF 谱面“小节内 x 间距”近似重建（4/4、♩=75.5），**未处理反复记号、
  连音线、渐慢/渐快**，属于可播放草稿；
- 全曲按 5 页顺序展开（不重复）。

## 用法

```cpp
#include "scores/ni.h"   // Hardware/Buzzer 已在 include 路径

static PwmBuzzer buzzer(&htim12, PwmBuzzer::Config{});
buzzer.Init();
buzzer.Play(buzzer_score::kNiSong, buzzer_score::kNiSongCount, HAL_GetTick());

// 主循环：
buzzer.Update(HAL_GetTick());
```

如只想播放某一段，可直接传数组子区间：

```cpp
buzzer.Play(&buzzer_score::kNiSong[start], count, HAL_GetTick());
```

## 限制

- 单音蜂鸣器无法还原钢琴左右手/和弦，这里取右手最高声部；
- 节奏是按谱面间距近似的，个别连音/休止可能与原谱有差异；
- 若需要精确时值，建议提供 MuseScore 源文件（`.mscz`）或 MIDI，可重新生成。
