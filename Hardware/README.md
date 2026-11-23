# V2 硬件接口

## 电机接入

在 CubeMX 完成初始化后，对每条使用的总线调用一次 `can_bus::start(&hfdcanX)`，检查返回值，然后初始化具体型号。电机对象须在整个使用期间存活。请在应用停机时先下发失能命令，再销毁实例。大疆的失能需要再调用一次分组发送。

| 所接总线 | 达妙默认帧格式 | 仲裁段 | 数据段 | 反馈入口 |
| --- | --- | --- | --- | --- |
| FDCAN1、FDCAN3 | 经典 CAN | 1 Mbps | 1 Mbps | FIFO0 |
| FDCAN2 | CAN FD + BRS | 1 Mbps | 5 Mbps | FIFO1 |

FDCAN2 的内核时钟为 80 MHz。仲裁段每位为 `2 × (1 + 32 + 7)` 个时钟，数据段每位为 `1 × (1 + 11 + 4)` 个时钟。两种格式均使用 11 位标准 ID 和 8 字节 MIT 载荷。电调的模式和波特率须与该节点匹配；发送 FD 帧的物理总线上，其他节点也须能容忍 FD 帧。

总线进入 Bus_Off 后，`hfdcan_bsp` 在错误状态中断里自动调用 `can_bus::Soft_Restart`：先中止未发完的 Tx 请求，再清除 `CCCR.INIT`，控制器自行完成 129 次 11 个连续隐性位的恢复序列后继续收发。若软复位本身失败（`HAL_FDCAN_Stop` 超时会让 HAL 状态机进入 ERROR），总线保持静默且不再上报，需要应用层调用 `can_bus::Hard_Restart` 兜底。

- 达妙使用 `init(command_id, feedback_id)` 注册。第一个参数是发往电调的命令 ID（CAN ID），第二个是电调反馈帧的返回 ID（Master ID）。在构造和注册时都可以直接写整数字面量或 `constexpr` 整数。编译器在缩窄转换前检查两个 ID 是否都在 `0x000…0x7FF` 内。
- 帧格式默认跟随总线。要显式选择经典 CAN 或 CAN FD + BRS，第三个参数传 `DmCanMode::Classic` 或 `DmCanMode::FdBrs`。原有 `Dm4310 joint(&hfdcan2, command_id, feedback_id)` 写法仍可用，随后调用 `joint.init()` 或 `joint.init(mode)` 注册构造时给定的地址。
- 在 `Motor/DM/dm_models.hpp` 中为使用的子型号填写与电调寄存器一致的 `T_MAX`。构造该型号时编译器要求它是有限正数，未使用的型号可以保留占位值。自定义型号继承 `DmMotorModel<型号>`，参照 `Dm4310` 声明 `T_MAX` 并继承构造函数。各型号共用 `P_MAX = 12.5664 rad` 和 `V_MAX = 45 rad/s`。
- 多台达妙共用一条物理总线时，把这条总线的地址写在同一个 `constexpr auto ids = dmBusAddresses({{0x01, 0x10}, {0x02, 0x10}});` 表中，各电机调用 `init(ids[0])`、`init(ids[1])`。表内命令 ID 重复，或者返回 ID 相同且命令 ID 的低四位相同时，编译失败。不同总线分别建表。独立注册之间的冲突、重复注册时改绑、总线模式不匹配和注册表已满，由 `init()` 返回 `false`。
- 一个业务对象写入每台电机的目标，观察方读取 `feedback()` 快照。角速度为 rad/s，力矩为 N·m。首帧前 `valid` 为假。单圈角范围为 `[−π, π)`，累计角包含首帧原始角。连续解圈要求相邻有效采样之间的机械位移小于半圈。

```cpp
#include "dm_models.hpp"
#include "m3508.hpp"
#include "hfdcan_bsp.h"

static Dm4310 joint(&hfdcan2);
static M3508 wheel(&hfdcan1, 1);

bool initMotors() {
    return can_bus::start(&hfdcan1) && can_bus::start(&hfdcan2) &&
           wheel.init() && joint.init(0x01, 0x11) && joint.enable();
}

bool sendMotorTargets(float joint_nm, float wheel_nm) {
    if (!joint.setTorque(joint_nm) || !wheel.setTorque(wheel_nm)) return false;
    const auto dm = DmMotor::sendAll();
    const auto dji = DjiMotorGroup::sendAll();
    return dm.queued_frames == dm.attempted_frames &&
           dji.queued_frames == dji.attempted_frames;
}
```

`setMit({position_rad, velocity_rad_s, kp, kd, torque_ff_nm})` 和 `setTorque()` 只更新目标。每拍调用 `sendAll()` 会重新提交当前目标，返回本次尝试和成功入队的 CAN 帧数。使能、失能、清故障操作单独检查返回值。M3508 的 `resetAccumulatedAngle()` 立即将软件累计角归零。

## 传感器

BMI088 使用 SPI2 DMA。初始化后用 `Start()` 发起采样，`Process() == 1` 表示已提交新帧。`Get_Data()` 返回值快照，其中 `gyro_deg_s` 供 Fusion 使用，`gyroRadS()` 返回同一帧的 rad/s 值，`accel_m_s2` 和 `temperature_c` 分别使用 m/s² 和 °C。FlySky SBUS 在 UART5 上使用 DMA，`Get_Snapshot()` 读取通道、丢帧和失控状态。

## 验证

原生 C++20 环境下运行 `cmake -S tests -B build/host`、`cmake --build build/host` 和 `ctest --test-dir build/host --output-on-failure`。STM32 固件使用根目录的 Debug、Release 预设。实机验收时，对照分析仪核对 FDCAN1/3 的经典帧和 FDCAN2 的 FDF、BRS、1/5 Mbps 位时序及双向反馈。
