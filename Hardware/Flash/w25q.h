//
// W25Q64 (8 MB) SPI NOR Flash —— OCTOSPI 间接模式驱动
//
// 从 RM25_firmware 的 Application/Src/bsp/w25q.c + w25q.h 移植，改为 C++ 类封装。
// 语义（命令码、状态位、WREN -> 操作 -> 忙等待 的时序、跨页拆分、错误码）与原版一致。
//
// ---------------------------------------------------------------------------
// 硬件（本工程，Core/Src/octospi.c 已配好，无需改 CubeMX）
// ---------------------------------------------------------------------------
//   OCTOSPI2 (hospi2)
//     PA1 = IO3   PA3 = IO2   PB0 = IO1   PB2 = CLK   PE11 = NCS   PD11 = IO0
//     ClockMode = 3, SampleShifting = HALFCYCLE, DeviceSize = 2^23 = 8 MB
//     DelayBlockBypass, MemoryType = MICRON
//     内核时钟 = D1HCLK (275 MHz), ClockPrescaler = 10  ->  OSPI 时钟 = 25 MHz
//     W25Q64 的 0xEB 快速读上限 133 MHz，25 MHz 下 DummyCycles = 6 是标准取值
//
// 注意：本驱动只做**间接模式**（indirect）读写擦；不做内存映射模式。
//       将来若要上内存映射模式，OCTOSPI2 的映射基址是 **0x70000000**
//       （OCTOSPI1 才是 0x90000000，别抄错），且进入映射后必须 HAL_OSPI_Abort()
//       才能回到间接模式。
//
// ---------------------------------------------------------------------------
// 用法
// ---------------------------------------------------------------------------
//   static W25Qxx flash(&hospi2);
//   if (flash.Init() != W25Qxx::Status::Ok) { /* JEDEC ID 不对，看 flash.jedecId() */ }
//
//   uint8_t buf[256];
//   flash.EraseSector(0x000000);
//   flash.Write(buf, 0x000000, sizeof(buf));   // 任意长度，自动跨页
//   flash.Read(buf, 0x000000, sizeof(buf));
//
// 上板验收清单：
//   Init() == Ok 且 jedecId() == 0xEF4017；
//   擦一个扇区 -> 读回全 0xFF；写 256 字节 -> 读回一致；再擦 -> 又全 0xFF。
//
// 单线程使用（无锁）；不做磨损均衡 / 文件系统，只提供裸读写擦。
//

#ifndef DM_02_BSP_W25Q_H
#define DM_02_BSP_W25Q_H

#include <cstdint>

#include "octospi.h"   // OSPI_HandleTypeDef + HAL_OSPI_* 枚举

class W25Qxx {
public:
    /** 与 RM25 的错误码一一对应；ErrorParam 是移植时新增的入参校验码 */
    enum class Status : int8_t {
        Ok               =  0,   ///< 成功
        ErrorInit        = -1,   ///< 初始化失败（JEDEC ID 不匹配）
        ErrorWriteEnable = -2,   ///< 写使能失败
        ErrorAutoPolling = -3,   ///< 轮询等待超时 / 无响应
        ErrorErase       = -4,   ///< 擦除命令发送失败
        ErrorTransmit    = -5,   ///< 传输失败
        ErrorParam       = -6,   ///< 入参非法（空指针 / 长度 0 / 越界 / 跨页 / 未对齐）
    };

    // -------------------- 容量常量（W25Q64） --------------------
    static constexpr uint32_t kPageSize           = 256U;
    static constexpr uint32_t kSectorSize         = 4096U;
    static constexpr uint32_t kBlock32KSize       = 32768U;
    static constexpr uint32_t kBlock64KSize       = 65536U;
    static constexpr uint32_t kFlashSize          = 0x00800000U;   ///< 8 MB
    static constexpr uint32_t kJedecIdW25Q64      = 0x00EF4017U;
    static constexpr uint32_t kChipEraseTimeoutMs = 100000U;       ///< 整片擦最大 100 s

    /**
     * @param hospi 该 flash 所在的 OCTOSPI 句柄（本工程为 &hospi2）
     * @warning 调用方需保证 MX_OCTOSPI2_Init() 已执行
     */
    explicit W25Qxx(OSPI_HandleTypeDef *hospi) noexcept;

    W25Qxx(const W25Qxx &) = delete;
    W25Qxx &operator=(const W25Qxx &) = delete;

    // -------------------- 初始化与识别 --------------------

    /** 读 JEDEC ID 并比对 W25Q64（0xEF4017）；成功记 ready */
    Status Init();

    /** 读 JEDEC ID；通信失败返回 0 */
    uint32_t ReadJedecId();

    // -------------------- 读 --------------------

    /** 任意长度读（0xEB，1-4-4 快速读） */
    Status Read(uint8_t *buf, uint32_t addr, uint32_t len);

    // -------------------- 写 --------------------

    /** 单页写：len <= 256 且不得跨页；写前必须已擦除 */
    Status WritePage(const uint8_t *buf, uint32_t addr, uint16_t len);

    /** 任意长度写，自动按页拆分；写前必须已擦除 */
    Status Write(const uint8_t *buf, uint32_t addr, uint32_t len);

    // -------------------- 擦 --------------------

    Status EraseSector(uint32_t addr);     ///< 4 KB，0x20
    Status EraseBlock32K(uint32_t addr);   ///< 32 KB，0x52
    Status EraseBlock64K(uint32_t addr);   ///< 64 KB，0xD8
    Status EraseChip();                    ///< 整片，0xC7（典型 20 s、最大 100 s）

    // -------------------- 状态 --------------------

    bool     IsReady() const noexcept { return ready_; }
    uint32_t jedecId() const noexcept { return jedec_id_; }
    OSPI_HandleTypeDef *bus() const noexcept { return hospi_; }

private:
    /**
     * 用一份显式基线填满整个 OSPI_RegularCmdTypeDef。
     * RM25 原版每个函数只赋一部分字段，AlternateBytes / AlternateBytesSize /
     * AlternateBytesDtrMode 等多处是未初始化栈垃圾；这里统一成基线 + 局部覆盖。
     */
    static void fillDefaultCmd(OSPI_RegularCmdTypeDef &cmd) noexcept;

    /** 0x06 写使能 + 轮询 SR1.WEL 置位 */
    Status writeEnable();

    /** 发 0x05 读状态寄存器1，并轮询 SR1.BUSY 清零 */
    Status waitMemReady(uint32_t timeout_ms);

    /** 通用擦除：WREN -> 发指令 -> 等 BUSY 清零 */
    Status eraseWithCmd(uint8_t instruction, uint32_t addr);

    OSPI_HandleTypeDef *hospi_ = nullptr;
    uint32_t jedec_id_ = 0U;
    bool     ready_ = false;
};

#endif // DM_02_BSP_W25Q_H
