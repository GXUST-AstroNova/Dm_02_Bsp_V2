//
// W25Q64 OCTOSPI 间接模式驱动实现
//
// 移植自 RM25_firmware: Application/Src/bsp/w25q.c
// 命令码、状态位与 WREN -> 操作 -> 忙等待 的时序与原版一致；
// 修正项见每处注释（基线填充 / 返回值检查 / 入参校验 / 去掉读后的多余轮询）。
//

#include "w25q.h"

namespace {

// -------------------- 指令 --------------------
constexpr uint8_t kCmdJedecId         = 0x9FU;   // 读 JEDEC ID
constexpr uint8_t kCmdWriteEnable     = 0x06U;   // 写使能
constexpr uint8_t kCmdReadStatusReg1  = 0x05U;   // 读状态寄存器 1
constexpr uint8_t kCmdSectorErase     = 0x20U;   // 扇区擦 4 KB
constexpr uint8_t kCmdBlockErase32K   = 0x52U;   // 块擦 32 KB
constexpr uint8_t kCmdBlockErase64K   = 0xD8U;   // 块擦 64 KB
constexpr uint8_t kCmdChipErase       = 0xC7U;   // 整片擦
constexpr uint8_t kCmdPageProgramQuad = 0x32U;   // 1-1-4 页编程
constexpr uint8_t kCmdFastReadQuadIo  = 0xEBU;   // 1-4-4 快速读

// -------------------- 状态寄存器 1 的位 --------------------
constexpr uint8_t kStatusReg1Busy = 0x01U;   // 忙标志
constexpr uint8_t kStatusReg1Wel  = 0x02U;   // 写使能锁存

// -------------------- 时序 --------------------
constexpr uint32_t kDefaultTimeoutMs = HAL_OSPI_TIMEOUT_DEFAULT_VALUE;   // 5 s
constexpr uint32_t kAutoPollingInterval = 0x10U;
constexpr uint32_t kFastReadDummyCycles = 6U;

} // namespace

// -------------------- 构造 --------------------

W25Qxx::W25Qxx(OSPI_HandleTypeDef *hospi) noexcept
    : hospi_(hospi) {
}

// -------------------- 私有工具 --------------------

void W25Qxx::fillDefaultCmd(OSPI_RegularCmdTypeDef &cmd) noexcept {
    cmd.OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId               = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction           = 0U;
    cmd.InstructionMode       = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Address               = 0U;
    cmd.AddressMode           = HAL_OSPI_ADDRESS_NONE;
    cmd.AddressSize           = HAL_OSPI_ADDRESS_24_BITS;
    cmd.AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytes        = 0U;
    cmd.AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
    cmd.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
    cmd.DataMode              = HAL_OSPI_DATA_NONE;
    cmd.NbData                = 0U;
    cmd.DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
    cmd.DummyCycles           = 0U;
    cmd.DQSMode               = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;
}

W25Qxx::Status W25Qxx::writeEnable() {
    OSPI_RegularCmdTypeDef cmd;
    OSPI_AutoPollingTypeDef poll;

    // 1) 发写使能指令（无地址、无数据）
    fillDefaultCmd(cmd);
    cmd.Instruction = kCmdWriteEnable;
    if (HAL_OSPI_Command(hospi_, &cmd, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorWriteEnable;
    }

    // 2) 发读状态寄存器 1 指令，进入自动轮询
    fillDefaultCmd(cmd);
    cmd.Instruction = kCmdReadStatusReg1;
    cmd.DataMode    = HAL_OSPI_DATA_1_LINE;
    cmd.NbData      = 1U;
    if (HAL_OSPI_Command(hospi_, &cmd, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorWriteEnable;
    }

    // 3) 轮询 SR1.WEL：屏蔽 WEL 位后与 0x02 比较，匹配即自动停止
    poll.Match         = kStatusReg1Wel;
    poll.Mask          = kStatusReg1Wel;
    poll.MatchMode     = HAL_OSPI_MATCH_MODE_AND;
    poll.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;
    poll.Interval      = kAutoPollingInterval;
    if (HAL_OSPI_AutoPolling(hospi_, &poll, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorAutoPolling;
    }
    return Status::Ok;
}

W25Qxx::Status W25Qxx::waitMemReady(uint32_t timeout_ms) {
    OSPI_RegularCmdTypeDef cmd;
    OSPI_AutoPollingTypeDef poll;

    fillDefaultCmd(cmd);
    cmd.Instruction = kCmdReadStatusReg1;
    cmd.DataMode    = HAL_OSPI_DATA_1_LINE;
    cmd.NbData      = 1U;
    if (HAL_OSPI_Command(hospi_, &cmd, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorAutoPolling;
    }

    // 屏蔽 BUSY 位后与 0 比较；整片擦时超时放宽到 100 s
    poll.Match         = 0U;
    poll.Mask          = kStatusReg1Busy;
    poll.MatchMode     = HAL_OSPI_MATCH_MODE_AND;
    poll.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;
    poll.Interval      = kAutoPollingInterval;
    if (HAL_OSPI_AutoPolling(hospi_, &poll, timeout_ms) != HAL_OK) {
        return Status::ErrorAutoPolling;
    }
    return Status::Ok;
}

// -------------------- 初始化与识别 --------------------

uint32_t W25Qxx::ReadJedecId() {
    if (hospi_ == nullptr) {
        return 0U;
    }

    OSPI_RegularCmdTypeDef cmd;
    uint8_t buf[3] = {0U, 0U, 0U};

    fillDefaultCmd(cmd);
    cmd.Instruction = kCmdJedecId;
    cmd.DataMode    = HAL_OSPI_DATA_1_LINE;
    cmd.NbData      = 3U;

    // RM25 原版这里不检查 HAL_OSPI_Command() 的返回值：
    // 通信失败时会把栈垃圾当 ID 返回。这里两个返回值都检查。
    if (HAL_OSPI_Command(hospi_, &cmd, kDefaultTimeoutMs) != HAL_OK) {
        return 0U;
    }
    if (HAL_OSPI_Receive(hospi_, buf, kDefaultTimeoutMs) != HAL_OK) {
        return 0U;
    }

    return (static_cast<uint32_t>(buf[0]) << 16) |
           (static_cast<uint32_t>(buf[1]) << 8) |
            static_cast<uint32_t>(buf[2]);
}

W25Qxx::Status W25Qxx::Init() {
    ready_ = false;
    jedec_id_ = 0U;

    if (hospi_ == nullptr) {
        return Status::ErrorInit;
    }

    jedec_id_ = ReadJedecId();
    if (jedec_id_ != kJedecIdW25Q64) {
        return Status::ErrorInit;
    }

    ready_ = true;
    return Status::Ok;
}

// -------------------- 读 --------------------

W25Qxx::Status W25Qxx::Read(uint8_t *buf, uint32_t addr, uint32_t len) {
    if (buf == nullptr || len == 0U) {
        return Status::ErrorParam;
    }
    if (addr >= kFlashSize || len > (kFlashSize - addr)) {
        return Status::ErrorParam;
    }
    if (hospi_ == nullptr) {
        return Status::ErrorTransmit;
    }

    OSPI_RegularCmdTypeDef cmd;
    fillDefaultCmd(cmd);
    cmd.Instruction  = kCmdFastReadQuadIo;
    cmd.Address      = addr;
    cmd.AddressMode  = HAL_OSPI_ADDRESS_4_LINES;   // 1-4-4
    cmd.DataMode     = HAL_OSPI_DATA_4_LINES;
    cmd.DummyCycles  = kFastReadDummyCycles;
    cmd.NbData       = len;

    if (HAL_OSPI_Command(hospi_, &cmd, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorTransmit;
    }
    if (HAL_OSPI_Receive(hospi_, buf, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorTransmit;
    }

    // 与 RM25 的差异：读操作不会把 BUSY 置位，因此不再调用忙轮询，
    // 省掉每次读都要多花的一条 0x05 事务。
    return Status::Ok;
}

// -------------------- 写 --------------------

W25Qxx::Status W25Qxx::WritePage(const uint8_t *buf, uint32_t addr, uint16_t len) {
    if (buf == nullptr || len == 0U) {
        return Status::ErrorParam;
    }
    if (len > kPageSize) {
        return Status::ErrorParam;
    }
    if (addr >= kFlashSize || static_cast<uint32_t>(len) > (kFlashSize - addr)) {
        return Status::ErrorParam;
    }
    // 页编程不能跨页（RM25 没查这一条，跨页会静默绕回页首覆盖数据）
    const uint32_t page_offset = addr % kPageSize;
    if (static_cast<uint32_t>(len) > (kPageSize - page_offset)) {
        return Status::ErrorParam;
    }
    if (hospi_ == nullptr) {
        return Status::ErrorTransmit;
    }

    const Status we = writeEnable();
    if (we != Status::Ok) {
        return we;
    }

    OSPI_RegularCmdTypeDef cmd;
    fillDefaultCmd(cmd);
    cmd.Instruction = kCmdPageProgramQuad;
    cmd.Address     = addr;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;   // 1-1-4
    cmd.DataMode    = HAL_OSPI_DATA_4_LINES;
    cmd.NbData      = len;

    if (HAL_OSPI_Command(hospi_, &cmd, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorTransmit;
    }
    // HAL_OSPI_Transmit 的形参不是 const，这里做一次显式转换（HAL 的既有约束）
    if (HAL_OSPI_Transmit(hospi_, const_cast<uint8_t *>(buf), kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorTransmit;
    }
    return waitMemReady(kDefaultTimeoutMs);
}

W25Qxx::Status W25Qxx::Write(const uint8_t *buf, uint32_t addr, uint32_t len) {
    if (buf == nullptr || len == 0U) {
        return Status::ErrorParam;
    }
    if (addr >= kFlashSize || len > (kFlashSize - addr)) {
        return Status::ErrorParam;
    }

    // 与 RM25 相同的拆分策略：先补满当前页，再整页写，最后写尾巴
    uint32_t current_addr = addr;
    uint32_t remaining = len;
    const uint8_t *src = buf;

    while (remaining > 0U) {
        const uint32_t page_offset = current_addr % kPageSize;
        uint32_t chunk = kPageSize - page_offset;
        if (chunk > remaining) {
            chunk = remaining;
        }

        const Status st = WritePage(src, current_addr, static_cast<uint16_t>(chunk));
        if (st != Status::Ok) {
            return st;
        }

        current_addr += chunk;
        src += chunk;
        remaining -= chunk;
    }

    return Status::Ok;
}

// -------------------- 擦 --------------------

W25Qxx::Status W25Qxx::eraseWithCmd(uint8_t instruction, uint32_t addr) {
    const Status we = writeEnable();
    if (we != Status::Ok) {
        return we;
    }

    OSPI_RegularCmdTypeDef cmd;
    fillDefaultCmd(cmd);
    cmd.Instruction = instruction;
    cmd.Address     = addr;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;

    // 与 RM25 的差异：原版把命令发送失败也报成 ErrorAutoPolling，
    // 这里用本来就定义好但从未被使用的 ErrorErase。
    if (HAL_OSPI_Command(hospi_, &cmd, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorErase;
    }
    return waitMemReady(kDefaultTimeoutMs);
}

W25Qxx::Status W25Qxx::EraseSector(uint32_t addr) {
    if (hospi_ == nullptr) {
        return Status::ErrorTransmit;
    }
    if (addr >= kFlashSize || (addr % kSectorSize) != 0U) {
        return Status::ErrorParam;   // 未对齐会静默擦错区域，直接拒绝
    }
    return eraseWithCmd(kCmdSectorErase, addr);
}

W25Qxx::Status W25Qxx::EraseBlock32K(uint32_t addr) {
    if (hospi_ == nullptr) {
        return Status::ErrorTransmit;
    }
    if (addr >= kFlashSize || (addr % kBlock32KSize) != 0U) {
        return Status::ErrorParam;
    }
    return eraseWithCmd(kCmdBlockErase32K, addr);
}

W25Qxx::Status W25Qxx::EraseBlock64K(uint32_t addr) {
    if (hospi_ == nullptr) {
        return Status::ErrorTransmit;
    }
    if (addr >= kFlashSize || (addr % kBlock64KSize) != 0U) {
        return Status::ErrorParam;
    }
    return eraseWithCmd(kCmdBlockErase64K, addr);
}

W25Qxx::Status W25Qxx::EraseChip() {
    if (hospi_ == nullptr) {
        return Status::ErrorTransmit;
    }

    const Status we = writeEnable();
    if (we != Status::Ok) {
        return we;
    }

    OSPI_RegularCmdTypeDef cmd;
    fillDefaultCmd(cmd);
    cmd.Instruction = kCmdChipErase;

    if (HAL_OSPI_Command(hospi_, &cmd, kDefaultTimeoutMs) != HAL_OK) {
        return Status::ErrorErase;
    }
    // 整片擦典型 20 s、最大 100 s
    return waitMemReady(kChipEraseTimeoutMs);
}
