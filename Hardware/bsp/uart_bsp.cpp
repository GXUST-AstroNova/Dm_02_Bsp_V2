//
// 通用 USART 故障分发 / 收养实现
//
// ---------------------------------------------------------------------------
// 设计要点
// ---------------------------------------------------------------------------
//   - 全工程唯一的 HAL 强符号定义点：HAL_UART_ErrorCallback 与
//     HAL_UARTEx_RxEventCallback 只在本文件定义（启动文件里的是弱符号）。
//   - 回调里只做短操作：查表 -> 置待处理标志/计数 -> 调一次驱动回调。
//     不在回调里做 arm()/DeInit()/Init() 之类的 HAL 状态变更。
//   - 容量固定、无动态分配；所有函数 noexcept。
//   - 句柄槽位是单字对齐写：Cortex-M7 上 32-bit 对齐存取是原子的，因此
//     中断侧查表与任务侧 Register/Unregister 之间不需要临界区（注册只在
//     初始化阶段、注销只在析构阶段发生）。
//
// ---------------------------------------------------------------------------
// 表满与 id 语义
// ---------------------------------------------------------------------------
//   Register() 取「最小的空闲槽位」作为 id，正常情况下 id 从 0 连续分配；
//   只有「先注册 2 个、再销毁其中靠前的那个」才会留下空洞。遍历请用：
//       for (i = 0; i < kMaxPorts; ++i) { if (auto *p = Get(i)) { ... } }
//   Count() 返回已注册端口数，不保证 Get(0..Count()-1) 全部非空。
//
#include "uart_bsp.h"

namespace uart_bsp {
namespace {

// ---------------------------------------------------------------------------
// 内部表
// ---------------------------------------------------------------------------

/** 已注册端口 + 错误诊断状态 */
struct PortEntry {
    Port port{};
    std::uint32_t error_count = 0U;
    bool pending = false;
};

/** 未注册 USART 的收养槽位 */
struct RawEntry {
    UART_HandleTypeDef *handle = nullptr;
    std::uint32_t error_count = 0U;
    bool pending = false;
};

PortEntry g_ports[kMaxPorts];
RawEntry g_raw[kMaxRaw];
UART_HandleTypeDef *g_reserved[kMaxReserved] = {};

std::uint32_t g_total_errors = 0U;      ///< 所有 USART 错误回调次数
std::uint32_t g_unhandled_errors = 0U;  ///< 既无端口也未收养成功（raw 表满）的次数

bool IsReserved(UART_HandleTypeDef *huart) noexcept {
    for (UART_HandleTypeDef *h : g_reserved) {
        if (h != nullptr && h == huart) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// 查表
// ---------------------------------------------------------------------------

PortEntry *find_port(UART_HandleTypeDef *huart) noexcept {
    if (huart == nullptr) {
        return nullptr;
    }
    for (std::uint8_t i = 0U; i < kMaxPorts; ++i) {
        if (g_ports[i].port.handle == huart) {
            return &g_ports[i];
        }
    }
    return nullptr;
}

RawEntry *find_raw(UART_HandleTypeDef *huart) noexcept {
    if (huart == nullptr) {
        return nullptr;
    }
    for (std::uint8_t i = 0U; i < kMaxRaw; ++i) {
        if (g_raw[i].handle == huart) {
            return &g_raw[i];
        }
    }
    return nullptr;
}

/** 收养一个未注册句柄；表满返回 nullptr。 */
RawEntry *adopt_raw(UART_HandleTypeDef *huart) noexcept {
    for (std::uint8_t i = 0U; i < kMaxRaw; ++i) {
        if (g_raw[i].handle == nullptr) {
            g_raw[i].handle = huart;
            g_raw[i].error_count = 0U;
            g_raw[i].pending = false;
            return &g_raw[i];
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// huart->Instance -> 该 USART 的 NVIC 中断号
// ---------------------------------------------------------------------------

bool irq_of(const USART_TypeDef *instance, IRQn_Type &irq) noexcept {
    if (instance == USART1) {
        irq = USART1_IRQn;
    } else if (instance == USART2) {
        irq = USART2_IRQn;
    } else if (instance == USART3) {
        irq = USART3_IRQn;
    } else if (instance == UART4) {
        irq = UART4_IRQn;
    } else if (instance == UART5) {
        irq = UART5_IRQn;
    } else if (instance == USART6) {
        irq = USART6_IRQn;
    } else if (instance == UART7) {
        irq = UART7_IRQn;
    } else if (instance == UART8) {
        irq = UART8_IRQn;
    } else if (instance == UART9) {
        irq = UART9_IRQn;
    } else if (instance == USART10) {
        irq = USART10_IRQn;
    } else {
        return false;
    }
    return true;
}

}  // namespace

// ===========================================================================
// 注册表
// ===========================================================================

int8_t Register(Port port) noexcept {
    if (port.handle == nullptr) {
        return -1;  // 参数非法
    }
    // 同一个句柄重复注册会让分发出现二义性，直接拒绝。
    for (std::uint8_t i = 0U; i < kMaxPorts; ++i) {
        if (g_ports[i].port.handle == port.handle) {
            return -1;
        }
    }
    for (std::uint8_t i = 0U; i < kMaxPorts; ++i) {
        if (g_ports[i].port.handle == nullptr) {
            g_ports[i].port = port;
            g_ports[i].port.id = i;
            g_ports[i].error_count = 0U;
            g_ports[i].pending = false;
            return static_cast<int8_t>(i);
        }
    }
    return -1;  // 表满
}

void Unregister(void *ctx) noexcept {
    for (std::uint8_t i = 0U; i < kMaxPorts; ++i) {
        if ((g_ports[i].port.handle != nullptr) && (g_ports[i].port.ctx == ctx)) {
            g_ports[i].port = Port{};
            g_ports[i].error_count = 0U;
            g_ports[i].pending = false;
        }
    }
}

// ===========================================================================
// 端口 / raw 查询
// ===========================================================================

void Reserve(UART_HandleTypeDef *huart) noexcept {
    if (huart == nullptr || IsReserved(huart)) {
        return;
    }
    for (UART_HandleTypeDef *&slot : g_reserved) {
        if (slot == nullptr) {
            slot = huart;
            return;
        }
    }
}

const Port *Get(std::uint8_t id) noexcept {
    if ((id >= kMaxPorts) || (g_ports[id].port.handle == nullptr)) {
        return nullptr;
    }
    return &g_ports[id].port;
}

std::uint8_t Count() noexcept {
    std::uint8_t n = 0U;
    for (std::uint8_t i = 0U; i < kMaxPorts; ++i) {
        if (g_ports[i].port.handle != nullptr) {
            ++n;
        }
    }
    return n;
}

bool Port_Error_Pending(std::uint8_t id) noexcept {
    return (id < kMaxPorts) && (g_ports[id].port.handle != nullptr) && g_ports[id].pending;
}

void Port_Clear_Pending(std::uint8_t id) noexcept {
    if ((id < kMaxPorts) && (g_ports[id].port.handle != nullptr)) {
        g_ports[id].pending = false;
    }
}

std::uint32_t Port_Error_Count(std::uint8_t id) noexcept {
    if ((id >= kMaxPorts) || (g_ports[id].port.handle == nullptr)) {
        return 0U;
    }
    return g_ports[id].error_count;
}

std::uint8_t Raw_Count() noexcept {
    std::uint8_t n = 0U;
    for (std::uint8_t i = 0U; i < kMaxRaw; ++i) {
        if (g_raw[i].handle != nullptr) {
            ++n;
        }
    }
    return n;
}

UART_HandleTypeDef *Raw_Handle(std::uint8_t index) noexcept {
    if (index >= kMaxRaw) {
        return nullptr;
    }
    return g_raw[index].handle;
}

bool Raw_Error_Pending(std::uint8_t index) noexcept {
    return (index < kMaxRaw) && (g_raw[index].handle != nullptr) && g_raw[index].pending;
}

void Raw_Clear_Pending(std::uint8_t index) noexcept {
    if ((index < kMaxRaw) && (g_raw[index].handle != nullptr)) {
        g_raw[index].pending = false;
    }
}

std::uint32_t Raw_Error_Count(std::uint8_t index) noexcept {
    if ((index >= kMaxRaw) || (g_raw[index].handle == nullptr)) {
        return 0U;
    }
    return g_raw[index].error_count;
}

// ===========================================================================
// 通用外设级恢复
// ===========================================================================

bool Restore_Raw(UART_HandleTypeDef *huart) noexcept {
    if ((huart == nullptr) || (huart->Instance == nullptr)) {
        return false;
    }

    // 只屏蔽该 USART 自己的 NVIC 线，绝不用全局 __disable_irq()：
    // 恢复过程要调 HAL（AbortReceive/DeInit/Init），期间不希望本线中断重入。
    IRQn_Type irq{};
    const bool have_irq = irq_of(huart->Instance, irq);
    if (have_irq) {
        HAL_NVIC_DisableIRQ(irq);
    }

    // AbortReceive 内部用阻塞式 HAL_DMA_Abort（XferAbortCallback 置空），
    // 不会再次触发 HAL_UART_ErrorCallback，所以这里没有递归风险。
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_DeInit(huart);

    // DeInit 不会清空 huart->Init / AdvancedInit，因此可以直接用原句柄配置
    // 重新 Init；HAL_UART_Init 会走 MspInit 重建 GPIO/DMA/时钟/NVIC。
    const bool ok = (HAL_UART_Init(huart) == HAL_OK);

    if (have_irq) {
        HAL_NVIC_EnableIRQ(irq);
    }
    return ok;
}

// ===========================================================================
// 全局计数
// ===========================================================================

std::uint32_t Total_Error_Count() noexcept {
    return g_total_errors;
}

std::uint32_t Unhandled_Error_Count() noexcept {
    return g_unhandled_errors;
}

// ===========================================================================
// HAL 强符号回调（全工程唯一定义点）
//
// 放在 namespace uart_bsp 内，C 链接的符号名仍是 HAL_UART_ErrorCallback /
// HAL_UARTEx_RxEventCallback，可直接覆盖 HAL 的 __weak 版本；同时又能直接
// 访问本翻译单元的文件级表。
// ===========================================================================

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    ++g_total_errors;

    if (IsReserved(huart)) {
        return;  // 调试口自行管理 TX，BSP 不做 raw 收养
    }

    PortEntry *entry = find_port(huart);
    if (entry != nullptr) {
        // 已注册端口：置待处理标志、累加计数，再交给驱动自己分类。
        entry->pending = true;
        ++entry->error_count;
        if (entry->port.on_error != nullptr) {
            entry->port.on_error(entry->port.ctx, huart);
        }
        return;
    }

    // 未注册 USART：收养到 raw 表，由故障定时器做外设级恢复。
    RawEntry *raw = find_raw(huart);
    if (raw == nullptr) {
        raw = adopt_raw(huart);
    }
    if (raw != nullptr) {
        raw->pending = true;
        ++raw->error_count;
    } else {
        ++g_unhandled_errors;  // raw 表满，本次错误无人处理
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    PortEntry *entry = find_port(huart);
    if ((entry != nullptr) && (entry->port.rx_event != nullptr)) {
        entry->port.rx_event(entry->port.ctx, huart, Size);
    }
    // 未注册的 USART 的接收事件与我方无关，直接忽略。
}

}  // namespace uart_bsp
