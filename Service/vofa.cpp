//
// Created by MrAsleep on 2026/9/20.
//
// VOFA+ JustFloat 调试输出实现：采样帧 + 单行调试文本帧。
//

#include "vofa.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "usbd_cdc.h"
#include "usbd_cdc_if.h"   // CDC_Transmit_HS
#include "dma_buffer.hpp"

extern "C" {
/** 定义在 USB_DEVICE/App/usb_device.c:44（C 文件），usb_device.h 未导出，需自行声明 */
extern USBD_HandleTypeDef hUsbDeviceHS;
}

namespace {
/** JustFloat 帧尾：+Inf 的小端表示 */
constexpr uint8_t kTail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

/** 取本工程 CDC 类的句柄。用库自己使用的那个表达式，保证看到的是同一个对象。 */
inline USBD_CDC_HandleTypeDef *cdcHandle(void) {
    return static_cast<USBD_CDC_HandleTypeDef *>(
        hUsbDeviceHS.pClassDataCmsit[hUsbDeviceHS.classId]);
}

/** 环形队列下标推进 */
inline uint8_t nextLine(uint8_t index) noexcept {
    return static_cast<uint8_t>((index + 1U) % vofa::kMaxLines);
}

/**
 * 阻塞发送超时：8N1 约 10 bit/字节，向上取整到毫秒再留 5 ms 余量。
 * 下限 10 ms（短包 1 ms 不够 HAL 轮询）、上限 100 ms（防止波特率配错时长时间卡住主循环）。
 */
inline uint32_t byteTimeoutMs(const UART_HandleTypeDef *uart, uint16_t len) noexcept {
    const uint32_t baud = (uart->Init.BaudRate != 0U) ? uart->Init.BaudRate : 115200U;
    const uint32_t need = (static_cast<uint32_t>(len) * 10U * 1000U + baud - 1U) / baud + 5U;
    if (need < 10U) {
        return 10U;
    }
    return (need > 100U) ? 100U : need;
}
}  // namespace

// ===========================================================================
// DMA 发送暂存缓冲
//
// 本工程 .data/.bss 全在 DTCM(0x20000000)（见 STM32H723XG_FLASH.ld），而 DMA1 访问不到
// DTCM，所以 DMA 发送前必须把整包拷到 D2 SRAM(0x30000000) —— 与 BMI088、Sbus 同一套做法
// （dma_buffer.hpp 的 RAM_D2_SECTION）。
//   - 放在文件作用域：GCC 不允许给非静态数据成员加 section 属性。
//   - 段是 NOLOAD：不保证上电清零，每次启动 DMA 前都会整包重写。
//   - 共享：同一时刻只允许一个 UART 后端实例使用它（本工程每块板只有一个）；s_dma_owner
//     记录当前归属，其它实例这一拍改为阻塞发送，避免把在途 DMA 的缓冲写坏。
//   - 未开 D-Cache（main.c 只开了 I-Cache），所以不需要 SCB_CleanDCache；若将来开启
//     D-Cache，必须在这里补 cache clean。
// ===========================================================================
static RAM_D2_SECTION uint8_t s_dma_tx[vofa::kMaxFrameBytes] {};
static const vofa *s_dma_owner = nullptr;

vofa::vofa(UART_HandleTypeDef *uart) noexcept : uart_(uart) {}
vofa::~vofa() noexcept {
    if (s_dma_owner == this) {
        if (uart_->gState != HAL_UART_STATE_READY) {
            (void)HAL_UART_AbortTransmit(uart_);
        }
        s_dma_owner = nullptr;   // 实例销毁时释放共享缓冲归属
    }
}

void vofa::add(float (*getter)()) noexcept {
    if (getter == nullptr) {
        return;
    }
    addReader([](const void *ctx) -> float {
        return (*reinterpret_cast<float (*)()>(const_cast<void *>(ctx)))();
    }, reinterpret_cast<const void *>(getter));
}

void vofa::addReader(Reader read, const void *ctx) noexcept {
    if (count_ >= kMaxChannels) {
        return;   // 表满，静默忽略
    }
    channels_[count_].read = read;
    channels_[count_].ctx  = ctx;
    ++count_;
}

void vofa::clear() noexcept {
    count_ = 0U;
}

// ===========================================================================
// 单行调试文本
// ===========================================================================
bool vofa::print(const char *text) noexcept {
    if (text == nullptr) {
        return false;
    }

    // ---- 1) 临界区之外组帧：USB 栈会直接引用这个槽，格式必须一次成形 ----
    //    文本帧 = [可见文本] + 换行符 + [可选 1 个空格] + [4 字节帧尾]
    char buf[kLineBytes];
    uint8_t n = 0U;
    while ((n < kLineTextMax) && (text[n] != '\0') &&
           (text[n] != '\n') && (text[n] != '\r')) {
        buf[n] = text[n];
        ++n;
    }
    if (n == 0U) {
        return false;   // 空行 / nullptr / 纯换行不入队
    }
    buf[n++] = '\n';

    // 让文本块长度不是 4 的倍数：官方文档说上位机要「判断它是采样数据帧、图片前导帧还是
    // 其他数据」但未公开判据；若按长度判，长度恰好合法的纯文本会被误判成采样帧。
    if ((n % 4U) == 0U) {
        buf[n++] = ' ';
    }

    std::memcpy(&buf[n], kTail, sizeof(kTail));
    n = static_cast<uint8_t>(n + sizeof(kTail));   // 最大 55+1+1+4 = 61 <= kLineBytes

    // ---- 2) 临界区：占用一个空槽并拷贝 ----
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();   // 生产端可能来自任意 ISR，必须与 send() 的取用互斥

    bool ok = false;
    const uint8_t next = nextLine(line_tail_);
    if (next != line_head_) {              // 队列未满
        std::memcpy(lines_[line_tail_], buf, n);
        line_bytes_[line_tail_] = n;
        line_tail_ = next;                 // 最后发布索引：数据先于索引对消费端可见
        ok = true;
    } else {
        ++dropped_texts_;                  // 队列满：丢弃新行，保住最早的现场
    }

    __set_PRIMASK(primask);
    return ok;
}

bool vofa::printf(const char *fmt, ...) noexcept {
    if (fmt == nullptr) {
        return false;
    }
    char buf[kLineTextMax + 1U];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return false;
    }
    return print(buf);   // 截断 / 换行规整 / 补帧尾统一交给 print()
}

bool vofa::Is_Ready() const noexcept {
    if (uart_ != nullptr) {
        return uart_->gState == HAL_UART_STATE_READY;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const auto *cdc = cdcHandle();
    const bool ready = hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED && cdc && cdc->TxState == 0U;
    __set_PRIMASK(primask);
    return ready;
}

uint8_t vofa::pendingTexts() const noexcept {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t used = static_cast<uint8_t>((line_tail_ + kMaxLines - line_head_) % kMaxLines);
    // 在途那一行已提交给 USB，对调用方而言不算「待发送」；
    // 读 line_inflight_ 必须与读索引同处一个临界区，否则两个值可能来自不同时刻。
    const bool inflight = line_inflight_;
    __set_PRIMASK(primask);
    return static_cast<uint8_t>(used - (inflight ? 1U : 0U));
}

bool vofa::send() noexcept {
    // ---- 0) 串口后端的错误恢复、卡死看门狗与共享缓冲归属释放（都只被 send() 读写）----
    if (uart_ != nullptr) {
        if (uart_->gState == HAL_UART_STATE_ERROR) {
            (void)HAL_UART_AbortTransmit(uart_);
            if (s_dma_owner == this) s_dma_owner = nullptr;
            line_inflight_ = false;
            ++tx_errors_;
            return false;
        }

        // DMA/中断没能把 gState 收回 READY 时 VOFA 会静默断流；abort 后下一拍自动重发，
        // 比一直等下去更容易在台架上发现问题（DMA 出错会被 HAL 记进 ErrorCode）。
        if (uart_->gState == HAL_UART_STATE_BUSY_TX) {
            if (++busy_ticks_ >= kBusyAbortTicks) {
                (void)HAL_UART_AbortTransmit(uart_);
                if (s_dma_owner == this) s_dma_owner = nullptr;
                line_inflight_ = false;
                ++tx_errors_;
                busy_ticks_ = 0U;
            }
        } else {
            busy_ticks_ = 0U;
            // 上一包 DMA 已完成：释放共享暂存缓冲（空闲时隙也要释放）。
            if (s_dma_owner == this) {
                s_dma_owner = nullptr;
            }
        }
    }

    // ---- 1) 推进分频：只被 send() 读写，无需临界区 ----
    if (++tick_ >= kFrameDiv) {
        tick_ = 0U;
        if (frame_pending_) {
            ++dropped_;   // 上一帧等满一个分频周期还没发出去，被这一帧顶掉
        }
        frame_pending_ = count_ != 0U;
    }
    const bool frame_due = frame_pending_;

    // ---- 2) 快路径：本时隙既没有待发/在途文本，也没轮到采样帧----
    if (!frame_due && !line_inflight_ && (line_head_ == line_tail_)) {
        return true;
    }

    // ---- 3) 采样：临界区之外（会调用用户的取值函数，可能不短）----
    //    文本可能挤掉本时隙的采样帧，但这里仍照常采样：最坏开销与「每 kFrameDiv 个节拍
    //    采一次」相同，换取「临界区内绝不可能少采」的可证明性。
    float sample[kMaxChannels];
    if (frame_due && (count_ != 0U)) {
        for (uint8_t i = 0U; i < count_; ++i) {
            sample[i] = channels_[i].read(channels_[i].ctx);
        }
    }

    // ---- 4) 临界区：状态检查 + 出队 + 组帧 + 提交/登记 ----
    //    阻塞发送不在这里做：HAL_UART_Transmit 用 HAL_GetTick 计时，需要 HAL 时基中断 推进，
    //    而这里关中断。所以这里只登记要阻塞发出的包，出临界区再发。
    const uint8_t *blocking_data = nullptr;
    uint16_t       blocking_len  = 0U;
    bool           blocking_text = false;   ///< true=文本行（发完就出队），false=采样帧

    bool ok         = false;
    bool sent_frame = false;
    bool sent_text  = false;
    bool used_dma   = false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool uart_mode = (uart_ != nullptr);
    USBD_CDC_HandleTypeDef *hcdc = uart_mode ? nullptr : cdcHandle();
    const bool tx_ready = uart_mode
                              ? (uart_->gState == HAL_UART_STATE_READY)
                              : ((hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) &&
                                 (hcdc != nullptr) && (hcdc->TxState == 0U));
    // 没有 TX DMA（hdmatx == nullptr）就没有 DMA 可提交，这一拍改为阻塞发送。
    const bool use_dma = uart_mode && (uart_->hdmatx != nullptr);

    if (tx_ready) {
        // 4a) 出队：上一时隙提交的那一包已传完（gState == READY / TxState == 0）
        if (line_inflight_) {
            line_head_     = nextLine(line_head_);
            line_inflight_ = false;
        }

        if (line_head_ != line_tail_) {
            // 4b) 文本优先：本时隙发文本帧，采样帧由 frame_pending_ 顺延到下一时隙
            const uint8_t index = line_head_;
            const uint8_t len   = line_bytes_[index];
            const uint8_t *src  = reinterpret_cast<const uint8_t *>(lines_[index]);
            if (!uart_mode) {
                if (CDC_Transmit_HS(reinterpret_cast<uint8_t *>(lines_[index]), len) ==
                    static_cast<uint8_t>(USBD_OK)) {
                    line_inflight_ = true;
                    ok             = true;
                    sent_text      = true;
                }
            } else if (use_dma && (s_dma_owner == nullptr)) {
                // 整包拷到 RAM_D2：对象缓冲区在 DTCM，DMA 读不到
                std::memcpy(s_dma_tx, src, len);
                if (HAL_UART_Transmit_DMA(uart_, s_dma_tx, len) == HAL_OK) {
                    s_dma_owner    = this;
                    line_inflight_ = true;
                    ok             = true;
                    sent_text      = true;
                    used_dma       = true;
                } else {
                    // DMA 起不来（流忙/配置不全）：本拍回退阻塞，不丢这一行
                    blocking_data = src;
                    blocking_len  = len;
                    blocking_text = true;
                    line_inflight_ = true;
                }
            } else {
                // 无 DMA，或共享暂存缓冲正被另一个实例的 DMA 使用
                blocking_data = src;
                blocking_len  = len;
                blocking_text = true;
                line_inflight_ = true;
            }
        } else if (frame_due && (count_ != 0U)) {
            // 4c) 组采样帧
            uint8_t *p = frame_;
            for (uint8_t i = 0U; i < count_; ++i) {
                std::memcpy(p, &sample[i], sizeof(float));   // 小端，直接取内存字节序
                p += sizeof(float);
            }
            std::memcpy(p, kTail, sizeof(kTail));

            const uint16_t len = static_cast<uint16_t>(count_ * 4U + 4U);
            if (!uart_mode) {
                if (CDC_Transmit_HS(frame_, len) == static_cast<uint8_t>(USBD_OK)) {
                    frame_pending_ = false;
                    ok             = true;
                    sent_frame     = true;
                }
            } else if (use_dma && (s_dma_owner == nullptr)) {
                std::memcpy(s_dma_tx, frame_, len);
                if (HAL_UART_Transmit_DMA(uart_, s_dma_tx, len) == HAL_OK) {
                    s_dma_owner    = this;
                    frame_pending_ = false;
                    ok             = true;
                    sent_frame     = true;
                    used_dma       = true;
                } else {
                    blocking_data = frame_;
                    blocking_len  = len;
                    blocking_text = false;
                }
            } else {
                blocking_data = frame_;
                blocking_len  = len;
                blocking_text = false;
            }
        } else {
            // 4d) 没有可发的包：要么只完成了 4a) 的出队，要么本时隙没轮到采样帧。
            //     顺手清掉到期的采样帧标记，避免「无通道」时 frame_pending_ 永久挂住快路径。
            frame_pending_ = false;
            ok             = true;
        }
    }
    // tx_ready == false：本时隙什么都发不出去，frame_pending_ 保持，下一时隙重试

    __set_PRIMASK(primask);   // 恢复到调用前的状态（不是无条件 __enable_irq）

    // ---- 5) 临界区之外：阻塞发送（HAL_GetTick 需要 HAL 时基中断，绝不能在关中断时调用）----
    if (blocking_data != nullptr) {
        const bool sent = (HAL_UART_Transmit(uart_, blocking_data, blocking_len,
                                             byteTimeoutMs(uart_, blocking_len)) == HAL_OK);
        const uint32_t primask2 = __get_PRIMASK();
        __disable_irq();
        if (sent) {
            if (blocking_text) {
                line_head_ = nextLine(line_head_);   // 文本行：出队
                sent_text  = true;
            } else {
                frame_pending_ = false;              // 采样帧：清到期标记
                sent_frame     = true;
            }
            ok = true;
            ++blocking_packets_;
        } else {
            ok = false;      // 超时/失败：数据仍留在队列，下一时隙重试
            ++tx_errors_;
        }
        line_inflight_ = false;
        __set_PRIMASK(primask2);
    }

    if (used_dma)   { ++dma_packets_; }
    if (sent_frame) { ++sent_; }
    if (sent_text)  { ++sent_texts_; }
    return ok;
}
