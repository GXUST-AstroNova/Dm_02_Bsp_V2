//
// Created by MrAsleep on 2026/9/20.
//

#ifndef DM_02_BSP_VOFA_H
#define DM_02_BSP_VOFA_H

#include "usart.h"      // UART_HandleTypeDef；USB 细节只在 .cpp 中

/**
 * VOFA+ JustFloat 调试输出：采样帧 + 单行调试文本帧。
 *
 * 用法：
 *   static vofa dbg;                                    // 默认 USB CDC
 *   static vofa dbg(&huart1);                           // 串口调试（构造注入）
 *   dbg.add(&some_float_var);                           // 任意标量变量指针
 *   dbg.add(+[]() -> float { return m.Get_Rpm(); });    // 无参取值函数（无捕获 lambda 亦可）
 *   dbg.print("imu ready");                             // [任意上下文] 入队一行调试文本
 *   dbg.printf("rpm=%d err=0x%02X", rpm, err);          // [任意上下文] 格式化入队（不支持 %f）
 *   ...主循环 1 ms 门限: dbg.send();                     // 业务层 Application_Loop 调用
 *
 * 后端：构造参数 uart=nullptr 使用 USB CDC；uart!=nullptr 使用该串口：
 *        uart->hdmatx != nullptr（CubeMX 配了 TX DMA）→ HAL_UART_Transmit_DMA，非阻塞；
 *        uart->hdmatx == nullptr（没有 DMA 配置）→ 回退 HAL_UART_Transmit 阻塞发送。
 *      串口需独占，且不能是 SBUS 的 UART5。
 *
 * 帧格式（两种帧共用同一个帧尾 00 00 80 7F）：
 *   采样帧：N 个小端 IEEE754 float + 帧尾，总长 N*4+4；由 send() 内部 kFrameDiv 分频产生 500 Hz。
 *   文本帧：可见文本 + '\n' + 空格填充 + 帧尾，总长 <= 63 字节（单 USB 包，不触发 ZLP）。
 *
 * 为什么文本也要带帧尾：VOFA+ 官方 JustFloat 文档明确「遇到 00 00 80 7F 才会开启一帧的解析……
 * 没有开启数据解析，就不会打印文本」，且持续不出现帧尾会让上位机「缓冲区爆满，软件卡死」。
 * 见 https://www.vofa.plus/docs/learning/dataengines/justfloat/ 的「文本打印」一节。
 *
 * 调度：文本行优先占用 1 ms 时隙；若某个时隙刚好轮到采样帧，该帧顺延到下一个时隙补发，
 * 所以插文本不会丢采样帧（只有文本积压超过 kFrameDiv 个时隙时才会顶掉一帧，计入 droppedFrames()）。
 *
 * send() 在临界区（保存/恢复 PRIMASK）内完成「状态检查 + 出队 + 组帧 + 提交/登记」，保证对
 * 不可重入的 USB 设备栈/UART 句柄的访问是原子的；采样（会调用用户函数）与阻塞回退的实际
 * 发送都放在临界区之外（后者必须让 HAL 时基中断 能进中断）。
 * send() 由 Application_Loop 的 1 ms 门限调用，不在 TIM7 中断里执行。
 *
 * @warning add() 保存的是指针，被指向的变量必须比本对象活得久（静态/全局/命名空间作用域）。
 * @warning USB CDC 发送完成前会持有对象内的缓冲区，USB 调试实例应覆盖整个传输生命周期。
 * @warning print() / printf() 会立即拷贝文本，调用方不必保持字符串存活。
 *
 * @note USB OTG HS 为从模式（usbd_conf.c: dma_enable = DISABLE），缓冲区由 CPU 搬运；
 *       UART 后端也由 vofa 持有缓冲区。本工程 .data/.bss 在 DTCM(0x20000000)，DMA1 读不到，
 *       所以 DMA 发送前必须把整包拷到 RAM_D2_SECTION 暂存（见 vofa.cpp 的 s_dma_tx）。
 *
 * @warning 阻塞回退只能在主循环上下文（Application_Loop 的 1 ms 门限）调用：HAL_UART_Transmit 用
 *          HAL_GetTick 计时，需要 HAL 时基中断 推进，绝不能在 ISR 里或在关中断状态下发送。
 * @warning 阻塞回退会占满当前时隙：921600 下 6 通道整帧 28 B ≈ 0.30 ms、16 通道 68 B ≈ 0.74 ms
 *          （若把调试口降到 115200，则分别为 2.4 ms / 5.9 ms，会明显吃掉 1 ms 节拍）。
 *          只有确实没有 TX DMA 的端口/移植场景才应走到这条路径。
 */
class vofa {
public:
    /** 通道数上限：JustFloat 帧 = N*4 + 4 字节，16 通道即 68 字节 */
    static constexpr uint8_t kMaxChannels = 16U;

    /** 采样帧字节上限 = kMaxChannels*4 + 4（68），也是 DMA 暂存缓冲的长度 */
    static constexpr uint8_t kMaxFrameBytes = static_cast<uint8_t>(kMaxChannels * 4U + 4U);

    /** 采样帧分频：send() 每 1 ms 调用一次 -> 采样帧 500 Hz */
    static constexpr uint8_t kFrameDiv = 2U;

    /** 文本行队列深度（满时丢弃新行，不覆盖旧行） */
    static constexpr uint8_t kMaxLines = 8U;

    /** 单行可见文本上限（不含自动补的 '\n' 与帧尾） */
    static constexpr uint8_t kLineTextMax = 55U;

    /** 文本帧单包字节上限 = 文本 + '\n' + 填充 + 4 字节帧尾（<=63 保证单包发出，不触发 ZLP） */
    static constexpr uint8_t kLineBytes = 63U;

    /** UART 卡死看门狗：gState 连续这么多个 1 ms 节拍停在 BUSY_TX 就 abort（DMA 未完成） */
    static constexpr uint16_t kBusyAbortTicks = 100U;

    /** uart == nullptr 使用 USB CDC；否则使用该 UART 发送。 */
    explicit vofa(UART_HandleTypeDef *uart = nullptr) noexcept;
    ~vofa() noexcept;
    vofa(const vofa &) = delete;
    vofa &operator=(const vofa &) = delete;

    /**
     * 登记一个标量变量（float / int16 / uint16 / int32 / uint32 / double …）。
     * 采样时 static_cast<float> 转换；nullptr 被忽略；超上限的登记被静默忽略。
     */
    template <typename T>
    void add(const T *var) noexcept {
        if (var == nullptr) {
            return;
        }
        addReader([](const void *ctx) -> float {
            return static_cast<float>(*static_cast<const T *>(ctx));
        }, static_cast<const void *>(var));
    }

    /** 登记一个无参取值函数；nullptr 被忽略。 */
    void add(float (*getter)()) noexcept;

    /** 清空全部通道（不释放任何资源，也不影响文本队列）。 */
    void clear() noexcept;

    uint8_t  channelCount()  const noexcept { return count_; }
    uint32_t sentFrames()    const noexcept { return sent_; }
    uint32_t droppedFrames() const noexcept { return dropped_; }

    // ======================= 单行调试文本 =======================

    /**
     * 入队一行调试文本，先入先出，每行只发送一次。
     *
     * 可在任意上下文调用（主循环 / 任意优先级的 ISR）；内部只在拷贝入槽时短暂关中断。
     * 文本会被规范化：截断到 kLineTextMax 字节，去掉调用者自带的 \r \n，统一补一个 '\n'。
     * nullptr / 空文本 / 纯换行不入队并返回 false。
     * 队列满时丢弃本行、累加 droppedTexts() 并返回 false。
     */
    bool print(const char *text) noexcept;

    /**
     * printf 风格格式化后入队，只发送第一行（格式化结果遇 '\n' 即截断）。
     *
     * @note 本工程用 --specs=nano.specs 且未加 -Wl,-u,_printf_float，%f 不可用；
     *       整型 %d/%u/%x 与 %s/%c 正常。浮点调试请走 add() 采样通道。
     */
    bool printf(const char *fmt, ...) noexcept __attribute__((format(printf, 2, 3)));

    /**
     * 当前链路是否可发送：USB 已枚举/CDC 空闲，或 UART gState == READY。
     * 只有为 true 时 send() 才可能真正把包发出去。
     */
    bool Is_Ready() const noexcept;

    /** 调试链路句柄；nullptr 表示 USB CDC。 */
    UART_HandleTypeDef *Uart() const noexcept { return uart_; }

    /**
     * 串口后端是否走 DMA 发送：uart != nullptr 且该 UART 已链接 TX DMA（hdmatx）。
     * 为 false 的串口后端回退阻塞发送；USB CDC 后端的返回值为 false。
     */
    bool UsesDmaTx() const noexcept {
        return (uart_ != nullptr) && (uart_->hdmatx != nullptr);
    }

    /** 走 DMA 的包数（采样帧 + 文本帧） */
    uint32_t dmaPackets()      const noexcept { return dma_packets_; }
    /** 走阻塞回退的包数（无 DMA、DMA 起不来或暂存缓冲被其它实例占用） */
    uint32_t blockingPackets() const noexcept { return blocking_packets_; }
    /** 阻塞发送失败（含超时）与卡死 abort 的累计次数；失败时数据仍留在队列重试 */
    uint32_t txErrors()        const noexcept { return tx_errors_; }

    /** 待发送的文本行数（不含正在 USB 传输中的那一行） */
    uint8_t  pendingTexts() const noexcept;
    uint32_t sentTexts()    const noexcept { return sent_texts_; }
    uint32_t droppedTexts() const noexcept { return dropped_texts_; }

    /**
     * 每个 1 ms 节拍调用一次（本工程由 Application_Loop 的 1 ms 门限驱动）：
     *   1) 串口后端先做错误恢复与卡死看门狗（BUSY_TX 连续 kBusyAbortTicks 个节拍 -> abort）；
     *   2) 推进 kFrameDiv 分频，判定本时隙是否轮到采样帧；
     *   3) 回收上一时隙已提交、且传输已完成的文本行（出队，"下一轮不再发送"）；
     *   4) 文本队列非空 -> 本时隙发队首文本帧（采样帧顺延 1 ms 补发）；
     *   5) 否则若轮到采样帧 -> 采样 + 组帧 + 发送。
     *
     * USB CDC 与 UART+DMA 路径非阻塞（提交后下一节拍查 gState 判定完成）；
     * UART 无 DMA 时在本函数内阻塞发出（见 @warning）。返回值：本时隙无包可发（含空闲时隙）
     * 返回 true；有待发数据却发不出去（USB 未枚举 / 上一包仍在发送 / 阻塞发送失败）返回 false。
     */
    bool send() noexcept;

private:
    using Reader = float (*)(const void *ctx);

    struct Channel {
        Reader      read;
        const void *ctx;
    };

    void addReader(Reader read, const void *ctx) noexcept;

    Channel  channels_[kMaxChannels] {};
    uint8_t  count_   = 0U;
    uint8_t  frame_[kMaxFrameBytes] {};  ///< 采样帧缓冲（USB 栈会持有其指针）
    uint32_t sent_    = 0U;
    uint32_t dropped_ = 0U;

    // ---------------- 单行调试文本 ----------------
    char             lines_[kMaxLines][kLineBytes] {};  ///< 文本行槽（USB 栈会持有其指针）
    uint8_t          line_bytes_[kMaxLines] {};         ///< 每行实际字节数（含帧尾）
    volatile uint8_t line_head_ = 0U;                   ///< 消费端（send）推进：队首/在途行
    volatile uint8_t line_tail_ = 0U;                   ///< 生产端（print）推进：下一个空槽
    bool             line_inflight_ = false;            ///< 队首行已提交给 USB，等传输完成
    uint8_t          tick_          = 0U;               ///< 采样帧分频计数 0..kFrameDiv-1
    bool             frame_pending_ = false;            ///< 采样帧到期但被文本行挤掉，待补发
    uint32_t         sent_texts_    = 0U;
    uint32_t         dropped_texts_ = 0U;
    uint32_t         dma_packets_      = 0U;  ///< 走 DMA 提交的包数
    uint32_t         blocking_packets_ = 0U;  ///< 走阻塞回退的包数
    uint32_t         tx_errors_        = 0U;  ///< 阻塞失败 / 卡死 abort 次数
    uint16_t         busy_ticks_       = 0U;  ///< UART BUSY_TX 连续节拍数（看门狗）
    UART_HandleTypeDef *uart_       = nullptr;
};

#endif //DM_02_BSP_VOFA_H
