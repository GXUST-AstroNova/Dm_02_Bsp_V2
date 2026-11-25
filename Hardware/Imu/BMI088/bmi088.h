//
// Bosch BMI088 六轴 IMU（SPI 模式）
//
// 本工程接线（Core/Src/spi.c、Core/Src/gpio.c 已配好）：
//   SPI2 (hspi2)：Master / 2Lines / 8bit / CPOL=High / CPHA=2Edge（SPI Mode 3）/ NSS=Soft
//                 BaudRatePrescaler = 64，SPI123 时钟 96 MHz(PLL3P) -> **实际 1.5 MHz**
//                 （.ioc 里的 CalculateBaudRate 写 3.0 MBits/s 是 ClockRecalculation 的
//                  失真值，以 spi.c 的 Prescaler 为准；BMI088 SPI 上限 10 MHz。）
//   DMA：SPI2_RX = DMA1_Stream1 (periph->mem) / SPI2_TX = DMA1_Stream2 (mem->periph)，
//        均为 DMA_NORMAL / MINC / 字节对齐 / 优先级 HIGH（见 Core/Src/spi.c 的 SPI2_MspInit）
//   ACC_CS  = PC0（推挽输出，高 = 未选中）   —— 加速度计事务
//   GYRO_CS = PC3（推挽输出，高 = 未选中）   —— 陀螺仪事务
//   ACC_INT = PE10 / GYRO_INT = PE12（本轮未使用，见下）
//
// ⚠【DMA 缓冲区必须在 D2 SRAM】STM32H7 的通用 DMA 访问不到 DTCM(0x20000000)，
//   而本工程栈和 .bss 都在 DTCMRAM 里（见 STM32H723XG_FLASH.ld 的 _estack）。
//   所以 tx_/rx_ 一律用 RAM_D2_SECTION 落到 0x30000000，**绝不能改成栈上局部数组**。
//
// BMI088 是一颗芯片、两个 SPI 从机（加速度计与陀螺各有独立片选与寄存器空间）。
//
// 寄存器与协议要点（依据工作区手册 C194919_..._BMI088_规格书_WJ1844327.PDF，
// 即 Bosch BST-BMI088-DS000-19 Rev 1.9；下面引用的页码都是该 PDF 的页码）：
//   - 【加速度计特有】p.13 §3 / p.45 §6.1：上电后加速度计处于 I2C 模式，必须检测到
//     CSB1 的【上升沿】才切到 SPI；【切过去之后的第一次读返回不可预测的 dummy 值，
//     必须丢弃】（手册推荐做法：对 ACC_CHIP_ID 做一次 dummy 读）。
//     陀螺仪的接口由 PS 引脚电平决定（本板 PS 接 GND = SPI），没有这个问题。
//   - 写：CS 低 -> [reg & 0x7F, value] -> CS 高                      （两者都无 dummy）
//   - 读加速度计：CS 低 -> [reg | 0x80, dummy, data...] -> CS 高      （多 1 个 dummy 字节）
//   - 读陀螺仪：  CS 低 -> [reg | 0x80, data...] -> CS 高            （无 dummy，p.45 §6.1）
//   - 数据寄存器小端（低字节在低地址）
//   - chip id：加速度计 0x1E、陀螺 0x0F；软复位命令 0xB6
//   - 加速度计数据是 16 位，**不需要 >> 4**
//   - 温度（手册 p.28 §5.3.7）：TEMP_MSB(0x22) 是高 8 位、TEMP_LSB(0x23) 的高 3 位是低 3 位；
//     Temp_uint11 = TEMP_MSB*8 + TEMP_LSB/32；11 位是【2 的补码】，>1023 要减 2048；
//     T = Temp_int11 * 0.125 + 23.0 °C。TEMP_MSB == 0x80 表示数据无效。
//
// 量程与灵敏度：
//   加速度计 ACC_RANGE(0x41)：0x00=±3g@10920 / 0x01=±6g@5460 / 0x02=±12g@2730 / 0x03=±24g@1365 (LSB/g)
//   陀螺仪   GYRO_RANGE(0x0F)：0x00=±2000@16.384 / 0x01=±1000 / 0x02=±500 / 0x03=±250 / 0x04=±125  (LSB/(°/s))
//   注意陀螺的量程码顺序与加速度计相反。默认取 ±6g 与 ±2000°/s。
//
// 输出单位：加速度 m/s²、角速度 °/s（同一快照可读取 rad/s）。
//
// 用法（异步：Start() 在节拍中断里发起，Process() 在主循环里推进）：
//   static Bmi088 imu(&hspi2);
//   if (imu.Init() != 0) { /* chip id 不对或 SPI 不通 */ }
//   // TIM13 的 1 kHz 节拍中断里：
//   imu.Start();
//   // 主循环里：
//   if (imu.Process() > 0) { /* 新数据已进缓存 */ }
//   const ImuData &d = imu.Get_Data();
//
// 一拍一次采样由**两个** DMA 事务组成（片选独立，必须分两次）：
//   accel 事务：0x12 起连续读 20 字节 —— 加速度计 0x12..0x17 + 温度 0x22..0x23
//              （中间 0x18..0x21 是保留字节，一并读回后丢弃）。
//              一次拿两样，省掉一个独立的温度事务。
//   gyro  事务：0x02 起连续读 7 字节 —— 陀螺仪 0x02..0x07（无 dummy）。
//   1.5 MHz 下 27 字节约 144 us（若把 SPI2 改到 3 MHz 则约 72 us）。
//
// 上板验收清单：
//   imu.accelChipId() == 0x1E、imu.gyroChipId() == 0x0F；
//   Get_UpdateCount() 每秒约增加 1000（1 kHz 未断流）、Get_RefreshErrors() 恒为 0；
//   静止时加速度模约为 9.81 m/s²，角速度接近 0 °/s，温度接近室温。
//
// 关于 data-ready 中断（本轮未实现，按需求留作扩展）：
//   若要改成中断驱动，需要额外写 5 个寄存器 ——
//     ACC_INT1_IO_CTRL(0x53)=0x0A（输出/推挽/高有效）
//     ACC_INT_MAP_DATA(0x58)=0x04（drdy 映射到 INT1）
//     GYRO_INT_CTRL(0x15)=0x80（使能新数据中断）
//     GYRO_INT3_INT4_IO_CONF(0x16)=0x01（推挽/高有效）
//     GYRO_INT3_INT4_IO_MAP(0x18)=0x01（drdy 映射到 INT3）
//   STM32 侧 PE10/PE12 已是 EXTI 上升沿、EXTI15_10_IRQn 已使能，只差一个
//   HAL_GPIO_EXTI_Callback() 转发进来；另需注意"ISR 写缓存 / 主循环读缓存"的撕裂问题
//   （现在已是异步模型，DMA 完成与缓存提交都在 Process() 里做，天然规避）。
//

#ifndef DM_02_BSP_BMI088_H
#define DM_02_BSP_BMI088_H

#include <cstdint>

#include "imu.hpp"
#include "main.h"   // GPIO_TypeDef / ACC_CS_Pin / GYRO_CS_Pin
#include "spi.h"    // SPI_HandleTypeDef

class Bmi088 final : public Imu {
public:
    /** 加速度计量程码（直接写进 ACC_RANGE 寄存器） */
    enum class AccelRange : uint8_t {
        G3  = 0x00,   ///< ±3g   @ 10920 LSB/g
        G6  = 0x01,   ///< ±6g   @  5460 LSB/g
        G12 = 0x02,   ///< ±12g  @  2730 LSB/g
        G24 = 0x03,   ///< ±24g  @  1365 LSB/g
    };

    /** 陀螺仪量程码（直接写进 GYRO_RANGE 寄存器；注意码值顺序与加速度计相反） */
    enum class GyroRange : uint8_t {
        Dps2000 = 0x00,   ///< ±2000°/s @ 16.384 LSB/(°/s)
        Dps1000 = 0x01,   ///< ±1000°/s @ 32.768
        Dps500  = 0x02,   ///< ±500°/s  @ 65.536
        Dps250  = 0x03,   ///< ±250°/s  @ 131.072
        Dps125  = 0x04,   ///< ±125°/s  @ 262.144
    };

    struct Config {
        AccelRange accel_range = AccelRange::G6;      ///< 默认 ±6g（5460 LSB/g）
        GyroRange  gyro_range  = GyroRange::Dps2000;  ///< 默认 ±2000°/s（抗高转速削顶）

        uint8_t accel_odr = 0x08;   ///< ACC_CONF[3:0]，0x08 = 100 Hz
        ///
        /// GYRO_BANDWIDTH(0x10)[2:0]（手册 p.39 §5.5.6）。bit7 是只读位、恒为 1，没有模式含义：
        ///   0x00 = 2000 Hz ODR / 532 Hz 滤波    0x01 = 2000 Hz / 230 Hz
        ///   0x02 = 1000 Hz ODR / 116 Hz 滤波    0x03 =  400 Hz /  47 Hz
        ///   0x04 =  200 Hz /  23 Hz             0x05 =  100 Hz /  12 Hz
        ///   0x06 =  200 Hz /  64 Hz             0x07 =  100 Hz /  32 Hz
        /// 取 0x02：与 Refresh() 的 1 kHz 节拍匹配（0x01 是 2000 Hz ODR，噪声更大）。
        uint8_t gyro_odr  = 0x02;

        GPIO_TypeDef *accel_cs_port = ACC_CS_GPIO_Port;
        uint16_t      accel_cs_pin  = ACC_CS_Pin;
        GPIO_TypeDef *gyro_cs_port  = GYRO_CS_GPIO_Port;
        uint16_t      gyro_cs_pin   = GYRO_CS_Pin;

        uint32_t spi_timeout_ms = 10U;

        /**
         * 单次 DMA 事务的超时（单位：HAL tick，即 ms）。
         * 1.5 MHz 下 20+7 字节只要约 144 us，正常情况下根本碰不到这个门限 ——
         * 它的唯一用途是硬件故障（MISO 短路、片选卡死）时让状态机自愈，
         * 避免 xfer_ 永远停在 Wait 而让采样彻底停摆。
         *
         * 取 5 而不是更小，是因为 HAL_GetTick() 由 TIM23 的 1 kHz 时基驱动，
         * 本身有最多 1 个 tick 的粒度误差；留足余量才不会把正常传输误判成超时。
         */
        uint32_t xfer_timeout_ms = 5U;
    };

    // -------------------- 构造 / 析构 --------------------

    /**
     * @param hspi 该 IMU 所在的 SPI 总线，使用默认 Config
     */
    explicit Bmi088(SPI_HandleTypeDef *hspi) noexcept;

    /** 指定 Config 构造 */
    Bmi088(SPI_HandleTypeDef *hspi, const Config &cfg) noexcept;

    Bmi088(const Bmi088 &) = delete;
    Bmi088 &operator=(const Bmi088 &) = delete;

    ~Bmi088() override;

    // -------------------- 基类接口 --------------------

    int8_t Init() override;
    int8_t Start() override;
    int8_t Process() override;

    // -------------------- SPI DMA 中断入口（由 .cpp 里的 HAL_SPI_*Callback 转发） --------------------

    /**
     * SPI DMA 事务完成（SPI2 的 TxRxCplt）。
     *
     * 只做「释放片选 + 立刻发起下一笔事务」的链式推进：加速度计完成就发陀螺仪，
     * 两笔串联后整帧在**同一拍（1 ms）内**读完，不再跨拍等下一次。
     * 解析与提交一律留给 Process()（TIM13 上下文），避免在更高优先级中断里改 data_。
     */
    void On_Spi_Complete(SPI_HandleTypeDef *hspi) noexcept;

    /** SPI 错误（SPI2 的 ErrorCallback）：释放片选、记录失败、回到 Idle 等下一拍重发。 */
    void On_Spi_Error(SPI_HandleTypeDef *hspi) noexcept;

    // -------------------- 诊断接口 --------------------

    /** 初始化时读到的加速度计 chip id，期望 0x1E */
    uint8_t accelChipId() const noexcept { return accel_chip_id_; }

    /** 初始化时读到的陀螺仪 chip id，期望 0x0F */
    uint8_t gyroChipId() const noexcept { return gyro_chip_id_; }

    /**
     * 初始化后读到的 ACC_ERR_REG(0x02)（手册 p.26 §5.3.2）：
     *   bit0  = fatal_err（芯片未处于可工作状态，只能靠 POR/软复位清除）
     *   [4:2] = error_code（0x01 = ACC_CONF 里有非法配置）
     * 0x00 = 无错误。纯诊断，不参与 Init() 的成功判定。
     */
    uint8_t accelErrorReg() const noexcept { return accel_err_reg_; }

    /**
     * SPI2 的收发 DMA 通道是否已在 Core/Src/spi.c 的 SPI2_MspInit 里挂上
     * （HAL_SPI_MspInit 里的 __HAL_LINKDMA 是否生效）。
     * Init() 和 Start() 都检查收发 DMA 通道。
     */
    bool dmaReady() const noexcept;

    /** 当前生效的加速度计灵敏度（LSB/g），由 accel_range 决定 */
    float accelSensitivityLsbPerG() const noexcept { return accel_lsb_per_g_; }

    /** 当前生效的陀螺仪灵敏度（LSB/(°/s)），由 gyro_range 决定 */
    float gyroSensitivityLsbPerDps() const noexcept { return gyro_lsb_per_dps_; }

    // -------------------- DMA 事务长度常量 --------------------
    // 公开：bmi088.cpp 里文件作用域的 DMA 缓冲要用它们定长（GCC 不允许给
    // 非静态数据成员加 section 属性，所以缓冲只能放文件作用域，长度常量就必须可见）。

    /**
     * 加速度计 DMA 事务长度：地址 + dummy + 0x12..0x23（含中间保留字节）。
     * 0x12..0x17 = 加速度 6 字节；0x18..0x21 保留 10 字节；0x22..0x23 = 温度 2 字节。
     */
    static constexpr uint16_t kAccTotal = 20U;

    /** 陀螺仪 DMA 事务长度：地址 + 0x02..0x07 */
    static constexpr uint16_t kGyroTotal = 7U;

private:
    // -------------------- 片选 --------------------
    void selectAccel() noexcept;
    void deselectAccel() noexcept;
    void selectGyro() noexcept;
    void deselectGyro() noexcept;

    // -------------------- 低层 SPI 事务 --------------------
    int8_t writeAccel(uint8_t reg, uint8_t value) noexcept;
    int8_t writeGyro(uint8_t reg, uint8_t value) noexcept;
    /** 读加速度计：自动在地址后插入 1 个 dummy 字节 */
    int8_t readAccel(uint8_t reg, uint8_t *out, uint16_t len) noexcept;
    /** 读陀螺仪：无 dummy 字节 */
    int8_t readGyro(uint8_t reg, uint8_t *out, uint16_t len) noexcept;

    int8_t readAccelId() noexcept;
    int8_t readGyroId() noexcept;

    /**
     * 【加速度计切到 SPI 之后的第一次读，内容不可预测，必须丢弃】
     * 手册 p.45 §6.1 原文：
     *   「To change the sensor to SPI mode in the initialization phase, the user
     *     could perform a dummy SPI read operation, e.g. of register ACC_CHIP_ID
     *     (the obtained value will be invalid).」
     * 内部就是对 ACC_CHIP_ID 读一次并扔掉结果。
     * @return 0 = SPI 事务成功；-1 = 失败
     */
    int8_t discardAccelFirstRead() noexcept;

    /** 读 ACC_ERR_REG(0x02) 存入 accel_err_reg_；失败不改变既有值，也不影响 Init() */
    void readAccelErrorReg() noexcept;

    // -------------------- 异步采样（DMA） --------------------

    /**
     * 发起加速度计事务：0x12 起连续读 kAccTotal 字节。
     * 布局：tx_[0]=0x92，rx_[0]=地址回读，rx_[1]=dummy，
     *       rx_[2..7]=ACC_X/Y/Z(0x12..0x17)，rx_[8..17]=保留(0x18..0x21)，
     *       rx_[18..19]=TEMP_MSB/LSB(0x22..0x23)。
     * @return 0 = DMA 已启动；-1 = 启动失败（片选已释放）
     */
    int8_t startReadAccel() noexcept;

    /**
     * 发起陀螺仪事务：0x02 起连续读 kGyroTotal 字节（无 dummy）。
     * 布局：tx_[0]=0x82，rx_[0]=地址回读，rx_[1..6]=GYRO_X/Y/Z(0x02..0x07)。
     */
    int8_t startReadGyro() noexcept;

    // -------------------- 采样解析 --------------------

    /** 把 rx_acc_ 解析进 pending_ 的加速度与温度（沿用原算法，逐行等价） */
    void parseAccel() noexcept;

    /** 把 rx_gyro_ 解析进 pending_ 的角速度 */
    void parseGyro() noexcept;

    /** 一次 SPI 事务的最大字节数：1(地址) + 1(dummy) + 6(数据) —— 阻塞式 Init() 用 */
    static constexpr uint16_t kMaxTransfer = 8U;

    /** 状态机：Idle 可发起新事务；Wait 表示事务在途（含已链式发起陀螺事务），Start() 会被忽略 */
    enum class Xfer : uint8_t { Idle = 0, Wait = 1 };

    SPI_HandleTypeDef *hspi_ = nullptr;
    Config cfg_{};

    uint8_t accel_chip_id_ = 0U;   ///< 第二次读（真正有效的那次）得到的加速度计 chip id
    uint8_t gyro_chip_id_ = 0U;
    uint8_t accel_err_reg_ = 0U;   ///< ACC_ERR_REG(0x02) 的内容

    float accel_lsb_per_g_ = 5460.0f;    ///< 对应 AccelRange::G6
    float gyro_lsb_per_dps_ = 16.384f;   ///< 对应 GyroRange::Dps2000

    // 以下三个标志由 SPI DMA 中断（优先级 5）与 TIM13（优先级 6）共享，统一用 volatile。
    volatile Xfer     xfer_ = Xfer::Idle;      ///< 当前事务状态
    volatile uint32_t xfer_start_tick_ = 0U;   ///< 本次事务的启动时刻（HAL_GetTick）
    volatile bool     accel_done_ = false;     ///< 加速度计事务已完成，陀螺事务在途
    volatile bool     sample_ready_ = false;   ///< 陀螺事务已完成，等 Process() 解析+提交
    ImuData  pending_{};                       ///< 跨调用暂存的半成品（禁止用栈局部变量跨调用）

    // 注意：DMA 收发缓冲**不在这里** —— GCC 不允许给非静态数据成员加 section 属性
    // （"section attribute not allowed for ..."）。它们定义在 bmi088.cpp 的文件作用域，
    // 用 RAM_D2_SECTION 落到 0x30000000，与 Sbus::s_rx_buf_ 同一套做法。
};

#endif // DM_02_BSP_BMI088_H
