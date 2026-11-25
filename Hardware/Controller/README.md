# Controller / 串口接收驱动接入约定

本目录的接收机驱动（当前 `FlySky/Sbus`）统一通过 `Hardware/bsp/uart_bsp` 接入
HAL 的 UART 错误/接收事件分发。**不要**在驱动里再定义全局
`HAL_UART_ErrorCallback` / `HAL_UARTEx_RxEventCallback`（全工程只允许
`uart_bsp.cpp` 一处定义）。

## 现有链路

```text
UARTx IRQ -> HAL_UART_IRQHandler
  -> HAL_UART_ErrorCallback      -> uart_bsp -> 已注册 Port.on_error / raw 收养
  -> HAL_UARTEx_RxEventCallback  -> uart_bsp -> 已注册 Port.rx_event
业务层 -> uart_bsp 端口/raw 表 -> 驱动的 recover(hard)
```

- 已注册端口：错误置 pending；退避/停帧/软硬升级由业务层
  决定，再调用驱动的 `recover(hard)`。
- 未注册 USART：错误自动收养到 raw 表，业务层调用 `uart_bsp::Restore_Raw`
  做外设级恢复（`HAL_UART_AbortReceive` + `DeInit` + `Init`），但不会自动重挂数据通路。

## 后续在 CubeMX 中新增 USART 的接入清单

1. 在 CubeMX 中启用 USART，生成 `MX_USARTx_Init()` / `huartx`。
2. 对于仅发送的端口，BSP 记录错误，业务层可调用 `Restore_Raw` 恢复外设。
3. 需要接收（SBUS/DBUS/视觉/上位机等）：在驱动构造函数里调用
   `uart_bsp::Register(...)`，实现以下回调（均可为空）：

   ```cpp
   uart_bsp::Port port{
       &huartx, this,
       rx_event_trampoline,   // HAL_UARTEx_RxEventCallback 分发
       error_trampoline,      // 只做分类计数/pending
       stale_trampoline,      // 收到过帧但超时无新帧（建议 500 ms）
       recover_trampoline,    // Recover_Soft()/Recover_Hard()
   };
   uart_bsp::Register(port);
   ```

4. 驱动析构时 `uart_bsp::Unregister(this)`。
5. `Recover_Hard()` 直接用 `HAL_UART_DeInit(huart) + HAL_UART_Init(huart)`
   复用句柄里的配置，**不要调用 `MX_USARTx_Init()`**（其中 `Error_Handler()`
   会死循环），然后重新挂载接收。
6. 不要在驱动内定义全局 HAL UART 回调；否则会与 `uart_bsp.cpp` 的强符号冲突。

业务层按自己的节拍读取端口状态并调用恢复接口。
