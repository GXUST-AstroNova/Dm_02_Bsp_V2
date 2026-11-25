#pragma once

// DMA1/2 cannot access the default DTCM data section on STM32H723.
#define RAM_D2_SECTION __attribute__((section(".RAM_D2")))
