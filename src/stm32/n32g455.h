#ifndef __N32G455_H
#define __N32G455_H

#include <stdint.h>

// N32G455 specific definitions

// GPIO registers - compatible with STM32F4 for easier integration
typedef struct {
    volatile uint32_t MODER;    // Mode register
    volatile uint32_t OTYPER;   // Output type register
    volatile uint32_t OSPEEDR;   // Output speed register
    volatile uint32_t PUPDR;     // Pull-up/pull-down register
    volatile uint32_t IDR;       // Input data register
    volatile uint32_t ODR;       // Output data register
    volatile uint32_t BSRR;      // Bit set/reset register
    volatile uint32_t LCKR;      // Configuration lock register
    volatile uint32_t AFRL;      // Alternate function low register
    volatile uint32_t AFRH;      // Alternate function high register
} GPIO_TypeDef;

// RCC registers - compatible with STM32F4 for easier integration
typedef struct {
    volatile uint32_t CR;         // Clock control register
    volatile uint32_t PLLCFGR;    // PLL configuration register
    volatile uint32_t CFGR;       // Clock configuration register
    volatile uint32_t CIR;        // Clock interrupt register
    volatile uint32_t AHB1RSTR;   // AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR;   // AHB2 peripheral reset register
    volatile uint32_t AHB3RSTR;   // AHB3 peripheral reset register
    uint32_t RESERVED0;           // Reserved
    volatile uint32_t APB1RSTR;   // APB1 peripheral reset register
    volatile uint32_t APB2RSTR;   // APB2 peripheral reset register
    uint32_t RESERVED1;           // Reserved
    volatile uint32_t AHB1ENR;    // AHB1 peripheral clock enable register
    volatile uint32_t AHB2ENR;    // AHB2 peripheral clock enable register
    volatile uint32_t AHB3ENR;    // AHB3 peripheral clock enable register
    uint32_t RESERVED2;           // Reserved
    volatile uint32_t APB1ENR;    // APB1 peripheral clock enable register
    volatile uint32_t APB2ENR;    // APB2 peripheral clock enable register
    uint32_t RESERVED3;           // Reserved
    volatile uint32_t BDCR;       // Backup domain control register
    volatile uint32_t CSR;        // Clock status register
    uint32_t RESERVED4[2];       // Reserved
    volatile uint32_t SSCGR;      // Spread spectrum clock generation register
    volatile uint32_t PLLI2SCFGR; // PLLI2S configuration register
    volatile uint32_t PLLSAICFGR; // PLLSAI configuration register
    volatile uint32_t DCKCFGR;    // Dedicated clocks configuration register
} RCC_TypeDef;

// Base addresses
#define GPIOA_BASE           0x40020000
#define GPIOB_BASE           0x40020400
#define GPIOC_BASE           0x40020800
#define GPIOD_BASE           0x40020C00
#define GPIOE_BASE           0x40021000
#define GPIOF_BASE           0x40021400

#define RCC_BASE            0x40021000

// Peripheral declarations
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)

#define RCC                 ((RCC_TypeDef *) RCC_BASE)

// RCC AHB1ENR register bits
#define RCC_AHB1ENR_GPIOAEN       (1 << 0)
#define RCC_AHB1ENR_GPIOBEN       (1 << 1)
#define RCC_AHB1ENR_GPIOCEN       (1 << 2)

#endif // __N32G455_H