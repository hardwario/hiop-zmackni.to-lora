#include "lis2dh12.h"
#include <bc_atci.h>
#include <bc_exti.h>
#include <bc_i2c.h>
#include <bc_log.h>
#include <bc_tick.h>
#include <stm32l0xx.h>

#define _LIS2DH12_I2C_ADDRESS 0x19
#define _LIS2DH12_THRESHOLD 4
#define _LIS2DH12_DURATION 1
#define _LIS2DH12_BLANKING_TIMEOUT (5 * 1000)
#define _LIS2DH12_BLANKING_TIMEOUT_BOOT (5 * 1000)

#define _LIS2DH12_REG_STATUS_AUX (0x07)
#define _LIS2DH12_REG_WHO_AM_I (0x0F)
#define _LIS2DH12_REG_CTRL0 (0x1E)
#define _LIS2DH12_REG_CFG0 (0x1F)

#define _LIS2DH12_REG_CTRL1 (0x20)
#define     _CTRL1_XEN       (1 << 0)
#define     _CTRL1_YEN       (1 << 1)
#define     _CTRL1_ZEN       (1 << 2)
#define     _CTRL1_LPEN      (1 << 3)
#define     _CTRL1_ODR_OFF   (0 << 4)
#define     _CTRL1_ODR_1HZ   (1 << 4)
#define     _CTRL1_ODR_10HZ  (2 << 4)
#define     _CTRL1_ODR_25HZ  (3 << 4)
#define     _CTRL1_ODR_50HZ  (4 << 4)
#define     _CTRL1_ODR_100HZ (5 << 4)
#define     _CTRL1_ODR_200HZ (6 << 4)
#define     _CTRL1_ODR_400HZ (7 << 4)

#define _LIS2DH12_REG_CTRL2 (0x21)
#define     _CTRL2_HP_IA1    (1 << 0) // High-pass filter enable for INT1
#define     _CTRL2_HP_IA2    (1 << 1) // High-pass filter enable for INT2
#define     _CTRL2_FDS       (1 << 3) // Filtered data selection
#define     _CTRL2_HP_CLICK  (1 << 2) // High-pass filter enabled for CLICK function

#define _LIS2DH12_REG_CTRL3 (0x22)
#define     _CTRL3_I1_IA2    (1 << 5)
#define     _CTRL3_I1_IA1    (1 << 6)

#define _LIS2DH12_REG_CTRL4 (0x23)
#define     _CTRL4_BDU       (1 << 7) // Block data update. 0: continuous; 1: output regs not updated until MSB and LSB have been read

#define _LIS2DH12_REG_CTRL5 (0x24)
#define     _CTRL5_BOOT      (1 << 7) // reboot memory content (reset)
#define     _CTRL5_LIR_INT1  (1 << 3) // Latch interrupt request, cleared by reading INT1_SRC (31h)
#define     _CTRL5_LIR_INT2  (1 << 1) // Latch interrupt request, cleared by reading INT2_SRC (35h)

#define _LIS2DH12_REG_CTRL6 (0x25)
#define     _CTRL6_INT_POLARITY (1 << 1) // interrupt pins polarity (default 0 == active high)
#define     _CTRL6_I2_ACT       (1 << 3) // activity on INT2
#define     _CTRL6_I2_BOOT      (1 << 4) // boot on INT2
#define     _CTRL6_I2_IA2       (1 << 5) // interrupt 2 function on INT2
#define     _CTRL6_I2_IA1       (1 << 6) // interrupt 1 function on INT2
#define     _CTRL6_I2_CLICK     (1 << 7) // click interrupt on INT2

#define _LIS2DH12_REG_REF (0x26)

#define _LIS2DH12_REG_STATUS (0x27)

#define _LIS2DH12_REG_INT1_CFG (0x30)
#define     _INT1_CFG_XLIE (1 << 0)
#define     _INT1_CFG_XHIE (1 << 1)
#define     _INT1_CFG_YLIE (1 << 2)
#define     _INT1_CFG_YHIE (1 << 3)
#define     _INT1_CFG_ZLIE (1 << 4)
#define     _INT1_CFG_ZHIE (1 << 5)
#define     _INT1_CFG_6D   (1 << 6)
#define     _INT1_CFG_AOI  (1 << 7)

#define _LIS2DH12_REG_INT1_SRC (0x31)

#define _LIS2DH12_REG_INT1_THS (0x32)

#define _LIS2DH12_REG_INT1_DURATION (0x33)

#define _LIS2DH12_REG_INT2_CFG (0x34)
#define     _INT2_CFG_XLIE (1 << 0)
#define     _INT2_CFG_XHIE (1 << 1)
#define     _INT2_CFG_YLIE (1 << 2)
#define     _INT2_CFG_YHIE (1 << 3)
#define     _INT2_CFG_ZLIE (1 << 4)
#define     _INT2_CFG_ZHIE (1 << 5)
#define     _INT2_CFG_6D   (1 << 6)
#define     _INT2_CFG_AOI  (1 << 7)

#define _LIS2DH12_REG_INT2_SRC (0x35)

#define _LIS2DH12_REG_INT2_THS (0x36)

#define _LIS2DH12_REG_INT2_DURATION (0x37)

static bool _lis2dh12_reg_write(uint8_t address, uint8_t value)
{
    if (!bc_i2c_memory_write_8b(BC_I2C_I2C0, _LIS2DH12_I2C_ADDRESS, address, value))
    {
        return false;
    }

    return true;
}

static bool _lis2dh12_reg_read(uint8_t address, uint8_t *value)
{
    if (!bc_i2c_memory_read_8b(BC_I2C_I2C0, _LIS2DH12_I2C_ADDRESS, address, value))
    {
        return false;
    }

    return true;
}

#define CHECK(EXPR)   \
    if (!(EXPR))      \
    {                 \
        return false; \
    }


bool volatile lis2dh12_irq;

static void _lis2dh12_interrupt(bc_exti_line_t line, void *param)
{
    (void) line;
    (void) param;

    lis2dh12_irq = true;
}

bool lis2dh12_init(void)
{
    // Enable GPIOB clock
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Errata workaround
    RCC->IOPENR;

    // Set input mode
    GPIOB->MODER &= ~GPIO_MODER_MODE6_Msk;

    uint8_t dummy;

    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_CTRL5, _CTRL5_BOOT));

    bc_tick_wait(10);

    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_CTRL1,
                              _CTRL1_XEN | _CTRL1_YEN | _CTRL1_ZEN | _CTRL1_LPEN | _CTRL1_ODR_100HZ));
    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_CTRL2, _CTRL2_FDS | _CTRL2_HP_IA1));
    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_CTRL3, _CTRL3_I1_IA1));
    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_CTRL4, _CTRL4_BDU));
    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_CTRL5, _CTRL5_LIR_INT1));
    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_CTRL6, 0x02));
    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_INT1_THS, _LIS2DH12_THRESHOLD));
    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_INT1_DURATION, _LIS2DH12_DURATION));
    CHECK(_lis2dh12_reg_read(_LIS2DH12_REG_REF, &dummy));
    CHECK(_lis2dh12_reg_write(_LIS2DH12_REG_INT1_CFG, _INT1_CFG_XHIE | _INT1_CFG_YHIE | _INT1_CFG_ZHIE));

    bc_exti_register(BC_EXTI_LINE_PB6, BC_EXTI_EDGE_FALLING, _lis2dh12_interrupt, NULL);

    return true;
}

bool lis2dh12_clear_irq(void)
{
    lis2dh12_irq = false;

    uint8_t int1;
    return _lis2dh12_reg_read(_LIS2DH12_REG_INT1_SRC, &int1);
}
