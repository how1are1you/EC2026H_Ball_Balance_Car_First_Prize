#include "imu_bus.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define IMU_BUS_TIMEOUT_MS (20UL)
#define IMU_BUS_SPIN_LIMIT (2000000UL)
#define IMU_BUS_ERROR_INTERRUPTS \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK | \
     DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

extern volatile unsigned long tick_ms;
extern void delay_ms(uint32_t ms);
extern void delay_us(uint32_t us);

static int imu_bus_wait_expired(unsigned long start_ms,
                                unsigned long *spin_count)
{
    (*spin_count)++;
    if (*spin_count >= IMU_BUS_SPIN_LIMIT)
    {
        return 1;
    }

    return (unsigned long)(tick_ms - start_ms) >= IMU_BUS_TIMEOUT_MS;
}

static void imu_bus_restore_peripheral(void)
{
    DL_I2C_reset(I2C_0_INST);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);
    DL_I2C_enablePower(I2C_0_INST);
    SYSCFG_DL_I2C_0_init();
}

static void imu_bus_recover(void)
{
    uint8_t pulse;

    DL_I2C_reset(I2C_0_INST);
    DL_GPIO_initDigitalOutput(GPIO_I2C_0_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_0_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);

    for (pulse = 0U; pulse < 9U; pulse++)
    {
        if (DL_GPIO_readPins(GPIO_I2C_0_SDA_PORT,
                            GPIO_I2C_0_SDA_PIN) != 0U)
        {
            break;
        }
        DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_us(5U);
        DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_us(5U);
    }

    imu_bus_restore_peripheral();
}

static int imu_bus_wait_idle(void)
{
    unsigned long start_ms = tick_ms;
    unsigned long spin_count = 0UL;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    {
        if (imu_bus_wait_expired(start_ms, &spin_count))
        {
            imu_bus_recover();
            return -1;
        }
    }
    return 0;
}

void imu_bus_init(void)
{
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    if (DL_I2C_getSDAStatus(I2C_0_INST) == DL_I2C_CONTROLLER_SDA_LOW)
    {
        imu_bus_recover();
    }
}

int imu_bus_write(unsigned char slave_addr,
                  unsigned char reg_addr,
                  unsigned char length,
                  unsigned char const *data)
{
    unsigned long start_ms;
    unsigned long spin_count = 0UL;
    uint16_t remaining = length;
    unsigned char const *cursor = data;
    uint32_t interrupt_status;

    if (length == 0U || data == NULL)
    {
        return -1;
    }
    if (imu_bus_wait_idle() != 0)
    {
        return -2;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | IMU_BUS_ERROR_INTERRUPTS);
    DL_I2C_transmitControllerData(I2C_0_INST, reg_addr);
    DL_I2C_startControllerTransfer(I2C_0_INST, slave_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, (uint16_t)length + 1U);

    start_ms = tick_ms;
    while (1)
    {
        if (remaining > 0U)
        {
            uint16_t written = DL_I2C_fillControllerTXFIFO(
                I2C_0_INST, (uint8_t *)cursor, remaining);
            remaining -= written;
            cursor += written;
        }

        interrupt_status = DL_I2C_getRawInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            IMU_BUS_ERROR_INTERRUPTS);
        if ((interrupt_status & IMU_BUS_ERROR_INTERRUPTS) != 0U)
        {
            imu_bus_recover();
            return -3;
        }
        if ((interrupt_status &
             DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U)
        {
            return (remaining == 0U) ? 0 : -4;
        }
        if (imu_bus_wait_expired(start_ms, &spin_count))
        {
            imu_bus_recover();
            return -5;
        }
    }
}

int imu_bus_read(unsigned char slave_addr,
                 unsigned char reg_addr,
                 unsigned char length,
                 unsigned char *data)
{
    unsigned long start_ms;
    unsigned long spin_count = 0UL;
    uint16_t received = 0U;
    uint32_t interrupt_status;

    if (length == 0U || data == NULL)
    {
        return -1;
    }
    if (imu_bus_wait_idle() != 0)
    {
        return -2;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | IMU_BUS_ERROR_INTERRUPTS);
    DL_I2C_transmitControllerData(I2C_0_INST, reg_addr);
    I2C_0_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_startControllerTransfer(I2C_0_INST, slave_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);

    start_ms = tick_ms;
    while (1)
    {
        while (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST))
        {
            unsigned char value =
                DL_I2C_receiveControllerData(I2C_0_INST);
            if (received < length)
            {
                data[received++] = value;
            }
        }

        interrupt_status = DL_I2C_getRawInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
            IMU_BUS_ERROR_INTERRUPTS);
        if ((interrupt_status & IMU_BUS_ERROR_INTERRUPTS) != 0U)
        {
            I2C_0_INST->MASTER.MCTR = 0U;
            imu_bus_recover();
            return -3;
        }
        if ((interrupt_status &
             DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) != 0U)
        {
            break;
        }
        if (imu_bus_wait_expired(start_ms, &spin_count))
        {
            I2C_0_INST->MASTER.MCTR = 0U;
            imu_bus_recover();
            return -4;
        }
    }

    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST))
    {
        unsigned char value = DL_I2C_receiveControllerData(I2C_0_INST);
        if (received < length)
        {
            data[received++] = value;
        }
    }

    I2C_0_INST->MASTER.MCTR = 0U;
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    return (received == length) ? 0 : -5;
}

void imu_bus_delay_ms(unsigned long ms)
{
    delay_ms((uint32_t)ms);
}

void imu_bus_get_ms(unsigned long *time_ms)
{
    if (time_ms != NULL)
    {
        *time_ms = tick_ms;
    }
}
