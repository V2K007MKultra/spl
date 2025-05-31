#ifndef SPL06_001_HAL_H
#define SPL06_001_HAL_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef spl06_write_reg(uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef spl06_read_reg(uint8_t reg_addr, uint8_t *p_data);
HAL_StatusTypeDef spl06_config_pressure(uint8_t rate, uint8_t oversampling);
HAL_StatusTypeDef spl06_config_temperature(uint8_t rate, uint8_t oversampling);
uint8_t spl06_get_measure_status(void);
HAL_StatusTypeDef spl06_set_measure_mode(uint8_t mode);
HAL_StatusTypeDef spl06_start_temperature(void);
HAL_StatusTypeDef spl06_start_pressure(void);
HAL_StatusTypeDef spl06_enter_standby(void);
HAL_StatusTypeDef spl06_set_interrupt(uint8_t interrupt, uint8_t enable);
HAL_StatusTypeDef spl06_set_spi_wire(uint8_t wire);
HAL_StatusTypeDef spl06_set_interrupt_level(uint8_t level);
uint8_t spl06_get_int_status(void);
uint8_t spl06_get_fifo_status(void);
HAL_StatusTypeDef spl06_soft_reset(void);
HAL_StatusTypeDef spl06_reset_fifo(void);
uint8_t spl06_get_chip_id(void);
HAL_StatusTypeDef spl06_get_calib_param(void);
HAL_StatusTypeDef spl06_init(void);
HAL_StatusTypeDef spl06_get_raw_temp(void);
HAL_StatusTypeDef spl06_get_raw_pressure(void);
float spl06_get_temperature(void);
float spl06_get_pressure(void);
float user_spl06_get_temperature(void);
float user_spl06_get_pressure(void);

#endif // SPL06_001_HAL_H