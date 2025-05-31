/************************************************************
 * SPL06-001 HAL 驱动程序
 * 设计用于内部温度传感器
 * @file spl06_001_hal.c
 ************************************************************/

#include "spl06_001_hal.h"
#include "i2c.h"

#define HW_ADR           0x76   // I2C 地址，当 SDO 为高电平或 NC 时
#define PRODUCT_ID       0x10    // 预期芯片 ID

// 寄存器定义
typedef enum {
    PRESSURE_REG = 0x00,
    TEMP_REG        = 0x03,
    PRS_CFG_REG   = 0x06,
    TMP_CFG_REG     = 0x07,
    MEAS_CFG_REG    = 0x08,
    CFG_REG         = 0x09,
    INT_STS_REG     = 0x0A,
    FIFO_STS_REG    = 0x0B,
    RESET_REG       = 0x0C,
    ID_REG          = 0x0D,
    COEF_REG        = 0x10
} spl06_reg_t;

// 校准参数结构
typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06_calib_param_t;

// 传感器数据结构
typedef struct {
    uint8_t chip_id;
    int32_t raw_pressure;
    int32_t raw_temperature;
    int32_t kP;
    int32_t kT;
} spl06_data_t;

// 全局变量
static spl06_calib_param_t calib_param;
static spl06_data_t spl06;

// 采样率定义
typedef enum {
    RATE_1_TIMES    = 0,
    RATE_2_TIMES    = 1,
    RATE_4_TIMES    = 2,
    RATE_8_TIMES    = 3,
    RATE_16_TIMES   = 4,
    RATE_32_TIMES   = 5,
    RATE_64_TIMES   = 6,
    RATE_128_TIMES  = 7
} spl06_rate_t;

// 测量配置
typedef enum {
    MEAS_STANDBY            = 0x00,
    MEAS_CMD_PRESSURE       = 0x01,
    MEAS_CMD_TEMPERATURE    = 0x02,
    MEAS_BG_PRESSURE        = 0x05,
    MEAS_BG_TEMPERATURE     = 0x06,
    MEAS_BG_PRES_TEMP       = 0x07
} spl06_meas_mode_t;

// 配置寄存器位定义
#define CFG_INT_LEVEL_ACTIVE_LOW  0x00
#define CFG_INT_LEVEL_ACTIVE_HIGH 0x80
#define CFG_INT_FIFO              0x40
#define CFG_INT_PRS               0x20
#define CFG_INT_TMP               0x10
#define CFG_T_SHIFT               0x08
#define CFG_P_SHIFT               0x04
#define CFG_FIF                   0x02
#define CFG_SPI_3_WIRE            0x01
#define CFG_SPI_4_WIRE            0x00

// 重置寄存器位定义
#define RESET_FIFO_FLUSH 0x80
#define RESET_SOFT       0x09

// 中断状态位定义
#define INT_STS_FIFO_FULL  0x04
#define INT_STS_FIFO_TMP   0x02
#define INT_STS_FIFO_PRS   0x01

// FIFO 状态位定义
#define FIFO_STS_FULL  0x02
#define FIFO_STS_EMPTY 0x01

// 测量状态位定义
#define MEAS_CFG_COEF_RDY    0x80
#define MEAS_CFG_SENSOR_RDY  0x40
#define MEAS_CFG_TMP_RDY     0x20
#define MEAS_CFG_PRS_RDY     0x10

/**
 * @brief 向 SPL06 寄存器写入单个字节
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_write_reg(uint8_t reg_addr, uint8_t data) {
    return HAL_I2C_Master_Transmit(&hi2c1, (HW_ADR << 1), &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief 从 SPL06 寄存器读取单个字节
 * @param reg_addr 寄存器地址
 * @param p_data 数据缓冲区指针
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_read_reg(uint8_t reg_addr, uint8_t *p_data) {
    return HAL_I2C_Master_Receive(&hi2c1, (HW_ADR << 1) | 1, p_data, 1, HAL_MAX_DELAY);
}

/**
 * @brief 配置气压采样率
 * @param rate 采样率
 * @param oversampling 过采样率
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_config_pressure(spl06_rate_t rate, spl06_rate_t oversampling) {
    uint8_t data = (rate << 4) | oversampling;
    HAL_StatusTypeDef status;

    if (oversampling > RATE_8_TIMES) {
        uint8_t cfg;
        status = spl06_read_reg(CFG_REG, &cfg);
        cfg |= CFG_P_SHIFT;
        status |= spl06_write_reg(CFG_REG, cfg);
    }

    switch (oversampling) {
        case RATE_2_TIMES: spl06.kP = 1572864; break;
        case RATE_4_TIMES: spl06.kP = 3670016; break;
        case RATE_8_TIMES: spl06.kP = 7864320; break;
        case RATE_16_TIMES: spl06.kP = 253952; break;
        case RATE_32_TIMES: spl06.kP = 516096; break;
        case RATE_64_TIMES: spl06.kP = 1040384; break;
        case RATE_128_TIMES: spl06.kP = 2088960; break;
        case RATE_1_TIMES:
        default: spl06.kP = 524288; break;
    }

    return spl06_write_reg(PRS_CFG_REG, data);
}

/**
 * @brief 配置温度采样率（使用内部传感器）
 * @param rate 采样率
 * @param oversampling 过采样率
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_config_temperature(spl06_rate_t rate, spl06_rate_t oversampling) {
    uint8_t data = (rate << 4) | oversampling; // 内部传感器 (位 7 = 0)
    HAL_StatusTypeDef status;

    if (oversampling > RATE_8_TIMES) {
        uint8_t cfg;
        status = spl06_read_reg(CFG_REG, &cfg);
        cfg |= CFG_T_SHIFT;
        status |= spl06_write_reg(CFG_REG, cfg);
    }

    switch (oversampling) {
        case RATE_2_TIMES: spl06.kT = 1572864; break;
        case RATE_4_TIMES: spl06.kT = 3670016; break;
        case RATE_8_TIMES: spl06.kT = 7864320; break;
        case RATE_16_TIMES: spl06.kT = 253952; break;
        case RATE_32_TIMES: spl06.kT = 516096; break;
        case RATE_64_TIMES: spl06.kT = 1040384; break;
        case RATE_128_TIMES: spl06.kT = 2088960; break;
        case RATE_1_TIMES:
        default: spl06.kT = 524288; break;
    }

    return spl06_write_reg(TMP_CFG_REG, data);
}

/**
 * @brief 获取测量状态
 * @return 状态寄存器值
 */
uint8_t spl06_get_measure_status(void) {
    uint8_t status;
    spl06_read_reg(MEAS_CFG_REG, &status);
    return status;
}

/**
 * @brief 设置测量模式
 * @param mode 测量模式
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_set_measure_mode(spl06_meas_mode_t mode) {
    return spl06_write_reg(MEAS_CFG_REG, (uint8_t)mode);
}

/**
 * @brief 启动命令模式读取温度
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_start_temperature(void) {
    return spl06_write_reg(MEAS_CFG_REG, MEAS_CMD_TEMPERATURE);
}

/**
 * @brief 启动命令模式读取气压
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_start_pressure(void) {
    return spl06_write_reg(MEAS_CFG_REG, MEAS_CMD_PRESSURE);
}

/**
 * @brief 进入待机模式
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_enter_standby(void) {
    return spl06_write_reg(MEAS_CFG_REG, MEAS_STANDBY);
}

/**
 * @brief 设置中断
 * @param interrupt 中断类型
 * @param enable 使能/禁用
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_set_interrupt(uint8_t interrupt, uint8_t enable) {
    uint8_t data;
    HAL_StatusTypeDef status = spl06_read_reg(CFG_REG, &data);
    if (enable)
        data |= interrupt;
    else
        data &= ~interrupt;
    return spl06_write_reg(CFG_REG, data);
}

/**
 * @brief 设置 SPI 线数
 * @param wire 线数 (0: 4 线, 1: 3 线)
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_set_spi_wire(uint8_t wire) {
    uint8_t data;
    HAL_StatusTypeDef status = spl06_read_reg(CFG_REG, &data);
    data &= 0xFE;
    data |= wire;
    return spl06_write_reg(CFG_REG, data);
}

/**
 * @brief 设置中断电平
 * @param level 电平 (0: 低电平有效, 1: 高电平有效)
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_set_interrupt_level(uint8_t level) {
    uint8_t data;
    HAL_StatusTypeDef status = spl06_read_reg(CFG_REG, &data);
    data &= 0x7F;
    data |= (level << 7);
    return spl06_write_reg(CFG_REG, data);
}

/**
 * @brief 获取中断状态
 * @return 中断状态
 */
uint8_t spl06_get_int_status(void) {
    uint8_t status;
    spl06_read_reg(INT_STS_REG, &status);
    return status;
}

/**
 * @brief 获取 FIFO 状态
 * @return FIFO 状态
 */
uint8_t spl06_get_fifo_status(void) {
    uint8_t status;
    spl06_read_reg(FIFO_STS_REG, &status);
    return status;
}

/**
 * @brief 软件复位
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_soft_reset(void) {
    return spl06_write_reg(RESET_REG, RESET_SOFT);
}

/**
 * @brief 清空 FIFO
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_reset_fifo(void) {
    return spl06_write_reg(RESET_REG, RESET_FIFO_FLUSH);
}

/**
 * @brief 获取芯片 ID
 * @return 芯片 ID
 */
uint8_t spl06_get_chip_id(void) {
    uint8_t id;
    spl06_read_reg(ID_REG, &id);
    return id;
}

/**
 * @brief 获取校准参数
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_get_calib_param(void) {
    uint8_t h, m, l;
    HAL_StatusTypeDef status;

    status = spl06_read_reg(0x10, &h);
    status |= spl06_read_reg(0x11, &l);
    calib_param.c0 = (int16_t)h << 4 | l >> 4;
    calib_param.c0 = (calib_param.c0 & 0x0800) ? (0xF000 | calib_param.c0) : calib_param.c0;

    status |= spl06_read_reg(0x11, &h);
    status |= spl06_read_reg(0x12, &l);
    calib_param.c1 = (int16_t)(h & 0x0F) << 8 | l;
    calib_param.c1 = (calib_param.c1 & 0x0800) ? (0xF000 | calib_param.c1) : calib_param.c1;

    status |= spl06_read_reg(0x13, &h);
    status |= spl06_read_reg(0x14, &m);
    status |= spl06_read_reg(0x15, &l);
    calib_param.c00 = (int32_t)h << 12 | (int32_t)m << 4 | (int32_t)l >> 4;
    calib_param.c00 = (calib_param.c00 & 0x080000) ? (0xFFF00000 | calib_param.c00) : calib_param.c00;

    status |= spl06_read_reg(0x15, &h);
    status |= spl06_read_reg(0x16, &m);
    status |= spl06_read_reg(0x17, &l);
    calib_param.c10 = (int32_t)h << 16 | (int32_t)m << 8 | l;
    calib_param.c10 = (calib_param.c10 & 0x080000) ? (0xFFF00000 | calib_param.c10) : calib_param.c10;

    status |= spl06_read_reg(0x18, &h);
    status |= spl06_read_reg(0x19, &l);
    calib_param.c01 = (int16_t)h << 8 | l;

    status |= spl06_read_reg(0x1A, &h);
    status |= spl06_read_reg(0x1B, &l);
    calib_param.c11 = (int16_t)h << 8 | l;

    status |= spl06_read_reg(0x1C, &h);
    status |= spl06_read_reg(0x1D, &l);
    calib_param.c20 = (int16_t)h << 8 | l;

    status |= spl06_read_reg(0x1E, &h);
    status |= spl06_read_reg(0x1F, &l);
    calib_param.c21 = (int16_t)h << 8 | l;

    status |= spl06_read_reg(0x20, &h);
    status |= spl06_read_reg(0x21, &l);
    calib_param.c30 = (int16_t)h << 8 | l;

    return status;
}

/**
 * @brief 初始化 SPL06 传感器
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_init(void) {
    HAL_StatusTypeDef status;
    uint8_t status_reg;

    // 等待校准数据就绪
    do {
        status = spl06_read_reg(MEAS_CFG_REG, &status_reg);
    } while (!(status_reg & MEAS_CFG_COEF_RDY));

    // 读取校准参数
    status = spl06_get_calib_param();

    // 等待传感器就绪
    do {
        status = spl06_read_reg(MEAS_CFG_REG, &status_reg);
    } while (!(status_reg & MEAS_CFG_SENSOR_RDY));

    // 读取芯片 ID
    status |= spl06_read_reg(ID_REG, &spl06.chip_id);
    if ((spl06.chip_id & 0xF0) != PRODUCT_ID) {
        return HAL_ERROR;
    }

    // 配置采样率（使用内部温度传感器）
    status |= spl06_config_pressure(RATE_128_TIMES, RATE_32_TIMES);
    status |= spl06_config_temperature(RATE_32_TIMES, RATE_8_TIMES);

    // 启动后台测量
    status |= spl06_set_measure_mode(MEAS_BG_PRES_TEMP);

    return status;
}

/**
 * @brief 获取原始温度值
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_get_raw_temp(void) {
    uint8_t h[3] = {0};
    HAL_StatusTypeDef status;

    status = spl06_read_reg(TEMP_REG, &h[0]);
    status |= spl06_read_reg(TEMP_REG + 1, &h[1]);
    status |= spl06_read_reg(TEMP_REG + 2, &h[2]);

    spl06.raw_temperature = (int32_t)h[0] << 16 | (int32_t)h[1] << 8 | (int32_t)h[2];
    spl06.raw_temperature = (spl06.raw_temperature & 0x800000) ? (0xFF000000 | spl06.raw_temperature) : spl06.raw_temperature;

    return status;
}

/**
 * @brief 获取原始气压值
 * @return HAL 状态
 */
HAL_StatusTypeDef spl06_get_raw_pressure(void) {
    uint8_t h[3] = {0};
    HAL_StatusTypeDef status;

    status = spl06_read_reg(PRESSURE_REG, &h[0]);
    status |= spl06_read_reg(PRESSURE_REG + 1, &h[1]);
    status |= spl06_read_reg(PRESSURE_REG + 2, &h[2]);

    spl06.raw_pressure = (int32_t)h[0] << 16 | (int32_t)h[1] << 8 | (int32_t)h[2];
    spl06.raw_pressure = (spl06.raw_pressure & 0x800000) ? (0xFF000000 | spl06.raw_pressure) : spl06.raw_pressure;

    return status;
}

/**
 * @brief 获取温度值
 * @return 温度值 (单位: °C)
 */
float spl06_get_temperature(void) {
    float fTsc = (float)spl06.raw_temperature / spl06.kT;
    return calib_param.c0 * 0.5f + calib_param.c1 * fTsc;
}

/**
 * @brief 获取气压值（含温度补偿）
 * @return 气压值 (单位: hPa)
 */
float spl06_get_pressure(void) {
    float fTsc = (float)spl06.raw_temperature / spl06.kT;
    float fPsc = (float)spl06.raw_pressure / spl06.kP;
    float qua2 = calib_param.c10 + fPsc * (calib_param.c20 + fPsc * calib_param.c30);
    float qua3 = fTsc * fPsc * (calib_param.c11 + fPsc * calib_param.c21);
    return calib_param.c00 + fPsc * qua2 + fTsc * calib_param.c01 + qua3;
}

/**
 * @brief 获取用户温度值
 * @return 温度值 (单位: °C)
 */
float user_spl06_get_temperature(void) {
    spl06_get_raw_temp();
    return spl06_get_temperature();
}

/**
 * @brief 获取用户气压值
 * @return 气压值 (单位: hPa)
 */
float user_spl06_get_pressure(void) {
    spl06_get_raw_pressure();
    return spl06_get_pressure();
}
