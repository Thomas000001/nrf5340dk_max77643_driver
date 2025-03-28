#include "max77643.h"
#include "max77643_2_regs.h"  // 寄存器定义
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

// 辅助宏：设置寄存器位字段
#define SET_BIT_FIELD(obj, reg_addr, reg_type, field, value) \
    do { \
        reg_type reg; \
        int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg); \
        if (ret != MAX77643_NO_ERROR) return ret; \
        reg.bits.field = value; \
        ret = max77643_write_register(obj, reg_addr, (uint8_t *)&reg); \
        if (ret != MAX77643_NO_ERROR) return ret; \
    } while (0)

// 初始化函数
int max77643_init(max77643_t *obj, const struct i2c_dt_spec *i2c_spec, uint32_t irq_pin) {
    if (obj == NULL || i2c_spec == NULL) {
        return MAX77643_VALUE_NULL;
    }
    obj->i2c_spec = i2c_spec;
    obj->irq_pin = irq_pin;
    if (irq_pin != 0) {
        const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
        if (!device_is_ready(gpio_dev)) {
            return -1;
        }
        gpio_pin_configure(gpio_dev, irq_pin, GPIO_INPUT | GPIO_PULL_UP);
    }
    return MAX77643_NO_ERROR;
}

// 寄存器读函数
int max77643_read_register(max77643_t *obj, uint8_t reg, uint8_t *value) {
    if (obj == NULL || obj->i2c_spec == NULL || value == NULL) {
        return MAX77643_VALUE_NULL;
    }
    int ret = i2c_write_dt(obj->i2c_spec, &reg, 1);  // 发送寄存器地址
    if (ret != 0) return MAX77643_WRITE_DATA_FAILED;
    ret = i2c_read_dt(obj->i2c_spec, value, 1);      // 读取数据
    if (ret != 0) return MAX77643_READ_DATA_FAILED;
    return MAX77643_NO_ERROR;
}

// 寄存器写函数
int max77643_write_register(max77643_t *obj, uint8_t reg, const uint8_t *value) {
    if (obj == NULL || obj->i2c_spec == NULL || value == NULL) {
        return MAX77643_VALUE_NULL;
    }
    uint8_t data[2] = {reg, *value};  // 数据格式：[寄存器地址, 值]
    int ret = i2c_write_dt(obj->i2c_spec, data, 2);
    if (ret != 0) return MAX77643_WRITE_DATA_FAILED;
    return MAX77643_NO_ERROR;
}

// 获取 ERCFLAG 寄存器位字段
int max77643_get_ercflag(max77643_t *obj, reg_bit_ercflag_t bit_field, uint8_t *flag) {
    reg_ercflag_t reg;
    int ret = max77643_read_register(obj, ERCFLAG, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case ERCFLAG_TOVLD: *flag = reg.bits.tovld; break;
        case ERCFLAG_INOVLO: *flag = reg.bits.inovlo; break;
        case ERCFLAG_INUVLO: *flag = reg.bits.inuvlo; break;
        case ERCFLAG_MRST_F: *flag = reg.bits.mrst_f; break;
        case ERCFLAG_SFT_OFF_F: *flag = reg.bits.sft_off_f; break;
        case ERCFLAG_SFT_CRST_F: *flag = reg.bits.sft_crst_f; break;
        case ERCFLAG_WDT_EXP_F: *flag = reg.bits.wdt_exp_f; break;
        case ERCFLAG_SBB_FAULT_F: *flag = reg.bits.sbb_fault_f; break;
        default: return MAX77643_INVALID_DATA;
    }
    return MAX77643_NO_ERROR;
}

// 获取 STAT_GLBL 寄存器位字段
int max77643_get_stat_glbl(max77643_t *obj, reg_bit_stat_glbl_t bit_field, uint8_t *status) {
    reg_stat_glbl_t reg;
    int ret = max77643_read_register(obj, STAT_GLBL, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case STAT_GLBL_STAT_IRQ: *status = reg.bits.stat_irq; break;
        case STAT_GLBL_STAT_EN: *status = reg.bits.stat_en; break;
        case STAT_GLBL_TJAL1_S: *status = reg.bits.tjal1_s; break;
        case STAT_GLBL_TJAL2_S: *status = reg.bits.tjal2_s; break;
        case STAT_GLBL_DOD_S: *status = reg.bits.dod_s; break;
        case STAT_GLBL_BOK: *status = reg.bits.bok; break;
        case STAT_GLBL_DIDM: *status = reg.bits.didm; break;
        default: return MAX77643_INVALID_DATA;
    }
    return MAX77643_NO_ERROR;
}

// 设置中断掩码
int max77643_set_interrupt_mask(max77643_t *obj, reg_bit_int_mask_t bit_field, uint8_t maskBit) {
    uint8_t reg_addr = (bit_field < INTM_GLBL1_GPI1_FM) ? INTM_GLBL0 : INTM_GLBL1;
    int ret;

    if (reg_addr == INTM_GLBL0) {
        reg_intm_glbl0_t reg;
        ret = max77643_read_register(obj, INTM_GLBL0, (uint8_t *)&reg);
        if (ret != MAX77643_NO_ERROR) return ret;
        switch (bit_field) {
            case INTM_GLBL0_GPI0_FM: reg.bits.gpi0_fm = maskBit; break;
            case INTM_GLBL0_GPI0_RM: reg.bits.gpi0_rm = maskBit; break;
            case INTM_GLBL0_nEN_FM: reg.bits.nen_fm = maskBit; break;
            case INTM_GLBL0_nEN_RM: reg.bits.nen_rm = maskBit; break;
            case INTM_GLBL0_TJAL1_RM: reg.bits.tjal1_rm = maskBit; break;
            case INTM_GLBL0_TJAL2_RM: reg.bits.tjal2_rm = maskBit; break;
            case INTM_GLBL0_DOD_RM: reg.bits.dod_rm = maskBit; break;
            default: return MAX77643_INVALID_DATA;
        }
        ret = max77643_write_register(obj, INTM_GLBL0, (uint8_t *)&reg);
    } else {
        reg_intm_glbl1_t reg;
        ret = max77643_read_register(obj, INTM_GLBL1, (uint8_t *)&reg);
        if (ret != MAX77643_NO_ERROR) return ret;
        switch (bit_field) {
            case INTM_GLBL1_GPI1_FM: reg.bits.gpi1_fm = maskBit; break;
            case INTM_GLBL1_GPI1_RM: reg.bits.gpi1_rm = maskBit; break;
            case INTM_GLBL1_SBB0_FM: reg.bits.sbb0_fm = maskBit; break;
            case INTM_GLBL1_SBB1_FM: reg.bits.sbb1_fm = maskBit; break;
            case INTM_GLBL1_SBB2_FM: reg.bits.sbb2_fm = maskBit; break;
            case INTM_GLBL1_LDO_M: reg.bits.ldo_m = maskBit; break;
            default: return MAX77643_INVALID_DATA;
        }
        ret = max77643_write_register(obj, INTM_GLBL1, (uint8_t *)&reg);
    }
    return ret;
}

// 获取中断掩码
int max77643_get_interrupt_mask(max77643_t *obj, reg_bit_int_mask_t bit_field, uint8_t *maskBit) {
    uint8_t reg_addr = (bit_field < INTM_GLBL1_GPI1_FM) ? INTM_GLBL0 : INTM_GLBL1;
    int ret;

    if (reg_addr == INTM_GLBL0) {
        reg_intm_glbl0_t reg;
        ret = max77643_read_register(obj, INTM_GLBL0, (uint8_t *)&reg);
        if (ret != MAX77643_NO_ERROR) return ret;
        switch (bit_field) {
            case INTM_GLBL0_GPI0_FM: *maskBit = reg.bits.gpi0_fm; break;
            case INTM_GLBL0_GPI0_RM: *maskBit = reg.bits.gpi0_rm; break;
            case INTM_GLBL0_nEN_FM: *maskBit = reg.bits.nen_fm; break;
            case INTM_GLBL0_nEN_RM: *maskBit = reg.bits.nen_rm; break;
            case INTM_GLBL0_TJAL1_RM: *maskBit = reg.bits.tjal1_rm; break;
            case INTM_GLBL0_TJAL2_RM: *maskBit = reg.bits.tjal2_rm; break;
            case INTM_GLBL0_DOD_RM: *maskBit = reg.bits.dod_rm; break;
            default: return MAX77643_INVALID_DATA;
        }
    } else {
        reg_intm_glbl1_t reg;
        ret = max77643_read_register(obj, INTM_GLBL1, (uint8_t *)&reg);
        if (ret != MAX77643_NO_ERROR) return ret;
        switch (bit_field) {
            case INTM_GLBL1_GPI1_FM: *maskBit = reg.bits.gpi1_fm; break;
            case INTM_GLBL1_GPI1_RM: *maskBit = reg.bits.gpi1_rm; break;
            case INTM_GLBL1_SBB0_FM: *maskBit = reg.bits.sbb0_fm; break;
            case INTM_GLBL1_SBB1_FM: *maskBit = reg.bits.sbb1_fm; break;
            case INTM_GLBL1_SBB2_FM: *maskBit = reg.bits.sbb2_fm; break;
            case INTM_GLBL1_LDO_M: *maskBit = reg.bits.ldo_m; break;
            default: return MAX77643_INVALID_DATA;
        }
    }
    return MAX77643_NO_ERROR;
}

// 设置 CNFG_GLBL0
int max77643_set_cnfg_glbl0(max77643_t *obj, reg_bit_cnfg_glbl0_t bit_field, uint8_t config) {
    reg_cnfg_glbl0_t reg;
    int ret = max77643_read_register(obj, CNFG_GLBL0, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_GLBL0_SFT_CTRL: reg.bits.sft_ctrl = config; break;
        case CNFG_GLBL0_DBEN_nEN: reg.bits.dben_nen = config; break;
        case CNFG_GLBL0_nEN_MODE: reg.bits.nen_mode = config; break;
        case CNFG_GLBL0_SBIA_LPM: reg.bits.sbia_lpm = config; break;
        case CNFG_GLBL0_T_MRST: reg.bits.t_mrst = config; break;
        case CNFG_GLBL0_PU_DIS: reg.bits.pu_dis = config; break;
        default: return MAX77643_INVALID_DATA;
    }
    return max77643_write_register(obj, CNFG_GLBL0, (uint8_t *)&reg);
}

// 获取 CNFG_GLBL0
int max77643_get_cnfg_glbl0(max77643_t *obj, reg_bit_cnfg_glbl0_t bit_field, uint8_t *config) {
    reg_cnfg_glbl0_t reg;
    int ret = max77643_read_register(obj, CNFG_GLBL0, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_GLBL0_SFT_CTRL: *config = reg.bits.sft_ctrl; break;
        case CNFG_GLBL0_DBEN_nEN: *config = reg.bits.dben_nen; break;
        case CNFG_GLBL0_nEN_MODE: *config = reg.bits.nen_mode; break;
        case CNFG_GLBL0_SBIA_LPM: *config = reg.bits.sbia_lpm; break;
        case CNFG_GLBL0_T_MRST: *config = reg.bits.t_mrst; break;
        case CNFG_GLBL0_PU_DIS: *config = reg.bits.pu_dis; break;
        default: return MAX77643_INVALID_DATA;
    }
    return MAX77643_NO_ERROR;
}

// 设置 CNFG_GLBL1
int max77643_set_cnfg_glbl1(max77643_t *obj, reg_bit_cnfg_glbl1_t bit_field, uint8_t config) {
    reg_cnfg_glbl1_t reg;
    int ret = max77643_read_register(obj, CNFG_GLBL1, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_GLBL1_AUTO_WKT: reg.bits.auto_wkt = config; break;
        case CNFG_GLBL1_SBB_F_SHUTDN: reg.bits.sbb_f_shutdn = config; break;
        default: return MAX77643_INVALID_DATA;
    }
    return max77643_write_register(obj, CNFG_GLBL1, (uint8_t *)&reg);
}

// 获取 CNFG_GLBL1
int max77643_get_cnfg_glbl1(max77643_t *obj, reg_bit_cnfg_glbl1_t bit_field, uint8_t *config) {
    reg_cnfg_glbl1_t reg;
    int ret = max77643_read_register(obj, CNFG_GLBL1, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_GLBL1_AUTO_WKT: *config = reg.bits.auto_wkt; break;
        case CNFG_GLBL1_SBB_F_SHUTDN: *config = reg.bits.sbb_f_shutdn; break;
        default: return MAX77643_INVALID_DATA;
    }
    return MAX77643_NO_ERROR;
}

// 设置 CNFG_GPIO
int max77643_set_cnfg_gpio(max77643_t *obj, reg_bit_cnfg_gpio_t bit_field, uint8_t channel, uint8_t config) {
    uint8_t reg_addr = (channel == 0) ? CNFG_GPIO0 : CNFG_GPIO1;
    reg_cnfg_gpio0_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_GPIO_DIR: reg.bits.gpo_dir = config; break;
        case CNFG_GPIO_DI: reg.bits.gpo_di = config; break;
        case CNFG_GPIO_DRV: reg.bits.gpo_drv = config; break;
        case CNFG_GPIO_DO: reg.bits.gpo_do = config; break;
        case CNFG_GPIO_DBEN_GPI: reg.bits.dben_gpi = config; break;
        case CNFG_GPIO_ALT_GPIO: reg.bits.alt_gpio = config; break;
        default: return MAX77643_INVALID_DATA;
    }
    return max77643_write_register(obj, reg_addr, (uint8_t *)&reg);
}

// 获取 CNFG_GPIO
int max77643_get_cnfg_gpio(max77643_t *obj, reg_bit_cnfg_gpio_t bit_field, uint8_t channel, uint8_t *config) {
    uint8_t reg_addr = (channel == 0) ? CNFG_GPIO0 : CNFG_GPIO1;
    reg_cnfg_gpio0_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_GPIO_DIR: *config = reg.bits.gpo_dir; break;
        case CNFG_GPIO_DI: *config = reg.bits.gpo_di; break;
        case CNFG_GPIO_DRV: *config = reg.bits.gpo_drv; break;
        case CNFG_GPIO_DO: *config = reg.bits.gpo_do; break;
        case CNFG_GPIO_DBEN_GPI: *config = reg.bits.dben_gpi; break;
        case CNFG_GPIO_ALT_GPIO: *config = reg.bits.alt_gpio; break;
        default: return MAX77643_INVALID_DATA;
    }
    return MAX77643_NO_ERROR;
}

// 获取 CID
int max77643_get_cid(max77643_t *obj) {
    uint8_t cid;
    int ret = max77643_read_register(obj, CID, &cid);
    if (ret != MAX77643_NO_ERROR) return ret;
    return cid;
}

// 设置 CNFG_WDT
int max77643_set_cnfg_wdt(max77643_t *obj, reg_bit_cnfg_wdt_t bit_field, uint8_t config) {
    reg_cnfg_wdt_t reg;
    int ret = max77643_read_register(obj, CNFG_WDT, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_WDT_WDT_LOCK: reg.bits.wdt_lock = config; break;
        case CNFG_WDT_WDT_EN: reg.bits.wdt_en = config; break;
        case CNFG_WDT_WDT_CLR: reg.bits.wdt_clr = config; break;
        case CNFG_WDT_WDT_MODE: reg.bits.wdt_mode = config; break;
        case CNFG_WDT_WDT_PER: reg.bits.wdt_per = config; break;
        default: return MAX77643_INVALID_DATA;
    }
    return max77643_write_register(obj, CNFG_WDT, (uint8_t *)&reg);
}

// 获取 CNFG_WDT
int max77643_get_cnfg_wdt(max77643_t *obj, reg_bit_cnfg_wdt_t bit_field, uint8_t *config) {
    reg_cnfg_wdt_t reg;
    int ret = max77643_read_register(obj, CNFG_WDT, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_WDT_WDT_LOCK: *config = reg.bits.wdt_lock; break;
        case CNFG_WDT_WDT_EN: *config = reg.bits.wdt_en; break;
        case CNFG_WDT_WDT_CLR: *config = reg.bits.wdt_clr; break;
        case CNFG_WDT_WDT_MODE: *config = reg.bits.wdt_mode; break;
        case CNFG_WDT_WDT_PER: *config = reg.bits.wdt_per; break;
        default: return MAX77643_INVALID_DATA;
    }
    return MAX77643_NO_ERROR;
}

// 设置 CNFG_SBB_TOP
int max77643_set_cnfg_sbb_top(max77643_t *obj, reg_bit_cnfg_sbb_top_t bit_field, uint8_t config) {
    reg_cnfg_sbb_top_t reg;
    int ret = max77643_read_register(obj, CNFG_SBB_TOP, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_SBB_TOP_DRV_SBB: reg.bits.drv_sbb = config; break;
        case CNFG_SBB_TOP_DIS_LPM: reg.bits.dis_lpm = config; break;
        default: return MAX77643_INVALID_DATA;
    }
    return max77643_write_register(obj, CNFG_SBB_TOP, (uint8_t *)&reg);
}

// 获取 CNFG_SBB_TOP
int max77643_get_cnfg_sbb_top(max77643_t *obj, reg_bit_cnfg_sbb_top_t bit_field, uint8_t *config) {
    reg_cnfg_sbb_top_t reg;
    int ret = max77643_read_register(obj, CNFG_SBB_TOP, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    switch (bit_field) {
        case CNFG_SBB_TOP_DRV_SBB: *config = reg.bits.drv_sbb; break;
        case CNFG_SBB_TOP_DIS_LPM: *config = reg.bits.dis_lpm; break;
        default: return MAX77643_INVALID_DATA;
    }
    return MAX77643_NO_ERROR;
}

// 设置 TV_SBB
int max77643_set_tv_sbb(max77643_t *obj, uint8_t channel, float voltV) {
    uint8_t value;
    float voltmV = voltV * 1000;
    if (voltmV < 500) voltmV = 500;
    else if (voltmV > 5500) voltmV = 5500;
    value = (voltmV - 500) / 25;

    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_A; break;
        case 1: reg_addr = CNFG_SBB1_A; break;
        case 2: reg_addr = CNFG_SBB2_A; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_a_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    reg.bits.tv_sbb0 = value;  // 所有通道的 TV_SBBx 字段名相同
    return max77643_write_register(obj, reg_addr, (uint8_t *)&reg);
}

// 获取 TV_SBB
int max77643_get_tv_sbb(max77643_t *obj, uint8_t channel, float *voltV) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_A; break;
        case 1: reg_addr = CNFG_SBB1_A; break;
        case 2: reg_addr = CNFG_SBB2_A; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_a_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    uint8_t bit_value = reg.bits.tv_sbb0;  // 所有通道的 TV_SBBx 字段名相同
    if (bit_value > 200) bit_value = 200;
    *voltV = (bit_value * 0.025f) + 0.5f;
    return MAX77643_NO_ERROR;
}

// 设置 OP_MODE
int max77643_set_op_mode(max77643_t *obj, uint8_t channel, decode_op_mode_t mode) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_B; break;
        case 1: reg_addr = CNFG_SBB1_B; break;
        case 2: reg_addr = CNFG_SBB2_B; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_b_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    reg.bits.op_mode0 = mode;  // 所有通道的 OP_MODE 字段名相同
    return max77643_write_register(obj, reg_addr, (uint8_t *)&reg);
}

// 获取 OP_MODE
int max77643_get_op_mode(max77643_t *obj, uint8_t channel, decode_op_mode_t *mode) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_B; break;
        case 1: reg_addr = CNFG_SBB1_B; break;
        case 2: reg_addr = CNFG_SBB2_B; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_b_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    *mode = (decode_op_mode_t)reg.bits.op_mode0;  // 所有通道的 OP_MODE 字段名相同
    return MAX77643_NO_ERROR;
}

// 设置 IP_SBB
int max77643_set_ip_sbb(max77643_t *obj, uint8_t channel, decode_ip_sbb_t ip_sbb) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_B; break;
        case 1: reg_addr = CNFG_SBB1_B; break;
        case 2: reg_addr = CNFG_SBB2_B; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_b_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    reg.bits.ip_sbb0 = ip_sbb;  // 所有通道的 IP_SBBx 字段名相同
    return max77643_write_register(obj, reg_addr, (uint8_t *)&reg);
}

// 获取 IP_SBB
int max77643_get_ip_sbb(max77643_t *obj, uint8_t channel, decode_ip_sbb_t *ip_sbb) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_B; break;
        case 1: reg_addr = CNFG_SBB1_B; break;
        case 2: reg_addr = CNFG_SBB2_B; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_b_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    *ip_sbb = (decode_ip_sbb_t)reg.bits.ip_sbb0;  // 所有通道的 IP_SBBx 字段名相同
    return MAX77643_NO_ERROR;
}

// 设置 ADE_SBB
int max77643_set_ade_sbb(max77643_t *obj, uint8_t channel, decode_ade_sbb_t ade_sbb) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_B; break;
        case 1: reg_addr = CNFG_SBB1_B; break;
        case 2: reg_addr = CNFG_SBB2_B; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_b_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    reg.bits.ade_sbb0 = ade_sbb;  // 所有通道的 ADE_SBBx 字段名相同
    return max77643_write_register(obj, reg_addr, (uint8_t *)&reg);
}

// 获取 ADE_SBB
int max77643_get_ade_sbb(max77643_t *obj, uint8_t channel, decode_ade_sbb_t *ade_sbb) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_B; break;
        case 1: reg_addr = CNFG_SBB1_B; break;
        case 2: reg_addr = CNFG_SBB2_B; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_b_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    *ade_sbb = (decode_ade_sbb_t)reg.bits.ade_sbb0;  // 所有通道的 ADE_SBBx 字段名相同
    return MAX77643_NO_ERROR;
}

// 设置 EN_SBB
int max77643_set_en_sbb(max77643_t *obj, uint8_t channel, decode_en_sbb_t en_sbb) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_B; break;
        case 1: reg_addr = CNFG_SBB1_B; break;
        case 2: reg_addr = CNFG_SBB2_B; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_b_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    reg.bits.en_sbb0 = en_sbb;  // 所有通道的 EN_SBBx 字段名相同
    return max77643_write_register(obj, reg_addr, (uint8_t *)&reg);
}

// 获取 EN_SBB
int max77643_get_en_sbb(max77643_t *obj, uint8_t channel, decode_en_sbb_t *en_sbb) {
    uint8_t reg_addr;
    switch (channel) {
        case 0: reg_addr = CNFG_SBB0_B; break;
        case 1: reg_addr = CNFG_SBB1_B; break;
        case 2: reg_addr = CNFG_SBB2_B; break;
        default: return MAX77643_INVALID_DATA;
    }
    reg_cnfg_sbb0_b_t reg;
    int ret = max77643_read_register(obj, reg_addr, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    *en_sbb = (decode_en_sbb_t)reg.bits.en_sbb0;  // 所有通道的 EN_SBBx 字段名相同
    return MAX77643_NO_ERROR;
}

// 设置 TV_SBB_DVS
int max77643_set_tv_sbb_dvs(max77643_t *obj, float voltV) {
    uint8_t value;
    float voltmV = voltV * 1000;
    if (voltmV < 500) voltmV = 500;
    else if (voltmV > 5500) voltmV = 5500;
    value = (voltmV - 500) / 25;
    SET_BIT_FIELD(obj, CNFG_DVS_SBB0_A, reg_cnfg_dvs_sbb0_a_t, tv_sbb0_dvs, value);
    return MAX77643_NO_ERROR;
}

// 获取 TV_SBB_DVS
int max77643_get_tv_sbb_dvs(max77643_t *obj, float *voltV) {
    reg_cnfg_dvs_sbb0_a_t reg;
    int ret = max77643_read_register(obj, CNFG_DVS_SBB0_A, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    uint8_t bit_value = reg.bits.tv_sbb0_dvs;
    if (bit_value > 200) bit_value = 200;
    *voltV = (bit_value * 0.025f) + 0.5f;
    return MAX77643_NO_ERROR;
}

// 设置 TV_LDO
int max77643_set_tv_ldo(max77643_t *obj, float voltV) {
    reg_cnfg_ldo0_a_t reg;
    int ret = max77643_read_register(obj, CNFG_LDO0_A, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;

    float voltmV = voltV * 1000;
    float lower_limit_voltmV = (reg.bits.tv_ofs_ldo == 0) ? 500 : 1825;
    float upper_limit_voltmV = (reg.bits.tv_ofs_ldo == 0) ? 3675 : 5000;
    float incrementmV = 25;

    if (voltmV < lower_limit_voltmV) voltmV = lower_limit_voltmV;
    else if (voltmV > upper_limit_voltmV) voltmV = upper_limit_voltmV;

    uint8_t value = (voltmV - lower_limit_voltmV) / incrementmV;
    reg.bits.tv_ldo0 = value;
    return max77643_write_register(obj, CNFG_LDO0_A, (uint8_t *)&reg);
}

// 获取 TV_LDO
int max77643_get_tv_ldo(max77643_t *obj, float *voltV) {
    reg_cnfg_ldo0_a_t reg;
    int ret = max77643_read_register(obj, CNFG_LDO0_A, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;

    float lower_limitV = (reg.bits.tv_ofs_ldo == 0) ? 0.5f : 1.825f;
    float incrementV = 0.025f;
    *voltV = lower_limitV + (reg.bits.tv_ldo0 * incrementV);
    return MAX77643_NO_ERROR;
}

// 设置 TV_OFS_LDO
int max77643_set_tv_ofs_ldo(max77643_t *obj, decode_tv_ofs_ldo_t offset) {
    SET_BIT_FIELD(obj, CNFG_LDO0_A, reg_cnfg_ldo0_a_t, tv_ofs_ldo, offset);
    return MAX77643_NO_ERROR;
}

// 获取 TV_OFS_LDO
int max77643_get_tv_ofs_ldo(max77643_t *obj, decode_tv_ofs_ldo_t *offset) {
    reg_cnfg_ldo0_a_t reg;
    int ret = max77643_read_register(obj, CNFG_LDO0_A, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    *offset = (decode_tv_ofs_ldo_t)reg.bits.tv_ofs_ldo;
    return MAX77643_NO_ERROR;
}

// 设置 EN_LDO
int max77643_set_en_ldo(max77643_t *obj, decode_en_ldo_t en_ldo) {
    SET_BIT_FIELD(obj, CNFG_LDO0_B, reg_cnfg_ldo0_b_t, en_ldo, en_ldo);
    return MAX77643_NO_ERROR;
}

// 获取 EN_LDO
int max77643_get_en_ldo(max77643_t *obj, decode_en_ldo_t *en_ldo) {
    reg_cnfg_ldo0_b_t reg;
    int ret = max77643_read_register(obj, CNFG_LDO0_B, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    *en_ldo = (decode_en_ldo_t)reg.bits.en_ldo;
    return MAX77643_NO_ERROR;
}

// 设置 ADE_LDO
int max77643_set_ade_ldo(max77643_t *obj, decode_ade_ldo_t ade_ldo) {
    SET_BIT_FIELD(obj, CNFG_LDO0_B, reg_cnfg_ldo0_b_t, ade_ldo, ade_ldo);
    return MAX77643_NO_ERROR;
}

// 获取 ADE_LDO
int max77643_get_ade_ldo(max77643_t *obj, decode_ade_ldo_t *ade_ldo) {
    reg_cnfg_ldo0_b_t reg;
    int ret = max77643_read_register(obj, CNFG_LDO0_B, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    *ade_ldo = (decode_ade_ldo_t)reg.bits.ade_ldo;
    return MAX77643_NO_ERROR;
}

// 设置 LDO_MD
int max77643_set_ldo_md(max77643_t *obj, decode_ldo_md_t mode) {
    SET_BIT_FIELD(obj, CNFG_LDO0_B, reg_cnfg_ldo0_b_t, ldo_md, mode);
    return MAX77643_NO_ERROR;
}

// 获取 LDO_MD
int max77643_get_ldo_md(max77643_t *obj, decode_ldo_md_t *mode) {
    reg_cnfg_ldo0_b_t reg;
    int ret = max77643_read_register(obj, CNFG_LDO0_B, (uint8_t *)&reg);
    if (ret != MAX77643_NO_ERROR) return ret;
    *mode = (decode_ldo_md_t)reg.bits.ldo_md;
    return MAX77643_NO_ERROR;
}

// 禁用所有中断
int max77643_irq_disable_all(max77643_t *obj) {
    uint8_t zero = 0;
    int ret = max77643_write_register(obj, INTM_GLBL0, &zero);
    if (ret != MAX77643_NO_ERROR) return ret;
    ret = max77643_write_register(obj, INTM_GLBL1, &zero);
    if (ret != MAX77643_NO_ERROR) return ret;
    return MAX77643_NO_ERROR;
}