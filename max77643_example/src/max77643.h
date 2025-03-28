#ifndef MAX77643_H
#define MAX77643_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

// 错误码定义
#define MAX77643_NO_ERROR          0
#define MAX77643_VALUE_NULL       -1
#define MAX77643_WRITE_DATA_FAILED -2
#define MAX77643_READ_DATA_FAILED  -3
#define MAX77643_INVALID_DATA     -4

// I2C 地址
#define MAX77643_I2C_ADDRESS       0x90

// 设备结构体
typedef struct {
    const struct i2c_dt_spec *i2c_spec;  // I2C 设备规范
    uint32_t irq_pin;                    // 中断引脚（可选）
} max77643_t;

// 寄存器地址枚举（从 MAX77643_2.h 转换）
typedef enum {
    INT_GLBL0     = 0x00,  // 中断状态 0
    INT_GLBL1     = 0x01,  // 中断状态 1
    ERCFLAG       = 0x02,  // 事件记录标志
    STAT_GLBL     = 0x03,  // 全局状态
    INTM_GLBL0    = 0x04,  // 中断掩码 0
    INTM_GLBL1    = 0x05,  // 中断掩码 1
    CNFG_GLBL0    = 0x06,  // 全局配置 0
    CNFG_GLBL1    = 0x07,  // 全局配置 1
    CNFG_GPIO0    = 0x08,  // GPIO0 配置
    CNFG_GPIO1    = 0x09,  // GPIO1 配置
    CID           = 0x10,  // 芯片识别码
    CNFG_WDT      = 0x17,  // 看门狗定时器配置
    CNFG_SBB_TOP  = 0x28,  // SIMO Buck-Boost 配置
    CNFG_SBB0_A   = 0x29,  // SIMO Buck-Boost 0 配置 A
    CNFG_SBB0_B   = 0x2A,  // SIMO Buck-Boost 0 配置 B
    CNFG_SBB1_A   = 0x2B,  // SIMO Buck-Boost 1 配置 A
    CNFG_SBB1_B   = 0x2C,  // SIMO Buck-Boost 1 配置 B
    CNFG_SBB2_A   = 0x2D,  // SIMO Buck-Boost 2 配置 A
    CNFG_SBB2_B   = 0x2E,  // SIMO Buck-Boost 2 配置 B
    CNFG_DVS_SBB0_A = 0x2F,// SIMO Buck-Boost 0 DVS 配置 A
    CNFG_LDO0_A   = 0x38,  // LDO0 输出电压
    CNFG_LDO0_B   = 0x39   // LDO0 输出电压配置
} max77643_reg_t;

// 从 MAX77643_2.h 转换的枚举类型
typedef enum {
    INT_GLBL0_GPI0_F, INT_GLBL0_GPI0_R, INT_GLBL0_NEN_F, INT_GLBL0_NEN_R,
    INT_GLBL0_TJAL1_R, INT_GLBL0_TJAL2_R, INT_GLBL0_DOD_R, INT_GLBL0_RSVD,
    INT_GLBL1_GPI1_F, INT_GLBL1_GPI1_R, INT_GLBL1_SBB0_F, INT_GLBL1_SBB1_F,
    INT_GLBL1_SBB2_F, INT_GLBL1_LDO_F, INT_GLBL1_RSVD, INT_CHG_END
} reg_bit_int_glbl_t;

typedef enum {
    ERCFLAG_TOVLD, ERCFLAG_INOVLO, ERCFLAG_INUVLO, ERCFLAG_MRST_F,
    ERCFLAG_SFT_OFF_F, ERCFLAG_SFT_CRST_F, ERCFLAG_WDT_EXP_F, ERCFLAG_SBB_FAULT_F
} reg_bit_ercflag_t;

typedef enum {
    STAT_GLBL_STAT_IRQ, STAT_GLBL_STAT_EN, STAT_GLBL_TJAL1_S, STAT_GLBL_TJAL2_S,
    STAT_GLBL_DOD_S, STAT_GLBL_RSVD, STAT_GLBL_BOK, STAT_GLBL_DIDM
} reg_bit_stat_glbl_t;

typedef enum {
    INTM_GLBL0_GPI0_FM, INTM_GLBL0_GPI0_RM, INTM_GLBL0_nEN_FM, INTM_GLBL0_nEN_RM,
    INTM_GLBL0_TJAL1_RM, INTM_GLBL0_TJAL2_RM, INTM_GLBL0_DOD_RM, INTM_GLBL0_RSVD,
    INTM_GLBL1_GPI1_FM, INTM_GLBL1_GPI1_RM, INTM_GLBL1_SBB0_FM, INTM_GLBL1_SBB1_FM,
    INTM_GLBL1_SBB2_FM, INTM_GLBL1_LDO_M, INTM_GLBL1_RSVD, INTM_NUM_OF_BIT
} reg_bit_int_mask_t;

typedef enum {
    CNFG_GLBL0_SFT_CTRL, CNFG_GLBL0_DBEN_nEN, CNFG_GLBL0_nEN_MODE,
    CNFG_GLBL0_SBIA_LPM, CNFG_GLBL0_T_MRST, CNFG_GLBL0_PU_DIS
} reg_bit_cnfg_glbl0_t;

typedef enum {
    CNFG_GLBL1_AUTO_WKT, CNFG_GLBL1_SBB_F_SHUTDN, CNFG_GLBL1_RSVD
} reg_bit_cnfg_glbl1_t;

typedef enum {
    CNFG_GPIO_DIR, CNFG_GPIO_DI, CNFG_GPIO_DRV, CNFG_GPIO_DO,
    CNFG_GPIO_DBEN_GPI, CNFG_GPIO_ALT_GPIO, CNFG_GPIO_RSVD
} reg_bit_cnfg_gpio_t;

typedef enum {
    CNFG_WDT_WDT_LOCK, CNFG_WDT_WDT_EN, CNFG_WDT_WDT_CLR,
    CNFG_WDT_WDT_MODE, CNFG_WDT_WDT_PER, CNFG_WDT_RSVD
} reg_bit_cnfg_wdt_t;

typedef enum {
    CNFG_SBB_TOP_DRV_SBB, CNFG_SBB_TOP_DIS_LPM
} reg_bit_cnfg_sbb_top_t;

typedef enum {
    OP_MODE_AUTOMATIC, OP_MODE_BUCK_MODE, OP_MODE_BOOST_MODE, OP_MODE_BUCK_BOOST_MODE
} decode_op_mode_t;

typedef enum {
    IP_SBB_AMP_1_000A, IP_SBB_AMP_0_750A, IP_SBB_AMP_0_500A, IP_SBB_AMP_0_333A
} decode_ip_sbb_t;

typedef enum {
    ADE_SBB_DISABLED, ADE_SBB_ENABLED
} decode_ade_sbb_t;

typedef enum {
    EN_SBB_FPS_SLOT_0, EN_SBB_FPS_SLOT_1, EN_SBB_FPS_SLOT_2, EN_SBB_FPS_SLOT_3,
    EN_SBB_OFF, EN_SBB_SAME_AS_0X04, EN_SBB_ON, EN_SBB_SAME_AS_0X06
} decode_en_sbb_t;

typedef enum {
    TV_OFS_LDO_NO_OFFSET, TV_OFS_LDO_NO_1_325V
} decode_tv_ofs_ldo_t;

typedef enum {
    EN_LDO_FPS_SLOT_0, EN_LDO_FPS_SLOT_1, EN_LDO_FPS_SLOT_2, EN_LDO_FPS_SLOT_3,
    EN_LDO_OFF, EN_LDO_SAME_AS_0X04, EN_LDO_ON, EN_LDO_SAME_AS_0X06
} decode_en_ldo_t;

typedef enum {
    ADE_LDO_DISABLED, ADE_LDO_ENABLED
} decode_ade_ldo_t;

typedef enum {
    LDO_MD_LDO_MODE, LDO_MD_LSW_MODE
} decode_ldo_md_t;

// 函数声明
int max77643_init(max77643_t *obj, const struct i2c_dt_spec *i2c_spec, uint32_t irq_pin);
int max77643_read_register(max77643_t *obj, uint8_t reg, uint8_t *value);
int max77643_write_register(max77643_t *obj, uint8_t reg, const uint8_t *value);

int max77643_get_ercflag(max77643_t *obj, reg_bit_ercflag_t bit_field, uint8_t *flag);
int max77643_get_stat_glbl(max77643_t *obj, reg_bit_stat_glbl_t bit_field, uint8_t *status);
int max77643_set_interrupt_mask(max77643_t *obj, reg_bit_int_mask_t bit_field, uint8_t maskBit);
int max77643_get_interrupt_mask(max77643_t *obj, reg_bit_int_mask_t bit_field, uint8_t *maskBit);
int max77643_set_cnfg_glbl0(max77643_t *obj, reg_bit_cnfg_glbl0_t bit_field, uint8_t config);
int max77643_get_cnfg_glbl0(max77643_t *obj, reg_bit_cnfg_glbl0_t bit_field, uint8_t *config);
int max77643_set_cnfg_glbl1(max77643_t *obj, reg_bit_cnfg_glbl1_t bit_field, uint8_t config);
int max77643_get_cnfg_glbl1(max77643_t *obj, reg_bit_cnfg_glbl1_t bit_field, uint8_t *config);
int max77643_set_cnfg_gpio(max77643_t *obj, reg_bit_cnfg_gpio_t bit_field, uint8_t channel, uint8_t config);
int max77643_get_cnfg_gpio(max77643_t *obj, reg_bit_cnfg_gpio_t bit_field, uint8_t channel, uint8_t *config);
int max77643_get_cid(max77643_t *obj);
int max77643_set_cnfg_wdt(max77643_t *obj, reg_bit_cnfg_wdt_t bit_field, uint8_t config);
int max77643_get_cnfg_wdt(max77643_t *obj, reg_bit_cnfg_wdt_t bit_field, uint8_t *config);
int max77643_set_cnfg_sbb_top(max77643_t *obj, reg_bit_cnfg_sbb_top_t bit_field, uint8_t config);
int max77643_get_cnfg_sbb_top(max77643_t *obj, reg_bit_cnfg_sbb_top_t bit_field, uint8_t *config);
int max77643_set_tv_sbb(max77643_t *obj, uint8_t channel, float voltV);
int max77643_get_tv_sbb(max77643_t *obj, uint8_t channel, float *voltV);
int max77643_set_op_mode(max77643_t *obj, uint8_t channel, decode_op_mode_t mode);
int max77643_get_op_mode(max77643_t *obj, uint8_t channel, decode_op_mode_t *mode);
int max77643_set_ip_sbb(max77643_t *obj, uint8_t channel, decode_ip_sbb_t ip_sbb);
int max77643_get_ip_sbb(max77643_t *obj, uint8_t channel, decode_ip_sbb_t *ip_sbb);
int max77643_set_ade_sbb(max77643_t *obj, uint8_t channel, decode_ade_sbb_t ade_sbb);
int max77643_get_ade_sbb(max77643_t *obj, uint8_t channel, decode_ade_sbb_t *ade_sbb);
int max77643_set_en_sbb(max77643_t *obj, uint8_t channel, decode_en_sbb_t en_sbb);
int max77643_get_en_sbb(max77643_t *obj, uint8_t channel, decode_en_sbb_t *en_sbb);
int max77643_set_tv_sbb_dvs(max77643_t *obj, float voltV);
int max77643_get_tv_sbb_dvs(max77643_t *obj, float *voltV);
int max77643_set_tv_ldo(max77643_t *obj, float voltV);
int max77643_get_tv_ldo(max77643_t *obj, float *voltV);
int max77643_set_tv_ofs_ldo(max77643_t *obj, decode_tv_ofs_ldo_t offset);
int max77643_get_tv_ofs_ldo(max77643_t *obj, decode_tv_ofs_ldo_t *offset);
int max77643_set_en_ldo(max77643_t *obj, decode_en_ldo_t en_ldo);
int max77643_get_en_ldo(max77643_t *obj, decode_en_ldo_t *en_ldo);
int max77643_set_ade_ldo(max77643_t *obj, decode_ade_ldo_t ade_ldo);
int max77643_get_ade_ldo(max77643_t *obj, decode_ade_ldo_t *ade_ldo);
int max77643_set_ldo_md(max77643_t *obj, decode_ldo_md_t mode);
int max77643_get_ldo_md(max77643_t *obj, decode_ldo_md_t *mode);
int max77643_irq_disable_all(max77643_t *obj);

#endif // MAX77643_H