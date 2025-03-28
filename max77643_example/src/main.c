#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "max77643.h"

// 从设备树获取I2C配置
#define MAX77643_NODE DT_NODELABEL(max77643)
static const struct i2c_dt_spec dev_max77643 = I2C_DT_SPEC_GET(MAX77643_NODE);

int main(void) {
    max77643_t device;  // MAX77643设备结构体
    int ret;            // 返回值，用于错误检查

    // 初始化MAX77643设备
    while(true){
        ret = max77643_init(&device, &dev_max77643, 0);  // 0表示不使用中断引脚
        if (ret != MAX77643_NO_ERROR) {
            printk("MAX77643初始化失败: %d\n", ret);
            return ret;
        }
        printk("MAX77643 initialized successfully\n");
        // 设置SBB0电压为3V
        ret = max77643_set_tv_sbb(&device, 0, 1.5f);  // 通道0，电压3.0V
        if (ret == MAX77643_NO_ERROR) {
            printk("SBB0电压已设置为3.0V\n");
            break;
        } else {
            printk("set SBB0 output 3V failure: %d\n", ret);
        }
        k_sleep(K_MSEC(100));
    }
    printk("SBB0 set successfully\n");
    return 0;
}