/*******************************************************************************
 * Copyright(C) Analog Devices Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files(the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Analog Devices Inc.
 * shall not be used except as stated in the Analog Devices Inc.
 * Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Analog Devices Inc.retains all ownership rights.
 *******************************************************************************
*/

#ifndef MAX77643_2_REGS_H_
#define MAX77643_2_REGS_H_

/**
 * @brief INT_GLBL0 Register
 *
 * Address : 0x00
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char gpi0_f    : 1;    /**< GPI Falling Interrupt. Bit 0.
                                            Note that "GPI" refers to the GPIO programmed to be an input.
                                            0 = No GPI falling edges have occurred since the last time this bit was read.
                                            1 = A GPI falling edge has occurred since the last time this bit was read. */
        unsigned char gpi0_r    : 1;    /**< GPI Rising Interrupt. Bit 1. 
                                            Note that "GPI" refers to the GPIO programmed to be an input.
                                            0 = No GPI rising edges have occurred since the last time this bit was read. 
                                            1 = A GPI rising edge has occurred since the last time this bit was read. */
        unsigned char nen_f     : 1;    /**< nEN Falling Interrupt.Bit 2.
                                            0 = No nEN falling edges have occurred since the last time this bit was read.
                                            1 = A nEN falling edge as occurred since the last time this bit was read. */
        unsigned char nen_r     : 1;    /**< nEN Rising Interrupt. Bit 3.
                                            0 = No nEN rising edges have occurred since the last time this bit was read.
                                            1 = A nEN rising edge as occurred since the last time this bit was read. */
        unsigned char tjal1_r   : 1;    /**< Thermal Alarm 1 Rising Interrupt. Bit 4.
                                            0 = The junction temperature has not risen above TJAL1 since the last time this bit was read.
                                            1 = The junction temperature has risen above TJAL1 since the last time this bit was read. */
        unsigned char tjal2_r   : 1;    /**< Thermal Alarm 2 Rising Interrupt. Bit 5.
                                            0 = The junction temperature has not risen above TJAL2 since the last time this bit was read.
                                            1 = The junction temperature has risen above TJAL2 since the last time this bit was read. */
        unsigned char dod_r     : 1;    /**< LDO Dropout Detector Rising Interrupt. Bit 6.
                                            0 = The LDO has not detected dropout since the last time this bit was read.
                                            1 = The LDO has detected dropout since the last time this bit was read.  */
        unsigned char rsvd      : 1;    /**< Reserved. Unutilized bit. Write to 0. Reads are don't care. Bit 7. */
    } bits;
} reg_int_glbl0_t;

/**
 * @brief INT_GLBL1 Register
 *
 * Address : 0x01
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char gpi1_f    : 1;    /**< GPI Falling Interrupt. Bit 0.
                                            Note that "GPI" refers to the GPIO programmed to be an input.
                                            0 = No GPI falling edges have occurred since the last time this bit was read.
                                            1 = A GPI falling edge has occurred since the last time this bit was read. */
        unsigned char gpi1_r    : 1;    /**< GPI Rising Interrupt. Bit 1.
                                            Note that "GPI" refers to the GPIO programmed to be an input.
                                            0 = No GPI rising edges have occurred since the last time this bit was read. 
                                            1 = A GPI rising edge has occurred since the last time this bit was read. */
        unsigned char sbb0_f    : 1;    /**< SBB0 Fault Indicator. Bit 2.
                                            0 = No fault has occurred on SBB0 since the last time this bit was read.
                                            1 = SBB0 has fallen out of regulation since the last time this bit was read. */
        unsigned char sbb1_f    : 1;    /**< SBB1 Fault Indicator. Bit 3.
                                            0 = No fault has occurred on SBB1 since the last time this bit was read.
                                            1 = SBB1 has fallen out of regulation since the last time this bit was read. */
        unsigned char sbb2_f    : 1;    /**< SBB2 Fault Indicator. Bit 4.
                                            0 = No fault has occurred on SBB2 since the last time this bit was read.
                                            1 = SBB2 has fallen out of regulation since the last time this bit was read. */
        unsigned char ldo_f     : 1;    /**< LDO0 Fault Interrupt. Bit 5.
                                            0 = No fault has occurred on LDO0 since the last time this bit was read.
                                            1 = LDO0 has fallen out of regulation since the last time this bit was read. */
        unsigned char rsvd      : 2;    /**< Reserved. Unutilized bit. Write to 0. Reads are don't care. Bit 7:6. */
    } bits;
} reg_int_glbl1_t;

/**
 * @brief ERCFLAG Register
 *
 * Address : 0x02
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char tovld         : 1;    /**< Thermal Overload. Bit 0.
                                                0 = Thermal overload has not occurred since the last read of this register.
                                                1 = Thermal overload has occurred since the list read of this register.
                                                This indicates that the junction temperature has exceeded 165ºC. */
        unsigned char inovlo        : 1;    /**< IN Domain Overvoltage Lockout. Bit 1.
                                                0 = The IN domain overvoltage lockout has not occurred since the last read of this register.
                                                1 = The IN domain overvoltage lockout has occurred since the last read of this register */
        unsigned char inuvlo        : 1;    /**< IN Domain Undervoltage Lockout. Bit 2.
                                                0 = The IN domain undervoltage lockout has not occurred since the last read of this register. 
                                                1 = The IN domain undervoltage lockout has occurred since the last read of this register */
        unsigned char mrst_f        : 1;    /**< Manual Reset Timer. Bit 3.
                                                0 = A Manual Reset has not occurred since this last read of this register.
                                                1 = A Manual Reset has occurred since this last read of this register. */
        unsigned char sft_off_f     : 1;    /**< Software Off Flag. Bit 4.
                                                0 = The SFT_OFF function has not occurred since the last read of this register.
                                                1 = The SFT_OFF function has occurred since the last read of this register. */
        unsigned char sft_crst_f    : 1;    /**< Software Cold Reset Flag. Bit 5.
                                                0 = The software cold reset has not occurred since the last read of this register.
                                                1 = The software cold reset has occurred since the last read of this register. */
        unsigned char wdt_exp_f     : 1;    /**< Watchdog Timer OFF or RESET Flag. Bit 6.
                                                This bit sets when the watchdog timer expires and causes a power-off or a reset; based on WDT_MODE bitfield setting. 
                                                0 = Watchdog timer has not caused a power-off or reset since the last time this bit was read.
                                                1 = Watchdog timer has expired and caused a power-off or reset since the last time this bit was read.  */
        unsigned char sbb_fault_f   : 1;    /**< SBBx Fault and Shutdown Flag. Bit 7.
                                                This bit sets when a SBBx fault and consequent SBBx shutdown occurs.  
                                                0 = No SBB shutdown occurred since the last time this bit was read.  
                                                1 = SBBx fault and SBB shutdown occurred since the last time this bit was read. */
    } bits;
} reg_ercflag_t;

/**
 * @brief STAT_GLBL Register
 *
 * Address : 0x03
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char stat_irq  : 1;    /**< Software Version of the nIRQ MOSFET gate drive. Bit 0.
                                            0 = unmasked gate drive is logic low 
                                            1 = unmasked gate drive is logic high */
        unsigned char stat_en   : 1;    /**< Debounced Status for the nEN input. Bit 1.
                                            0 = nEN is not active (logic high) 
                                            1 = nEN is active (logic low) */
        unsigned char tjal1_s   : 1;    /**< Thermal Alarm 1 Status. Bit 2.
                                            0 = The junction temperature is less than TJAL1 
                                            1 = The junction temperature is greater than TJAL1 */
        unsigned char tjal2_s   : 1;    /**< Thermal Alarm 2 Status. Bit 3.
                                            0 = The junction temperature is less than TJAL2 
                                            1 = The junction temperature is greater than TJAL2 */
        unsigned char dod_s     : 1;    /**< LDO1 Dropout Detector Rising Status. Bit 4.
                                            0 = LDO1 is not in dropout 
                                            1 = LDO1 is in dropout */
        unsigned char rsvd      : 1;    /**< Reserved. Unutilized bit. Write to 0. Reads are don't care. Bit 5. */
        unsigned char bok       : 1;    /**< BOK Interrupt Status. Bit 6.
                                            0 = Main Bias is not ready. 
                                            1 = Main Bias enabled and ready.  */
        unsigned char didm      : 1;    /**< Device Identification Bits for Metal Options. Bit 7.
                                            0 = MAX77643_2 
                                            1 = Reserved */
    } bits;
} reg_stat_glbl_t;

/**
 * @brief INTM_GLBL0 Register
 *
 * Address : 0x04
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char gpi0_fm   : 1;    /**< GPI Falling Interrupt Mask. Bit 0. 
                                            0 = Unmasked. If GPI_F goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to GPI_F. */
        unsigned char gpi0_rm   : 1;    /**< GPI Rising Interrupt Mask. Bit 1. 
                                            0 = Unmasked. If GPI_R goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to GPI_R. */
        unsigned char nen_fm    : 1;    /**< nEN Falling Interrupt Mask. Bit 2.
                                            0 = Unmasked. If nEN_F goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to nEN_F. */
        unsigned char nen_rm    : 1;    /**< nEN Rising Interrupt Mask. Bit 3.
                                            0 = Unmasked. If nEN_R goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to nEN_R. */
        unsigned char tjal1_rm  : 1;    /**< Thermal Alarm 1 Rising Interrupt Mask. Bit 4.
                                            0 = Unmasked. If TJAL1_R goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to TJAL1_R. */
        unsigned char tjal2_rm  : 1;    /**< Thermal Alarm 2 Rising Interrupt Mask. Bit 5.
                                            0 = Unmasked. If TJAL2_R goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to TJAL2_R. */
        unsigned char dod_rm    : 1;    /**< LDO Dropout Detector Rising Interrupt Mask. Bit 6.
                                            0 = Unmasked. If DOD1_R goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to DOD1_R.  */
        unsigned char rsvd      : 1;    /**< Reserved. Unutilized bit. Write to 0. Reads are don't care. Bit 7. */
    } bits;
} reg_intm_glbl0_t;

/**
 * @brief INTM_GLBL1 Register
 *
 * Address : 0x05
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char gpi1_fm   : 1;    /**< GPI Falling Interrupt Mask. Bit 0. 
                                            0 = Unmasked. If GPI_F goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to GPI_F. */
        unsigned char gpi1_rm   : 1;    /**< GPI Rising Interrupt Mask. Bit 1. 
                                            0 = Unmasked. If GPI_R goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared. 
                                            1 = Masked. nIRQ does not go low due to GPI_R. */
        unsigned char sbb0_fm  : 1;    /**< SBB0 Fault Interrupt Mask. Bit 2.
                                            0 = Unmasked. If SBB0_F goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared..
                                            1 = Masked. nIRQ does not go low due to SBB0_F. */
        unsigned char sbb1_fm  : 1;    /**< SBB1 Fault Interrupt Mask. Bit 3.
                                            0 = Unmasked. If SBB1_F goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared..
                                            1 = Masked. nIRQ does not go low due to SBB1_F. */
        unsigned char sbb2_fm  : 1;    /**< SBB2 Fault Interrupt Mask. Bit 4.
                                            0 = Unmasked. If SBB2_F goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared..
                                            1 = Masked. nIRQ does not go low due to SBB2_F. */
        unsigned char ldo_m     : 1;    /**< LDO0 Fault Interrupt. Bit 5.
                                            0 = Unmasked. If LDO0_F goes from 0 to 1, then nIRQ goes low. 
                                            nIRQ goes high when all interrupt bits are cleared.
                                            1 = Masked. nIRQ does not go low due to LDO0_F. */
        unsigned char rsvd      : 2;    /**< Reserved. Unutilized bit. Write to 0. Reads are don't care. Bit 7:6. */
    } bits;
} reg_intm_glbl1_t;

/**
 * @brief CNFG_GLBL0 Register
 *
 * Address : 0x06
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char sft_ctrl  : 2;    /**< Software Reset Functions. Bit 1:0. 
                                            0b00 = No Action 
                                            0b01 = Software Cold Reset (SFT_CRST). The device powers down, resets, and the powers up again. 
                                            0b10 = Software Off (SFT_OFF). The device powers down, resets, and then remains off and waiting for a wake-up event. 
                                            0b11 = Auto Wake Up (SFT_AUTO). */
        unsigned char dben_nen  : 1;    /**< Debounce Timer Enable for the nEN Pin. Bit 2.
                                            0 = 500μs Debounce 
                                            1 = 30ms Debounce */
        unsigned char nen_mode  : 2;    /**< nEN Input (ON-KEY) Default Configuration Mode. Bit 4:3.
                                            0b00 = Push-button mode 
                                            0b01 = Slide-switch mode 
                                            0b10 = Logic mode 
                                            0b11 = Reserved */
        unsigned char sbia_lpm  : 1;    /**< Main Bias Low-Power Mode Software Request. Bit 5.
                                            0 = Main Bias requested to be in Normal-Power Mode by software. 
                                            1 = Main Bias request to be in Low-Power Mode by software. */
        unsigned char t_mrst    : 1;    /**< Sets the Manual Reset Time (tMRST). Bit 6.
                                            0 = 8s 
                                            1 = 4s  */
        unsigned char pu_dis    : 1;    /**< nEN Internal Pullup Resistor. Bit 7.
                                            0 = Strong internal nEN pullup (200kΩ) 
                                            1 = Weak internal nEN pullup (10MΩ) */
    } bits;
} reg_cnfg_glbl0_t;

/**
 * @brief CNFG_GLBL1 Register
 *
 * Address : 0x07
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char auto_wkt      : 2;    /**< Auto Wake-Up Timer. Bit 1:0. 
                                                0b00 = 100ms Auto Wake-up Time
                                                0b01 = 200ms Auto Wake-up Time
                                                0b10 = 500ms Auto Wake-up Time
                                                0b11 = 1000ms Auto Wake-up Time */
        unsigned char sbb_f_shutdn  : 1;    /**< SBB Shutdown from SBB Fault. Bit 2.
                                                0 = 500μs Debounce 
                                                1 = 30ms Debounce */
        unsigned char rsvd          : 5;    /**< Reserved. Unutilized bit. Write to 0. Reads are don't care */
} bits;
} reg_cnfg_glbl1_t;

/**
 * @brief CNFG_GPIO0 Register
 *
 * Address : 0x08
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char gpo_dir   : 1;    /**< GPIO Direction. Bit 0. 
                                            0 = General purpose output (GPO) 
                                            1 = General purpose input (GPI) */
        unsigned char gpo_di    : 1;    /**< GPIO Digital Input Value. Bit 1.
                                            0 = Input logic low 
                                            1 = Input logic high */
        unsigned char gpo_drv   : 1;    /**< General Purpose Output Driver Type. Bit 2.
                                            This bit is a don't care when DIR = 1 (configured as input) When set for GPO (DIR = 0): 
                                            0 = Open-Drain 
                                            1 = Push-Pull */
        unsigned char gpo_do    : 1;    /**< General Purpose Output Data Output. Bit 3.
                                            This bit is a don't care when DIR = 1 (configured as input). When set for GPO (DIR = 0): 
                                            0 = GPIO is output is logic low 
                                            1 = GPIO is output logic high when set as push-pull output (DRV = 1). */
        unsigned char dben_gpi  : 1;    /**< General Purpose Input Debounce Timer Enable. Bit 4.
                                            0 = no debounce 
                                            1 = 30ms debounce */
        unsigned char alt_gpio  : 1;    /**< Alternate Mode Enable for GPIO0. Bit 5.
                                            0 = Standard GPIO. 
                                            1 = Active-high input, Force USB Suspend (FUS). FUS is only active if the FUS_M bit is set to 0.  */
        unsigned char rsvd      : 2;    /**< Reserved. Bit 7:6. Unutilized bit. Write to 0. Reads are don't care. */
    } bits;
} reg_cnfg_gpio0_t;

/**
 * @brief CNFG_GPIO1 Register
 *
 * Address : 0x09
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char gpo_dir   : 1;    /**< GPIO Direction. Bit 0. 
                                            0 = General purpose output (GPO) 
                                            1 = General purpose input (GPI) */
        unsigned char gpo_di    : 1;    /**< GPIO Digital Input Value. Bit 1.
                                            0 = Input logic low 
                                            1 = Input logic high */
        unsigned char gpo_drv   : 1;    /**< General Purpose Output Driver Type. Bit 2.
                                            This bit is a don't care when DIR = 1 (configured as input) When set for GPO (DIR = 0): 
                                            0 = Open-Drain 
                                            1 = Push-Pull */
        unsigned char gpo_do    : 1;    /**< General Purpose Output Data Output. Bit 3.
                                            This bit is a don't care when DIR = 1 (configured as input). When set for GPO (DIR = 0): 
                                            0 = GPIO is output is logic low 
                                            1 = GPIO is output logic high when set as push-pull output (DRV = 1). */
        unsigned char dben_gpi  : 1;    /**< General Purpose Input Debounce Timer Enable. Bit 4.
                                            0 = no debounce 
                                            1 = 30ms debounce */
        unsigned char alt_gpio  : 1;    /**< Alternate Mode Enable for GPIO1. Bit 5.
                                            0 = Standard GPIO. 
                                            1 = Active-high output of SBB2's Flexible Power Sequencer (FPS) slot. */
        unsigned char rsvd      : 2;    /**< Reserved. Bit 7:6.
                                            Unutilized bit. Write to 0. Reads are don't care. */
    } bits;
} reg_cnfg_gpio1_t;

/**
 * @brief CID Register
 *
 * Address : 0x10
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char cid   : 5;    /**< Chip Identification Code. Bit 4:0.
                                    The Chip Identification Code refers to a set of reset values in the register map, or the "OTP configuration.". */
        unsigned char       : 3;    /**< Bit 7:5. */
    } bits;
} reg_cid_t;

/**
 * @brief CNFG_WDT Register
 *
 * Address : 0x17
 */
typedef union {
    unsigned char raw;
    struct {
        unsigned char wdt_lock  : 1;    /**< Factory-Set Safety Bit for the Watchdog Timer. Bit 0. 
                                            0 = Watchdog timer can be enabled and disabled with WDT_EN. 
                                            1 = Watchdog timer can not be disabled with WDT_EN. 
                                            However, WDT_EN can still be used to enable the watchdog timer. */
        unsigned char wdt_en    : 1;    /**< Watchdog Timer Enable. Bit 1.
                                            0 = Watchdog timer is not enabled. 
                                            1 = Watchdog timer is enabled. The timer will expire if not reset by setting WDT_CLR. */
        unsigned char wdt_clr   : 1;    /**< Watchdog Timer Clear Control. Bit 2.
                                            0 = Watchdog timer period is not reset. 
                                            1 = Watchdog timer is reset back to tWD. */
        unsigned char wdt_mode  : 1;    /**< Watchdog Timer Expired Action. Bit 3.
                                            0 = Watchdog timer expire causes power-off. 
                                            1 = Watchdog timer expire causes power-reset. */
        unsigned char wdt_per   : 2;    /**< Watchdog Timer Period. Bit 5:4.
                                            0b00 = 16 seconds      0b01 = 32 seconds 
                                            0b10 = 64 seconds      0b11 = 128 seconds. */
        unsigned char rsvd      : 2;    /**< Reserved. Bit 7:6.
                                            Unutilized bit. Write to 0. Reads are don't care. */
    } bits;
} reg_cnfg_wdt_t;

/**
 * @brief CNFG_SBB_TOP
 * 
 * Address : 0x28
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char drv_sbb   : 2;    /**< SIMO Buck-Boost (all channels) Drive Strength Trim. Bit 1:0.
                                            0b00 = Fastest transition time 
                                            0b01 = A little slower than 0b00 
                                            0b10 = A little slower than 0b01 
                                            0b11 = A little slower than 0b10 */
        unsigned char 		    : 5;    /**< Bit 6:2.*/
        unsigned char dis_lpm   : 1;    /**< Disables the automatic Low Power Mode for Each SIMO Channel. Bit 7.
                                            0b0 = Automatic Low Power Mode for each SIMO channel 
                                            0b1 = Disable LPM feature for each SIMO channel */
    } bits;
} reg_cnfg_sbb_top_t;

/**
 * @brief CNFG_SBB0_A
 * 
 * Address : 0x29
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char tv_sbb0   : 8;    /**< SIMO Buck-Boost Channel 0 Target Output Voltage. Bit 7:0.
                                            0x00 = 0.500V 0x01 = 0.525V 0x02 = 0.550V 
                                            0x03 = 0.575V 0x04 = 0.600V 0x05 = 0.625V 
                                            0x06 = 0.650V 0x07 = 0.675V 0x08 = 0.700V 
                                            ... 
                                            0xC5 = 5.425V 0xC6 = 5.450V 0xC7 = 5.475V 
                                            0xC8 to 0xFF = 5.500V */
    } bits;
} reg_cnfg_sbb0_a_t;

/**
 * @brief CNFG_SBB0_B
 * 
 * Address : 0x2A
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char en_sbb0       : 3;    /**< Enable Control for SIMO Buck-Boost Channel 0, 
                                                selecting either an FPS slot the channel powers-up and powers-down in 
                                                or whether the channel is forced on or off. Bit 2:0.
                                                0b000 = FPS slot 0      0b001 = FPS slot 1 
                                                0b010 = FPS slot 2      0b011 = FPS slot 3      
                                                0b100 = Off irrespective of FPS 
                                                0b101 = same as 0b100   0b110 = On irrespective of FPS 
                                                0b111 = same as 0b110 */
        unsigned char ade_sbb0      : 1;    /**< SIMO Buck-Boost Channel 0 Active-Discharge Enable. Bit 3.
                                                0 = The active discharge function is disabled. 
                                                When SBB0 is disabled, its discharge rate is a function of the output capacitance and the external load. 
                                                1 = The active discharge function is enabled. 
                                                When SBB0 is disabled, an internal resistor (RAD_SBB0) is activated from SBB0 to PGND to help the output voltage discharge. */
        unsigned char ip_sbb0       : 2;    /**< SIMO Buck-Boost Channel 0 Peak Current Limit. Bit 5:4
                                                0b00 = 1.000A       0b01 = 0.750A 
                                                0b10 = 0.500A       0b11 = 0.333A*/
        unsigned char op_mode0      : 2;    /**<  Operation mode of SBB0. Bit 6.
                                                0b00 = Automatic 
                                                0b01 = Buck mode 
                                                0b10 = Boost mode 
                                                0b11 = Buck-boost mode*/
    } bits;
} reg_cnfg_sbb0_b_t;

/**
 * @brief CNFG_SBB1_A
 * 
 * Address : 0x2B
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char tv_sbb1   : 8;    /**< SIMO Buck-Boost Channel 1 Target Output Voltage. Bit 7:0.
                                            0x00 = 0.500V 0x01 = 0.525V 0x02 = 0.550V 
                                            0x03 = 0.575V 0x04 = 0.600V 0x05 = 0.625V 
                                            0x06 = 0.650V 0x07 = 0.675V 0x08 = 0.700V 
                                            ... 
                                            0xC5 = 5.425V 0xC6 = 5.450V 0xC7 = 5.475V 
                                            0xC8 to 0xFF = 5.500V */
    } bits;
} reg_cnfg_sbb1_a_t;

/**
 * @brief CNFG_SBB1_B
 * 
 * Address : 0x3C
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char en_sbb1       : 3;    /**< Enable Control for SIMO Buck-Boost Channel 1, 
                                                selecting either an FPS slot the channel powers-up and powers-down in 
                                                or whether the channel is forced on or off. Bit 2:0.
                                                0b000 = FPS slot 0      0b001 = FPS slot 1 
                                                0b010 = FPS slot 2      0b011 = FPS slot 3      
                                                0b100 = Off irrespective of FPS 
                                                0b101 = same as 0b100   0b110 = On irrespective of FPS 
                                                0b111 = same as 0b110 */
        unsigned char ade_sbb1      : 1;    /**< SIMO Buck-Boost Channel 1 Active-Discharge Enable. Bit 3.
                                                0 = The active discharge function is disabled. 
                                                When SBB0 is disabled, its discharge rate is a function of the output capacitance and the external load. 
                                                1 = The active discharge function is enabled. 
                                                When SBB0 is disabled, an internal resistor (RAD_SBB0) is activated from SBB0 to PGND to help the output voltage discharge. */
        unsigned char ip_sbb1       : 2;    /**< SIMO Buck-Boost Channel 1 Peak Current Limit. Bit 5:4.
                                                0b00 = 1.000A       0b01 = 0.750A 
                                                0b10 = 0.500A       0b11 = 0.333A*/
        unsigned char op_mode1      : 2;    /**<  Operation mode of SBB1. Bit 7:6.
                                                0b00 = Automatic 
                                                0b01 = Buck mode 
                                                0b10 = Boost mode 
                                                0b11 = Buck-boost mode*/
    } bits;
} reg_cnfg_sbb1_b_t;

/**
 * @brief CNFG_SBB2_A
 * 
 * Address : 0x2D
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char tv_sbb2   : 8;    /**< SIMO Buck-Boost Channel 2 Target Output Voltage. Bit 7:0.
                                            0x00 = 0.500V 0x01 = 0.525V 0x02 = 0.550V 
                                            0x03 = 0.575V 0x04 = 0.600V 0x05 = 0.625V 
                                            0x06 = 0.650V 0x07 = 0.675V 0x08 = 0.700V 
                                            ... 
                                            0xC5 = 5.425V 0xC6 = 5.450V 0xC7 = 5.475V 
                                            0xC8 to 0xFF = 5.500V */
    } bits;
} reg_cnfg_sbb2_a_t;

/**
 * @brief CNFG_SBB2_B
 * 
 * Address : 0x2E
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char en_sbb2       : 3;    /**< Enable Control for SIMO Buck-Boost Channel 2, 
                                                selecting either an FPS slot the channel powers-up and powers-down in 
                                                or whether the channel is forced on or off. Bit 2:0.
                                                0b000 = FPS slot 0      0b001 = FPS slot 1 
                                                0b010 = FPS slot 2      0b011 = FPS slot 3      
                                                0b100 = Off irrespective of FPS 
                                                0b101 = same as 0b100   0b110 = On irrespective of FPS 
                                                0b111 = same as 0b110 */
        unsigned char ade_sbb2      : 1;    /**< SIMO Buck-Boost Channel 2 Active-Discharge Enable Bit 3.
                                                0 = The active discharge function is disabled. 
                                                When SBB0 is disabled, its discharge rate is a function of the output capacitance and the external load. 
                                                1 = The active discharge function is enabled. 
                                                When SBB0 is disabled, an internal resistor (RAD_SBB0) is activated from SBB0 to PGND to help the output voltage discharge. */
        unsigned char ip_sbb2       : 2;    /**< SIMO Buck-Boost Channel 2 Peak Current Limit. Bit 5:4.
                                                0b00 = 1.000A       0b01 = 0.750A 
                                                0b10 = 0.500A       0b11 = 0.333A*/
        unsigned char op_mode2      : 2;    /**< Operation mode of SBB2. Bit 7:6.
                                                0b00 = Automatic 
                                                0b01 = Buck mode 
                                                0b10 = Boost mode 
                                                0b11 = Buck-boost mode*/
    } bits;
} reg_cnfg_sbb2_b_t;

/**
 * @brief CNFG_DVS_SBB0_A
 * 
 * Address : 0x2F
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char tv_sbb0_dvs   : 8;    /**<  SIMO Buck-Boost Channel 0 Target Output Voltage. Bit 7:0.
                                                0x00 = 0.500V 0x01 = 0.525V 0x02 = 0.550V 
                                                0x03 = 0.575V 0x04 = 0.600V 0x05 = 0.625V 
                                                0x06 = 0.650V 0x07 = 0.675V 0x08 = 0.700V 
                                                ... 
                                                0xC5 = 5.425V 0xC6 = 5.450V 0xC7 = 5.475V 
                                                0xC8 to 0xFF = 5.500V */
    } bits;
} reg_cnfg_dvs_sbb0_a_t;

/**
 * @brief CNFG_LDO0_A
 * 
 * Address : 0x38
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char tv_ldo0      : 7;    /**<  LDO0 Target Output Voltage. Bit 6:0.
                                                0x00 = 0.500V 0x01 = 0.525V 0x02 = 0.550V 
                                                0x03 = 0.575V 0x04 = 0.600V 0x05 = 0.625V 
                                                0x06 = 0.650V 0x07 = 0.675V 0x08 = 0.700V 
                                                ... 
                                                0x7E = 3.650V
                                                0x7F = 3.675V 
                                                When TV_LDO[7] = 0, TV_LDO[6:0] sets the
                                                LDO's output voltage range from 0.5V to 3.675V.
                                                When TV_LDO[7] = 1, TV_LDO[6:0] sets the
                                                LDO's output voltage from 1.825V to 5V. */
        unsigned char tv_ofs_ldo   	: 1; 	/**< LDO0 Output Voltage. Bit7. 
                                                This bit applies a 1.325V offset to the output voltage of the LDO0. 
                                                0b0 = No Offset, 0b1 = 1.325V Offset
                                                */
        
    } bits;
} reg_cnfg_ldo0_a_t;

/**
 * @brief CNFG_LDO0_B
 * 
 * Address : 0x39
 */
typedef union {
    unsigned char raw;
    struct 
    {
        unsigned char en_ldo        : 3;    /**< Enable Control for LDO0, 
                                                selecting either an FPS slot the channel powers-up and 
                                                powersdown in or whether the channel is forced on or off.  Bit 2:0.
                                                0b000 = FPS slot 0      0b001 = FPS slot 1 
                                                0b010 = FPS slot 2      0b011 = FPS slot 3      
                                                0b100 = Off irrespective of FPS 
                                                0b101 = same as 0b100   0b110 = On irrespective of FPS 
                                                0b111 = same as 0b110 */
        unsigned char ade_ldo       : 1;    /**< LDO0 Active-Discharge Enable. Bit 3.
                                                0 = The active discharge function is disabled.                                            
                                                1 = The active discharge function is enabled.*/
        unsigned char ldo_md        : 1;    /**< Operation Mode of LDO0. Bit 4.
                                                0b0 = Low dropout linear regulator (LDO) mode
                                                0b1 = Load switch (LSW) mode*/
        unsigned char rsvd    		: 3;    /**< Reserved. Unutilized bit. Write to 0. Reads are don't care.  */
    } bits;
} reg_cnfg_ldo0_b_t;

#endif /* MAX77643_2_REGS_H_ */