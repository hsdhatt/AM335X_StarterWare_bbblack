/**
 * \file  bl_platform.c
 *
 * \brief Initializes AM335x Device Peripherals.
 *
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "hw_types.h"
#include "hw_cm_cefuse.h"
#include "hw_cm_device.h"
#include "hw_cm_dpll.h"
#include "hw_cm_gfx.h"
#include "hw_cm_mpu.h"
#include "hw_cm_per.h"
#include "hw_cm_rtc.h"
#include "hw_cm_wkup.h"
#include "hw_control_AM335x.h"
#include "hw_emif4d.h"
#include "bl.h"
#include "gpmc.h"
#include "bl_copy.h"
#include "bl_platform.h"
#include "uartStdio.h"
#include "watchdog.h"
#include "hsi2c.h"
#include "gpio_v2.h"
#include "board.h"
#include "device.h"
#include "string.h"
#ifdef evmAM335x
    #include "hw_tps65910.h"
#elif  (defined beaglebone)
    #include "hw_tps65217.h"
#elif  (defined evmskAM335x)
    #include "hw_tps65910.h"
    #include "pin_mux.h"
#endif

#if defined(SPI)
    #include "bl_spi.h"
#elif defined(MMCSD)
    #include "bl_mmcsd.h"
#elif defined(NAND)
    #include "bl_nand.h"
    #include "nandlib.h"
    #include "nand_gpmc.h"
#endif

/******************************************************************************
**                     Internal Macro Definitions
*******************************************************************************/
#define INTVECMAX                          (9)
#define BIT(n)                             (1 << (n))
#define PAD_CTRL_PULLUDDISABLE             (BIT(3))
#define PAD_CTRL_PULLUPSEL                 (BIT(4))
#define PAD_CTRL_RXACTIVE                  (BIT(5))
#define PAD_CTRL_SLOWSLEW                  (BIT(6))
#define PAD_CTRL_MUXMODE(n)                ((n))

/*	I2C instance	*/
#define  I2C_0				   (0x0u)
/*	System clock fed to I2C module - 48Mhz	*/
#define  I2C_SYSTEM_CLOCK		   (48000000u)
/*	Internal clock used by I2C module - 12Mhz	*/
#define  I2C_INTERNAL_CLOCK		   (12000000u)
/*	I2C bus speed or frequency - 100Khz	*/
#define	 I2C_OUTPUT_CLOCK		   (100000u)
/*	I2C interrupt flags to clear	*/
#define  I2C_INTERRUPT_FLAG_TO_CLR         (0x7FF)
#define  PMIC_SR_I2C_SLAVE_ADDR            (0x12)
#define SMPS_DRIVE_SCLSR_EN1	           (0x0u)
#define SMPS_DRIVE_SDASR_EN2	           (0x1u)

/* TODO : These are not there in the control module header file */
#define DDR_PHY_CTRL_REGS                  (SOC_CONTROL_REGS + 0x2000)
#define CMD0_SLAVE_RATIO_0                 (DDR_PHY_CTRL_REGS + 0x1C)
#define CMD0_SLAVE_FORCE_0                 (DDR_PHY_CTRL_REGS + 0x20)
#define CMD0_SLAVE_DELAY_0                 (DDR_PHY_CTRL_REGS + 0x24)
#define CMD0_LOCK_DIFF_0                   (DDR_PHY_CTRL_REGS + 0x28)
#define CMD0_INVERT_CLKOUT_0               (DDR_PHY_CTRL_REGS + 0x2C)
#define CMD1_SLAVE_RATIO_0                 (DDR_PHY_CTRL_REGS + 0x50)
#define CMD1_SLAVE_FORCE_0                 (DDR_PHY_CTRL_REGS + 0x54)
#define CMD1_SLAVE_DELAY_0                 (DDR_PHY_CTRL_REGS + 0x58)
#define CMD1_LOCK_DIFF_0                   (DDR_PHY_CTRL_REGS + 0x5C)
#define CMD1_INVERT_CLKOUT_0               (DDR_PHY_CTRL_REGS + 0x60)
#define CMD2_SLAVE_RATIO_0                 (DDR_PHY_CTRL_REGS + 0x84)
#define CMD2_SLAVE_FORCE_0                 (DDR_PHY_CTRL_REGS + 0x88)
#define CMD2_SLAVE_DELAY_0                 (DDR_PHY_CTRL_REGS + 0x8C)
#define CMD2_LOCK_DIFF_0                   (DDR_PHY_CTRL_REGS + 0x90)
#define CMD2_INVERT_CLKOUT_0               (DDR_PHY_CTRL_REGS + 0x94)
#define DATA0_RD_DQS_SLAVE_RATIO_0         (DDR_PHY_CTRL_REGS + 0xC8)
#define DATA0_RD_DQS_SLAVE_RATIO_1         (DDR_PHY_CTRL_REGS + 0xCC)
#define DATA0_WR_DQS_SLAVE_RATIO_0         (DDR_PHY_CTRL_REGS + 0xDC)
#define DATA0_WR_DQS_SLAVE_RATIO_1         (DDR_PHY_CTRL_REGS + 0xE0)
#define DATA0_WRLVL_INIT_RATIO_0           (DDR_PHY_CTRL_REGS + 0xF0)
#define DATA0_WRLVL_INIT_RATIO_1           (DDR_PHY_CTRL_REGS + 0xF4)
#define DATA0_GATELVL_INIT_RATIO_0         (DDR_PHY_CTRL_REGS + 0xFC)
#define DATA0_GATELVL_INIT_RATIO_1         (DDR_PHY_CTRL_REGS + 0x100)
#define DATA0_FIFO_WE_SLAVE_RATIO_0        (DDR_PHY_CTRL_REGS + 0x108)
#define DATA0_FIFO_WE_SLAVE_RATIO_1        (DDR_PHY_CTRL_REGS + 0x10C)
#define DATA0_WR_DATA_SLAVE_RATIO_0        (DDR_PHY_CTRL_REGS + 0x120)
#define DATA0_WR_DATA_SLAVE_RATIO_1        (DDR_PHY_CTRL_REGS + 0x124)
#define DATA0_USE_RANK0_DELAYS_0           (DDR_PHY_CTRL_REGS + 0x134)
#define DATA0_LOCK_DIFF_0                  (DDR_PHY_CTRL_REGS + 0x138)
#define DATA1_RD_DQS_SLAVE_RATIO_0         (DDR_PHY_CTRL_REGS + 0x16c)
#define DATA1_RD_DQS_SLAVE_RATIO_1         (DDR_PHY_CTRL_REGS + 0x170)
#define DATA1_WR_DQS_SLAVE_RATIO_0         (DDR_PHY_CTRL_REGS + 0x180)
#define DATA1_WR_DQS_SLAVE_RATIO_1         (DDR_PHY_CTRL_REGS + 0x184)
#define DATA1_WRLVL_INIT_RATIO_0           (DDR_PHY_CTRL_REGS + 0x194)
#define DATA1_WRLVL_INIT_RATIO_1           (DDR_PHY_CTRL_REGS + 0x198)
#define DATA1_GATELVL_INIT_RATIO_0         (DDR_PHY_CTRL_REGS + 0x1a0)
#define DATA1_GATELVL_INIT_RATIO_1         (DDR_PHY_CTRL_REGS + 0x1a4)
#define DATA1_FIFO_WE_SLAVE_RATIO_0        (DDR_PHY_CTRL_REGS + 0x1ac)
#define DATA1_FIFO_WE_SLAVE_RATIO_1        (DDR_PHY_CTRL_REGS + 0x1b0)
#define DATA1_WR_DATA_SLAVE_RATIO_0        (DDR_PHY_CTRL_REGS + 0x1c4)
#define DATA1_WR_DATA_SLAVE_RATIO_1        (DDR_PHY_CTRL_REGS + 0x1c8)
#define DATA1_USE_RANK0_DELAYS_0           (DDR_PHY_CTRL_REGS + 0x1d8)
#define DATA1_LOCK_DIFF_0                  (DDR_PHY_CTRL_REGS + 0x1dc)


/* DDR3 init values */
#ifdef evmAM335x

#define DDR3_CMD0_SLAVE_RATIO_0            (0x80)
#define DDR3_CMD0_INVERT_CLKOUT_0          (0x0)
#define DDR3_CMD1_SLAVE_RATIO_0            (0x80)
#define DDR3_CMD1_INVERT_CLKOUT_0          (0x0)
#define DDR3_CMD2_SLAVE_RATIO_0            (0x80)
#define DDR3_CMD2_INVERT_CLKOUT_0          (0x0)

#define DDR3_DATA0_RD_DQS_SLAVE_RATIO_0    (0x3B)
#define DDR3_DATA0_WR_DQS_SLAVE_RATIO_0    (0x3C)
#define DDR3_DATA0_FIFO_WE_SLAVE_RATIO_0   (0xA5)
#define DDR3_DATA0_WR_DATA_SLAVE_RATIO_0   (0x74)

#define DDR3_DATA0_RD_DQS_SLAVE_RATIO_1    (0x3B)
#define DDR3_DATA0_WR_DQS_SLAVE_RATIO_1    (0x3C)
#define DDR3_DATA0_FIFO_WE_SLAVE_RATIO_1   (0xA5)
#define DDR3_DATA0_WR_DATA_SLAVE_RATIO_1   (0x74)

#define DDR3_CONTROL_DDR_CMD_IOCTRL_0      (0x18B)
#define DDR3_CONTROL_DDR_CMD_IOCTRL_1      (0x18B)
#define DDR3_CONTROL_DDR_CMD_IOCTRL_2      (0x18B)

#define DDR3_CONTROL_DDR_DATA_IOCTRL_0      (0x18B)
#define DDR3_CONTROL_DDR_DATA_IOCTRL_1      (0x18B)

#define DDR3_CONTROL_DDR_IO_CTRL           (0xefffffff)

#define DDR3_EMIF_DDR_PHY_CTRL_1           (0x06)
#define DDR3_EMIF_DDR_PHY_CTRL_1_DY_PWRDN         (0x00100000)
#define DDR3_EMIF_DDR_PHY_CTRL_1_SHDW      (0x06)
#define DDR3_EMIF_DDR_PHY_CTRL_1_SHDW_DY_PWRDN    (0x00100000)
#define DDR3_EMIF_DDR_PHY_CTRL_2           (0x06)

#define DDR3_EMIF_SDRAM_TIM_1              (0x0888A39B)
#define DDR3_EMIF_SDRAM_TIM_1_SHDW         (0x0888A39B)

#define DDR3_EMIF_SDRAM_TIM_2              (0x26517FDA)
#define DDR3_EMIF_SDRAM_TIM_2_SHDW         (0x26517FDA)

#define DDR3_EMIF_SDRAM_TIM_3              (0x501F84EF)
#define DDR3_EMIF_SDRAM_TIM_3_SHDM         (0x501F84EF)

#define DDR3_EMIF_SDRAM_REF_CTRL_VAL1      (0x0000093B)
#define DDR3_EMIF_SDRAM_REF_CTRL_SHDW_VAL1 (0x0000093B)

#define DDR3_EMIF_ZQ_CONFIG_VAL            (0x50074BE4)

/*
** termination = 1 (RZQ/4)
** dynamic ODT = 2 (RZQ/2)
** SDRAM drive = 0 (RZQ/6)
** CWL = 0 (CAS write latency = 5)
** CL = 2 (CAS latency = 5)
** ROWSIZE = 7 (16 row bits)
** PAGESIZE = 2 (10 column bits)
*/
#define DDR3_EMIF_SDRAM_CONFIG             (0x61C04BB2)

#else

#define DDR3_CMD0_SLAVE_RATIO_0            (0x40)
#define DDR3_CMD0_INVERT_CLKOUT_0          (0x1)
#define DDR3_CMD1_SLAVE_RATIO_0            (0x40)
#define DDR3_CMD1_INVERT_CLKOUT_0          (0x1)
#define DDR3_CMD2_SLAVE_RATIO_0            (0x40)
#define DDR3_CMD2_INVERT_CLKOUT_0          (0x1)

#define DDR3_DATA0_RD_DQS_SLAVE_RATIO_0    (0x3B)
#define DDR3_DATA0_WR_DQS_SLAVE_RATIO_0    (0x85)
#define DDR3_DATA0_FIFO_WE_SLAVE_RATIO_0   (0x100)
#define DDR3_DATA0_WR_DATA_SLAVE_RATIO_0   (0xC1)

#define DDR3_DATA0_RD_DQS_SLAVE_RATIO_1    (0x3B)
#define DDR3_DATA0_WR_DQS_SLAVE_RATIO_1    (0x85)
#define DDR3_DATA0_FIFO_WE_SLAVE_RATIO_1   (0x100)
#define DDR3_DATA0_WR_DATA_SLAVE_RATIO_1   (0xC1)

#define DDR3_CONTROL_DDR_CMD_IOCTRL_0      (0x18B)
#define DDR3_CONTROL_DDR_CMD_IOCTRL_1      (0x18B)
#define DDR3_CONTROL_DDR_CMD_IOCTRL_2      (0x18B)

#define DDR3_CONTROL_DDR_DATA_IOCTRL_0      (0x18B)
#define DDR3_CONTROL_DDR_DATA_IOCTRL_1      (0x18B)

//#define DDR3_CONTROL_DDR_IO_CTRL           (0x0fffffff)
#define DDR3_CONTROL_DDR_IO_CTRL           (0xefffffff)

#define DDR3_EMIF_DDR_PHY_CTRL_1           (0x06)
#define DDR3_EMIF_DDR_PHY_CTRL_1_DY_PWRDN         (0x00100000)
#define DDR3_EMIF_DDR_PHY_CTRL_1_SHDW      (0x06)
#define DDR3_EMIF_DDR_PHY_CTRL_1_SHDW_DY_PWRDN    (0x00100000)
#define DDR3_EMIF_DDR_PHY_CTRL_2           (0x06)

#define DDR3_EMIF_SDRAM_TIM_1              (0x0888A39B)
#define DDR3_EMIF_SDRAM_TIM_1_SHDW         (0x0888A39B)

#define DDR3_EMIF_SDRAM_TIM_2              (0x26337FDA)
#define DDR3_EMIF_SDRAM_TIM_2_SHDW         (0x26337FDA)

#define DDR3_EMIF_SDRAM_TIM_3              (0x501F830F)
#define DDR3_EMIF_SDRAM_TIM_3_SHDM         (0x501F830F)

#define DDR3_EMIF_SDRAM_REF_CTRL_VAL1      (0x0000093B)
#define DDR3_EMIF_SDRAM_REF_CTRL_SHDW_VAL1 (0x0000093B)

#define DDR3_EMIF_ZQ_CONFIG_VAL            (0x50074BE4)
#define DDR3_EMIF_SDRAM_CONFIG             (0x61C04AB2)//termination = 1 (RZQ/4)
                                                       //dynamic ODT = 2 (RZQ/2)
                                                       //SDRAM drive = 0 (RZQ/6)
                                                       //CWL = 0 (CAS write latency = 5)
                                                       //CL = 2 (CAS latency = 5)
                                                       //ROWSIZE = 5 (14 row bits)
                                                       //PAGESIZE = 2 (10 column bits)
													   
#endif

/* DDR2 init values */

#define DDR2_CMD0_SLAVE_RATIO_0            (0x80)
#define DDR2_CMD0_SLAVE_FORCE_0            (0x0)
#define DDR2_CMD0_SLAVE_DELAY_0            (0x0)
#define DDR2_CMD0_LOCK_DIFF_0              (0x4)
#define DDR2_CMD0_INVERT_CLKOUT_0          (0x0)

#define DDR2_CMD1_SLAVE_RATIO_0            (0x80)
#define DDR2_CMD1_SLAVE_FORCE_0            (0x0)
#define DDR2_CMD1_SLAVE_DELAY_0            (0x0)
#define DDR2_CMD1_LOCK_DIFF_0              (0x4)
#define DDR2_CMD1_INVERT_CLKOUT_0          (0x0)

#define DDR2_CMD2_SLAVE_RATIO_0            (0x80)
#define DDR2_CMD2_SLAVE_FORCE_0            (0x0)
#define DDR2_CMD2_SLAVE_DELAY_0            (0x0)
#define DDR2_CMD2_LOCK_DIFF_0              (0x4)
#define DDR2_CMD2_INVERT_CLKOUT_0          (0x0)

#define DDR2_DATA0_RD_DQS_SLAVE_RATIO_0    (0x12)
#define DDR2_DATA0_WR_DQS_SLAVE_RATIO_0    (0x0)
#define DDR2_DATA0_FIFO_WE_SLAVE_RATIO_0   (0x80)
#define DDR2_DATA0_WR_DATA_SLAVE_RATIO_0   (0x40)

#define DDR2_DATA1_RD_DQS_SLAVE_RATIO_0    (0x12)
#define DDR2_DATA1_WR_DQS_SLAVE_RATIO_0    (0x0)
#define DDR2_DATA1_FIFO_WE_SLAVE_RATIO_0   (0x80)
#define DDR2_DATA1_WR_DATA_SLAVE_RATIO_0   (0x40)

#define DDR2_CONTROL_DDR_CMD_IOCTRL_0      (0x18B)
#define DDR2_CONTROL_DDR_CMD_IOCTRL_1      (0x18B)
#define DDR2_CONTROL_DDR_CMD_IOCTRL_2      (0x18B)
#define DDR2_CONTROL_DDR_DATA_IOCTRL_0     (0x18B)
#define DDR2_CONTROL_DDR_DATA_IOCTRL_1     (0x18B)

#define DDR2_CONTROL_DDR_IO_CTRL           (0x0fffffff)
#define DDR2_EMIF_DDR_PHY_CTRL_1           (0x05)
#define DDR2_EMIF_DDR_PHY_CTRL_1_DY_PWRDN         (0x00100000)
#define DDR2_EMIF_DDR_PHY_CTRL_1_SHDW      (0x05)
#define DDR2_EMIF_DDR_PHY_CTRL_1_SHDW_DY_PWRDN    (0x00100000)
#define DDR2_EMIF_DDR_PHY_CTRL_2           (0x05)

#define DDR2_EMIF_SDRAM_TIM_1		   (0x0666B3D6)
#define DDR2_EMIF_SDRAM_TIM_1_SHDW         (0x0666B3D6)
#define DDR2_EMIF_SDRAM_TIM_2              (0x143731DA)
#define DDR2_EMIF_SDRAM_TIM_2_SHDW	   (0x143731DA)
#define DDR2_EMIF_SDRAM_TIM_3              (0x00000347)
#define DDR2_EMIF_SDRAM_TIM_3_SHDM         (0x00000347)
#define DDR2_EMIF_SDRAM_CONFIG             (0x41805332)
#define DDR2_EMIF_SDRAM_REF_CTRL_VAL1	   (0x00004650)
#define DDR2_EMIF_SDRAM_REF_CTRL_SHDW_VAL1 (0x00004650)

#define DDR2_EMIF_SDRAM_REF_CTRL_VAL2	   (0x0000081a)
#define DDR2_EMIF_SDRAM_REF_CTRL_SHDW_VAL2 (0x0000081a)

#define GPIO_INSTANCE_PIN_NUMBER      (7)
/******************************************************************************
**                     Local function Declarations
*******************************************************************************/

#ifdef evmskAM335x
static void DDRVTTEnable(void);
#endif
extern void SPIConfigure(void);
extern void I2C1ModuleClkConfig(void);


/******************************************************************************
**                     Global variable Definitions
*******************************************************************************/
char *deviceType = "AM335x";
volatile unsigned char dataFromSlave[2];
volatile unsigned char dataToSlave[3];
volatile unsigned int tCount;
volatile unsigned int rCount;
volatile unsigned int flag = 1;
volatile unsigned int numOfBytes;
volatile unsigned int oppMaxIdx;
volatile unsigned int deviceVersion;
volatile unsigned int freqMultDDR;

/*
** OPP table for mpu multiplier and pmic voltage select.
** MPUPLL_N and MPUPLL_M2 are divider and post divider values.
*/
tOPPConfig oppTable[] =
{
    {MPUPLL_M_275_MHZ, PMIC_VOLT_SEL_1100MV},  /* OPP100 275Mhz - 1.1v */
    {MPUPLL_M_500_MHZ, PMIC_VOLT_SEL_1100MV},  /* OPP100 500Mhz - 1.1v */
    {MPUPLL_M_600_MHZ, PMIC_VOLT_SEL_1200MV},  /* OPP120 600Mhz - 1.2v */
    {MPUPLL_M_720_MHZ, PMIC_VOLT_SEL_1260MV},  /* OPP TURBO 720Mhz - 1.26v */
    {MPUPLL_M_300_MHZ, PMIC_VOLT_SEL_0950MV},  /* OPP50 300Mhz - 950mv */
    {MPUPLL_M_300_MHZ, PMIC_VOLT_SEL_1100MV},  /* OPP100 300Mhz - 1.1v */
    {MPUPLL_M_600_MHZ, PMIC_VOLT_SEL_1100MV},  /* OPP100 600Mhz - 1.1v */
    {MPUPLL_M_720_MHZ, PMIC_VOLT_SEL_1200MV},  /* OPP120 720Mhz - 1.2v */
    {MPUPLL_M_800_MHZ, PMIC_VOLT_SEL_1260MV},  /* OPP TURBO 800Mhz - 1.26v */
    {MPUPLL_M_1000_MHZ, PMIC_VOLT_SEL_1325MV}  /* OPP NITRO 1000Mhz - 1.325v */
};

#if defined(NAND)
  static GPMCNANDTimingInfo_t nandTimingInfo;
#endif


/******************************************************************************
**                     Local variable Definitions
*******************************************************************************/

/******************************************************************************
**                     Function Definitions
*******************************************************************************/

/*
** Determine maximum OPP configuration of SoC.
*/
unsigned int BootMaxOppGet(void)
{
    unsigned int oppIdx;
    unsigned int oppSupport = SysConfigOppDataGet();

    if(DEVICE_VERSION_1_0 == deviceVersion)
    {
        oppIdx = EFUSE_OPPTB_720;
    }
    else if(DEVICE_VERSION_2_0 == deviceVersion)
    {
        oppIdx = EFUSE_OPPTB_800;
    }
    else if(DEVICE_VERSION_2_1 == deviceVersion)
    {
        if(!(oppSupport & EFUSE_OPPNT_1000_MASK))
        {
            oppIdx = EFUSE_OPPNT_1000;
        }
        else if(!(oppSupport & EFUSE_OPPTB_800_MASK))
        {
            oppIdx = EFUSE_OPPTB_800;
        }
        else if(!(oppSupport & EFUSE_OPP120_720_MASK))
        {
            oppIdx = EFUSE_OPP120_720;
        }
        else if(!(oppSupport & EFUSE_OPP100_600_MASK))
        {
            oppIdx = EFUSE_OPP100_600;
        }
        else if(!(oppSupport & EFUSE_OPP100_300_MASK))
        {
            oppIdx = EFUSE_OPP100_300;
        }
        else
        {
            oppIdx = EFUSE_OPP50_300;
        }
    }
    else
    {
        return OPP_NONE;
    }

    return oppIdx;
}

/* \brief This function initialize sthe CORE PLL 
 * 
 * \param none
 *
 * \return none
 *
 */
void CorePLLInit(void)
{
    volatile unsigned int regVal = 0;

    /* Enable the Core PLL */

    /* Put the PLL in bypass mode */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_CORE) &
                ~CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_DPLL_MN_BYP_MODE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_CORE) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_CORE) &
                      CM_WKUP_CM_IDLEST_DPLL_CORE_ST_MN_BYPASS));

    /* Set the multipler and divider values for the PLL */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_CORE) =
        ((COREPLL_M << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT_SHIFT) |
         (COREPLL_N << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV_SHIFT));

    /* Configure the High speed dividers */
    /* Set M4 divider */    
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M4_DPLL_CORE);
    regVal = regVal & ~CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_DIV;
    regVal = regVal | (COREPLL_HSD_M4 << 
                CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_DIV_SHIFT);
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M4_DPLL_CORE) = regVal;
    
    /* Set M5 divider */    
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M5_DPLL_CORE);
    regVal = regVal & ~CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_DIV;
    regVal = regVal | (COREPLL_HSD_M5 << 
                CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_DIV_SHIFT);
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M5_DPLL_CORE) = regVal;        
        
    /* Set M6 divider */    
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M6_DPLL_CORE);
    regVal = regVal & ~CM_WKUP_CM_DIV_M6_DPLL_CORE_HSDIVIDER_CLKOUT3_DIV;
    regVal = regVal | (COREPLL_HSD_M6 << 
                CM_WKUP_CM_DIV_M6_DPLL_CORE_HSDIVIDER_CLKOUT3_DIV_SHIFT);
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M6_DPLL_CORE) = regVal;         

    /* Now LOCK the PLL by enabling it */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_CORE) &
                ~CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_CORE) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_CORE) &
                        CM_WKUP_CM_IDLEST_DPLL_CORE_ST_DPLL_CLK));
}

/* \brief This function initializes the DISPLAY PLL
 * 
 * \param none
 *
 * \return none
 *
 */
void DisplayPLLInit(void)
{
    volatile unsigned int regVal = 0;

    /* Put the PLL in bypass mode */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DISP) &
                ~CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_DPLL_MN_BYP_MODE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DISP) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DISP) &
                        CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS));

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DISP) &=
                           ~(CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV |
                             CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT);

    /* Set the multipler and divider values for the PLL */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DISP) |=
        ((DISPLL_M << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT_SHIFT) |
         (DISPLL_N << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV_SHIFT));

    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_DISP);
    regVal = regVal & ~CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_DIV;
    regVal = regVal | DISPLL_M2;

    /* Set the CLKOUT2 divider */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_DISP) = regVal;

    /* Now LOCK the PLL by enabling it */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DISP) &
                ~CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DISP) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DISP) &
                         CM_WKUP_CM_IDLEST_DPLL_DISP_ST_DPLL_CLK));
}

/* \brief This function initializes the PER PLL
 * 
 * \param none
 *
 * \return none
 *
 */
void PerPLLInit(void)
{
    volatile unsigned int regVal = 0;

    /* Put the PLL in bypass mode */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_PER) &
                ~CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_DPLL_MN_BYP_MODE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_PER) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_PER) &
                      CM_WKUP_CM_IDLEST_DPLL_PER_ST_MN_BYPASS));

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_PERIPH) &=
                             ~(CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT |
                                    CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV);

    /* Set the multipler and divider values for the PLL */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_PERIPH) |=
        ((PERPLL_M << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT_SHIFT) |
         (PERPLL_N << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV_SHIFT));

    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_PER);
    regVal = regVal & ~CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_DIV;
    regVal = regVal | PERPLL_M2;

    /* Set the CLKOUT2 divider */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_PER) = regVal;
    
    /* Now LOCK the PLL by enabling it */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_PER) &
                ~CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_PER) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_PER) &
                           CM_WKUP_CM_IDLEST_DPLL_PER_ST_DPLL_CLK));

}

/* \brief This function initializes the DDR PLL
 * 
 * \param none
 *
 * \return none
 *
 */
void DDRPLLInit(unsigned int freqMult)
{
    volatile unsigned int regVal = 0;

    /* Put the PLL in bypass mode */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DDR) &
                 ~CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_MN_BYP_MODE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DDR) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DDR) &
                      CM_WKUP_CM_IDLEST_DPLL_DDR_ST_MN_BYPASS));

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DDR) &=
                     ~(CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT |
                           CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV);

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DDR) |=
                     ((freqMult << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT) |
                      (DDRPLL_N << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT));

    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_DDR);
    regVal = regVal & ~CM_WKUP_CM_DIV_M2_DPLL_DDR_DPLL_CLKOUT_DIV;
    regVal = regVal | DDRPLL_M2;

    /* Set the CLKOUT2 divider */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_DDR) = regVal;

    /* Now LOCK the PLL by enabling it */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DDR) &
                ~CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DDR) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DDR) &
                           CM_WKUP_CM_IDLEST_DPLL_DDR_ST_DPLL_CLK));
}

/*
 * \brief This function initializes the Basic Clocks
 *
 * \param  none
 *
 * \return none
 */
void BASICInit(void)
{
	volatile unsigned int regval = 0x0;

	regval = HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) 
			& ~CM_PER_L3_CLKCTRL_MODULEMODE;	
	regval |= CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;
	HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) = regval;


	regval = HWREG(SOC_CM_PER_REGS + CM_PER_L4FW_CLKSTCTRL) 
			& ~CM_PER_L4FW_CLKSTCTRL_CLKTRCTRL;	
	regval |= CM_PER_L4FW_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	HWREG(SOC_CM_PER_REGS + CM_PER_L4FW_CLKSTCTRL) = regval;

	regval = HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) 
			& ~CM_PER_L3S_CLKSTCTRL_CLKTRCTRL;	
	regval |= CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) = regval;

	regval = HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) 
			& ~CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL;	
	regval |= CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) = regval;
	
	regval = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) 
			& ~CM_WKUP_CLKSTCTRL_CLKTRCTRL;	
	regval |= CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) = regval;
	
#if 0
	regval = HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL) 
			& ~CM_PER_EMIF_FW_CLKCTRL_MODULEMODE;	
	regval |= CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_ENABLE;
	HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL) = regval;
#endif	
	regval = HWREG(SOC_CM_RTC_REGS + CM_RTC_CLKSTCTRL) 
			& ~CM_RTC_CLKSTCTRL_CLKTRCTRL;	
	regval |= CM_RTC_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	HWREG(SOC_CM_PER_REGS + CM_RTC_CLKSTCTRL) = regval;
	
}
/*
 * \brief This function initializes the MPU PLL
 *
 * \param  none
 *
 * \return none
 */
void MPUPLLInit(unsigned int freqMult)
{
    volatile unsigned int regVal = 0;

    /* Put the PLL in bypass mode */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) &
                ~CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_MN_BYP_MODE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) = regVal;

    /* Wait for DPLL to go in to bypass mode */
    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU) &
                CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS));

    /* Clear the MULT and DIV field of DPLL_MPU register */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU) &=
                      ~(CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT |
                              CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV);

    /* Set the multiplier and divider values for the PLL */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU) |=
                     ((freqMult << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT) |
                      (MPUPLL_N << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV_SHIFT));

    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_MPU);

    regVal = regVal & ~CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIV;

    regVal = regVal | MPUPLL_M2;

    /* Set the CLKOUT2 divider */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_MPU) = regVal;

    /* Now LOCK the PLL by enabling it */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) &
                ~CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU) &
                             CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK));
}

/* \brief This function initializes the interface clock 
 * 
 * \param none
 *
 * \return none
 *
 */
void InterfaceClkInit(void)
{
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) |=
                                   CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
        CM_PER_L3_CLKCTRL_MODULEMODE) != CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) |=
                                       CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) &
      CM_PER_L4LS_CLKCTRL_MODULEMODE) != CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_L4FW_CLKCTRL) |=
                                 CM_PER_L4FW_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L4FW_CLKCTRL) &
      CM_PER_L4FW_CLKCTRL_MODULEMODE) != CM_PER_L4FW_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) |=
                                          CM_PER_L4FW_CLKCTRL_MODULEMODE_ENABLE;
    while((HWREG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
                        CM_WKUP_L4WKUP_CLKCTRL_MODULEMODE) !=
                                         CM_PER_L4FW_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
                                      CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
                        CM_PER_L3_INSTR_CLKCTRL_MODULEMODE) !=
                        CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_L4HS_CLKCTRL) |=
                              CM_PER_L4HS_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L4HS_CLKCTRL) &
                        CM_PER_L4HS_CLKCTRL_MODULEMODE) !=
                  CM_PER_L4HS_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
                             CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE; 

    while((HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
                        CM_WKUP_GPIO0_CLKCTRL_MODULEMODE) !=
                  CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
                             CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE; 

    while((HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
                        CM_WKUP_CONTROL_CLKCTRL_MODULEMODE) !=
                  CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_TIMER2_CLKCTRL) |=
                             CM_PER_TIMER2_CLKCTRL_MODULEMODE_ENABLE; 

    while((HWREG(SOC_CM_PER_REGS + CM_PER_TIMER2_CLKCTRL) & 
                        CM_PER_TIMER2_CLKCTRL_MODULEMODE) !=
                  CM_PER_TIMER2_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL) |=
                             CM_PER_GPMC_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL) &
                        CM_PER_GPMC_CLKCTRL_MODULEMODE) !=
                  CM_PER_GPMC_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_ELM_CLKCTRL) |=
                             CM_PER_ELM_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_ELM_CLKCTRL) &
                        CM_PER_ELM_CLKCTRL_MODULEMODE) !=
                  CM_PER_ELM_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_MMC0_CLKCTRL) |=
                             CM_PER_MMC0_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_MMC0_CLKCTRL) &
                        CM_PER_MMC0_CLKCTRL_MODULEMODE) !=
                  CM_PER_MMC0_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_MMC1_CLKCTRL) |=
                             CM_PER_MMC1_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_MMC1_CLKCTRL) &
                        CM_PER_MMC1_CLKCTRL_MODULEMODE) !=
                  CM_PER_MMC1_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) |=
                             CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
                        CM_WKUP_I2C0_CLKCTRL_MODULEMODE) !=
                  CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
                             CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
                        CM_PER_GPIO1_CLKCTRL_MODULEMODE) !=
                  CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO2_CLKCTRL) |=
                             CM_PER_GPIO2_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_GPIO2_CLKCTRL) &
                        CM_PER_GPIO2_CLKCTRL_MODULEMODE) !=
                  CM_PER_GPIO2_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) |=
                             CM_PER_GPIO3_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) &
                        CM_PER_GPIO3_CLKCTRL_MODULEMODE) !=
                  CM_PER_GPIO3_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_I2C1_CLKCTRL) |=
                             CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_I2C1_CLKCTRL) &
                        CM_PER_I2C1_CLKCTRL_MODULEMODE) !=
                  CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_CPGMAC0_CLKCTRL) |=
                             CM_PER_CPGMAC0_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_CPGMAC0_CLKCTRL) &
                        CM_PER_CPGMAC0_CLKCTRL_MODULEMODE) !=
                  CM_PER_CPGMAC0_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_SPI0_CLKCTRL) |=
                             CM_PER_SPI0_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_SPI0_CLKCTRL) &
                        CM_PER_SPI0_CLKCTRL_MODULEMODE) !=
                  CM_PER_SPI0_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_CM_RTC_REGS + CM_RTC_RTC_CLKCTRL) |=
			CM_RTC_RTC_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_RTC_REGS + CM_RTC_RTC_CLKCTRL) &
                        CM_RTC_RTC_CLKCTRL_MODULEMODE) !=
                  CM_RTC_RTC_CLKCTRL_MODULEMODE_ENABLE);

   HWREG(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL) |=
                             CM_PER_USB0_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL) &
                        CM_PER_USB0_CLKCTRL_MODULEMODE) !=
                  CM_PER_USB0_CLKCTRL_MODULEMODE_ENABLE);

#if 0
   HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL) |=
                             CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL) &
                        CM_PER_EMIF_FW_CLKCTRL_MODULEMODE) !=
                  CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_ENABLE);
#endif
   HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_CLKCTRL) |=
                             CM_PER_EMIF_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_CLKCTRL) &
                        CM_PER_EMIF_CLKCTRL_MODULEMODE) !=
                  CM_PER_EMIF_CLKCTRL_MODULEMODE_ENABLE);


}
/* \brief This function initializes the power domain transition.
 * 
 * \param none
 *
 * \return none
 *
 */
void PowerDomainTransitionInit(void)
{
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) |=
                             CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

     HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) |=
                             CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
                             CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    HWREG(SOC_CM_PER_REGS + CM_PER_L4FW_CLKSTCTRL) |=
                              CM_PER_L4FW_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) |=
                                CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

}

/*
 * \brief This function sets up various PLLs  
 *
 * \param  none
 *
 * \return none
 */
void PLLInit(void)
{
    BASICInit();	/* init the basic clocks */
    InterfaceClkInit();
    MPUPLLInit(oppTable[oppMaxIdx].pllMult);
    CorePLLInit();
    PerPLLInit();
    DDRPLLInit(freqMultDDR);
    //PowerDomainTransitionInit();
    //DisplayPLLInit();
}

/*
 * \brief Configure I2C0 on which the PMIC is interfaced
 *
 * \param  none
 *
 * \return none
 */
void SetupI2C(void)
{
    I2C0ModuleClkConfig();

    I2CPinMuxSetup(I2C_0);

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(SOC_I2C_0_REGS);

    I2CSoftReset(SOC_I2C_0_REGS);

    /* Disable auto Idle functionality */
    I2CAutoIdleDisable(SOC_I2C_0_REGS);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(SOC_I2C_0_REGS, I2C_SYSTEM_CLOCK, I2C_INTERNAL_CLOCK,
							   I2C_OUTPUT_CLOCK);
    I2CMasterEnable(SOC_I2C_0_REGS);

    while(!I2CSystemStatusGet(SOC_I2C_0_REGS));
}

/*
 * \brief Clear the status of all interrupts 
 *
 * \param  none.
 *
 * \return none
 */

void CleanupInterrupts(void)
{
    I2CMasterIntClearEx(SOC_I2C_0_REGS,  I2C_INTERRUPT_FLAG_TO_CLR);
}

/*
 * \brief Transmits data over I2C0 bus 
 *
 * \param  none
 *
 * \return none
 */
void SetupI2CTransmit(unsigned int dcount)
{
    I2CSetDataCount(SOC_I2C_0_REGS, dcount);

    CleanupInterrupts();

    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_TX);

    I2CMasterStart(SOC_I2C_0_REGS);

    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    while((I2C_INT_TRANSMIT_READY == (I2CMasterIntRawStatus(SOC_I2C_0_REGS)
                                     & I2C_INT_TRANSMIT_READY)) && dcount--)
    {
        I2CMasterDataPut(SOC_I2C_0_REGS, dataToSlave[tCount++]);

        I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);
    }

    I2CMasterStop(SOC_I2C_0_REGS);

    while(0 == (I2CMasterIntRawStatus(SOC_I2C_0_REGS) & I2C_INT_STOP_CONDITION));

    I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_STOP_CONDITION);

}

/*
 * \brief Receives data over I2C0 bus 
 *
 * \param  dcount - Number of bytes to receive.
 *
 * \return none
 */

void SetupReception(unsigned int dcount)
{
    I2CSetDataCount(SOC_I2C_0_REGS, 1);

    CleanupInterrupts();

    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_TX);

    I2CMasterStart(SOC_I2C_0_REGS);

    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    I2CMasterDataPut(SOC_I2C_0_REGS, dataToSlave[tCount]);

    I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);

    while(0 == (I2CMasterIntRawStatus(SOC_I2C_0_REGS) & I2C_INT_ADRR_READY_ACESS));

    I2CSetDataCount(SOC_I2C_0_REGS, dcount);

    CleanupInterrupts();

    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_RX);

    I2CMasterStart(SOC_I2C_0_REGS);

    /* Wait till the bus if free */
    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    /* Read the data from slave of dcount */
    while((dcount--))
    {
        while(0 == (I2CMasterIntRawStatus(SOC_I2C_0_REGS) & I2C_INT_RECV_READY));

        dataFromSlave[rCount++] = I2CMasterDataGet(SOC_I2C_0_REGS);

        I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_RECV_READY);
    }

    I2CMasterStop(SOC_I2C_0_REGS);

    while(0 == (I2CMasterIntRawStatus(SOC_I2C_0_REGS) & I2C_INT_STOP_CONDITION));

    I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_STOP_CONDITION);
}

/*
 * \brief Generic function that can read a TPS65217 register
 *
 * \param regOffset -  Source register address
 *
 * \return dest     -  Place holder for read bytes.
 */
#ifdef beaglebone

void TPS65217RegRead(unsigned char regOffset, unsigned char* dest)
{
    dataToSlave[0] = regOffset;
    tCount = 0;

    SetupReception(1);

    *dest = dataFromSlave[0];
}

/**
 *  \brief            - Generic function that can write a TPS65217 PMIC
 *                      register or bit field regardless of protection
 *                      level.
 *
 * \param prot_level  - Register password protection.
 *                      use PROT_LEVEL_NONE, PROT_LEVEL_1, or PROT_LEVEL_2
 * \param regOffset:  - Register address to write.
 *
 * \param dest_val    - Value to write.
 *
 * \param mask        - Bit mask (8 bits) to be applied.  Function will only
 *                      change bits that are set in the bit mask.
 *
 * \return:            None.
 */
void TPS65217RegWrite(unsigned char port_level, unsigned char regOffset,
                      unsigned char dest_val, unsigned char mask)
{
    unsigned char read_val;
    unsigned xor_reg;

    dataToSlave[0] = regOffset;
    tCount = 0;
    rCount = 0;

    if(mask != MASK_ALL_BITS)
    {
         SetupReception(1);

         read_val = dataFromSlave[0];
         read_val &= (~mask);
         read_val |= (dest_val & mask);
         dest_val = read_val;
    }

    if(port_level > 0)
    {
         xor_reg = regOffset ^ PASSWORD_UNLOCK;

         dataToSlave[0] = PASSWORD;
         dataToSlave[1] = xor_reg;
         tCount = 0;

         SetupI2CTransmit(2);
    }

    dataToSlave[0] = regOffset;
    dataToSlave[1] = dest_val;
    tCount = 0;

    SetupI2CTransmit(2);

    if(port_level == PROT_LEVEL_2)
    {
         dataToSlave[0] = PASSWORD;
         dataToSlave[1] = xor_reg;
         tCount = 0;

         SetupI2CTransmit(2);

         dataToSlave[0] = regOffset;
         dataToSlave[1] = dest_val;
         tCount = 0;

         SetupI2CTransmit(2);
    }
}

/**
 *  \brief              - Controls output voltage setting for the DCDC1,
 *                        DCDC2, or DCDC3 control registers in the PMIC.
 *
 * \param  dc_cntrl_reg   DCDC Control Register address.
 *                        Must be DEFDCDC1, DEFDCDC2, or DEFDCDC3.
 *
 * \param  volt_sel       Register value to set.  See PMIC TRM for value set.
 *
 * \return:               None.
 */
void TPS65217VoltageUpdate(unsigned char dc_cntrl_reg, unsigned char volt_sel)
{
    /* set voltage level */
    TPS65217RegWrite(PROT_LEVEL_2, dc_cntrl_reg, volt_sel, MASK_ALL_BITS);

    /* set GO bit to initiate voltage transition */
    TPS65217RegWrite(PROT_LEVEL_2, DEFSLEW, DCDC_GO, DCDC_GO);
}

#else

/**
 *  \brief                 - Configure vdd2 for various parameters such as 
 *                           Multiplier, Maximum Load Current etc .
 *
 * \param  opVolMultiplier - Multiplier.
 *
 * \param  maxLoadCurrent  - Maximum Load Current.
 * 
 * \param  timeStep        - Time step - voltage change per us(micro sec).
 *  
 * \param  supplyState     - Supply state (on (high/low power mode), off)
 *
 * \return:               None.
 */

void ConfigureVdd2(unsigned int opVolMultiplier, unsigned maxLoadCurrent,
                   unsigned int timeStep, unsigned int supplyState)
{
    dataToSlave[0] = VDD2_REG;
    dataToSlave[1] = ((opVolMultiplier << PMIC_VDD2_REG_VGAIN_SEL_SHIFT) |
		              (maxLoadCurrent << PMIC_VDD2_REG_ILMAX_SHIFT)      |
		              (timeStep << PMIC_VDD2_REG_TSTEP_SHIFT)            |
		              (supplyState << PMIC_VDD2_REG_ST_SHIFT));
    tCount = 0;
    rCount = 0;
    SetupI2CTransmit(2);
}


/**
 *  \brief Select the VDD2 value. VDD2_OP_REG or VDD2_SR_REG.
 *
 * \param  vddSource  - VDD2 value.
 *
 * \return None.
 */
void SelectVdd2Source(unsigned int vddSource)
{
    /*	Read reg value	*/
    dataToSlave[0] = VDD2_OP_REG;
    dataFromSlave[0] = 0; // clear receive buffer
    dataFromSlave[1] = 0;
    rCount = 0;
    tCount = 0;
    SetupReception(1);

    /*	Modify reg value	*/
    vddSource = (dataFromSlave[0] & (~PMIC_VDD2_OP_REG_CMD)) |
		 (vddSource << PMIC_VDD2_OP_REG_CMD_SHIFT);

	/*	Write reg value	*/
    dataToSlave[0] = VDD2_OP_REG;
    dataToSlave[1] = vddSource;
    tCount = 0;
    rCount = 0;

    SetupI2CTransmit(2);
}

/**
 *  \brief set VDD2_OP voltage value.
 *
 * \param  opVolSelector  - VDD2_OP voltage value.
 *
 * \return None.
 */
void SetVdd2OpVoltage(unsigned int opVolSelector)
{
    /*	Read reg value	*/
    dataToSlave[0] = VDD2_OP_REG;
    dataFromSlave[0] = 0; // clear receive buffer
    dataFromSlave[1] = 0;
    rCount = 0;
    tCount = 0;
    SetupReception(1);

    /*	Modify reg value	*/
    opVolSelector = (dataFromSlave[0] & (~PMIC_VDD2_OP_REG_SEL)) |
		            (opVolSelector << PMIC_VDD2_OP_REG_SEL_SHIFT);

    /*	Write reg value	*/
    dataToSlave[0] = VDD2_OP_REG;
    dataToSlave[1] = opVolSelector;
    tCount = 0;
    rCount = 0;
    SetupI2CTransmit(2);

     /*	Read reg value to verify */
    dataToSlave[0] = VDD2_OP_REG;
    dataFromSlave[0] = 0; // clear receive buffer
    dataFromSlave[1] = 0;
    rCount = 0;
    tCount = 0;

    SetupReception(1);

    while((dataFromSlave[0] & PMIC_VDD2_OP_REG_SEL) != 
          (opVolSelector << PMIC_VDD2_OP_REG_SEL_SHIFT));
}

/**
 *  \brief set VDD2_SR voltage value
 *
 * \param  opVolSelector  - VDD2_SR voltage value.
 *
 * \return None.
 */
void SetVdd2SrVoltage(unsigned int opVolSelector)
{
    /*	Write reg value	*/
    dataToSlave[0] = VDD2_SR_REG;
    dataToSlave[1] = opVolSelector;
    tCount = 0;
    SetupI2CTransmit(2);	
}

/**
 *  \brief Select I2C interface whether SR I2C or Control I2C
 *
 * \param  i2cInstance  - I2c instance to select.
 *
 * \return None.
 */
void SelectI2CInstance(unsigned int i2cInstance)
{
    /*	Read reg value	*/
    dataToSlave[0] = DEVCTRL_REG;
    dataFromSlave[0] = 0; // clear receive buffer
    dataFromSlave[1] = 0;
    rCount = 0;
    tCount = 0;
    SetupReception(1);

    /*	Modify reg value */
    i2cInstance = (dataFromSlave[0] & (~PMIC_DEVCTRL_REG_SR_CTL_I2C_SEL)) |
                  (i2cInstance << PMIC_DEVCTRL_REG_SR_CTL_I2C_SEL_SHIFT);

    /*	Write reg value	*/
    dataToSlave[0] = DEVCTRL_REG;
    dataToSlave[1] = i2cInstance;
    tCount = 0;
    rCount = 0;

    SetupI2CTransmit(2);
}

/**
 *  \brief                 - Configure vdd1 for various parameters such as 
 *                           Multiplier, Maximum Load Current etc .
 *
 * \param  opVolMultiplier - Multiplier.
 *
 * \param  maxLoadCurrent  - Maximum Load Current.
 * 
 * \param  timeStep        - Time step - voltage change per us(micro sec).
 *  
 * \param  supplyState     - Supply state (on (high/low power mode), off)
 *
 * \return:               None.
 */
void ConfigureVdd1(unsigned int opVolMultiplier, unsigned maxLoadCurrent,
                   unsigned int timeStep, unsigned int supplyState)
{
    dataToSlave[0] = VDD1_REG;
    dataToSlave[1] = ((opVolMultiplier << PMIC_VDD1_REG_VGAIN_SEL_SHIFT) |
		      (maxLoadCurrent << PMIC_VDD1_REG_ILMAX_SHIFT) |
		      (timeStep << PMIC_VDD1_REG_TSTEP_SHIFT) |
		      (supplyState << PMIC_VDD1_REG_ST_SHIFT) );
    tCount = 0;
    rCount = 0;
    SetupI2CTransmit(2);
}

/**
 *  \brief Select the VDD1 value. VDD1_OP_REG or VDD1_SR_REG.
 *
 * \param  vddSource  - VDD2 value.
 *
 * \return None.
 */
void SelectVdd1Source(unsigned int vddSource)
{
    /*	Read reg value	*/
    dataToSlave[0] = VDD1_OP_REG;
    dataFromSlave[0] = 0; // clear receive buffer
    dataFromSlave[1] = 0;
    rCount = 0;
    tCount = 0;
    SetupReception(1);

    /*	Modify reg value */
    vddSource = (dataFromSlave[0] & (~PMIC_VDD1_OP_REG_CMD)) |
		 (vddSource << PMIC_VDD1_OP_REG_CMD_SHIFT);

     /*	Write reg value	*/
    dataToSlave[0] = VDD1_OP_REG;
    dataToSlave[1] = vddSource;
    tCount = 0;
    rCount = 0;
    SetupI2CTransmit(2);
}

#endif

/**
 *  \brief set VDD1_OP voltage value.
 *
 * \param  opVolSelector  - VDD2_OP voltage value.
 *
 * \return None.
 */
void SetVdd1OpVoltage(unsigned int opVolSelector)
{
#ifdef beaglebone

    /* Set DCDC2 (MPU) voltage */
    TPS65217VoltageUpdate(DEFDCDC2, opVolSelector);

#elif  defined (evmAM335x) || defined (evmskAM335x)

    /*	Read reg value	*/
    dataToSlave[0] = VDD1_OP_REG;
    dataFromSlave[0] = 0; // clear receive buffer
    dataFromSlave[1] = 0;
    rCount = 0;
    tCount = 0;
    SetupReception(1);

    /*	Modify reg value */
    opVolSelector = (dataFromSlave[0] & (~PMIC_VDD1_OP_REG_SEL)) |
		    (opVolSelector << PMIC_VDD1_OP_REG_SEL_SHIFT);

    /*	Write reg value	*/
    dataToSlave[0] = VDD1_OP_REG;
    dataToSlave[1] = opVolSelector;
    tCount = 0;
    rCount = 0;
    SetupI2CTransmit(2);

    /*	Read reg value to verify */
    dataToSlave[0] = VDD1_OP_REG;
    dataFromSlave[0] = 0; // clear receive buffer
    dataFromSlave[1] = 0;
    rCount = 0;
    tCount = 0;
    SetupReception(1);

    while((dataFromSlave[0] & PMIC_VDD1_OP_REG_SEL) != 
          (opVolSelector << PMIC_VDD1_OP_REG_SEL_SHIFT));

#endif
}

/*
 * \brief Configures the VDD OP voltage. 
 *
 * \param  none.
 *
 * \return none
 */

void ConfigVddOpVoltage(void)
{
    SetupI2C();

#ifdef beaglebone

    unsigned char pmic_status = 0;

    /* Configure PMIC slave address */
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_TPS65217_I2C_SLAVE_ADDR);

    TPS65217RegRead(STATUS, &pmic_status);

    /* Increase USB current limit to 1300mA */
    TPS65217RegWrite(PROT_LEVEL_NONE, POWER_PATH, USB_INPUT_CUR_LIMIT_1300MA,
                       USB_INPUT_CUR_LIMIT_MASK);

    /* Set DCDC2 (MPU) voltage to 1.275V */
    TPS65217VoltageUpdate(DEFDCDC2, DCDC_VOLT_SEL_1275MV);

    /* Set LDO3, LDO4 output voltage to 3.3V */
    TPS65217RegWrite(PROT_LEVEL_2, DEFLS1, LDO_VOLTAGE_OUT_3_3, LDO_MASK);


    TPS65217RegWrite(PROT_LEVEL_2, DEFLS2, LDO_VOLTAGE_OUT_3_3, LDO_MASK);

#elif  defined (evmAM335x) || defined (evmskAM335x)

    /* Configure PMIC slave address */
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_CNTL_I2C_SLAVE_ADDR);

	/* Select SR I2C(0) */
    SelectI2CInstance(PMIC_DEVCTRL_REG_SR_CTL_I2C_SEL_CTL_I2C);

    /* Configure vdd1- need to validate these parameters */
    ConfigureVdd1(PMIC_VDD1_REG_VGAIN_SEL_X1, PMIC_VDD1_REG_ILMAX_1_5_A,
	              PMIC_VDD1_REG_TSTEP_12_5, PMIC_VDD1_REG_ST_ON_HI_POW);

    /* Select the source for VDD1 control */
    SelectVdd1Source(PMIC_VDD1_OP_REG_CMD_OP);

#else

    #error Unsupported EVM !!

#endif
}

/*
 * \brief This function sets up the DDR PHY
 *
 * \param  none
 *
 * \return none
 */
#ifdef am335x
static void DDR2PhyInit(void)
{
    /* Enable VTP */
    HWREG(SOC_CONTROL_REGS + CONTROL_VTP_CTRL) |= CONTROL_VTP_CTRL_ENABLE;
    HWREG(SOC_CONTROL_REGS + CONTROL_VTP_CTRL) &= ~CONTROL_VTP_CTRL_CLRZ;
    HWREG(SOC_CONTROL_REGS + CONTROL_VTP_CTRL) |= CONTROL_VTP_CTRL_CLRZ;
    while((HWREG(SOC_CONTROL_REGS + CONTROL_VTP_CTRL) & CONTROL_VTP_CTRL_READY) !=
                CONTROL_VTP_CTRL_READY);

    /* DDR PHY CMD0 Register configuration */
    HWREG(CMD0_SLAVE_RATIO_0)   = DDR2_CMD0_SLAVE_RATIO_0;
    HWREG(CMD0_SLAVE_FORCE_0)   = DDR2_CMD0_SLAVE_FORCE_0;
    HWREG(CMD0_SLAVE_DELAY_0)   = DDR2_CMD0_SLAVE_DELAY_0;
    HWREG(CMD0_LOCK_DIFF_0)     = DDR2_CMD0_LOCK_DIFF_0;
    HWREG(CMD0_INVERT_CLKOUT_0) = DDR2_CMD0_INVERT_CLKOUT_0;

    /* DDR PHY CMD1 Register configuration */
    HWREG(CMD1_SLAVE_RATIO_0)   = DDR2_CMD1_SLAVE_RATIO_0;
    HWREG(CMD1_SLAVE_FORCE_0)   =  DDR2_CMD1_SLAVE_FORCE_0;
    HWREG(CMD1_SLAVE_DELAY_0)   = DDR2_CMD1_SLAVE_DELAY_0;
    HWREG(CMD1_LOCK_DIFF_0)     = DDR2_CMD1_LOCK_DIFF_0;
    HWREG(CMD1_INVERT_CLKOUT_0) = DDR2_CMD1_INVERT_CLKOUT_0;

    /* DDR PHY CMD2 Register configuration */
    HWREG(CMD2_SLAVE_RATIO_0)   = DDR2_CMD2_SLAVE_RATIO_0;
    HWREG(CMD2_SLAVE_FORCE_0)   = DDR2_CMD2_SLAVE_FORCE_0;
    HWREG(CMD2_SLAVE_DELAY_0)   = DDR2_CMD2_SLAVE_DELAY_0;
    HWREG(CMD2_LOCK_DIFF_0)     = DDR2_CMD2_LOCK_DIFF_0;
    HWREG(CMD2_INVERT_CLKOUT_0) = DDR2_CMD2_INVERT_CLKOUT_0;

    /* DATA macro configuration */
    HWREG(DATA0_RD_DQS_SLAVE_RATIO_0)  = DDR2_DATA0_RD_DQS_SLAVE_RATIO_0;
    HWREG(DATA0_WR_DQS_SLAVE_RATIO_0)  = DDR2_DATA0_WR_DQS_SLAVE_RATIO_0;
    HWREG(DATA0_FIFO_WE_SLAVE_RATIO_0) = DDR2_DATA0_FIFO_WE_SLAVE_RATIO_0;
    HWREG(DATA0_WR_DATA_SLAVE_RATIO_0) = DDR2_DATA0_WR_DATA_SLAVE_RATIO_0;

    HWREG(DATA1_RD_DQS_SLAVE_RATIO_0)  = DDR2_DATA1_RD_DQS_SLAVE_RATIO_0;
    HWREG(DATA1_WR_DQS_SLAVE_RATIO_0)  = DDR2_DATA1_WR_DQS_SLAVE_RATIO_0;
    HWREG(DATA1_FIFO_WE_SLAVE_RATIO_0) = DDR2_DATA1_FIFO_WE_SLAVE_RATIO_0;
    HWREG(DATA1_WR_DATA_SLAVE_RATIO_0) = DDR2_DATA1_WR_DATA_SLAVE_RATIO_0;
}
/*
 * \brief This function sets up the DDR PHY
 *  1. enables VTP - config_vtp
 *  2. configures DDR CMD - config_cmd_ctrl
 *  3. configure DDR DATA - config_ddr_data
 * \param  none
 *
 * \return none
 */
static void DDR3PhyInit(void)
{
    /* Enable VTP */
    HWREG(SOC_CONTROL_REGS + CONTROL_VTP_CTRL) |= CONTROL_VTP_CTRL_ENABLE;
    HWREG(SOC_CONTROL_REGS + CONTROL_VTP_CTRL) &= ~CONTROL_VTP_CTRL_CLRZ;
    HWREG(SOC_CONTROL_REGS + CONTROL_VTP_CTRL) |= CONTROL_VTP_CTRL_CLRZ;
    while((HWREG(SOC_CONTROL_REGS + CONTROL_VTP_CTRL) & CONTROL_VTP_CTRL_READY) !=
                CONTROL_VTP_CTRL_READY);

    /* DDR PHY CMD0 Register configuration */
    HWREG(CMD0_SLAVE_RATIO_0)   = DDR3_CMD0_SLAVE_RATIO_0;
    HWREG(CMD0_INVERT_CLKOUT_0) = DDR3_CMD0_INVERT_CLKOUT_0;

    /* DDR PHY CMD1 Register configuration */
    HWREG(CMD1_SLAVE_RATIO_0)   = DDR3_CMD1_SLAVE_RATIO_0;
    HWREG(CMD1_INVERT_CLKOUT_0) = DDR3_CMD1_INVERT_CLKOUT_0;

    /* DDR PHY CMD2 Register configuration */
    HWREG(CMD2_SLAVE_RATIO_0)   = DDR3_CMD2_SLAVE_RATIO_0;
    HWREG(CMD2_INVERT_CLKOUT_0) = DDR3_CMD2_INVERT_CLKOUT_0;

    /* DATA macro configuration */
    HWREG(DATA0_RD_DQS_SLAVE_RATIO_0)  = DDR3_DATA0_RD_DQS_SLAVE_RATIO_0;
    HWREG(DATA0_WR_DQS_SLAVE_RATIO_0)  = DDR3_DATA0_WR_DQS_SLAVE_RATIO_0;
    HWREG(DATA0_FIFO_WE_SLAVE_RATIO_0) = DDR3_DATA0_FIFO_WE_SLAVE_RATIO_0;
    HWREG(DATA0_WR_DATA_SLAVE_RATIO_0) = DDR3_DATA0_WR_DATA_SLAVE_RATIO_0;
    HWREG(DATA1_RD_DQS_SLAVE_RATIO_0)  = DDR3_DATA0_RD_DQS_SLAVE_RATIO_1;
    HWREG(DATA1_WR_DQS_SLAVE_RATIO_0)  = DDR3_DATA0_WR_DQS_SLAVE_RATIO_1;
    HWREG(DATA1_FIFO_WE_SLAVE_RATIO_0) = DDR3_DATA0_FIFO_WE_SLAVE_RATIO_1;
    HWREG(DATA1_WR_DATA_SLAVE_RATIO_0) = DDR3_DATA0_WR_DATA_SLAVE_RATIO_1;

}


/* \brief This function initializes the DDR2
 *  
 * config_io_ctrl
 * config_ddr_phy
 * set_sdram_timings
 * config_sdram
 * \param none
 *
 * \return none
 *
 */
void DDR3Init(void)
{
    /* DDR3 Phy Initialization */
    DDR3PhyInit();

    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(0)) =
                                                 DDR3_CONTROL_DDR_CMD_IOCTRL_0;
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(1)) =
                                                 DDR3_CONTROL_DDR_CMD_IOCTRL_1;
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(2)) =
                                                 DDR3_CONTROL_DDR_CMD_IOCTRL_2;
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_DATA_IOCTRL(0)) =
                                                 DDR3_CONTROL_DDR_DATA_IOCTRL_0;
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_DATA_IOCTRL(1)) =
                                                 DDR3_CONTROL_DDR_DATA_IOCTRL_1;

    /* IO to work for DDR3 */
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_IO_CTRL) &= DDR3_CONTROL_DDR_IO_CTRL;

    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_CKE_CTRL) |= CONTROL_DDR_CKE_CTRL_DDR_CKE_CTRL;

    HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1) = DDR3_EMIF_DDR_PHY_CTRL_1;

    /* Dynamic Power Down */
    if((DEVICE_VERSION_2_0 == deviceVersion) ||
       (DEVICE_VERSION_2_1 == deviceVersion))
    {
        HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1) |=
                                              DDR3_EMIF_DDR_PHY_CTRL_1_DY_PWRDN;
    }

    HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1_SHDW) =
                                                 DDR3_EMIF_DDR_PHY_CTRL_1_SHDW;

    /* Dynamic Power Down */
    if((DEVICE_VERSION_2_0 == deviceVersion) ||
       (DEVICE_VERSION_2_1 == deviceVersion))
    {
        HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1_SHDW) |=
                                         DDR3_EMIF_DDR_PHY_CTRL_1_SHDW_DY_PWRDN;
    }

    HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_2) = DDR3_EMIF_DDR_PHY_CTRL_2;

    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_1)      = DDR3_EMIF_SDRAM_TIM_1;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_1_SHDW) = DDR3_EMIF_SDRAM_TIM_1_SHDW;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_2)      = DDR3_EMIF_SDRAM_TIM_2;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_2_SHDW) = DDR3_EMIF_SDRAM_TIM_2_SHDW;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_3)      = DDR3_EMIF_SDRAM_TIM_3;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_3_SHDW) = DDR3_EMIF_SDRAM_TIM_3_SHDM;

    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL)   = DDR3_EMIF_SDRAM_REF_CTRL_VAL1;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL_SHDW) =
                                                 DDR3_EMIF_SDRAM_REF_CTRL_SHDW_VAL1;

    HWREG(SOC_EMIF_0_REGS + EMIF_ZQ_CONFIG)     = DDR3_EMIF_ZQ_CONFIG_VAL;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_CONFIG)     = DDR3_EMIF_SDRAM_CONFIG;
	
    /* The CONTROL_SECURE_EMIF_SDRAM_CONFIG register exports SDRAM configuration 
       information to the EMIF */
    HWREG(SOC_CONTROL_REGS + CONTROL_SECURE_EMIF_SDRAM_CONFIG) = DDR3_EMIF_SDRAM_CONFIG;

}

/* \brief This function initializes the DDR2
 * 
 * \param none
 *
 * \return none
 *
 */
void DDR2Init(void)
{
    volatile unsigned int delay = 5000;

    /* DDR2 Phy Initialization */
    DDR2PhyInit();

    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(0)) =
                                                 DDR2_CONTROL_DDR_CMD_IOCTRL_0;
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(1)) =
                                                 DDR2_CONTROL_DDR_CMD_IOCTRL_1;
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_CMD_IOCTRL(2)) =
                                                 DDR2_CONTROL_DDR_CMD_IOCTRL_2;
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_DATA_IOCTRL(0)) =
                                                 DDR2_CONTROL_DDR_DATA_IOCTRL_0;
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_DATA_IOCTRL(1)) =
                                                 DDR2_CONTROL_DDR_DATA_IOCTRL_1;

    /* IO to work for DDR2 */
    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_IO_CTRL) &= DDR2_CONTROL_DDR_IO_CTRL;

    HWREG(SOC_CONTROL_REGS + CONTROL_DDR_CKE_CTRL) |= CONTROL_DDR_CKE_CTRL_DDR_CKE_CTRL;


    HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1) = DDR2_EMIF_DDR_PHY_CTRL_1;

    /* Dynamic Power Down */
    if((DEVICE_VERSION_2_0 == deviceVersion) ||
       (DEVICE_VERSION_2_1 == deviceVersion))
    {
        HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1) |=
                                              DDR2_EMIF_DDR_PHY_CTRL_1_DY_PWRDN;
    }

    HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1_SHDW) =
                                                 DDR2_EMIF_DDR_PHY_CTRL_1_SHDW;

    /* Dynamic Power Down */
    if((DEVICE_VERSION_2_0 == deviceVersion) ||
       (DEVICE_VERSION_2_1 == deviceVersion))
    {
        HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_1_SHDW) |=
                                         DDR2_EMIF_DDR_PHY_CTRL_1_SHDW_DY_PWRDN;
    }

    HWREG(SOC_EMIF_0_REGS + EMIF_DDR_PHY_CTRL_2) = DDR2_EMIF_DDR_PHY_CTRL_2;

    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_1)      =  DDR2_EMIF_SDRAM_TIM_1;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_1_SHDW) = DDR2_EMIF_SDRAM_TIM_1_SHDW;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_2)      = DDR2_EMIF_SDRAM_TIM_2;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_2_SHDW) = DDR2_EMIF_SDRAM_TIM_2_SHDW;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_3)      = DDR2_EMIF_SDRAM_TIM_3;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_TIM_3_SHDW) = DDR2_EMIF_SDRAM_TIM_3_SHDM;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_CONFIG)     = DDR2_EMIF_SDRAM_CONFIG;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL)   = DDR2_EMIF_SDRAM_REF_CTRL_VAL1;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL_SHDW) =
                                                 DDR2_EMIF_SDRAM_REF_CTRL_SHDW_VAL1;

    while(delay--);

    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL) = DDR2_EMIF_SDRAM_REF_CTRL_VAL2;
    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_REF_CTRL_SHDW) =
                                                 DDR2_EMIF_SDRAM_REF_CTRL_SHDW_VAL2;

    HWREG(SOC_EMIF_0_REGS + EMIF_SDRAM_CONFIG)   = DDR2_EMIF_SDRAM_CONFIG;
	
    /* The CONTROL_SECURE_EMIF_SDRAM_CONFIG register exports SDRAM configuration 
       information to the EMIF */
    HWREG(SOC_CONTROL_REGS + CONTROL_SECURE_EMIF_SDRAM_CONFIG) = DDR2_EMIF_SDRAM_CONFIG;

}
#else
#error "---------------------------------------------------"
#error "           UNSUPPORTED MEMORY CONFIGURATION        "
#error "---------------------------------------------------"
#endif

typedef unsigned int u32;

#define dmb()           __asm__ __volatile__ ("" : : : "memory")
#define __iowmb()       dmb()
#define __iormb()       dmb()
#define __arch_putl(v,a)                (*(volatile unsigned int *)(a) = (v))
#define __arch_getl(a)                  (*(volatile unsigned int *)(a))

#define writel(v,c)     ({ u32 __v = v; __iowmb(); __arch_putl(__v,c); __v; })
#define readl(c)        ({ u32 __v = __arch_getl(c); __iormb(); __v; })


/* VTP Registers */
struct vtp_reg {
        unsigned int vtp0ctrlreg;
};     

/* VTP Base Address */
#define VTP0_CTRL_ADDR	0x44E10E0C
#define VTP1_CTRL_ADDR	0x44E10E10

/* AM335X EMIF Register values */
#define VTP_CTRL_READY          (0x1 << 5)
#define VTP_CTRL_ENABLE         (0x1 << 6)
#define VTP_CTRL_START_EN       (0x1)

static struct vtp_reg *vtpreg[2] = {
                                (struct vtp_reg *)VTP0_CTRL_ADDR,
                                (struct vtp_reg *)VTP1_CTRL_ADDR};


void config_vtp(int nr)
{
      writel(readl(&vtpreg[nr]->vtp0ctrlreg) | VTP_CTRL_ENABLE,
                        &vtpreg[nr]->vtp0ctrlreg);
        writel(readl(&vtpreg[nr]->vtp0ctrlreg) & (~VTP_CTRL_START_EN),
                        &vtpreg[nr]->vtp0ctrlreg);
        writel(readl(&vtpreg[nr]->vtp0ctrlreg) | VTP_CTRL_START_EN,
                        &vtpreg[nr]->vtp0ctrlreg);
        
        /* Poll for READY */
        while ((readl(&vtpreg[nr]->vtp0ctrlreg) & VTP_CTRL_READY) !=
                        VTP_CTRL_READY)
                ;

}

/**
 * Encapsulates DDR CMD control registers.
 */     
struct cmd_control {
        unsigned long cmd0csratio;
        unsigned long cmd0csforce;
        unsigned long cmd0csdelay;
        unsigned long cmd0iclkout;
        unsigned long cmd1csratio;
        unsigned long cmd1csforce;
        unsigned long cmd1csdelay;
        unsigned long cmd1iclkout;
        unsigned long cmd2csratio;
        unsigned long cmd2csforce;
        unsigned long cmd2csdelay;
        unsigned long cmd2iclkout;
};      


/* Micron MT41K256M16HA-125E */
#define MT41K256M16HA125E_EMIF_READ_LATENCY     0x100007
#define MT41K256M16HA125E_EMIF_TIM1             0x0AAAD4DB
#define MT41K256M16HA125E_EMIF_TIM2             0x266B7FDA
#define MT41K256M16HA125E_EMIF_TIM3             0x501F867F
#define MT41K256M16HA125E_EMIF_SDCFG            0x61C05332
#define MT41K256M16HA125E_EMIF_SDREF            0xC30
#define MT41K256M16HA125E_ZQ_CFG                0x50074BE4
#define MT41K256M16HA125E_RATIO                 0x80
#define MT41K256M16HA125E_INVERT_CLKOUT         0x0
#define MT41K256M16HA125E_RD_DQS                0x38
#define MT41K256M16HA125E_WR_DQS                0x44
#define MT41K256M16HA125E_PHY_WR_DATA           0x7D
#define MT41K256M16HA125E_PHY_FIFO_WE           0x94
#define MT41K256M16HA125E_IOCTRL_VALUE          0x18B

static const struct cmd_control ddr3_beagleblack_cmd_ctrl_data = {
        .cmd0csratio = MT41K256M16HA125E_RATIO,
        .cmd0iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

        .cmd1csratio = MT41K256M16HA125E_RATIO,
        .cmd1iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

        .cmd2csratio = MT41K256M16HA125E_RATIO,
        .cmd2iclkout = MT41K256M16HA125E_INVERT_CLKOUT,
};

/* DDR Base address */
#define DDR_PHY_CMD_ADDR                0x44E12000
#define DDR_PHY_DATA_ADDR               0x44E120C8
#define DDR_PHY_CMD_ADDR2               0x47C0C800
#define DDR_PHY_DATA_ADDR2              0x47C0C8C8
#define DDR_DATA_REGS_NR                2


struct ddr_cmd_regs {
        unsigned int resv0[7];  
        unsigned int cm0csratio;        /* offset 0x01C */
        unsigned int resv1[3];
        unsigned int cm0iclkout;        /* offset 0x02C */
        unsigned int resv2[8];  
        unsigned int cm1csratio;        /* offset 0x050 */
        unsigned int resv3[3];
        unsigned int cm1iclkout;        /* offset 0x060 */
        unsigned int resv4[8];
        unsigned int cm2csratio;        /* offset 0x084 */
        unsigned int resv5[3];
        unsigned int cm2iclkout;        /* offset 0x094 */
        unsigned int resv6[3];
};

/**
 * Base addresses for DDR PHY cmd/data regs
 */
static struct ddr_cmd_regs *ddr_cmd_reg[2] = {
                                (struct ddr_cmd_regs *)DDR_PHY_CMD_ADDR,
                                (struct ddr_cmd_regs *)DDR_PHY_CMD_ADDR2};


/**
 * Configure DDR CMD control registers
 */     
void config_cmd_ctrl(const struct cmd_control *cmd, int nr)
{

      if (!cmd)
                return;
                
        writel(cmd->cmd0csratio, &ddr_cmd_reg[nr]->cm0csratio);
        writel(cmd->cmd0iclkout, &ddr_cmd_reg[nr]->cm0iclkout);

        writel(cmd->cmd1csratio, &ddr_cmd_reg[nr]->cm1csratio);
        writel(cmd->cmd1iclkout, &ddr_cmd_reg[nr]->cm1iclkout);
        
        writel(cmd->cmd2csratio, &ddr_cmd_reg[nr]->cm2csratio);
        writel(cmd->cmd2iclkout, &ddr_cmd_reg[nr]->cm2iclkout);
}

/**
 * Encapsulates DDR DATA registers.
 */
struct ddr_data {
        unsigned long datardsratio0;
        unsigned long datawdsratio0;
        unsigned long datawiratio0;
        unsigned long datagiratio0;
        unsigned long datafwsratio0;
        unsigned long datawrsratio0;
};    

struct ddr_data_regs {
        unsigned int dt0rdsratio0;      /* offset 0x0C8 */
        unsigned int resv1[4];  
        unsigned int dt0wdsratio0;      /* offset 0x0DC */
        unsigned int resv2[4];
        unsigned int dt0wiratio0;       /* offset 0x0F0 */
        unsigned int resv3;     
        unsigned int dt0wimode0;        /* offset 0x0F8 */
        unsigned int dt0giratio0;       /* offset 0x0FC */
        unsigned int resv4;
        unsigned int dt0gimode0;        /* offset 0x104 */
        unsigned int dt0fwsratio0;      /* offset 0x108 */
        unsigned int resv5[4];
        unsigned int dt0dqoffset;       /* offset 0x11C */
        unsigned int dt0wrsratio0;      /* offset 0x120 */
        unsigned int resv6[4];
        unsigned int dt0rdelays0;       /* offset 0x134 */
        unsigned int dt0dldiff0;        /* offset 0x138 */
        unsigned int resv7[12];
};

static const struct ddr_data ddr3_beagleblack_data = {
        .datardsratio0 = MT41K256M16HA125E_RD_DQS,
        .datawdsratio0 = MT41K256M16HA125E_WR_DQS,
        .datafwsratio0 = MT41K256M16HA125E_PHY_FIFO_WE,
        .datawrsratio0 = MT41K256M16HA125E_PHY_WR_DATA,
};

static struct ddr_data_regs *ddr_data_reg[2] = {
                                (struct ddr_data_regs *)DDR_PHY_DATA_ADDR,
                                (struct ddr_data_regs *)DDR_PHY_DATA_ADDR2};


void config_ddr_data(const struct ddr_data *data, int nr)
{
       int i;

        if (!data)
                return;

        for (i = 0; i < DDR_DATA_REGS_NR; i++) {
                writel(data->datardsratio0,
                        &(ddr_data_reg[nr]+i)->dt0rdsratio0);
                writel(data->datawdsratio0,
                        &(ddr_data_reg[nr]+i)->dt0wdsratio0);
                writel(data->datawiratio0,
                        &(ddr_data_reg[nr]+i)->dt0wiratio0);
                writel(data->datagiratio0,
                        &(ddr_data_reg[nr]+i)->dt0giratio0);
                writel(data->datafwsratio0,
                        &(ddr_data_reg[nr]+i)->dt0fwsratio0);
                writel(data->datawrsratio0,
                        &(ddr_data_reg[nr]+i)->dt0wrsratio0);
        }

}

/* DDR Base address */ 
#define DDR_CTRL_ADDR                   0x44E10E04
#define DDR_CONTROL_BASE_ADDR           0x44E11404

/**
 * This structure represents the DDR io control on AM33XX devices.
 */
struct ddr_cmdtctrl {
        unsigned int cm0ioctl;
        unsigned int cm1ioctl;
        unsigned int cm2ioctl;
        unsigned int resv2[12];
        unsigned int dt0ioctl;
        unsigned int dt1ioctl;
        unsigned int dt2ioctrl;
        unsigned int dt3ioctrl;
        unsigned int resv3[4];
        unsigned int emif_sdram_config_ext;
};


struct ddr_ctrl {
        unsigned int ddrioctrl;
        unsigned int resv1[325];
        unsigned int ddrckectrl;
};

/**
 * Base address for ddr io control instances
 */
static struct ddr_cmdtctrl *ioctrl_reg = {
                        (struct ddr_cmdtctrl *)DDR_CONTROL_BASE_ADDR};

static struct ddr_ctrl *ddrctrl = (struct ddr_ctrl *)DDR_CTRL_ADDR;
#define DDR_CKE_CTRL_NORMAL     0x1


struct ctrl_ioregs {
        unsigned int cm0ioctl;
        unsigned int cm1ioctl;
        unsigned int cm2ioctl;
        unsigned int dt0ioctl;
        unsigned int dt1ioctl;
        unsigned int dt2ioctrl;
        unsigned int dt3ioctrl;
        unsigned int emif_sdram_config_ext;
};      

const struct ctrl_ioregs ioregs_bonelt = {
        .cm0ioctl               = MT41K256M16HA125E_IOCTRL_VALUE,
        .cm1ioctl               = MT41K256M16HA125E_IOCTRL_VALUE,
        .cm2ioctl               = MT41K256M16HA125E_IOCTRL_VALUE,
        .dt0ioctl               = MT41K256M16HA125E_IOCTRL_VALUE,
        .dt1ioctl               = MT41K256M16HA125E_IOCTRL_VALUE,
};


void config_io_ctrl(const struct ctrl_ioregs *ioregs)
{

        if (!ioregs)
                return;
        
        writel(ioregs->cm0ioctl, &ioctrl_reg->cm0ioctl);
        writel(ioregs->cm1ioctl, &ioctrl_reg->cm1ioctl);
        writel(ioregs->cm2ioctl, &ioctrl_reg->cm2ioctl);
        writel(ioregs->dt0ioctl, &ioctrl_reg->dt0ioctl);
        writel(ioregs->dt1ioctl, &ioctrl_reg->dt1ioctl);
}


/* Base address */
#define EMIF1_BASE                              0x4c000000
#define EMIF2_BASE                              0x4d000000

/*
 * Structure containing shadow of important registers in EMIF
 * The calculation function fills in this structure to be later used for
 * initialization and DVFS
 */
struct emif_regs {
        unsigned int freq;
        unsigned int sdram_config_init;
        unsigned int sdram_config;
        unsigned int sdram_config2;
        unsigned int ref_ctrl;
        unsigned int sdram_tim1;
        unsigned int sdram_tim2;
        unsigned int sdram_tim3;
        unsigned int read_idle_ctrl;
        unsigned int zq_config;
        unsigned int temp_alert_config;
        unsigned int emif_ddr_phy_ctlr_1_init;
        unsigned int emif_ddr_phy_ctlr_1;
        unsigned int emif_ddr_ext_phy_ctrl_1;
        unsigned int emif_ddr_ext_phy_ctrl_2;
        unsigned int emif_ddr_ext_phy_ctrl_3;
        unsigned int emif_ddr_ext_phy_ctrl_4;
        unsigned int emif_ddr_ext_phy_ctrl_5;
        unsigned int emif_rd_wr_lvl_rmp_win;
        unsigned int emif_rd_wr_lvl_rmp_ctl;
        unsigned int emif_rd_wr_lvl_ctl;
        unsigned int emif_rd_wr_exec_thresh;
        unsigned int emif_prio_class_serv_map;
        unsigned int emif_connect_id_serv_1_map;
        unsigned int emif_connect_id_serv_2_map;
        unsigned int emif_cos_config;
};
static struct emif_regs ddr3_beagleblack_emif_reg_data = {
        .sdram_config = MT41K256M16HA125E_EMIF_SDCFG,
        .ref_ctrl = MT41K256M16HA125E_EMIF_SDREF,
        .sdram_tim1 = MT41K256M16HA125E_EMIF_TIM1,
        .sdram_tim2 = MT41K256M16HA125E_EMIF_TIM2,
        .sdram_tim3 = MT41K256M16HA125E_EMIF_TIM3,
        .zq_config = MT41K256M16HA125E_ZQ_CFG,
        .emif_ddr_phy_ctlr_1 = MT41K256M16HA125E_EMIF_READ_LATENCY,
};


/* EMIF Base address */ 
#define EMIF4_0_CFG_BASE                0x4C000000
#define EMIF4_1_CFG_BASE                0x4D000000

/* Reg mapping structure */
struct emif_reg_struct {        
        u32 emif_mod_id_rev;    
        u32 emif_status;
        u32 emif_sdram_config;
        u32 emif_lpddr2_nvm_config;
        u32 emif_sdram_ref_ctrl;
        u32 emif_sdram_ref_ctrl_shdw;
        u32 emif_sdram_tim_1;   
        u32 emif_sdram_tim_1_shdw;
        u32 emif_sdram_tim_2;
        u32 emif_sdram_tim_2_shdw;
        u32 emif_sdram_tim_3;   
        u32 emif_sdram_tim_3_shdw;
        u32 emif_lpddr2_nvm_tim;
        u32 emif_lpddr2_nvm_tim_shdw;
        u32 emif_pwr_mgmt_ctrl;
        u32 emif_pwr_mgmt_ctrl_shdw;
        u32 emif_lpddr2_mode_reg_data; 
        u32 padding1[1];
        u32 emif_lpddr2_mode_reg_data_es2;
        u32 padding11[1];
        u32 emif_lpddr2_mode_reg_cfg;
        u32 emif_l3_config;
        u32 emif_l3_cfg_val_1;
        u32 emif_l3_cfg_val_2;
        u32 emif_iodft_tlgc;
        u32 padding2[7];
        u32 emif_perf_cnt_1;
        u32 emif_perf_cnt_2;
        u32 emif_perf_cnt_cfg;
        u32 emif_perf_cnt_sel;
        u32 emif_perf_cnt_tim;
        u32 padding3;
        u32 emif_read_idlectrl;
        u32 emif_read_idlectrl_shdw;
        u32 padding4;
        u32 emif_irqstatus_raw_sys;
 u32 emif_irqstatus_raw_ll;
        u32 emif_irqstatus_sys;
        u32 emif_irqstatus_ll;
        u32 emif_irqenable_set_sys;
        u32 emif_irqenable_set_ll;
        u32 emif_irqenable_clr_sys;
        u32 emif_irqenable_clr_ll;
        u32 padding5;
        u32 emif_zq_config;
        u32 emif_temp_alert_config;
        u32 emif_l3_err_log;
        u32 emif_rd_wr_lvl_rmp_win;
        u32 emif_rd_wr_lvl_rmp_ctl;
        u32 emif_rd_wr_lvl_ctl;
        u32 padding6[1];
        u32 emif_ddr_phy_ctrl_1;
        u32 emif_ddr_phy_ctrl_1_shdw;
        u32 emif_ddr_phy_ctrl_2;
        u32 padding7[4];
        u32 emif_prio_class_serv_map;
        u32 emif_connect_id_serv_1_map;
        u32 emif_connect_id_serv_2_map;
        u32 padding8[5];
        u32 emif_rd_wr_exec_thresh;
        u32 emif_cos_config;
        u32 padding9[6];
        u32 emif_ddr_phy_status[21];
        u32 padding10[27];
        u32 emif_ddr_ext_phy_ctrl_1;
        u32 emif_ddr_ext_phy_ctrl_1_shdw;
        u32 emif_ddr_ext_phy_ctrl_2;
        u32 emif_ddr_ext_phy_ctrl_2_shdw;
        u32 emif_ddr_ext_phy_ctrl_3;
        u32 emif_ddr_ext_phy_ctrl_3_shdw;
        u32 emif_ddr_ext_phy_ctrl_4;
        u32 emif_ddr_ext_phy_ctrl_4_shdw;
u32 emif_ddr_ext_phy_ctrl_5;
        u32 emif_ddr_ext_phy_ctrl_5_shdw;
        u32 emif_ddr_ext_phy_ctrl_6;
        u32 emif_ddr_ext_phy_ctrl_6_shdw;
        u32 emif_ddr_ext_phy_ctrl_7;
        u32 emif_ddr_ext_phy_ctrl_7_shdw;
        u32 emif_ddr_ext_phy_ctrl_8;
        u32 emif_ddr_ext_phy_ctrl_8_shdw;
        u32 emif_ddr_ext_phy_ctrl_9;
        u32 emif_ddr_ext_phy_ctrl_9_shdw;
        u32 emif_ddr_ext_phy_ctrl_10;
        u32 emif_ddr_ext_phy_ctrl_10_shdw;
        u32 emif_ddr_ext_phy_ctrl_11;
        u32 emif_ddr_ext_phy_ctrl_11_shdw;
        u32 emif_ddr_ext_phy_ctrl_12;
        u32 emif_ddr_ext_phy_ctrl_12_shdw;
        u32 emif_ddr_ext_phy_ctrl_13;
        u32 emif_ddr_ext_phy_ctrl_13_shdw;
        u32 emif_ddr_ext_phy_ctrl_14;
        u32 emif_ddr_ext_phy_ctrl_14_shdw;
        u32 emif_ddr_ext_phy_ctrl_15;
        u32 emif_ddr_ext_phy_ctrl_15_shdw;
        u32 emif_ddr_ext_phy_ctrl_16;
        u32 emif_ddr_ext_phy_ctrl_16_shdw;
        u32 emif_ddr_ext_phy_ctrl_17;
        u32 emif_ddr_ext_phy_ctrl_17_shdw;
        u32 emif_ddr_ext_phy_ctrl_18;
        u32 emif_ddr_ext_phy_ctrl_18_shdw;
        u32 emif_ddr_ext_phy_ctrl_19;
        u32 emif_ddr_ext_phy_ctrl_19_shdw;
        u32 emif_ddr_ext_phy_ctrl_20;
        u32 emif_ddr_ext_phy_ctrl_20_shdw;
        u32 emif_ddr_ext_phy_ctrl_21;
        u32 emif_ddr_ext_phy_ctrl_21_shdw;
        u32 emif_ddr_ext_phy_ctrl_22;
        u32 emif_ddr_ext_phy_ctrl_22_shdw;
        u32 emif_ddr_ext_phy_ctrl_23;
u32 emif_ddr_ext_phy_ctrl_23_shdw;
        u32 emif_ddr_ext_phy_ctrl_24;
        u32 emif_ddr_ext_phy_ctrl_24_shdw;
        u32 padding[22];
        u32 emif_ddr_fifo_misaligned_clear_1;
        u32 emif_ddr_fifo_misaligned_clear_2;
};

/**
 * Base address for EMIF instances
 */
static struct emif_reg_struct *emif_reg[2] = {
                                (struct emif_reg_struct *)EMIF4_0_CFG_BASE,
                                (struct emif_reg_struct *)EMIF4_1_CFG_BASE};

#define EMIF_REG_INITREF_DIS_MASK                       (1 << 31)
#define EMIF_REG_MAJOR_REVISION_MASK            (0x7 << 8)
#define EMIF_REG_MAJOR_REVISION_SHIFT           8
#define EMIF_4D5                                0x5


#define EMIF_EXT_PHY_CTRL_TIMING_REG    0x5

/*
 * Configure EXT PHY registers
 */
static void ext_phy_settings(const struct emif_regs *regs, int nr)
{
      u32 *ext_phy_ctrl_base = 0;
        u32 *emif_ext_phy_ctrl_base = 0;
        u32 i = 0;
        u32 size = 0;

        ext_phy_ctrl_base = (u32 *)&(regs->emif_ddr_ext_phy_ctrl_1);
        emif_ext_phy_ctrl_base =
                        (u32 *)&(emif_reg[nr]->emif_ddr_ext_phy_ctrl_1);

        /* Configure external phy control timing registers */
        for (i = 0; i < EMIF_EXT_PHY_CTRL_TIMING_REG; i++) {
                writel(*ext_phy_ctrl_base, emif_ext_phy_ctrl_base++);
                /* Update shadow registers */
                writel(*ext_phy_ctrl_base++, emif_ext_phy_ctrl_base++);
        }
	 if (!size)
                return;



}
static inline unsigned int get_emif_rev(u32 base)
{               
        struct emif_reg_struct *emif = (struct emif_reg_struct *)base;
        
        return (readl(&emif->emif_mod_id_rev) & EMIF_REG_MAJOR_REVISION_MASK)
                >> EMIF_REG_MAJOR_REVISION_SHIFT;
}
# define cpu_to_le32(x)         (x)
# define le32_to_cpu(x)         (x)

#define __raw_readl(a)          __arch_getl(a)
#define __raw_writel(v,a)       __arch_putl(v,a)

#define out_arch(type,endian,a,v)       __raw_write##type(cpu_to_##endian(v),a)
#define in_arch(type,endian,a)          endian##_to_cpu(__raw_read##type(a))


#define in_le32(a)      in_arch(l,le32,a)
#define out_le32(a,v)   out_arch(l,le32,a,v)

#define setbits(type, addr, set) \
        out_##type((addr), in_##type(addr) | (set))
#define setbits_le32(addr, set) setbits(le32, addr, set)

void config_ddr_phy(const struct emif_regs *regs, int nr)
{

      /*      
         * disable initialization and refreshes for now until we
         * finish programming EMIF regs.
         */
	setbits_le32(&emif_reg[nr]->emif_sdram_ref_ctrl,
                     EMIF_REG_INITREF_DIS_MASK);

        
        writel(regs->emif_ddr_phy_ctlr_1,
                &emif_reg[nr]->emif_ddr_phy_ctrl_1);
        writel(regs->emif_ddr_phy_ctlr_1,
                &emif_reg[nr]->emif_ddr_phy_ctrl_1_shdw);

      if (get_emif_rev((u32)emif_reg[nr]) == EMIF_4D5)
                ext_phy_settings(regs, nr);


}



/**                        
 * Set SDRAM timings
 */     
void set_sdram_timings(const struct emif_regs *regs, int nr)
{

        writel(regs->sdram_tim1, &emif_reg[nr]->emif_sdram_tim_1);
        writel(regs->sdram_tim1, &emif_reg[nr]->emif_sdram_tim_1_shdw);
        writel(regs->sdram_tim2, &emif_reg[nr]->emif_sdram_tim_2);
        writel(regs->sdram_tim2, &emif_reg[nr]->emif_sdram_tim_2_shdw);
        writel(regs->sdram_tim3, &emif_reg[nr]->emif_sdram_tim_3);
        writel(regs->sdram_tim3, &emif_reg[nr]->emif_sdram_tim_3_shdw);
}

/* Control Module Base Address */
#define CTRL_BASE                       0x44E10000
#define CTRL_DEVICE_BASE                0x44E10600


/* Control Status Register */
struct ctrl_stat {
        unsigned int resv1[16];
        unsigned int statusreg;         /* ofset 0x40 */
        unsigned int resv2[51];
        unsigned int secure_emif_sdram_config;  /* offset 0x0110 */
        unsigned int resv3[319];
        unsigned int dev_attr;
};

struct ctrl_stat *cstat = (struct ctrl_stat *)CTRL_BASE;

void config_sdram(const struct emif_regs *regs, int nr)
{
      if (regs->zq_config) {
                /*
                 * A value of 0x2800 for the REF CTRL will give us
                 * about 570us for a delay, which will be long enough
                 * to configure things.
                 */
                writel(0x2800, &emif_reg[nr]->emif_sdram_ref_ctrl);
                writel(regs->zq_config, &emif_reg[nr]->emif_zq_config);
                writel(regs->sdram_config, &cstat->secure_emif_sdram_config);
                writel(regs->sdram_config, &emif_reg[nr]->emif_sdram_config);
                writel(regs->ref_ctrl, &emif_reg[nr]->emif_sdram_ref_ctrl);
                writel(regs->ref_ctrl, &emif_reg[nr]->emif_sdram_ref_ctrl_shdw);
        }
        writel(regs->ref_ctrl, &emif_reg[nr]->emif_sdram_ref_ctrl);
        writel(regs->ref_ctrl, &emif_reg[nr]->emif_sdram_ref_ctrl_shdw);
        writel(regs->sdram_config, &emif_reg[nr]->emif_sdram_config);

}
/* \brief This function initializes the EMIF
 * 
 * \param none
 *
 * \return none
 *
 */
void EMIFInit(void)
{
    volatile unsigned int regVal;

    /* Enable the clocks for EMIF */
    regVal = HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL) &
                ~(CM_PER_EMIF_FW_CLKCTRL_MODULEMODE);

    regVal |= CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_ENABLE;

    HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL) = regVal;

    regVal = HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_CLKCTRL) &
                ~(CM_PER_EMIF_CLKCTRL_MODULEMODE);

    regVal |= CM_PER_EMIF_CLKCTRL_MODULEMODE_ENABLE;

    HWREG(SOC_CM_PER_REGS + CM_PER_EMIF_CLKCTRL) = regVal;

    while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
          (CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK |
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK)) !=
          (CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK |
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));
}

#if defined(NAND)

/******************************************************************************
*                                                                             *
*                                                                             *
* \brief  Function to initalize the GPMC NAND timing and base addr info.      *
*                                                                             *
* \param  nandTimimgInfo : Pointer to structure containing                    *
*                          NAND timing info.                                  *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void NANDTimingInfoInit(void *TimingInfo)
{

    GPMCNANDTimingInfo_t *nandTimingInfo;

    nandTimingInfo = (GPMCNANDTimingInfo_t * )TimingInfo;

    nandTimingInfo->CSWrOffTime               = NAND_CSWROFFTIME;
    nandTimingInfo->CSRdOffTime               = NAND_CSRDOFFTIME;
    nandTimingInfo->CSExtDelayFlag            = GPMC_CS_EXTRA_NODELAY;
    nandTimingInfo->CSOnTime                  = NAND_CSONTIME;

    nandTimingInfo->ADVAADMuxWrOffTime        = NAND_ADVAADMUXWROFFTIME;
    nandTimingInfo->ADVAADMuxRdOffTime        = NAND_ADVAADMUXRDOFFTIME;
    nandTimingInfo->ADVWrOffTime              = NAND_ADVWROFFTIME;
    nandTimingInfo->ADVRdOffTime              = NAND_ADVRDOFFTIME;
    nandTimingInfo->ADVExtDelayFlag           = GPMC_ADV_EXTRA_NODELAY;
    nandTimingInfo->ADVAADMuxOnTime           = NAND_ADVAADMUXONTIME;
    nandTimingInfo->ADVOnTime                 = NAND_ADVONTIME;

    nandTimingInfo->WEOffTime                 = NAND_WEOFFTIME;
    nandTimingInfo->WEExtDelayFlag            = GPMC_WE_EXTRA_NODELAY;
    nandTimingInfo->WEOnTime                  = NAND_WEONTIME;
    nandTimingInfo->OEAADMuxOffTime           = NAND_OEAADMUXOFFTIME;
    nandTimingInfo->OEOffTime                 = NAND_OEOFFTIME;
    nandTimingInfo->OEExtDelayFlag            = GPMC_OE_EXTRA_NODELAY;
    nandTimingInfo->OEAADMuxOnTime            = NAND_OEAADMUXONTIME;
    nandTimingInfo->OEOnTime                  = NAND_OEONTIME;

    nandTimingInfo->rdCycleTime               = NAND_RDCYCLETIME;
    nandTimingInfo->wrCycleTime               = NAND_WRCYCLETIME;
    nandTimingInfo->rdAccessTime              = NAND_RDACCESSTIME;
    nandTimingInfo->pageBurstAccessTime       = NAND_PAGEBURSTACCESSTIME;

    nandTimingInfo->cycle2CycleDelay          = NAND_CYCLE2CYCLEDELAY;
    nandTimingInfo->cycle2CycleDelaySameCSCfg = NAND_CYCLE2CYCLESAMECSEN;
    nandTimingInfo->cycle2CycleDelayDiffCSCfg = NAND_CYCLE2CYCLEDIFFCSEN;
    nandTimingInfo->busTAtime                 = NAND_BUSTURNAROUND;
}


/******************************************************************************
*                                                                             *
* \brief  Function to initialize the device and controller info.              *
*                                                                             *
* \param  nandInfo      : Pointer to structure containing controller and      *
*                         device information.                                 *
*                                                                             *
* \param  csNum         : Chip select where device is interfaced.             *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void BlPlatformNANDInfoInit(NandInfo_t *nandInfo)
{
    NandCtrlInfo_t *hNandCtrlInfo = nandInfo->hNandCtrlInfo;
    NandDmaInfo_t  *hNandDmaInfo  = nandInfo->hNandDmaInfo;
    NandEccInfo_t  *hNandEccInfo  = nandInfo->hNandEccInfo;

    /* Init the NAND Device Info */
    nandInfo->opMode                        = NAND_DATA_XFER_MODE;
    nandInfo->eccType                       = NAND_ECC_ALGO_BCH_8BIT;

    nandInfo->chipSelectCnt                 = 1;
    nandInfo->dieCnt                        = 1;
    nandInfo->chipSelects[0]                = NAND_CHIP_SELECT;
    nandInfo->busWidth                      = NAND_BUSWIDTH;
    nandInfo->pageSize                      = NAND_PAGE_SIZE_IN_BYTES;
    nandInfo->blkSize                       = NAND_BLOCK_SIZE_IN_BYTES;
    nandInfo->manId                         = NAND_MANUFATURER_MICRON_ID;
    nandInfo->devId                         = NAND_DEVICE_ID;
    nandInfo->dataRegAddr                   = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_DATA(GPMC_CHIP_SELECT_0));
    nandInfo->addrRegAddr                   = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_ADDRESS(GPMC_CHIP_SELECT_0));
    nandInfo->cmdRegAddr                    = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_COMMAND(GPMC_CHIP_SELECT_0));
    /* Init the NAND Controller Info struct */
    hNandCtrlInfo->CtrlInit                 = GPMCNANDInit;
    hNandCtrlInfo->WaitPinStatusGet         = GPMCNANDWaitPinStatusGet;
    hNandCtrlInfo->currChipSelect           = NAND_CHIP_SELECT;
    hNandCtrlInfo->baseAddr                 = SOC_GPMC_0_REGS;
    hNandCtrlInfo->eccSupported             = (NAND_ECC_ALGO_HAMMING_1BIT |
                                          NAND_ECC_ALGO_BCH_4BIT |
                                          NAND_ECC_ALGO_BCH_8BIT |
                                          NAND_ECC_ALGO_BCH_16BIT );

    hNandCtrlInfo->waitPin                  = GPMC_WAIT_PIN0;
    hNandCtrlInfo->waitPinPol               = GPMC_WAIT_PIN_POLARITY_LOW;
    hNandCtrlInfo->wpPinPol                 = GPMC_WP_PIN_LEVEL_HIGH;
    hNandCtrlInfo->chipSelectBaseAddr[0]    = NAND_CS0_BASEADDR;
    hNandCtrlInfo->chipSelectRegionSize[0]  = NAND_CS0_REGIONSIZE;
    hNandCtrlInfo->hNandTimingInfo          = &nandTimingInfo;
    NANDTimingInfoInit(hNandCtrlInfo->hNandTimingInfo);


    /* Init the NAND Ecc Info */
    hNandEccInfo->baseAddr                  = SOC_ELM_0_REGS;
    hNandEccInfo->ECCInit                   = GPMCNANDECCInit;
    hNandEccInfo->ECCEnable                 = GPMCNANDECCEnable;
    hNandEccInfo->ECCDisable                = GPMCNANDECCDisable;
    hNandEccInfo->ECCWriteSet               = GPMCNANDECCWriteSet;
    hNandEccInfo->ECCReadSet                = GPMCNANDECCReadSet;
    hNandEccInfo->ECCCalculate              = GPMCNANDECCCalculate;
    hNandEccInfo->ECCCheckAndCorrect        = GPMCNANDECCCheckAndCorrect;

    /* Init the NAND DMA info */
    hNandDmaInfo->DMAXfer                   = NULL;
    hNandDmaInfo->DMAInit                   = NULL;
    hNandDmaInfo->DMAXferSetup              = NULL;
    hNandDmaInfo->DMAXferStatusGet          = NULL;
}

/*
 * \brief This function is used to initialize and configure NAND.
 *
 * \param  none.
 *
 * \return none
*/
void BlPlatformNANDSetup(void)
{
    /* Enable clock for NAND and Do the PINMUXing */
    NANDPinMuxSetup();
    GPMCClkConfig();
}
#endif

#if defined(MMCSD)
/*
 * \brief This function is used to initialize and configure NAND.
 *
 * \param  none.
 *
 * \return none
*/
void BlPlatformMMCSDSetup(void)
{
    /* Enable clock for MMCSD and Do the PINMUXing */
    HSMMCSDPinMuxSetup();
    HSMMCSDModuleClkConfig();
}
#endif


#if defined(SPI)
/*
 * \brief This function is used to initialize and configure SPI Module.
 *
 * \param  none.
 *
 * \return none
*/
void BlPlatformSPISetup(void)
{
    unsigned int regVal;

    /* Enable clock for SPI */
    regVal = (HWREG(SOC_CM_PER_REGS + CM_PER_SPI0_CLKCTRL) &
                    ~(CM_PER_SPI0_CLKCTRL_MODULEMODE));

    regVal |= CM_PER_SPI0_CLKCTRL_MODULEMODE_ENABLE;

    HWREG(SOC_CM_PER_REGS + CM_PER_SPI0_CLKCTRL) = regVal;

    /* Setup SPI PINMUX */
    McSPIPinMuxSetup(0);
    McSPI0CSPinMuxSetup(0);

    SPIConfigure();
}
#endif

/*
 * \brief This function is used to initialize and configure UART Module.
 *
 * \param  none.
 *
 * \return none
*/

void UARTSetup(void)
{
    volatile unsigned int regVal;

    /* Enable clock for UART0 */
    regVal = (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_UART0_CLKCTRL) &
                    ~(CM_WKUP_UART0_CLKCTRL_MODULEMODE));

    regVal |= CM_WKUP_UART0_CLKCTRL_MODULEMODE_ENABLE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_UART0_CLKCTRL) = regVal;

    UARTStdioInit();
}




/*
 * \brief This function Initializes Pll, DDR and Uart
 *
 * \param  none
 *
 * \return none
*/
void BlPlatformConfig(void)
{
    BoardInfoInit();
    deviceVersion = DeviceVersionGet();
    
#if 0
    ConfigVddOpVoltage();

    oppMaxIdx = BootMaxOppGet();

    SetVdd1OpVoltage(oppTable[oppMaxIdx].pmicVolt);
#endif
	/* Disable the WDT */
    HWREG(SOC_WDT_1_REGS + WDT_WSPR) = 0xAAAAu;
    while(HWREG(SOC_WDT_1_REGS + WDT_WWPS) != 0x00);

    HWREG(SOC_WDT_1_REGS + WDT_WSPR) = 0x5555u;
    while(HWREG(SOC_WDT_1_REGS + WDT_WWPS) != 0x00);

#if 0
    /* Configure DDR frequency */
#ifdef evmskAM335x
    freqMultDDR = DDRPLL_M_DDR3;
#elif evmAM335x
    if(BOARD_ID_EVM_DDR3 == BoardIdGet())
    {
        freqMultDDR = DDRPLL_M_DDR3;
    }
    else if(BOARD_ID_EVM_DDR2 == BoardIdGet())
    {
        freqMultDDR = DDRPLL_M_DDR2;
    }
#else
    freqMultDDR = DDRPLL_M_DDR2;
#endif
#endif
    /* Set the PLL0 to generate 300MHz for ARM */
    freqMultDDR = DDRPLL_M_DDR3;
    PLLInit();
    EMIFInit();
    //DDRVTTEnable();
    /*    DDR3Init();*/
#if 1
    /* Enable the control module */ /*already done in InterfaceClock init*/
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) =
            CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;
    /* EMIF Initialization */

    /* DDR Initialization */

#ifdef evmskAM335x

    /* Enable DDR_VTT */
    DDR3Init();
#elif evmAM335x

    if(BOARD_ID_EVM_DDR3 == BoardIdGet())
    {
    }
    else if(BOARD_ID_EVM_DDR2 == BoardIdGet())
    {
        DDR2Init();
    }
#else

//    DDR2Init();
#endif
#endif
   
    config_vtp(0);
    config_cmd_ctrl(&ddr3_beagleblack_cmd_ctrl_data,0);
    config_ddr_data(&ddr3_beagleblack_data, 0);
    config_io_ctrl(&ioregs_bonelt);

        /* Set CKE to be controlled by EMIF/DDR PHY */
    writel(DDR_CKE_CTRL_NORMAL, &ddrctrl->ddrckectrl);

    /* Program EMIF instance */
    config_ddr_phy(&ddr3_beagleblack_emif_reg_data, 0);

    set_sdram_timings(&ddr3_beagleblack_emif_reg_data, 0);

    config_sdram(&ddr3_beagleblack_emif_reg_data, 0); /*emif not equal to EMIF_4D5*/    
    /* UART Initialization */
    UARTSetup();
}

/*
** Enable DDR_VTT.
**
*/
#ifdef evmskAM335x
static void DDRVTTEnable(void)
{
    GPIO0ModuleClkConfig();

    GPIO_PMUX_OFFADDR_VALUE(0, 7, PAD_FS_RXE_PD_PUPDE(7));

    /* Resetting the GPIO module. */
    GPIOModuleReset(SOC_GPIO_0_REGS);

    /* Enabling the GPIO module. */
    GPIOModuleEnable(SOC_GPIO_0_REGS);

    /* Setting the GPIO pin as an input pin. */    
    GPIODirModeSet(SOC_GPIO_0_REGS,
                   GPIO_INSTANCE_PIN_NUMBER,
                   GPIO_DIR_OUTPUT);

    GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_INSTANCE_PIN_NUMBER,
                 GPIO_PIN_HIGH);

}
#endif

/*
 * \brief This function does any post boot setup/init
 *
 * \param  none
 *
 * \return none
*/
void BlPlatformConfigPostBoot( void )
{
}

/*
 * \brief This function copies the image form SPI to RAM
 *
 * \param  none
 *
 * \return Status of the copy operation.
*/
#if defined(SPI)

unsigned int BlPlatformSPIImageCopy()
{
    ti_header header;

    /* Spi read function to read the header */
    BlSPIReadFlash(IMAGE_OFFSET, sizeof(header), (unsigned char *)&header);

    /* Copies application from SPI flash to DDR RAM */
    BlSPIReadFlash( IMAGE_OFFSET + 8,
                    header.image_size,
                    (unsigned char *)(header.load_addr) );
                     
    entryPoint = (unsigned int)header.load_addr;
    
    return (TRUE);
}
#endif

/*
 * \brief This function copies the image form MMCSD to RAM
 *
 * \param  none
 *
 * \return Status of the copy operation.
*/

#if defined(MMCSD)

unsigned int BlPlatformMMCSDImageCopy()
{
    UARTPuts("in BlPlatformMMCSDImageCopy\r\n\n", -1);
    HSMMCSDInit();
    UARTPuts("in BlPlatformMMCSDImageCopyinit done\r\n\n", -1);
    HSMMCSDImageCopy();

    return (TRUE);
}

#endif

/*
 * \brief This function copies the image form NAND to RAM
 *
 * \param  none
 *
 * \return Status of the copy operation.
*/

#if defined(NAND)

unsigned int BlPlatformNANDImageCopy(NandInfo_t *nandInfo)
{

    ti_header header;

    /* Spi read function to read the header */
    BlNANDReadFlash(nandInfo, IMAGE_OFFSET, sizeof(header), (unsigned char *)&header);

    /* Copies application from SPI flash to DDR RAM */
    BlNANDReadFlash( nandInfo, IMAGE_OFFSET + sizeof(header),
                     header.image_size, (unsigned char *)(header.load_addr) );

    entryPoint = (unsigned int)header.load_addr;

    return (TRUE);
}

#endif

/******************************************************************************
**                              END OF FILE
*******************************************************************************/
