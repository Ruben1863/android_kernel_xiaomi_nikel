/*
 * Generated by MTK SP DrvGen Version: 2.2.160113 for MT6797. Copyright MediaTek Inc. (C) 2015.
 * 01.18.2022 23:42:18
 * Do Not Modify the File.
 */

#ifndef __CUST_SCP_GPIO_USAGE_H
#define __CUST_SCP_GPIO_USAGE_H

#define GPIO_CAMERA_CMPDN_PIN			(GPIO28 | 0x80000000)
#define GPIO_CAMERA_CMPDN_PIN_M_CLK		GPIO_MODE_01
#define GPIO_CAMERA_CMPDN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CAMERA_CMPDN_PIN_M_PWM		GPIO_MODE_06

#define GPIO_CAMERA_CMPDN1_PIN			(GPIO29 | 0x80000000)
#define GPIO_CAMERA_CMPDN1_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_CMMCLK			(GPIO30 | 0x80000000)
#define GPIO_CMMCLK_M_CLK		GPIO_MODE_01
#define GPIO_CMMCLK_M_GPIO		GPIO_MODE_00

#define GPIO_CMMCLK1_PIN			(GPIO31 | 0x80000000)
#define GPIO_CMMCLK1_PIN_M_CLK		GPIO_MODE_01
#define GPIO_CMMCLK1_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_CAMERA_CMRST_PIN			(GPIO32 | 0x80000000)
#define GPIO_CAMERA_CMRST_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_CAMERA_CMRST1_PIN			(GPIO33 | 0x80000000)
#define GPIO_CAMERA_CMRST1_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_CAMERA_2_CMPDN_PIN			(GPIO34 | 0x80000000)
#define GPIO_CAMERA_2_CMPDN_PIN_M_CLK		GPIO_MODE_02
#define GPIO_CAMERA_2_CMPDN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CAMERA_2_CMPDN_PIN_CLK		CLK_OUT0
#define GPIO_CAMERA_2_CMPDN_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_CAMERA_2_CMRST_PIN			(GPIO35 | 0x80000000)
#define GPIO_CAMERA_2_CMRST_PIN_M_CLK		GPIO_MODE_01
#define GPIO_CAMERA_2_CMRST_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CAMERA_2_CMRST_PIN_M_PWM		GPIO_MODE_06
#define GPIO_CAMERA_2_CMRST_PIN_CLK		CLK_OUT1
#define GPIO_CAMERA_2_CMRST_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_CAMERA_2_CMMCLK_PIN			(GPIO36 | 0x80000000)
#define GPIO_CAMERA_2_CMMCLK_PIN_M_CLK		GPIO_MODE_01
#define GPIO_CAMERA_2_CMMCLK_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CAMERA_2_CMMCLK_PIN_M_PWM		GPIO_MODE_06
#define GPIO_CAMERA_2_CMMCLK_PIN_CLK		CLK_OUT2
#define GPIO_CAMERA_2_CMMCLK_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_I2C0_SCA_PIN			(GPIO37 | 0x80000000)
#define GPIO_I2C0_SCA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C0_SCA_PIN_M_SCL0_		GPIO_MODE_01

#define GPIO_I2C0_SDA_PIN			(GPIO38 | 0x80000000)
#define GPIO_I2C0_SDA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C0_SDA_PIN_M_SDA0_		GPIO_MODE_01

#define GPIO_LCDBL_EN_PIN			(GPIO50 | 0x80000000)
#define GPIO_LCDBL_EN_PIN_M_CLK		GPIO_MODE_04
#define GPIO_LCDBL_EN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_LCDBL_EN_PIN_M_PWM		GPIO_MODE_03
#define GPIO_LCDBL_EN_PIN_CLK		CLK_OUT2
#define GPIO_LCDBL_EN_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_I2C1_SCA_PIN			(GPIO55 | 0x80000000)
#define GPIO_I2C1_SCA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C1_SCA_PIN_M_SCL1_		GPIO_MODE_01

#define GPIO_I2C1_SDA_PIN			(GPIO56 | 0x80000000)
#define GPIO_I2C1_SDA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C1_SDA_PIN_M_SDA1_		GPIO_MODE_01

#define GPIO_NFC_EINT_PIN			(GPIO57 | 0x80000000)
#define GPIO_NFC_EINT_PIN_M_CLK		GPIO_MODE_01
#define GPIO_NFC_EINT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_NFC_EINT_PIN_M_PWM		GPIO_MODE_03

#define GPIO_IRQ_NFC_PIN			(GPIO58 | 0x80000000)
#define GPIO_IRQ_NFC_PIN_M_CLK		GPIO_MODE_05
#define GPIO_IRQ_NFC_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_NFC_RST_PIN			(GPIO59 | 0x80000000)
#define GPIO_NFC_RST_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_NFC_RST_PIN_M_PWM		GPIO_MODE_03

#define GPIO_NFC_VENB_PIN			(GPIO60 | 0x80000000)
#define GPIO_NFC_VENB_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_EXT_BUCK_OC_EINT_PIN			(GPIO61 | 0x80000000)
#define GPIO_EXT_BUCK_OC_EINT_PIN_M_CLK		GPIO_MODE_03
#define GPIO_EXT_BUCK_OC_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_EXT_BUCK_OC_EINT_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_GYRO_EINT_PIN			(GPIO63 | 0x80000000)
#define GPIO_GYRO_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_GYRO_EINT_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_GSE_1_EINT_PIN			(GPIO63 | 0x80000000)
#define GPIO_GSE_1_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_GSE_1_EINT_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_MSE_EINT_PIN			(GPIO64 | 0x80000000)
#define GPIO_MSE_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_MSE_EINT_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_LCD_DRV_EN_PIN			(GPIO65 | 0x80000000)
#define GPIO_LCD_DRV_EN_PIN_M_CLK		GPIO_MODE_02
#define GPIO_LCD_DRV_EN_PIN_M_EINT		GPIO_MODE_01
#define GPIO_LCD_DRV_EN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_LCD_DRV_EN_PIN_M_PWM		GPIO_MODE_05
#define GPIO_LCD_DRV_EN_PIN_CLK		CLK_OUT0
#define GPIO_LCD_DRV_EN_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_SDHC_EINT_PIN			(GPIO67 | 0x80000000)
#define GPIO_SDHC_EINT_PIN_M_CLK		GPIO_MODE_02
#define GPIO_SDHC_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_SDHC_EINT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SDHC_EINT_PIN_M_PWM		GPIO_MODE_05
#define GPIO_SDHC_EINT_PIN_CLK		CLK_OUT2
#define GPIO_SDHC_EINT_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_CTP_RST_PIN			(GPIO68 | 0x80000000)
#define GPIO_CTP_RST_PIN_M_CLK		GPIO_MODE_02
#define GPIO_CTP_RST_PIN_M_EINT		GPIO_MODE_01
#define GPIO_CTP_RST_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CTP_RST_PIN_M_PWM		GPIO_MODE_05
#define GPIO_CTP_RST_PIN_CLK		CLK_OUT3
#define GPIO_CTP_RST_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_SMARTPA_I2S_WS_PIN			(GPIO69 | 0x80000000)
#define GPIO_SMARTPA_I2S_WS_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_I2S_WS_PIN_M_I2S0_LRCK		GPIO_MODE_01
#define GPIO_SMARTPA_I2S_WS_PIN_M_I2S1_LRCK		GPIO_MODE_03

#define GPIO_SMARTPA_I2S_BCK_PIN			(GPIO70 | 0x80000000)
#define GPIO_SMARTPA_I2S_BCK_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_I2S_BCK_PIN_M_I2S0_BCK		GPIO_MODE_01
#define GPIO_SMARTPA_I2S_BCK_PIN_M_I2S1_BCK		GPIO_MODE_03

#define GPIO_SMARTPA_EINT_PIN			(GPIO71 | 0x80000000)
#define GPIO_SMARTPA_EINT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_EINT_PIN_M_I2S0_MCK		GPIO_MODE_01

#define GPIO_SMARTPA_I2S_DIN_PIN			(GPIO72 | 0x80000000)
#define GPIO_SMARTPA_I2S_DIN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_I2S_DIN_PIN_M_I2S0_DI		GPIO_MODE_01
#define GPIO_SMARTPA_I2S_DIN_PIN_M_I2S2_DI		GPIO_MODE_03

#define GPIO_SMARTPA_I2S_DOUT_PIN			(GPIO73 | 0x80000000)
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_I2S3_DO		GPIO_MODE_01
#define GPIO_SMARTPA_I2S_DOUT_PIN_M_I2S1_DO		GPIO_MODE_03

#define GPIO_I2C3_SCA_PIN			(GPIO74 | 0x80000000)
#define GPIO_I2C3_SCA_PIN_M_CLK		GPIO_MODE_07
#define GPIO_I2C3_SCA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C3_SCA_PIN_M_SCL3_		GPIO_MODE_01

#define GPIO_I2C3_SDA_PIN			(GPIO75 | 0x80000000)
#define GPIO_I2C3_SDA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C3_SDA_PIN_M_SDA3_		GPIO_MODE_01

#define GPIO_CTP_EINT_PIN			(GPIO85 | 0x80000000)
#define GPIO_CTP_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_CTP_EINT_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_FP_EINT_PIN			(GPIO86 | 0x80000000)
#define GPIO_FP_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_FP_EINT_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_LCD_BIAS_ENP_PIN			(GPIO87 | 0x80000000)
#define GPIO_LCD_BIAS_ENP_PIN_M_EINT		GPIO_MODE_01
#define GPIO_LCD_BIAS_ENP_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_ALS_EINT_PIN			(GPIO88 | 0x80000000)
#define GPIO_ALS_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_ALS_EINT_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_CAMERA_FLASH_EN_PIN			(GPIO90 | 0x80000000)
#define GPIO_CAMERA_FLASH_EN_PIN_M_CLK		GPIO_MODE_03
#define GPIO_CAMERA_FLASH_EN_PIN_M_EINT		GPIO_MODE_01
#define GPIO_CAMERA_FLASH_EN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CAMERA_FLASH_EN_PIN_CLK		CLK_OUT1
#define GPIO_CAMERA_FLASH_EN_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_TORCH_EN			(GPIO91 | 0x80000000)
#define GPIO_TORCH_EN_M_CLK		GPIO_MODE_03
#define GPIO_TORCH_EN_M_EINT		GPIO_MODE_01
#define GPIO_TORCH_EN_M_GPIO		GPIO_MODE_00
#define GPIO_TORCH_EN_M_PWM		GPIO_MODE_02
#define GPIO_TORCH_EN_CLK		CLK_OUT2
#define GPIO_TORCH_EN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_ACCDET_EINT_PIN			(GPIO92 | 0x80000000)
#define GPIO_ACCDET_EINT_PIN_M_CLK		GPIO_MODE_03
#define GPIO_ACCDET_EINT_PIN_M_EINT		GPIO_MODE_01
#define GPIO_ACCDET_EINT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_ACCDET_EINT_PIN_M_PWM		GPIO_MODE_02
#define GPIO_ACCDET_EINT_PIN_CLK		CLK_OUT3
#define GPIO_ACCDET_EINT_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_OTG_DRVVBUS_PIN			(GPIO94 | 0x80000000)
#define GPIO_OTG_DRVVBUS_PIN_M_CLK		GPIO_MODE_03
#define GPIO_OTG_DRVVBUS_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_OTG_DRVVBUS_PIN_M_PWM		GPIO_MODE_02
#define GPIO_OTG_DRVVBUS_PIN_M_USB_DRVVBUS		GPIO_MODE_01
#define GPIO_OTG_DRVVBUS_PIN_CLK		CLK_OUT5
#define GPIO_OTG_DRVVBUS_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_I2C2_SDA_PIN			(GPIO95 | 0x80000000)
#define GPIO_I2C2_SDA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C2_SDA_PIN_M_SDA2_		GPIO_MODE_01

#define GPIO_I2C2_SCA_PIN			(GPIO96 | 0x80000000)
#define GPIO_I2C2_SCA_PIN_M_CLK		GPIO_MODE_07
#define GPIO_I2C2_SCA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C2_SCA_PIN_M_SCL2_		GPIO_MODE_01

#define GPIO_UART_URXD0_PIN			(GPIO97 | 0x80000000)
#define GPIO_UART_URXD0_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_UART_URXD0_PIN_M_URXD		GPIO_MODE_01
#define GPIO_UART_URXD0_PIN_M_UTXD		GPIO_MODE_02
#define GPIO_UART_URXD0_PIN_M_MD_URXD		GPIO_MODE_03
#define GPIO_UART_URXD0_PIN_M_MD_URXD		GPIO_MODE_04
#define GPIO_UART_URXD0_PIN_M_MD_URXD		GPIO_MODE_05
#define GPIO_UART_URXD0_PIN_M_C2K_URXD		GPIO_MODE_06
#define GPIO_UART_URXD0_PIN_M_C2K_URXD		GPIO_MODE_07

#define GPIO_UART_UTXD0_PIN			(GPIO98 | 0x80000000)
#define GPIO_UART_UTXD0_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_UART_UTXD0_PIN_M_UTXD		GPIO_MODE_01
#define GPIO_UART_UTXD0_PIN_M_URXD		GPIO_MODE_02
#define GPIO_UART_UTXD0_PIN_M_MD_UTXD		GPIO_MODE_03
#define GPIO_UART_UTXD0_PIN_M_MD_UTXD		GPIO_MODE_04
#define GPIO_UART_UTXD0_PIN_M_MD_UTXD		GPIO_MODE_05
#define GPIO_UART_UTXD0_PIN_M_C2K_UTXD		GPIO_MODE_06
#define GPIO_UART_UTXD0_PIN_M_C2K_UTXD		GPIO_MODE_07

#define GPIO_NFC_OSC_EN_PIN			(GPIO100 | 0x80000000)
#define GPIO_NFC_OSC_EN_PIN_M_CLK		GPIO_MODE_01
#define GPIO_NFC_OSC_EN_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_KPD_KROW0_PIN			(GPIO106 | 0x80000000)
#define GPIO_KPD_KROW0_PIN_M_CLK		GPIO_MODE_03
#define GPIO_KPD_KROW0_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_KPD_KROW0_PIN_M_KPROW		GPIO_MODE_01
#define GPIO_KPD_KROW0_PIN_CLK		CLK_OUT4
#define GPIO_KPD_KROW0_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_LCD_BIAS_ENN_PIN			(GPIO108 | 0x80000000)
#define GPIO_LCD_BIAS_ENN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_LCD_BIAS_ENN_PIN_M_PWM		GPIO_MODE_03
#define GPIO_LCD_BIAS_ENN_PIN_M_DAP_SIB1_SWCK		GPIO_MODE_07

#define GPIO_KPD_KCOL0_PIN			(GPIO109 | 0x80000000)
#define GPIO_KPD_KCOL0_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_KPD_KCOL0_PIN_M_KPCOL		GPIO_MODE_01

#define GPIO_CAMERA_LDO_EN_PIN			(GPIO110 | 0x80000000)
#define GPIO_CAMERA_LDO_EN_PIN_M_CLK		GPIO_MODE_04
#define GPIO_CAMERA_LDO_EN_PIN_M_EINT		GPIO_MODE_07
#define GPIO_CAMERA_LDO_EN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CAMERA_LDO_EN_PIN_M_PWM		GPIO_MODE_03
#define GPIO_CAMERA_LDO_EN_PIN_M_SDA1_		GPIO_MODE_02
#define GPIO_CAMERA_LDO_EN_PIN_CLK		CLK_OUT0
#define GPIO_CAMERA_LDO_EN_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_VMBUCK_I2C_SDA			(GPIO110 | 0x80000000)
#define GPIO_VMBUCK_I2C_SDA_M_CLK		GPIO_MODE_04
#define GPIO_VMBUCK_I2C_SDA_M_EINT		GPIO_MODE_07
#define GPIO_VMBUCK_I2C_SDA_M_GPIO		GPIO_MODE_00
#define GPIO_VMBUCK_I2C_SDA_M_PWM		GPIO_MODE_03
#define GPIO_VMBUCK_I2C_SDA_M_SDA1_		GPIO_MODE_02
#define GPIO_VMBUCK_I2C_SDA_CLK		CLK_OUT0
#define GPIO_VMBUCK_I2C_SDA_FREQ		GPIO_CLKSRC_NONE

#define GPIO_HP_DEPOP_SWITCH_PIN			(GPIO111 | 0x80000000)
#define GPIO_HP_DEPOP_SWITCH_PIN_M_EINT		GPIO_MODE_07
#define GPIO_HP_DEPOP_SWITCH_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_HP_DEPOP_SWITCH_PIN_M_PWM		GPIO_MODE_03
#define GPIO_HP_DEPOP_SWITCH_PIN_M_SCL1_		GPIO_MODE_02

#define GPIO_VMBUCK_I2C_SCL			(GPIO111 | 0x80000000)
#define GPIO_VMBUCK_I2C_SCL_M_EINT		GPIO_MODE_07
#define GPIO_VMBUCK_I2C_SCL_M_GPIO		GPIO_MODE_00
#define GPIO_VMBUCK_I2C_SCL_M_PWM		GPIO_MODE_03
#define GPIO_VMBUCK_I2C_SCL_M_SCL1_		GPIO_MODE_02

#define GPIO_SIM1_HOT_PLUG			(GPIO112 | 0x80000000)
#define GPIO_SIM1_HOT_PLUG_M_EINT		GPIO_MODE_07
#define GPIO_SIM1_HOT_PLUG_M_GPIO		GPIO_MODE_00
#define GPIO_SIM1_HOT_PLUG_M_MD_INT1_C2K_UIM1_HOT_PLUG_IN		GPIO_MODE_01

#define GPIO_SIM2_HOT_PLUG			(GPIO113 | 0x80000000)
#define GPIO_SIM2_HOT_PLUG_M_EINT		GPIO_MODE_07
#define GPIO_SIM2_HOT_PLUG_M_GPIO		GPIO_MODE_00
#define GPIO_SIM2_HOT_PLUG_M_MD_INT0_C2K_UIM0_HOT_PLUG_IN		GPIO_MODE_01

#define GPIO_MSDC0_DAT0			(GPIO114 | 0x80000000)
#define GPIO_MSDC0_DAT0_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DAT0_M_MSDC0_DAT		GPIO_MODE_01

#define GPIO_MSDC0_DAT1			(GPIO115 | 0x80000000)
#define GPIO_MSDC0_DAT1_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DAT1_M_MSDC0_DAT		GPIO_MODE_01

#define GPIO_MSDC0_DAT2			(GPIO116 | 0x80000000)
#define GPIO_MSDC0_DAT2_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DAT2_M_MSDC0_DAT		GPIO_MODE_01

#define GPIO_MSDC0_DAT3			(GPIO117 | 0x80000000)
#define GPIO_MSDC0_DAT3_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DAT3_M_MSDC0_DAT		GPIO_MODE_01

#define GPIO_MSDC0_DAT4			(GPIO118 | 0x80000000)
#define GPIO_MSDC0_DAT4_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DAT4_M_MSDC0_DAT		GPIO_MODE_01

#define GPIO_MSDC0_DAT5			(GPIO119 | 0x80000000)
#define GPIO_MSDC0_DAT5_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DAT5_M_MSDC0_DAT		GPIO_MODE_01

#define GPIO_MSDC0_DAT6			(GPIO120 | 0x80000000)
#define GPIO_MSDC0_DAT6_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DAT6_M_MSDC0_DAT		GPIO_MODE_01

#define GPIO_MSDC0_DAT7			(GPIO121 | 0x80000000)
#define GPIO_MSDC0_DAT7_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DAT7_M_MSDC0_DAT		GPIO_MODE_01

#define GPIO_MSDC0_CMD			(GPIO122 | 0x80000000)
#define GPIO_MSDC0_CMD_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_CMD_M_MSDC0_CMD		GPIO_MODE_01

#define GPIO_MSDC0_CLK			(GPIO123 | 0x80000000)
#define GPIO_MSDC0_CLK_M_CLK		GPIO_MODE_01
#define GPIO_MSDC0_CLK_M_GPIO		GPIO_MODE_00

#define GPIO_MSDC0_DSL			(GPIO124 | 0x80000000)
#define GPIO_MSDC0_DSL_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_DSL_M_MSDC0_DSL		GPIO_MODE_01

#define GPIO_MSDC0_RSTB			(GPIO125 | 0x80000000)
#define GPIO_MSDC0_RSTB_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC0_RSTB_M_MSDC0_RSTB		GPIO_MODE_01

#define GPIO_SIM1_SCLK			(GPIO126 | 0x80000000)
#define GPIO_SIM1_SCLK_M_CLK		GPIO_MODE_01
#define GPIO_SIM1_SCLK_M_GPIO		GPIO_MODE_00
#define GPIO_SIM1_SCLK_M_C2K_UIM0_CLK		GPIO_MODE_03

#define GPIO_SIM1_SRST			(GPIO127 | 0x80000000)
#define GPIO_SIM1_SRST_M_GPIO		GPIO_MODE_00
#define GPIO_SIM1_SRST_M_MD1_SIM1_SRST		GPIO_MODE_01
#define GPIO_SIM1_SRST_M_C2K_UIM0_RST		GPIO_MODE_03

#define GPIO_SIM1_SIO			(GPIO128 | 0x80000000)
#define GPIO_SIM1_SIO_M_GPIO		GPIO_MODE_00
#define GPIO_SIM1_SIO_M_MD1_SIM1_SIO		GPIO_MODE_01
#define GPIO_SIM1_SIO_M_C2K_UIM0_IO		GPIO_MODE_03

#define GPIO_MSDC1_CMD			(GPIO129 | 0x80000000)
#define GPIO_MSDC1_CMD_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC1_CMD_M_MSDC1_CMD		GPIO_MODE_01
#define GPIO_MSDC1_CMD_M_LTE_JTAG_TMS		GPIO_MODE_03
#define GPIO_MSDC1_CMD_M_UDI_TMS		GPIO_MODE_04
#define GPIO_MSDC1_CMD_M_C2K_TMS		GPIO_MODE_05

#define GPIO_MSDC1_DAT0			(GPIO130 | 0x80000000)
#define GPIO_MSDC1_DAT0_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC1_DAT0_M_MSDC1_DAT		GPIO_MODE_01
#define GPIO_MSDC1_DAT0_M_LTE_JTAG_TDI		GPIO_MODE_03
#define GPIO_MSDC1_DAT0_M_UDI_TDI		GPIO_MODE_04
#define GPIO_MSDC1_DAT0_M_C2K_TDI		GPIO_MODE_05

#define GPIO_MSDC1_DAT1			(GPIO131 | 0x80000000)
#define GPIO_MSDC1_DAT1_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC1_DAT1_M_MSDC1_DAT		GPIO_MODE_01
#define GPIO_MSDC1_DAT1_M_LTE_JTAG_TDO		GPIO_MODE_03
#define GPIO_MSDC1_DAT1_M_UDI_TDO		GPIO_MODE_04
#define GPIO_MSDC1_DAT1_M_C2K_TDO		GPIO_MODE_05

#define GPIO_MSDC1_DAT2			(GPIO132 | 0x80000000)
#define GPIO_MSDC1_DAT2_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC1_DAT2_M_MSDC1_DAT		GPIO_MODE_01
#define GPIO_MSDC1_DAT2_M_C2K_RTCK		GPIO_MODE_05

#define GPIO_MSDC1_DAT3			(GPIO133 | 0x80000000)
#define GPIO_MSDC1_DAT3_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC1_DAT3_M_MSDC1_DAT		GPIO_MODE_01
#define GPIO_MSDC1_DAT3_M_LTE_JTAG_TRSTN		GPIO_MODE_03
#define GPIO_MSDC1_DAT3_M_UDI_NTRST		GPIO_MODE_04
#define GPIO_MSDC1_DAT3_M_C2K_NTRST		GPIO_MODE_05

#define GPIO_MSDC1_CLK			(GPIO134 | 0x80000000)
#define GPIO_MSDC1_CLK_M_CLK		GPIO_MODE_01
#define GPIO_MSDC1_CLK_M_GPIO		GPIO_MODE_00
#define GPIO_MSDC1_CLK_M_LTE_JTAG_TCK		GPIO_MODE_03
#define GPIO_MSDC1_CLK_M_UDI_TCK_XI		GPIO_MODE_04
#define GPIO_MSDC1_CLK_M_C2K_TCK		GPIO_MODE_05

#define GPIO_MHL_I2S_OUT_WS_PIN			(GPIO135 | 0x80000000)
#define GPIO_MHL_I2S_OUT_WS_PIN_M_CLK		GPIO_MODE_03
#define GPIO_MHL_I2S_OUT_WS_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_MHL_I2S_OUT_WS_PIN_M_PWM		GPIO_MODE_05
#define GPIO_MHL_I2S_OUT_WS_PIN_M_TDM_LRCK		GPIO_MODE_01
#define GPIO_MHL_I2S_OUT_WS_PIN_CLK		CLK_OUT0
#define GPIO_MHL_I2S_OUT_WS_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_MHL_I2S_OUT_CK_PIN			(GPIO136 | 0x80000000)
#define GPIO_MHL_I2S_OUT_CK_PIN_M_CLK		GPIO_MODE_03
#define GPIO_MHL_I2S_OUT_CK_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_MHL_I2S_OUT_CK_PIN_M_PWM		GPIO_MODE_05
#define GPIO_MHL_I2S_OUT_CK_PIN_M_TDM_BCK		GPIO_MODE_01
#define GPIO_MHL_I2S_OUT_CK_PIN_CLK		CLK_OUT1
#define GPIO_MHL_I2S_OUT_CK_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_IRTX_OUT_PIN			(GPIO137 | 0x80000000)
#define GPIO_IRTX_OUT_PIN_M_CLK		GPIO_MODE_03
#define GPIO_IRTX_OUT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_IRTX_OUT_PIN_M_IRTX_OUT		GPIO_MODE_05
#define GPIO_IRTX_OUT_PIN_CLK		CLK_OUT2
#define GPIO_IRTX_OUT_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_MHL_I2S_OUT_DAT_PIN			(GPIO138 | 0x80000000)
#define GPIO_MHL_I2S_OUT_DAT_PIN_M_CLK		GPIO_MODE_03
#define GPIO_MHL_I2S_OUT_DAT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_MHL_I2S_OUT_DAT_PIN_M_PWM		GPIO_MODE_05
#define GPIO_MHL_I2S_OUT_DAT_PIN_M_TDM_DATA		GPIO_MODE_01
#define GPIO_MHL_I2S_OUT_DAT_PIN_CLK		CLK_OUT3
#define GPIO_MHL_I2S_OUT_DAT_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_MHL_RST_B_PIN			(GPIO139 | 0x80000000)
#define GPIO_MHL_RST_B_PIN_M_CLK		GPIO_MODE_03
#define GPIO_MHL_RST_B_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_MHL_RST_B_PIN_CLK		CLK_OUT4
#define GPIO_MHL_RST_B_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_MHL_EINT_PIN			(GPIO140 | 0x80000000)
#define GPIO_MHL_EINT_PIN_M_CLK		GPIO_MODE_03
#define GPIO_MHL_EINT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_MHL_EINT_PIN_M_PWM		GPIO_MODE_02
#define GPIO_MHL_EINT_PIN_M_TDM_DATA		GPIO_MODE_01
#define GPIO_MHL_EINT_PIN_CLK		CLK_OUT5
#define GPIO_MHL_EINT_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_GPS_LNA_PIN			(GPIO141 | 0x80000000)
#define GPIO_GPS_LNA_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_PWRAP_SPI0_MI_PIN			(GPIO142 | 0x80000000)
#define GPIO_PWRAP_SPI0_MI_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_PWRAP_SPI0_MI_PIN_M_PWRAP_SPI0_MI		GPIO_MODE_01
#define GPIO_PWRAP_SPI0_MI_PIN_M_PWRAP_SPI0_MO		GPIO_MODE_02

#define GPIO_PWRAP_SPI0_MO_PIN			(GPIO143 | 0x80000000)
#define GPIO_PWRAP_SPI0_MO_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_PWRAP_SPI0_MO_PIN_M_PWRAP_SPI0_MO		GPIO_MODE_01
#define GPIO_PWRAP_SPI0_MO_PIN_M_PWRAP_SPI0_MI		GPIO_MODE_02

#define GPIO_PWRAP_SPI0_CK_PIN			(GPIO144 | 0x80000000)
#define GPIO_PWRAP_SPI0_CK_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_PWRAP_SPI0_CK_PIN_M_PWRAP_SPI0_CK		GPIO_MODE_01

#define GPIO_PWRAP_SPI0_CSN_PIN			(GPIO145 | 0x80000000)
#define GPIO_PWRAP_SPI0_CSN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_PWRAP_SPI0_CSN_PIN_M_PWRAP_SPI0_CSN		GPIO_MODE_01

#define GPIO_AUD_CLK_MOSI_PIN			(GPIO146 | 0x80000000)
#define GPIO_AUD_CLK_MOSI_PIN_M_CLK		GPIO_MODE_01
#define GPIO_AUD_CLK_MOSI_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_AUD_DAT_MISO_PIN			(GPIO147 | 0x80000000)
#define GPIO_AUD_DAT_MISO_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_AUD_DAT_MISO_PIN_M_AUD_DAT_MISO		GPIO_MODE_01

#define GPIO_AUD_DAT_MOSI_PIN			(GPIO148 | 0x80000000)
#define GPIO_AUD_DAT_MOSI_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_AUD_DAT_MOSI_PIN_M_AUD_DAT_MOSI		GPIO_MODE_01

#define GPIO_VOW_CLK_MISO_PIN			(GPIO149 | 0x80000000)
#define GPIO_VOW_CLK_MISO_PIN_M_CLK		GPIO_MODE_01
#define GPIO_VOW_CLK_MISO_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_I2C6_SCA_PIN			(GPIO151 | 0x80000000)
#define GPIO_I2C6_SCA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C6_SCA_PIN_M_SCL6_		GPIO_MODE_01

#define GPIO_I2C6_SDA_PIN			(GPIO152 | 0x80000000)
#define GPIO_I2C6_SDA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C6_SDA_PIN_M_SDA6_		GPIO_MODE_01

#define GPIO_I2C7_SCA_PIN			(GPIO153 | 0x80000000)
#define GPIO_I2C7_SCA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C7_SCA_PIN_M_SCL7_		GPIO_MODE_01

#define GPIO_I2C7_SDA_PIN			(GPIO154 | 0x80000000)
#define GPIO_I2C7_SDA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C7_SDA_PIN_M_SDA7_		GPIO_MODE_01

#define GPIO_SIM2_SCLK			(GPIO155 | 0x80000000)
#define GPIO_SIM2_SCLK_M_CLK		GPIO_MODE_01
#define GPIO_SIM2_SCLK_M_GPIO		GPIO_MODE_00
#define GPIO_SIM2_SCLK_M_C2K_UIM1_CLK		GPIO_MODE_04

#define GPIO_SIM2_SRST			(GPIO156 | 0x80000000)
#define GPIO_SIM2_SRST_M_GPIO		GPIO_MODE_00
#define GPIO_SIM2_SRST_M_MD1_SIM2_SRST		GPIO_MODE_01
#define GPIO_SIM2_SRST_M_C2K_UIM1_RST		GPIO_MODE_04

#define GPIO_SIM2_SIO			(GPIO157 | 0x80000000)
#define GPIO_SIM2_SIO_M_GPIO		GPIO_MODE_00
#define GPIO_SIM2_SIO_M_MD1_SIM2_SIO		GPIO_MODE_01
#define GPIO_SIM2_SIO_M_C2K_UIM1_IO		GPIO_MODE_04

#define GPIO_DSI_TE_PIN			(GPIO179 | 0x80000000)
#define GPIO_DSI_TE_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_DSI_TE_PIN_M_DSI_TE		GPIO_MODE_01

#define GPIO_LCM_RST			(GPIO180 | 0x80000000)
#define GPIO_LCM_RST_M_GPIO		GPIO_MODE_00
#define GPIO_LCM_RST_M_LCM_RST		GPIO_MODE_01

#define GPIO_OTG_IDDIG_EINT_PIN			(GPIO181 | 0x80000000)
#define GPIO_OTG_IDDIG_EINT_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_OTG_IDDIG_EINT_PIN_M_IDDIG		GPIO_MODE_01

#define GPIO_RFIC0_BSI_CK			(GPIO183 | 0x80000000)
#define GPIO_RFIC0_BSI_CK_M_GPIO		GPIO_MODE_00
#define GPIO_RFIC0_BSI_CK_M_RFIC0_BSI_CK		GPIO_MODE_01

#define GPIO_RFIC0_BSI_D0			(GPIO185 | 0x80000000)
#define GPIO_RFIC0_BSI_D0_M_GPIO		GPIO_MODE_00
#define GPIO_RFIC0_BSI_D0_M_RFIC0_BSI_D		GPIO_MODE_01

#define GPIO_RFIC0_BSI_D1			(GPIO186 | 0x80000000)
#define GPIO_RFIC0_BSI_D1_M_GPIO		GPIO_MODE_00
#define GPIO_RFIC0_BSI_D1_M_RFIC0_BSI_D		GPIO_MODE_01

#define GPIO_RFIC0_BSI_D2			(GPIO187 | 0x80000000)
#define GPIO_RFIC0_BSI_D2_M_GPIO		GPIO_MODE_00
#define GPIO_RFIC0_BSI_D2_M_RFIC0_BSI_D		GPIO_MODE_01

#define GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN			(GPIO194 | 0x80000000)
#define GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN			(GPIO195 | 0x80000000)
#define GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_U3_REDRIVER_C2_PIN			(GPIO196 | 0x80000000)
#define GPIO_U3_REDRIVER_C2_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_U3_REDRIVER_C2_PIN_M_BPI_BUS		GPIO_MODE_01

#define GPIO_U3_REDRIVER_C1_PIN			(GPIO197 | 0x80000000)
#define GPIO_U3_REDRIVER_C1_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_U3_REDRIVER_C1_PIN_M_BPI_BUS		GPIO_MODE_01

#define GPIO_UART_URXD1_PIN			(GPIO232 | 0x80000000)
#define GPIO_UART_URXD1_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_UART_URXD1_PIN_M_URXD		GPIO_MODE_01

#define GPIO_UART_UTXD1_PIN			(GPIO233 | 0x80000000)
#define GPIO_UART_UTXD1_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_UART_UTXD1_PIN_M_UTXD		GPIO_MODE_01

#define GPIO_SPI_SCK_PIN			(GPIO234 | 0x80000000)
#define GPIO_SPI_SCK_PIN_M_CLK		GPIO_MODE_01
#define GPIO_SPI_SCK_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SPI_SCK_PIN_M_PWM		GPIO_MODE_06

#define GPIO_SPI_MISO_PIN			(GPIO235 | 0x80000000)
#define GPIO_SPI_MISO_PIN_M_CLK		GPIO_MODE_06
#define GPIO_SPI_MISO_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SPI_MISO_PIN_M_SPI1_MI_B		GPIO_MODE_01
#define GPIO_SPI_MISO_PIN_CLK		CLK_OUT0
#define GPIO_SPI_MISO_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_SPI_MOSI_PIN			(GPIO236 | 0x80000000)
#define GPIO_SPI_MOSI_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SPI_MOSI_PIN_M_PWM		GPIO_MODE_06
#define GPIO_SPI_MOSI_PIN_M_SPI1_MO_B		GPIO_MODE_01

#define GPIO_SPI_CS_PIN			(GPIO237 | 0x80000000)
#define GPIO_SPI_CS_PIN_M_CLK		GPIO_MODE_06
#define GPIO_SPI_CS_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SPI_CS_PIN_M_SPI1_CS_B		GPIO_MODE_01
#define GPIO_SPI_CS_PIN_CLK		CLK_OUT1
#define GPIO_SPI_CS_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_I2C4_SDA_PIN			(GPIO238 | 0x80000000)
#define GPIO_I2C4_SDA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C4_SDA_PIN_M_SDA4_		GPIO_MODE_01

#define GPIO_I2C4_SCA_PIN			(GPIO239 | 0x80000000)
#define GPIO_I2C4_SCA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C4_SCA_PIN_M_SCL4_		GPIO_MODE_01

#define GPIO_I2C5_SDA_PIN			(GPIO240 | 0x80000000)
#define GPIO_I2C5_SDA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C5_SDA_PIN_M_SDA5_		GPIO_MODE_01

#define GPIO_I2C5_SCA_PIN			(GPIO241 | 0x80000000)
#define GPIO_I2C5_SCA_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_I2C5_SCA_PIN_M_SCL5_		GPIO_MODE_01

#define GPIO_SPI2_SCK_PIN			(GPIO242 | 0x80000000)
#define GPIO_SPI2_SCK_PIN_M_CLK		GPIO_MODE_01
#define GPIO_SPI2_SCK_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SPI2_SCK_PIN_M_PWM		GPIO_MODE_06

#define GPIO_SPI2_MISO_PIN			(GPIO243 | 0x80000000)
#define GPIO_SPI2_MISO_PIN_M_CLK		GPIO_MODE_06
#define GPIO_SPI2_MISO_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SPI2_MISO_PIN_M_SPI2_MI_B		GPIO_MODE_01
#define GPIO_SPI2_MISO_PIN_CLK		CLK_OUT2
#define GPIO_SPI2_MISO_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_SPI2_MOSI_PIN			(GPIO244 | 0x80000000)
#define GPIO_SPI2_MOSI_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SPI2_MOSI_PIN_M_PWM		GPIO_MODE_06
#define GPIO_SPI2_MOSI_PIN_M_SPI2_MO_B		GPIO_MODE_01

#define GPIO_SPI2_CS_PIN			(GPIO245 | 0x80000000)
#define GPIO_SPI2_CS_PIN_M_CLK		GPIO_MODE_06
#define GPIO_SPI2_CS_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_SPI2_CS_PIN_M_SPI2_CS_B		GPIO_MODE_01
#define GPIO_SPI2_CS_PIN_CLK		CLK_OUT3
#define GPIO_SPI2_CS_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_FUSB340_USBTYPEC_NOE_PIN			(GPIO251 | 0x80000000)
#define GPIO_FUSB340_USBTYPEC_NOE_PIN_M_GPIO		GPIO_MODE_00

#define GPIO_FUSB340_USBTYPEC_SEL_PIN			(GPIO252 | 0x80000000)
#define GPIO_FUSB340_USBTYPEC_SEL_PIN_M_CLK		GPIO_MODE_01
#define GPIO_FUSB340_USBTYPEC_SEL_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_FUSB340_USBTYPEC_SEL_PIN_M_PWM		GPIO_MODE_03

#define GPIO_CAMERA_VCAMA_ENABLE_PIN			(GPIO253 | 0x80000000)
#define GPIO_CAMERA_VCAMA_ENABLE_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CAMERA_VCAMA_ENABLE_PIN_M_PWM		GPIO_MODE_03

#define GPIO_CHR_CE_PIN			(GPIO254 | 0x80000000)
#define GPIO_CHR_CE_PIN_M_CLK		GPIO_MODE_05
#define GPIO_CHR_CE_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CHR_CE_PIN_CLK		CLK_OUT0
#define GPIO_CHR_CE_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_FINGERPRINT_RST_PIN			(GPIO256 | 0x80000000)
#define GPIO_FINGERPRINT_RST_PIN_M_CLK		GPIO_MODE_01
#define GPIO_FINGERPRINT_RST_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_FINGERPRINT_RST_PIN_M_PWM		GPIO_MODE_03
#define GPIO_FINGERPRINT_RST_PIN_CLK		CLK_OUT2
#define GPIO_FINGERPRINT_RST_PIN_FREQ		GPIO_CLKSRC_NONE

#define GPIO_JTAG_TMS_PIN			(GPIO257 | 0x80000000)
#define GPIO_JTAG_TMS_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_JTAG_TMS_PIN_M_IO_JTAG_TMS		GPIO_MODE_01
#define GPIO_JTAG_TMS_PIN_M_LTE_JTAG_TMS		GPIO_MODE_02
#define GPIO_JTAG_TMS_PIN_M_DFD_TMS		GPIO_MODE_03
#define GPIO_JTAG_TMS_PIN_M_DAP_SIB1_SWD		GPIO_MODE_04
#define GPIO_JTAG_TMS_PIN_M_ANC_JTAG_TMS		GPIO_MODE_05
#define GPIO_JTAG_TMS_PIN_M_SCP_JTAG_TMS		GPIO_MODE_06
#define GPIO_JTAG_TMS_PIN_M_C2K_DM_OTMS		GPIO_MODE_07

#define GPIO_JTAG_TCK_PIN			(GPIO258 | 0x80000000)
#define GPIO_JTAG_TCK_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_JTAG_TCK_PIN_M_IO_JTAG_TCK		GPIO_MODE_01
#define GPIO_JTAG_TCK_PIN_M_LTE_JTAG_TCK		GPIO_MODE_02
#define GPIO_JTAG_TCK_PIN_M_DFD_TCK_XI		GPIO_MODE_03
#define GPIO_JTAG_TCK_PIN_M_DAP_SIB1_SWCK		GPIO_MODE_04
#define GPIO_JTAG_TCK_PIN_M_ANC_JTAG_TCK		GPIO_MODE_05
#define GPIO_JTAG_TCK_PIN_M_SCP_JTAG_TCK		GPIO_MODE_06
#define GPIO_JTAG_TCK_PIN_M_C2K_DM_OTCK		GPIO_MODE_07

#define GPIO_JTAG_TDI_PIN			(GPIO259 | 0x80000000)
#define GPIO_JTAG_TDI_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_JTAG_TDI_PIN_M_IO_JTAG_TDI		GPIO_MODE_01
#define GPIO_JTAG_TDI_PIN_M_LTE_JTAG_TDI		GPIO_MODE_02
#define GPIO_JTAG_TDI_PIN_M_DFD_TDI		GPIO_MODE_03
#define GPIO_JTAG_TDI_PIN_M_ANC_JTAG_TDI		GPIO_MODE_05
#define GPIO_JTAG_TDI_PIN_M_SCP_JTAG_TDI		GPIO_MODE_06
#define GPIO_JTAG_TDI_PIN_M_C2K_DM_OTDI		GPIO_MODE_07

#define GPIO_JTAG_TDO_PIN			(GPIO260 | 0x80000000)
#define GPIO_JTAG_TDO_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_JTAG_TDO_PIN_M_IO_JTAG_TDO		GPIO_MODE_01
#define GPIO_JTAG_TDO_PIN_M_LTE_JTAG_TDO		GPIO_MODE_02
#define GPIO_JTAG_TDO_PIN_M_DFD_TDO		GPIO_MODE_03
#define GPIO_JTAG_TDO_PIN_M_ANC_JTAG_TDO		GPIO_MODE_05
#define GPIO_JTAG_TDO_PIN_M_SCP_JTAG_TDO		GPIO_MODE_06
#define GPIO_JTAG_TDO_PIN_M_C2K_DM_OTDO		GPIO_MODE_07

#define GPIO_JTAG_TRSTN_PIN			(GPIO261 | 0x80000000)
#define GPIO_JTAG_TRSTN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_JTAG_TRSTN_PIN_M_LTE_JTAG_TRSTN		GPIO_MODE_02
#define GPIO_JTAG_TRSTN_PIN_M_DFD_NTRST		GPIO_MODE_03
#define GPIO_JTAG_TRSTN_PIN_M_ANC_JTAG_TRSTN		GPIO_MODE_05
#define GPIO_JTAG_TRSTN_PIN_M_SCP_JTAG_TRSTN		GPIO_MODE_06
#define GPIO_JTAG_TRSTN_PIN_M_C2K_DM_JTINTP		GPIO_MODE_07



#endif /* __CUST_SCP_GPIO_USAGE_H */

