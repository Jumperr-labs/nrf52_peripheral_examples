/*
 * (C) Copyright 2012 Samsung Electronics Co. Ltd
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <common.h>
#include <asm/arch-exynos/gpio.h>
#include <asm/io.h>
#undef FEATURE_BQ24160_DEBUG
#if defined(CONFIG_MACH_SHIRI) || defined(CONFIG_MACH_KOI)
//I2C	-	SDA	-	GPA0_6,	SCL	-	GPA0_7
#define GPA0CON		*(volatile unsigned long *)(0x11400000)
#define GPA0DAT		*(volatile unsigned long *)(0x11400004)
#define GPA0PUD		*(volatile unsigned long *)(0x11400008)
#define IIC0_ESCL_Hi	GPA0DAT |= (0x1<<7)
#define IIC0_ESCL_Lo	GPA0DAT &= ~(0x1<<7)
#define IIC0_ESDA_Hi	GPA0DAT |= (0x1<<6)
#define IIC0_ESDA_Lo	GPA0DAT &= ~(0x1<<6)
#define IIC0_ESCL_INP	GPA0CON &= ~(0xf<<28)
#define IIC0_ESCL_OUTP	GPA0CON = (GPA0CON & ~(0xf<<28))|(0x1<<28)
#define IIC0_ESDA_INP	GPA0CON &= ~(0xf<<24)
#define IIC0_ESDA_OUTP	GPA0CON = (GPA0CON & ~(0xf<<24))|(0x1<<24)
#define GPIO_DAT	GPA0DAT
#define GPIO_DAT_SHIFT	(6)
#define DIS_GPIO_PUD	GPA0PUD &= ~(0xf<<12)
//nPOWER, PMIC_ONOB  XEINT23/GPX2_7
#define GPX2CON    0x11000C40
#define GPX2DAT    0x11000C44
#define GPX2PUD    0x11000C48
#define POWER_KEY_CON	GPX2CON
#define POWER_KEY_DAT	GPX2DAT
#define POWER_KEY_PUD	GPX2PUD
//CHG_INT	XEINT17/GPX2_1
#define BQ24160_CHG_INT_CON	GPX2CON
#define BQ24160_CHG_INT_DAT	GPX2DAT
#define BQ24160_CHG_INT_PUD	GPX2PUD
//CHG_STAT      XEINT11/GPX1_3
#define GPX1CON    0x11000C20
#define GPX1DAT    0x11000C24
#define GPX1PUD    0x11000C28
#define BQ24160_CHG_STAT_CON	GPX1CON
#define BQ24160_CHG_STAT_DAT	GPX1DAT
#define BQ24160_CHG_STAT_PUD	GPX1PUD
#define EXYNOS3250_PS_HOLD_CONTROL 0x1002330C
#define PS_HOLD_CONTROL EXYNOS3250_PS_HOLD_CONTROL
#define PS_HOLD_CONTORL_EN_BIT 9
#else
#endif
// BQ24160 Register
#define REG_STATUS	      0x00
#define REG_BR_STATUS	   0x01
#define REG_CONTROL	     0x02
#define REG_BR_VOLTAGE	  0x03
#define REG_VENDOR	      0x04
#define REG_TERMINATION	 0x05
#define REG_DPM		 0x06
#define REG_NTC		 0x07
#define REG_STATUS_TMR_RST_BIT	7
#define REG_STATUS_STAT_MASK	0x70
#define REG_STATUS_SUPPLY_SEL_BIT	3
#define REG_STATUS_FAULT_MASK	   0x07
#define REG_CONTROL_RESET_BIT	   7
#define REG_CONTROL_IUSB_LIM_MASK       0x70
#define REG_CONTROL_IUSB_LIM_BIT 4
#define REG_CONTROL_EN_STAT_BIT	 3
#define REG_CONTROL_TE_BIT	      2
#define REG_CONTROL_CE_BIT	      1
#define REG_CONTROL_HZ_MODE_BIT	 0
#define REG_NTC_2XTMR_EN_BIT	    7
#define REG_NTC_TMR_MASK		0x60
#define REG_NTC_TMR_BIT     5
#define REG_NTC_BATGD_EN_BIT	    4
#define REG_NTC_TS_EN_BIT	       3
#define REG_NTC_TS_FAULT_MASK	   0x06
#define REG_NTC_LOW_CHG_BIT	     0
#define STAT_NO_VALID_SOURCE	0x00
#define STAT_IN_READY	0x01
#define STAT_USB_READ	0x02
#define STAT_CHARGING_FROM_IN	0x03
#define STAT_CHARGING_FROM_USB 0x04
#define STAT_CHARGING_DONE	0x05
#define STAT_CHARGING_NA	0x06
#define STAT_CHARGING_FAULT	0x07
#define TMR_27MIN 0x0
#define TMR_6H	0x1
#define TMR_9H	0x2
#define TMR_OFF	0x3
#define IIN_LIM_1500MA 0x0
#define IIN_LIM_2500MA 0x1
#define IUSB_LIM_100MA 0x0
#define IUSB_LIM_150MA 0x1
#define IUSB_LIM_500MA 0x2
#define IUSB_LIM_800MA 0x3
#define IUSB_LIM_900MA 0x4
#define IUSB_LIM_1500MA 0x5
#define REG_BR_VOLTAGE_MASK 0xFC
#define REG_BR_VOLTAGE_IIN_LIM_MASK 0x1
#define REG_BR_VOLTAGE_IIN_LIM_BIT 1
#define BATTERY_REGULATION_VOLTAGE_640mV (1<<7)
#define BATTERY_REGULATION_VOLTAGE_320mV (1<<6)
#define BATTERY_REGULATION_VOLTAGE_160mV (1<<5)
#define BATTERY_REGULATION_VOLTAGE_80mV (1<<4)
#define BATTERY_REGULATION_VOLTAGE_40mV (1<<3)
#define BATTERY_REGULATION_VOLTAGE_20mV (1<<2)
#define SET_BIT(bit, val, data) ((val << bit) | ((data) & ~(1 << bit)))
#define SET_BIT_2(bit, val, data) ((val << bit) | ((data) & ~(11 << bit)))
#define SET_BIT_3(bit, val, data) ((val << bit) | ((data) & ~(111 << bit)))
#define DATA_MASK(mask, data) ((data) << (ffs(mask) - 1))
#define SET_MASK(mask, val, data) (((data) & ~(mask)) | (val))
extern int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
typedef enum {
	UNPRESS_POWER_KEY = 0,
	PRESS_POWER_KEY
} power_key;
static int bq24160_has_fault = 0;
#define DELAY_TIME		100
#define BQ24160_ADDR  0xD6
void BQ24160_Delay(void)
{
	unsigned long i;
	for (i = 0; i < DELAY_TIME; i++)
		;
}
void BQ24160_IIC0_SCLH_SDAH(void)
{
	IIC0_ESCL_Hi;
	IIC0_ESDA_Hi;
	BQ24160_Delay();
}
void BQ24160_IIC0_SCLH_SDAL(void)
{
	IIC0_ESCL_Hi;
	IIC0_ESDA_Lo;
	BQ24160_Delay();
}
void BQ24160_IIC0_SCLL_SDAH(void)
{
	IIC0_ESCL_Lo;
	IIC0_ESDA_Hi;
	BQ24160_Delay();
}
void BQ24160_IIC0_SCLL_SDAL(void)
{
	IIC0_ESCL_Lo;
	IIC0_ESDA_Lo;
	BQ24160_Delay();
}
void BQ24160_IIC0_ELow(void)
{
	BQ24160_IIC0_SCLL_SDAL();
	BQ24160_IIC0_SCLH_SDAL();
	BQ24160_IIC0_SCLH_SDAL();
	BQ24160_IIC0_SCLL_SDAL();
}
void BQ24160_IIC0_EHigh(void)
{
	BQ24160_IIC0_SCLL_SDAH();
	BQ24160_IIC0_SCLH_SDAH();
	BQ24160_IIC0_SCLH_SDAH();
	BQ24160_IIC0_SCLL_SDAH();
}
void BQ24160_IIC0_EStart(void)
{
	BQ24160_IIC0_SCLH_SDAH();
	BQ24160_IIC0_SCLH_SDAL();
	BQ24160_Delay();
	BQ24160_IIC0_SCLL_SDAL();
}
void BQ24160_IIC0_EEnd(void)
{
	BQ24160_IIC0_SCLL_SDAL();
	BQ24160_IIC0_SCLH_SDAL();
	BQ24160_Delay();
	BQ24160_IIC0_SCLH_SDAH();
}
void BQ24160_IIC0_EAck_write(void)
{
	unsigned long ack;
	/* Function <- Input */
	IIC0_ESDA_INP;
	IIC0_ESCL_Lo;
	BQ24160_Delay();
	IIC0_ESCL_Hi;
	BQ24160_Delay();
	ack = GPIO_DAT;
	IIC0_ESCL_Hi;
	BQ24160_Delay();
	IIC0_ESCL_Hi;
	BQ24160_Delay();
	/* Function <- Output (SDA) */
	IIC0_ESDA_OUTP;
	ack = (ack >> GPIO_DAT_SHIFT) & 0x1;
	BQ24160_IIC0_SCLL_SDAL();
}
void BQ24160_IIC0_EAck_read(void)
{
	/* Function <- Output */
	IIC0_ESDA_OUTP;
	IIC0_ESCL_Lo;
	IIC0_ESCL_Lo;
	IIC0_ESDA_Hi;
	IIC0_ESCL_Hi;
	IIC0_ESCL_Hi;
	/* Function <- Input (SDA) */
	IIC0_ESDA_INP;
	BQ24160_IIC0_SCLL_SDAL();
}
void BQ24160_IIC0_ESetport(void)
{
	/* Pull Up/Down Disable	SCL, SDA */
	DIS_GPIO_PUD;
	IIC0_ESCL_Hi;
	IIC0_ESDA_Hi;
	/* Function <- Output (SCL) */
	IIC0_ESCL_OUTP;
	/* Function <- Output (SDA) */
	IIC0_ESDA_OUTP;
	BQ24160_Delay();
}
void BQ24160_IIC0_EWrite(unsigned char ChipId,
		unsigned char IicAddr, unsigned char IicData)
{
	unsigned long i;
	BQ24160_IIC0_EStart();
	/* write chip id */
	for (i = 7; i > 0; i--) {
		if ((ChipId >> i) & 0x0001)
			BQ24160_IIC0_EHigh();
		else
			BQ24160_IIC0_ELow();
	}
	/* write */
	BQ24160_IIC0_ELow();
	/* ACK */
	BQ24160_IIC0_EAck_write();
	/* write reg. addr. */
	for (i = 8; i > 0; i--) {
		if ((IicAddr >> (i-1)) & 0x0001)
			BQ24160_IIC0_EHigh();
		else
			BQ24160_IIC0_ELow();
	}
	/* ACK */
	BQ24160_IIC0_EAck_write();
	/* write reg. data. */
	for (i = 8; i > 0; i--) {
		if ((IicData >> (i-1)) & 0x0001)
			BQ24160_IIC0_EHigh();
		else
			BQ24160_IIC0_ELow();
	}
	/* ACK */
	BQ24160_IIC0_EAck_write();
	BQ24160_IIC0_EEnd();
}
void BQ24160_IIC0_ERead(unsigned char ChipId,
		unsigned char IicAddr, unsigned char *IicData)
{
	unsigned long i, reg;
	unsigned char data = 0;
	BQ24160_IIC0_EStart();
	/* write chip id */
	for (i = 7; i > 0; i--) {
		if ((ChipId >> i) & 0x0001)
			BQ24160_IIC0_EHigh();
		else
			BQ24160_IIC0_ELow();
	}
	/* write */
	BQ24160_IIC0_ELow();
	/* ACK */
	BQ24160_IIC0_EAck_write();
	/* write reg. addr. */
	for (i = 8; i > 0; i--) {
		if ((IicAddr >> (i-1)) & 0x0001)
			BQ24160_IIC0_EHigh();
		else
			BQ24160_IIC0_ELow();
	}
	/* ACK */
	BQ24160_IIC0_EAck_write();
	BQ24160_IIC0_EStart();
	/* write chip id */
	for (i = 7; i > 0; i--) {
		if ((ChipId >> i) & 0x0001)
			BQ24160_IIC0_EHigh();
		else
			BQ24160_IIC0_ELow();
	}
	/* read */
	BQ24160_IIC0_EHigh();
	/* ACK */
	BQ24160_IIC0_EAck_write();
	/* read reg. data. */
	IIC0_ESDA_INP;
	IIC0_ESCL_Lo;
	IIC0_ESCL_Lo;
	BQ24160_Delay();
	for (i = 8; i > 0; i--) {
		IIC0_ESCL_Lo;
		IIC0_ESCL_Lo;
		BQ24160_Delay();
		IIC0_ESCL_Hi;
		IIC0_ESCL_Hi;
		BQ24160_Delay();
		reg = GPIO_DAT;
		IIC0_ESCL_Hi;
		IIC0_ESCL_Hi;
		BQ24160_Delay();
		IIC0_ESCL_Lo;
		IIC0_ESCL_Lo;
		BQ24160_Delay();
		reg = (reg >> GPIO_DAT_SHIFT) & 0x1;
		data |= reg << (i-1);
	}
	/* ACK */
	BQ24160_IIC0_EAck_read();
	IIC0_ESDA_OUTP;
	BQ24160_IIC0_EEnd();
	*IicData = data;
}
int check_powerkey(void)
{
	int temp = 0;
  //PMIC_ONOB XEINT23/GPX2_7
	writel(((readl(POWER_KEY_PUD)&(~0x3<<14))|(0x0<<14)), POWER_KEY_PUD);
	temp = (readl(POWER_KEY_DAT)&(1<<7))>>7;
	if(temp == 1) // active low
		return UNPRESS_POWER_KEY; // no power key
	return PRESS_POWER_KEY; // power key
}
extern int get_boot_cause(void);
#ifdef CONFIG_USE_LCD
extern unsigned char *charger_logo[4];
extern void s5p_lcd_draw_bitmap32(const unsigned char *buf);
extern void Display_Turnon(void);
extern void Display_Turnoff(void);
extern void Backlight_Turnon(void);
extern void Backlight_Turnoff(void);
#endif
void bq24160_watchdog_reset(void)
{
	unsigned char value;
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_STATUS, &value);
	value = SET_BIT(REG_STATUS_TMR_RST_BIT, 1, value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_STATUS, value);
}
void enable_charging(void)
{
	unsigned char value;
	// hz_disable
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_CONTROL, &value);
	value = SET_BIT(REG_CONTROL_HZ_MODE_BIT, 0, value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_CONTROL, value);
	// set_ce
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_CONTROL, &value);
	value = SET_BIT(REG_CONTROL_CE_BIT, 0, value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_CONTROL, value);
	/* Because TS input is tied to the ground in the xyref b'd, disable TS monitoring function.*/
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_NTC, &value);
	value = SET_BIT(REG_NTC_TS_EN_BIT, 0, value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_NTC, value);
	/* Sets any charging relates registers */
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_BR_VOLTAGE, &value);
	value &= ~(REG_BR_VOLTAGE_MASK);
	value = BATTERY_REGULATION_VOLTAGE_640mV | BATTERY_REGULATION_VOLTAGE_40mV | BATTERY_REGULATION_VOLTAGE_20mV;
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_BR_VOLTAGE, value);
//	bq24160_low_chg_enable(bd, 0);
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_NTC, &value);
	value = SET_BIT(REG_NTC_LOW_CHG_BIT, 0, value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_NTC, value);
#if 0
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_TERMINATION, &value);
	value = SET_MASK(REG_TERMINATION_ICHRG_MASK,	DATA_MASK(REG_TERMINATION_ICHRG_MASK, vichrg), value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_TERMINATION, value);
#endif
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_CONTROL, &value);
	value = SET_BIT_3(REG_CONTROL_IUSB_LIM_BIT, IUSB_LIM_900MA,	value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_CONTROL, value);
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_BR_VOLTAGE, &value);
	value = SET_BIT(REG_BR_VOLTAGE_IIN_LIM_BIT, IIN_LIM_2500MA, value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_BR_VOLTAGE, value);
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_NTC, &value);
	value = SET_BIT_2(REG_NTC_TMR_BIT,TMR_6H,value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_NTC, value);
	bq24160_watchdog_reset();
}
int bq24160_version_check(void)
{
	unsigned char value;
	/* read pmic revision number */
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_VENDOR, &value);
	if (value <= 0)
		printf("Failed read vendor info\n");
#ifdef FEATURE_BQ24160_DEBUG
	else
		printf("BQ24160 version = 0x%x\n",value);
#endif
	return 0;
}
int bq24160_reset(void)
{
	unsigned char value;
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_CONTROL, &value);
	value = SET_BIT(REG_CONTROL_RESET_BIT, 1, value);
	BQ24160_IIC0_EWrite(BQ24160_ADDR, REG_CONTROL_RESET_BIT, &value);
}
int bq24160_check_charging(int log)
{
	int charging, fault;
	unsigned char value;
	BQ24160_IIC0_ERead(BQ24160_ADDR, REG_STATUS,&value);
	charging = (value & REG_STATUS_STAT_MASK) >>	(ffs(REG_STATUS_STAT_MASK) - 1);
	fault = (value & REG_STATUS_FAULT_MASK) >>	(ffs(REG_STATUS_FAULT_MASK) - 1);
	if(log == 1)
		printf("[BQ24160] value=0x%x, charging=0x%x, fault=0x%x\n",value,charging,fault);
	if((log == 1)&&((fault & 0x7) != 0))
	{
		bq24160_has_fault = 1;
		printf("[BQ24160] Charger has a fault.\n");
	}
	if(charging == STAT_IN_READY || charging == STAT_USB_READ || charging == STAT_CHARGING_FROM_IN \
		|| charging == STAT_CHARGING_FROM_USB || charging == STAT_CHARGING_DONE)
		charging = 1;
	else
		charging = 0;
	return charging;
}
void exynos_bq24160_gpio_init(void)
{
	//CHG_INT	XEINT17/GPX2_1/EXT_INT,Pull-up/down disable
	writel((readl(BQ24160_CHG_INT_DAT)&(~(1<<1)))|(0<<1), BQ24160_CHG_INT_DAT);
	writel((readl(BQ24160_CHG_INT_CON)&(~(0xF<<0x4)))|0xF, BQ24160_CHG_INT_CON);
	writel(readl(BQ24160_CHG_INT_PUD)&(~(0x3<<0x2)), BQ24160_CHG_INT_PUD);
	//CHG_STAT	XEINT11/GPX1_3/INTPUT,Pull-up/down disable
	writel((readl(BQ24160_CHG_STAT_DAT)&(~(1<<3)))|(0<<3), BQ24160_CHG_STAT_DAT);
	writel((readl(BQ24160_CHG_STAT_CON)&(~(0xF<<0xC)))|0x0, BQ24160_CHG_STAT_CON);
	writel(readl(BQ24160_CHG_STAT_PUD)&(~(0x3<<0x6)), BQ24160_CHG_STAT_PUD);
}
void exynos_bq24160_init(void)
{
	exynos_bq24160_gpio_init();
	BQ24160_IIC0_ESetport();
}
void dump_register(void)
{
	unsigned char value;
	int i = 0;
	for(i=0;i<8;i++) {
		BQ24160_IIC0_ERead(BQ24160_ADDR, i, &value);
		printf("BQ24160 dump : i=%d,value=0x%x\n",i,value);
	}
}
void turnoff_unused_ip_for_charging_mode(void)
{
#ifdef CONFIG_MACH_ESPRESSO5260
	unsigned int tmp = 0;
	__raw_writel(tmp, EXYNOS5260_ISP_CONFIGURATION);
	__raw_writel(tmp, EXYNOS5260_G3D_CONFIGURATION);
	__raw_writel(tmp, EXYNOS5260_G2D_CONFIGURATION);
	__raw_writel(tmp, EXYNOS5260_GSCL_CONFIGURATION);
	__raw_writel(tmp, EXYNOS5260_MFC_CONFIGURATION);
	__raw_writel(0x80000000, EXYNOS5260_CORTEXA15_EAGLE0_CONFIGURATION);
	__raw_writel(0x80000000, EXYNOS5260_CORTEXA15_EAGLE1_CONFIGURATION);
#endif
}
static int bPressedKey = 0;
int bq24160_charging_loop()
{
	int i, iTmp, iBQ24160_DelayCounter, iFrameNr, iPKNr = 0;
	int lcdon = 0;
	int charging_check = 0;
	int boot_cause = 0;
	boot_cause = get_boot_cause();
	exynos_bq24160_init();
	bq24160_reset();
	if(boot_cause == SOFTWARE_RESET)
		return lcdon;
	bq24160_version_check();
	charging_check = bq24160_check_charging(1);
	if ((charging_check == 1) && (bq24160_has_fault == 0)) {
		printf("[BQ24160] battery charger detected.\n");
	} else {
		printf("[BQ24160] battery or charger NOT found\n");
		return lcdon;
	}
	enable_charging();
#ifdef FEATURE_BQ24160_DEBUG
	dump_register();
#endif
////////// To enter no charging mode ///////////
	iTmp = 1;
	iBQ24160_DelayCounter = 2;
	printf("**********************************\n");
	printf(" WILL ENTER BATTERY CHARGING LOOP\n");
	printf("**********************************\n");
	printf("Hit any key to skip to normal mode: %d", iBQ24160_DelayCounter);
	for (; iBQ24160_DelayCounter>0; )
	{
		bq24160_watchdog_reset();
		iBQ24160_DelayCounter--;
		for (i=0; i<50; i++)
		{
			if (tstc())	{
				iTmp = 0;
				iBQ24160_DelayCounter = 0;
				getc();
				break;
			}
			udelay(30000);
		}
		printf("\b%d", iBQ24160_DelayCounter);
		if (!iTmp) {
			printf("\n");
			return lcdon;
		}
	}
#ifdef CONFIG_USE_LCD
	Lcd_read_bootlogo();
	run_command("lcdtext 5 5 Charging...", 0);
	run_command("lcdtext 'Press power key to'", 0);
	run_command("lcdtext 'go to kernel.'", 0);
	lcdon = 1;
#else
	lcdon = 1;
#endif
	turnoff_unused_ip_for_charging_mode();
	printf("\nEnter Battery Charging Loop......\n");
	iFrameNr = 0;
	iPKNr = 0;
	iBQ24160_DelayCounter = 0;
	for (;;)
	{
		iTmp = 1;
		if(bq24160_check_charging(lcdon)==0) {
			if(lcdon==1)
				printf("unplug charger\n");
				iTmp = 0;
		}
		udelay(200);
		bq24160_watchdog_reset();
		if(lcdon==1)
			printf(" >> To go kernel, press the power key\n");
		switch (iTmp) {
			case 0:			/* Charging OFF */
				goto POWEROFF;
				break;
			case 1:			/* Charging ON */
			case 2:			/* EOC */
			case 3:			/* Error */
			case 4:
			case 5:
			default:
				break;
		}
		for (i = 0; i < 20; i++) {
			if (check_powerkey()) {
				//press power key
				iPKNr++;
				iBQ24160_DelayCounter = 0;
				bPressedKey = 1;
				if (lcdon != 1) {
#ifdef CONFIG_USE_LCD
					Display_Turnon();
					Lcd_read_bootlogo();
					s5p_lcd_draw_bootlogo();
					run_command("lcdtext 5 5 Charging...", 0);
					run_command("lcdtext 'Press power key to'", 0);
					run_command("lcdtext 'go to kernel.'", 0);
					Backlight_Turnon();
#endif
					lcdon = 1;
				}
			}
			else {
				bPressedKey = 0;
				iPKNr = 0;
			}
			if (iPKNr > 40) {
				printf("Booting the system...\n");
				if (lcdon != 0) {
					lcdon = 0;
				}
#ifdef CONFIG_USE_LCD
				Backlight_Turnoff();
#endif
				printf("Press the power key\n");
#ifdef CONFIG_USE_LCD
				Display_Turnoff();
#endif
				do_reset(NULL,0, 0, NULL);
			}
			if (iBQ24160_DelayCounter++ > 400) {
				iBQ24160_DelayCounter = 0;
				if (lcdon != 0) {
#ifdef CONFIG_USE_LCD
					Display_Turnoff();
					Backlight_Turnoff();
#endif
					lcdon = 0;
				}
			}
			udelay(40000);
		}
		iFrameNr++;
		if (iFrameNr>2)
			iFrameNr = 0;
	}
POWEROFF:
	printf("Power off the device...\n");
	writel(readl(PS_HOLD_CONTROL)|(1<<PS_HOLD_CONTORL_EN_BIT),PS_HOLD_CONTROL);
	printf("Error!. Can't power off, value=0x%x\n",readl(PS_HOLD_CONTROL));
	for (;;) ;
}