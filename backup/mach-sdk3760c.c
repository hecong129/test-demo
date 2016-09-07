/**
 * arch/arm/mach-ak37/mach-spi-sdk3753.c
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/mxc622x.h>
#include <linux/mmc328x.h>
#include <linux/dma-mapping.h>
#include <media/soc_camera.h>
#include <linux/switch.h>
#include <linux/i2c-gpio.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <plat/l2.h>
#include <plat/rtc.h>
#include <plat/bat.h>
#include <plat-anyka/mmc_sd.h>
#include <plat-anyka/wifi.h>
#include <plat-anyka/otg-hshcd.h>
#include <plat-anyka/aksensor.h>
#include <plat-anyka/drv_module_lock.h>
#include <plat-anyka/adkey.h>
#include <plat-anyka/spi.h>
#include <plat-anyka/akgpios.h>
#include <linux/cp2528.h>



#include <mach/map.h>
#include <mach/regs-comm.h>
#include <mach/clock.h>
#include <mach/devices.h>
#include <mach/reboot.h>
#include <mach/adc.h>
#include <mach/akpcmL0.h>
#include <mach/ak37cfb.h>
#include <mach/pwm.h>
#include "cpu.h"
#include "irq.h"

/* declare function */
extern int ak37_reboot_level;
void (*ak37_arch_reset)(void);
static void ak37_power_off(void);
void  backlight_timer_func(unsigned long arg);
void  backlight_timer_init(void);

#define DBG(fmt...)		//{printk(":%s:%s:%d",__FILE__,__FUNCTION__,__LINE__);printk(fmt);}


#ifndef BACKLIGHT_ON_FLAG
#define  BACKLIGHT_ON_FLAG   0x81
#endif

/* declare function */
void i2c_set_gpio(void);

/**
 * @brief   the param drv_shpin_lock[i].name indicate different lock
 * @author  caolianming
 * @date      2012-07-19
 */
static struct ak_drv_sharepin_lock drv_shpin_lock[] = {
	{DRV_MODULE_SDMMC,	AK_MODULE_LOCK_1, TYPE_LOCK_SEMAPHORE, 0, NULL},
	{DRV_MODULE_SDIO,	AK_MODULE_LOCK_1, TYPE_LOCK_SEMAPHORE, 0, NULL},
    {DRV_MODULE_SPI,    AK_MODULE_LOCK_1, TYPE_LOCK_SEMAPHORE, 0, NULL},
	{DRV_MODULE_I2C,	AK_MODULE_LOCK_2, TYPE_LOCK_SEMAPHORE, 0, i2c_set_gpio},
};

void *sharepin_lock_array[AK_MODULE_COUNT] = {0};
struct timer_list backlight_timer;

/**
* @brief		i2c device struct,
			In this device,wo should initialize i2c data line and clock line
			to make sure it can work well.
* @author	caolianming
* @date	  	2012-07-19
* notice, wrap_sensor.h use sda_pin scl_pin  , if you change it ,please chang it
 */
struct i2c_gpio_platform_data ak37_i2c_data={
	.sda_pin = AK_GPIO_13,//AK_GPIO_82,						/*GPIO of i2C data line*/
	.scl_pin = AK_GPIO_6,//AK_GPIO_80,						/*GPIO of i2C clock line*/
	.udelay = 10,
	.timeout = 200,
};

struct platform_device ak37_i2c_device = {
	.name	= "i2c-gpio",
	.id		= 0,
	.dev	= {
		.platform_data = &ak37_i2c_data,
	},
};

struct i2c_gpio_platform_data key_i2c_data={
	.sda_pin = AK_GPIO_82,						/*GPIO of i2C data line*/
	.scl_pin = AK_GPIO_80,						/*GPIO of i2C clock line*/
	.udelay = 10,
	.timeout = 200,
};

struct platform_device key_i2c_device = {
	.name	= "i2c-gpio",
	.id		= 1,
	.dev	= {
		.platform_data = &key_i2c_data,
	},
};

struct i2c_gpio_platform_data at_i2c_data={
	.sda_pin = AK_GPIO_79,						/*GPIO of i2C data line*/
	.scl_pin = AK_GPIO_23,						/*GPIO of i2C clock line*/
	.udelay = 10,
	.timeout = 200,
};

struct platform_device at_i2c_device = {
	.name	= "i2c-gpio",
	.id		= 2,
	.dev	= {
		.platform_data = &at_i2c_data,
	},
};

/**
* @brief		i2c_set_gpio
			This function set i2c's data GPIO and clock GPIO
			aim at solve problem of share with other pins,
* @author	caolianming
* @date   	2012-07-19
* @param
* @return	  void
* @retval
*/
void i2c_set_gpio(void)
{
	ak_setpin_as_gpio(ak37_i2c_data.sda_pin);
	ak_setpin_as_gpio(ak37_i2c_data.scl_pin);

	ak_setpin_as_gpio(key_i2c_data.sda_pin);
	ak_setpin_as_gpio(key_i2c_data.scl_pin);

	ak_setpin_as_gpio(at_i2c_data.sda_pin);
	ak_setpin_as_gpio(at_i2c_data.scl_pin);
}


#define SPI_ONCHIP_CS 	(0)/*means not need gpio*/
static unsigned long ak37_spidev_cs[AKSPI_CS_NUM] = {
	[AKSPI_ONCHIP_CS] = SPI_ONCHIP_CS,/*gpio 25, spidev0: ak-spiflash*/
};

struct ak_spi_info ak37_spi1_info = {
	.pin_cs = ak37_spidev_cs,
	.num_cs = ARRAY_SIZE(ak37_spidev_cs),
	.bus_num = AKSPI_BUS_NUM1,
	.clk_name = "spi_clk",
	.mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH,
	.xfer_mode = AKSPI_XFER_MODE_DMA,
};

#if defined(CONFIG_MTD_AK_SPINAND)
static struct flash_platform_data ak37_spiflash_info= {
	.bus_width = FLASH_BUS_WIDTH_4WIRE,
	.type 	   = NULL,
};

static struct spi_board_info ak37_spi_board_dev[] = {
	{
		.modalias = "ak-sflash",
		.bus_num = AKSPI_BUS_NUM1,
		.chip_select = AKSPI_ONCHIP_CS,
		.mode = SPI_MODE_0,
		.max_speed_hz = 40*1000*1000,
		.platform_data = &ak37_spiflash_info,
	},
};

#else
static struct flash_platform_data ak37_spiflash_info= {
	.bus_width = FLASH_BUS_WIDTH_4WIRE,
	.type 	   = NULL,
};

static struct spi_board_info ak37_spi_board_dev[] = {
	{
		.modalias = "ak-spiflash",
		.bus_num = AKSPI_BUS_NUM1,
		.chip_select = AKSPI_ONCHIP_CS,
		.mode = SPI_MODE_0,
		.max_speed_hz = 40*1000*1000,
		.platform_data = &ak37_spiflash_info,
	},
};
#endif

/**
* @brief		MMC/SD device
			There are two conditions of this type device,
			we can use four or eight data line to transfer date,
			we use "define" to deal this problem,
* @author	caolianming
* @date	  	2012-07-19
 */
#ifdef CONFIG_FOUR_DATA_LINE						//when we use four data line to transfer data.
struct ak_mci_platform_data mci_plat_data = {
    .gpio_init = ak_gpio_set,						/*initialize the function of GPIO setting*/
    .gpio_cd = {
		.pin = AK_GPIO_16,									/*initialize GPIO number of identify device and -1 show not use GPIO */
        .pulldown = -1,			/*initialize function of this GPIO for pulldown(AK_PULLDOWN_DISABLE | AK_PULLDOWN_ENABLE | -1)*/
        .pullup = AK_PULLUP_ENABLE,								/*initialize function of this GPIO for pullup(AK_PULLUP_DISABLE | AK_PULLUP_ENABLE | -1)*/
        .value = 1,								/*initialize value of this GPIO(high | low | -1)*/
        .dir = AK_GPIO_DIR_INPUT,					/*initialize direction of this GPIO*/
        .int_pol = AK_GPIO_INT_LOWLEVEL,								/*initialize mode of irq*/
    },
    .gpio_wp = {
        .pin = -1,
        .pulldown = AK_PULLDOWN_DISABLE,
        .pullup = AK_PULLUP_ENABLE,
        .value = -1,
        .dir = AK_GPIO_DIR_INPUT,
        .int_pol = -1,
    },
};

#elif defined CONFIG_EIGHT_DATA_LINE				//when we use eight line to transfer data.
struct ak_mci_platform_data mci_plat_data = {
    .gpio_init = ak_gpio_set,
    .gpio_cd = {
		.pin = -1,
        .pulldown = AK_PULLDOWN_DISABLE,
        .pullup = -1,
        .value = -1,
        .dir = AK_GPIO_DIR_INPUT,
        .int_pol = -1,
    },
    .gpio_wp = {
        .pin = -1,
        .pulldown = AK_PULLDOWN_DISABLE,
        .pullup = AK_PULLUP_ENABLE,
        .value = -1,
        .dir = AK_GPIO_DIR_INPUT,
        .int_pol = -1,
    },
};
#else
struct ak_mci_platform_data mci_plat_data = {

};
#endif

static struct resource ak37_mmc_resource[] = {
	[0] = {
		.start = 0x20020000,						/*initialize the start address*/
		.end = 0x20020000 + 0x43,					/*initialize the end address*/
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MMC_SD,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ak37_mmc_device = {
	.name = "ak_mci",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak37_mmc_resource),/*initialize the number of device*/
	.resource = ak37_mmc_resource,
	.dev = {
		.platform_data = &mci_plat_data,
	},
};


/**
* @brief		SDIO device struct
			we should initialize GPIO number while we use GPIO identify device
			to make sure this device can work well.
			or wen can initialize pin as -1,
* @author	caolianming
* @date   	2012-07-19

*/
struct ak_mci_platform_data sdio_plat_data = {

    .gpio_init = ak_gpio_set,
    .gpio_cd = {
        .pin = AK_GPIO_16,
        .pulldown = -1,
        .pullup = AK_PULLUP_ENABLE,
        .value = -1,
        .dir = AK_GPIO_DIR_INPUT,
        .int_pol = -1,
    },
    .gpio_wp = {
        .pin = -1,
        .pulldown = AK_PULLDOWN_DISABLE,
        .pullup = AK_PULLUP_ENABLE,
        .value = -1,
        .dir = AK_GPIO_DIR_INPUT,
        .int_pol = -1,
    },
};

static struct resource ak37_sdio_resource[] = {
	[0] = {
		.start = 0x20021000,						/*initialize the start address*/
		.end = 0x20021000 + 0x43,					/*initialize the end address*/
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ak37_sdio_device = {
	.name = "ak_sdio",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak37_sdio_resource),/*initialize the number of device*/
	.resource = ak37_sdio_resource,
	.dev = {
		.platform_data = &sdio_plat_data,
	},
};


/**
* @brief		GPIO KEY device struct
			In this struct we should initialize GPIO for the button.
* @author	caolianming
* @date	  2012-07-19
*/
static struct ak_gpio_keys_button aimer37_buttons[] = {
#if 1
	[0] = {
		.code		= KEY_9,						/*set this button code*/
		.gpio		= AK_GPIO_0,						/*initialize GPIO number of this button*/
		.active_low	= 0,
		.desc		= "holk-key",
		.debounce_interval = 400,
		.wakeup		= -1,								/*set this button have the function of wake up the chip */
		.pullup		= -1,
		.pulldown	= -1,
		.dir		= AK_GPIO_DIR_INPUT,
		.int_pol	= AK_GPIO_INT_HIGHLEVEL,
	},
#else
	[0] = {
		.code		= KEY_RIGHT,						/*set this button code*/
		.gpio		= AK_GPIO_79,						/*initialize GPIO number of this button*/
		.active_low	= 1,
		.desc		= "menu-key",
		.debounce_interval = 30,
		.wakeup		= -1,								/*set this button have the function of wake up the chip */
		.pullup		= -1,
		.pulldown	= AK_PULLDOWN_DISABLE,
		.dir		= AK_GPIO_DIR_INPUT,
		.int_pol	= AK_GPIO_INT_LOWLEVEL,
	},
	[1] = {
		.code		= KEY_LEFT,						/*set this button code*/
		.gpio		= AK_GPIO_23,						/*initialize GPIO number of this button*/
		.active_low	= 1,
		.desc		= "left-key",
		.debounce_interval = 30,
		.wakeup		= -1,								/*set this button have the function of wake up the chip */
		.pullup		= -1,
		.pulldown	= AK_PULLDOWN_DISABLE,
		.dir		= AK_GPIO_DIR_INPUT,
		.int_pol	= AK_GPIO_INT_LOWLEVEL,
	},
	[2] = {
		.code		= KEY_MENU,						/*set this button code*/
		.gpio		= AK_GPIO_18,						/*initialize GPIO number of this button*/
		.active_low	= 1,
		.desc		= "right-key",
		.debounce_interval = 30,
		.wakeup		= -1,								/*set this button have the function of wake up the chip */
		.pullup		= -1,
		.pulldown	= AK_PULLDOWN_DISABLE,
		.dir		= AK_GPIO_DIR_INPUT,
		.int_pol	= AK_GPIO_INT_LOWLEVEL,
	},
#endif
};

static struct ak_gpio_keys_platform_data aimer37_button_data = {
	.rep = 1,
	.buttons = aimer37_buttons,
	.nbuttons = ARRAY_SIZE(aimer37_buttons),			/*initialize the number of device*/
};

static struct platform_device ak37_button_device = {
	.name = "gpio_keys",
	.id = -1,
	.dev = {
		.platform_data = &aimer37_button_data,
	},
};


/**
* @brief		ad key platform device struct
			we should initialize the correct voltage for each key.
* @author	caolianming
* @date   2012-07-19
*/

struct adgpio_key adkey[][3] = {
// ain * avdd/1024= ref
	{	{ .code = KEY_1,	.min = 600,	 	.max = 800},
    		{ .code = KEY_2,  .min = 1600,  	.max = 1800},
    		{ .code = KEY_3,  .min = 2400, 		.max = 2600}
	},
};

struct multi_addetect multi_det[] = {
	{.unpress_min = 3200, .unpress_max = 3400, .fixkeys = adkey[0], .plugdev = PLUGIN_NODEV},
};

struct analog_gpio_key ak37_adkey_data = {
	.desc = "adkey",
	.interval = 200,
	.debounce_interval = 30,
	.addet = multi_det,
	.naddet = ARRAY_SIZE(multi_det),						/*initialize the number of device*/
	.nkey = ARRAY_SIZE(adkey[0]),
	.wakeup = 1,											//initialize key have function of wake up chip.
};

static struct platform_device ak37_adkey_device = {
	.name = "analog-key",
	.id = -1,
	.dev = {
		.platform_data = &ak37_adkey_data,
	}
};


/* Keypad Initialization */
#define KEYPAD_BUTTON(ev_type, ev_code, act_low) \
{								\
	.type		= ev_type,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
}

#define KEYPAD_BUTTON_LOW(event_code) KEYPAD_BUTTON(EV_KEY, event_code, 0)

static struct cp2528_button cp2528_gpio_keys[] = {
	KEYPAD_BUTTON_LOW(KEY_LEFT),
	KEYPAD_BUTTON_LOW(KEY_UP),
	KEYPAD_BUTTON_LOW(KEY_MENU),
	KEYPAD_BUTTON_LOW(KEY_RIGHT),
	KEYPAD_BUTTON_LOW(KEY_DOWN),
};

static struct cp2528_keys_platform_data cp2528_keys_info = {
	.buttons	= cp2528_gpio_keys,
	.nbuttons	= ARRAY_SIZE(cp2528_gpio_keys),
	.rep		= 1,
	.use_polling	= 1,
	.pinmask	= 0x1F,
};



/**
* @brief		camera device struct
			we should initialize the GPIO power for camera.
* @author	dengzhou
* @date	  2012-07-19
*/
struct i2c_board_info ak_camara_devices[] = {
	{
		I2C_BOARD_INFO("aksensor", 0x1),
	},
};

static struct aksensor_camera_info  ak_soc_camera_info = {
	.buswidth = SOCAM_DATAWIDTH_8,
	.pin_avdd = INVALID_GPIO,
	.pin_power = INVALID_GPIO,		//initialize GPIO for the power of camera.
	.pin_reset = INVALID_GPIO,		//initialize GPIO for reset of camera.
	.link = {
		.bus_id = 37,
		.board_info = &ak_camara_devices[0],
		.i2c_adapter_id = 0,
	}
};

/* fake device for soc_camera subsystem */
static struct platform_device soc_camera_interface = {
	.name = "soc-camera-pdrv",
	.id   = -1,
	.dev = {
		.platform_data = &ak_soc_camera_info.link,
	}
};

static struct i2c_board_info  i2c_key1[] __initdata = {
	{
		I2C_BOARD_INFO("cp2528-keys", 0x2c),
		.platform_data = &cp2528_keys_info,
	},
};

/**
* @brief		akwifi platform device struct
			There are two conditions of wifi struct,
			one is use GPIO for the power of wifi.
			the other is use usb as host.
* @author	caolianming
* @date	  2012-07-19
*/
struct wifi_control_data akwifi_control_data = {
#if defined(CONFIG_SDIO_WIFI)
		.gpio_on = {
			.pin		= AK_GPIO_33,
			.pulldown	= -1,
			.pullup 	= AK_PULLUP_DISABLE,
			.value		= AK_GPIO_OUT_HIGH,
			.dir		= AK_GPIO_DIR_OUTPUT,
			.int_pol	= -1,
		},
		.gpio_off = {
			.pin		= AK_GPIO_33,
			.pulldown	= -1,
			.pullup 	= AK_PULLUP_DISABLE,
			.value		= AK_GPIO_OUT_LOW,
			.dir		= AK_GPIO_DIR_OUTPUT,
			.int_pol	= -1,
		},
#else

	.gpio_on = {
		.pin		= -1,
		.pulldown	= AK_PULLDOWN_ENABLE,
		.pullup		= -1,
		.value		= AK_GPIO_OUT_LOW,
		.dir		= AK_GPIO_DIR_OUTPUT,
		.int_pol	= -1,
	},
	.gpio_off = {
		.pin		= -1,
		.pulldown	= AK_PULLDOWN_ENABLE,
		.pullup		= -1,
		.value		= AK_GPIO_OUT_LOW,
		.dir		= AK_GPIO_DIR_OUTPUT,
		.int_pol	= -1,
	},
	.power_on_delay   = 2000,
	.power_off_delay  = 0,
#endif
};

static int akwifi_power_on(void)
	{
	struct wifi_control_data *p = &akwifi_control_data;
	if (p->gpio_on.pin > 0) {
		ak_gpio_set(&p->gpio_on);
	}
	msleep(p->power_on_delay);
	return 0;
	}

static int akwifi_power_off(void)
{
	struct wifi_control_data *p = &akwifi_control_data;

	if (p->gpio_off.pin > 0) {
		ak_gpio_set(&p->gpio_off);						//set GPIO's function
	}
	msleep(p->power_off_delay);
	return 0;
}

static struct wifi_power_platform_data akwifi_info = {
	.total_usb_ep_num = 2,
	.power_on	= akwifi_power_on,						//initialize the power turn on function
	.power_off	= akwifi_power_off,						//initialize the power turn off function.
};

struct platform_device anyka_wifi_device = {
	.name = "anyka-wifi",
	.id = -1,
	.dev = {
		.platform_data = &akwifi_info,
	},
};


/*
this struct is
power on for devices when usb otg as host.
we should initialize it while we use usb as host
to make sure wifi can work.
*/
static struct akotghc_usb_platform_data akotghc_plat_data = {
	.gpio_init = ak_gpio_set,
	.gpio_pwr_on = {
		.pin = -1,												/*initialize GPIO number of turn on power when usb as host*/
		.pulldown = AK_PULLDOWN_DISABLE,
		.pullup = -1,
		.value = AK_GPIO_OUT_HIGH,
		.dir = AK_GPIO_DIR_OUTPUT,
		.int_pol = -1,
	},
	.gpio_pwr_off = {
		.pin = -1,												/*initialize GPIO number of turn off power when usb as host*/
		.pulldown = AK_PULLDOWN_DISABLE,
		.pullup = -1,
		.value = AK_GPIO_OUT_LOW,
		.dir = AK_GPIO_DIR_OUTPUT,
		.int_pol = -1,
	},
	.switch_onboard = {
		.pin = -1,										/*initialize GPIO number of turn on switch of usb when usb as host */
		.pulldown = AK_PULLDOWN_DISABLE,
		.pullup = -1,
		.value = AK_GPIO_OUT_HIGH,
		.dir = AK_GPIO_DIR_OUTPUT,
		.int_pol = -1,
	},
	.switch_extport = {
		.pin = -1,										/*initialize GPIO number of turn off switch of usb when usb as host */
		.pulldown = AK_PULLDOWN_DISABLE,
		.pullup = -1,
		.value = AK_GPIO_OUT_LOW,
		.dir = AK_GPIO_DIR_OUTPUT,
		.int_pol = -1,
	},
};



/**
* @brief		ak battery mach info
* @author	caolianming
* @date	  2012-07-19
*/

static struct ak_bat_mach_info ak37_bat_info = {
	.gpio_init	= ak_gpio_set,
	.usb_gpio	= {
		.active 	= -1,
		.irq		= -ENOSYS, 					//initilaize the irq type.
		.delay		= 0,
		.pindata	={
			.pin		= -1,
			.pulldown	= -1,
			.pullup 	= -1,
			.value		= -1,
			.dir		= -1,
			.int_pol	= -1,
		},
	},

	.ac_gpio	= {
		.active 	= -1,
		.irq		= -ENOSYS,
		.delay		= 0,
		.pindata	={
			.pin		= -1,
			.pulldown	= -1,
			.pullup 	= -1,
			.value		= -1,
			.dir		= -1,
			.int_pol	= -1,
		},
	},

	.full_gpio = {
		.active 	= -1,
		.irq		= -ENOSYS,
		.delay		= 0,
		.pindata	={
			.pin		= -1,
			.pulldown	= -1,
			.pullup 	= -1,
			.value		= -1,
			.dir		= -1,
			.int_pol	= -1,
		},
	},

	.bat_mach_info	= {
		.voltage_sample 	= 6,			// the sample of read voltage
		.power_on_voltage	= 3650, 		// discharge power on voltage limit
		.power_on_correct	= 64,			// mv
		.charge_min_voltage = 3550, 		// charge minute voltage (mv)
		.max_voltage		= 4200, 		// max battery voltage
		.min_voltage		= 3500, 		// min battery voltage
		.power_off			= poweroff_enable,
		.full_capacity		= 100,			// battery full
		.poweroff_cap		= 0,			// user read value to power off
		.low_cap			= 5,			// user read value to low power warring
		.recover_cap		= 30,
		.cpower_on_voltage	= 3700, 		// charge power on voltage limit
		.full_delay 		= 30,			// unit is minute
		.full_voltage		= 4100,
	},

	.bat_adc	= {
		.up_resistance		= 51,
		.dw_resistance		= 51,
		.voltage_correct	= 32,			// battery correct factor
		.adc_avdd			= 3300, 		// avdd voltage
	},
};


/**
 * @brief   	check_poweroff
 			This function is check whether power is off.
 * @author 	dengzhou
 * @date       2012-07-19
 * @param
 * @return    void
 * @retval
 */

void check_poweroff(void)
{
	int voltage;
	struct ak_bat_mach_info *info = &ak37_bat_info;

#ifdef CONFIG_AK37_REBOOT_POLICY
	if (3 == ak37_reboot_level)
	{
		return;
	}
#endif

	voltage = (int)adc1_read_bat();
	voltage = (voltage * info->bat_adc.adc_avdd) / (1024);	// real read voltage
	voltage = voltage * (info->bat_adc.up_resistance + info->bat_adc.dw_resistance)
		/ info->bat_adc.dw_resistance;
	voltage += info->bat_adc.voltage_correct;				// correct battery voltage
	voltage -= info->bat_mach_info.power_on_correct;		// correct power on voltage

	if (voltage <= info->bat_mach_info.min_voltage)
	{
		printk("=========voltage=%d;power off;while(1)=========\n",voltage);
		ak37_power_off();
	}
}
// check powerof end



/**
* @brief		ak pcm device struct
			hp and spk can identify by GPIO or AD.
			wo can initialize it in this struct.
* @author	dengzhou
* @date	  2012-07-19
*/
struct akpcm_platform_data akpcm_plat_data =
{
	.hpdet_gpio =
	{
		.pin        = INVALID_GPIO,//AK_GPIO_18,
		.dir		= AK_GPIO_DIR_INPUT,
		.pullup		= -1,
		.pulldown	= AK_PULLDOWN_DISABLE,
		.value      = -1,
		.int_pol	= -1,
	},
	.spk_down_gpio =
	{
		.pin        = INVALID_GPIO,
		.dir		= AK_GPIO_DIR_OUTPUT,
		.pullup		= AK_PULLUP_ENABLE,
		.pulldown	= -1,
		.value      = AK_GPIO_LOW,
		.int_pol	= -1,
	},
	.hpmute_gpio =
	{
		.pin        = INVALID_GPIO,
		.dir		= AK_GPIO_DIR_OUTPUT,
		.pullup		= -1,
		.pulldown	= -1,
		.value      = AK_GPIO_LOW,
		.int_pol	= -1,
	},
	.linindet_gpio =
	{
		.pin		= INVALID_GPIO,
		.dir		= INVALID_GPIO,
		.pullup 	= AK_PULLUP_DISABLE,
		.pulldown	= -1,
		.value		= AK_GPIO_OUT_LOW,	/* linein detect level */
		.int_pol	= -1,
	},

	.hp_on_value          = -1,//AK_GPIO_OUT_LOW,
	.hpdet_irq            = 0,// IRQ_GPIO_18,
	.linindet_irq		  = 0,//IRQ_GPIO_18,
	.bIsHPmuteUsed        = 0,
	.hp_mute_enable_value = -1,//AK_GPIO_OUT_HIGH,
	.bIsMetalfixed        = 0,
	.boutput_only		  = 1, /* HP only */
};
struct resource akpcm_resources[] = {
	[0] = {
			.start = 0x08000000,
			.end = 0x0800FFFF,
			.flags = (int)IORESOURCE_MEM,
			.name = "ak37pcm_AnalogCtrlRegs",
		  },
	[1] = {
			.start = 0x2002E000,
			.end = 0x2002E00F,
			.flags = (int)IORESOURCE_MEM,
			.name = "ak37pcm_I2SCtrlRegs",
		  },
	[2] = {
			.start = 0x20072000,
			.end = 0x2007200F,
			.flags = (int)IORESOURCE_MEM,
			.name = "ak37pcm_ADC2ModeCfgRegs",
		  },
};
static u64 snd_dma_mask = DMA_BIT_MASK(32);

struct platform_device akpcm_device = {
	.name = "akpcm",
	.id = 0,

	.dev = {
			 .dma_mask	   = &snd_dma_mask,
			 .coherent_dma_mask = DMA_BIT_MASK(32),
			 .platform_data = &akpcm_plat_data,
		   },
	.resource = akpcm_resources,
	.num_resources = ARRAY_SIZE(akpcm_resources),
};

static struct platform_device akfha_char_device = {
	.name = "ak-fhachar",
	.id = -1,
	.dev = {
			.platform_data = /*&ak37_evt_nand_info*/NULL,
	},
};

static struct platform_device asic_div_char_device = {
	.name = "ak_asic_div",
	.id = -1,
	.dev = {
			.platform_data = NULL,
	},
};

/* Added by panqihe for backlight device 2014-06-03 */
static int backlight_init(struct ak37_pwm *dev)
{
    int ret;

    BUG_ON(!dev->pwm_ops);

	ret = dev->pwm_ops->enable(dev);
    ret = dev->pwm_ops->enable(dev);
    DBG(KERN_INFO "AK37C backlight init RET = %x\n", ret);
    if (ret == 0)
        ak_gpio_pulldown(dev->gpio, AK_PULLDOWN_ENABLE);

    return ret;
}

static void backlight_exit(struct ak37_pwm *dev)
{
    BUG_ON(!dev->pwm_ops);

    dev->pwm_ops->disable(dev);
    DBG(KERN_INFO "AK37C backlight EXIT \n");
}

static struct ak37_platform_pwm_bl_data ak37_backlight_data = {
    .pwm_id = 2,
    /*max brightness can be set by application. Notes: the range of
       birghtness can be set by AP is 0 ~ max_brightness */
    .max_brightness = 255,
   /*default brightness that is set when driver start,
      which will be changed by android at one moment
      during system starting, that is to say, actual
      default brightness is provided by android */
    // dft_brightness = (std_table_brightness - low_limit) * max / (high_limit - low_limit);
    .dft_brightness = 200,  // no effective to android,android will be set to 128
    .high_limit = 250,      //high limit of brightness in driver
    .low_limit = 0,        //low limit of brightness in driver
    .pwm_clk = 4000,    //Hz
    .pwm_div = 50,
    .init = backlight_init,
    .notify = NULL,
    .exit = backlight_exit,
};

static struct platform_device ak37_backlight_device = {
    .name = "ak37-backlight",
    .id = -1,
    .dev        = {
        .platform_data = &ak37_backlight_data,
    },
};

/* Added by panqihe for fb device 2014-06-03
   for blight lcd and avoid white screen when power on. hecong 2016-04-05
   case mst702 driver display logo takes 300ms to take effect  */
static void aklcd_set_power(unsigned int enable)
{
	static int flag = 1;

	DBG("hecong ak lcd_set_power %d %d\n",enable,flag);

	if (0x80 & BACKLIGHT_ON_FLAG) {
		if (flag) {
			backlight_timer_init();
			flag = 0;
		}
	}

	return;
}
static struct akfb_display ak37cfb_disp[] = {
//
};
static struct aklcd_platform_data aklcd_pdata = {
	.displays = ak37cfb_disp,
  	.num_displays = ARRAY_SIZE(ak37cfb_disp),
  	.default_display = 0,
	.set_power = aklcd_set_power,
};

static struct resource ak37cfb_resources[] = {
	[0] = {
		.start	= 0x2001d000,
		.end	= 0x2001d123,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_DISPLAY_CTRL,
		.end	= IRQ_DISPLAY_CTRL,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = DMA_BIT_MASK(32);
static struct platform_device ak37cfb_lcd_device = {
	.name		= "ak-lcd",
	.id		= -1,
	.dev		= {
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &aklcd_pdata,
	},
	.num_resources	= ARRAY_SIZE(ak37cfb_resources),
	.resource	= ak37cfb_resources,
};

static struct platform_device akget_batval_device = {
	.name = "ak-getbattery",
	.id = -1,
	.dev = {
			.platform_data = NULL,
	},
};

/* unused GPIO number for the machine board is left*/
static unsigned int ak37c_custom_gpiopin[] = {
	AK_GPIO_45,
	AK_GPIO_44,
	AK_GPIO_55,
	AK_GPIO_35,
	AK_GPIO_38,
	AK_GPIO_39,
	AK_GPIO_1,
	AK_GPIO_5,
	AK_GPIO_83,
	AK_GPIO_84,
	AK_GPIO_85,
	AK_GPIO_61,
};

static struct custom_gpio_data ak37c_custom_gpios= {
	.gpiopin = ak37c_custom_gpiopin,
	.ngpiopin = ARRAY_SIZE(ak37c_custom_gpiopin),
};

static struct platform_device ak37c_custom_gpio = {
	.name = "akgpio",
	.id = -1,
	.dev = {
		.platform_data = &ak37c_custom_gpios,
	},
};



static struct gpio_info backlight_gpio ={

	.pin=AK_GPIO_1,
	.pulldown=-1,
	.pullup=-1,
	.value=AK_GPIO_HIGH,
	.dir=AK_GPIO_DIR_OUTPUT,
};

void  backlight_timer_func(unsigned long arg)
{
	int ret=0;

	DBG("lcd backlight open now: %ld\n", arg);
	ak_gpio_set(&backlight_gpio);

	ret = del_timer(&backlight_timer);
	DBG("now del backlight timer: %d\n", ret);

}

void  backlight_timer_init(void)
{

	DBG("lcd backlight init here\n");

	init_timer(&backlight_timer);     //初始化定时器

	backlight_timer.expires = jiffies+50;//设定超时时间，100代表1秒
    backlight_timer.data = 250;    //传递给定时器超时函数的值
    backlight_timer.function = backlight_timer_func;//设置定时器超时函数

    add_timer(&backlight_timer); //添加定时器，定时器开始生效

	return;
}

/* @brief		platform devices
			This struct contain all the device we should initialize while we
			turn on the chip.
			we can add or delete some inside,
* @author	caolianming
* @date   2012-07-19
*/


static struct platform_device *ak37_evt_platform_devices[] __initdata = {

	&ak37_uart0_device,
	&ak37_spi1_device,
	&ak37_mmx_device,
	&ak37_mmx_pmem,
	&akfha_char_device,
	&asic_div_char_device,
	&ak37_usb_device,
	&ak37_adkey_device,
	&akpcm_device,
	&soc_camera_interface,
    	&ak_camera_interface,
    	&ak37_i2c_device,
    	&key_i2c_device,
    	&ak37_button_device,
	&ak37_sdio_device,
  	//&ak_ethernet_device,
  	&ak37_usb_otg_hcd_device,
  	&ak37cfb_lcd_device,
	&ak37c_custom_gpio,
	&ak37_uart1_device,
	&ak_rtc_device,
	&ak37_uart2_device,
	&akget_batval_device,

#if 0
	//&ak37_backlight_device,
	//&ak37_mmc_device,
  	&anyka_wifi_device,
	&ak37_battery_power,
#endif
};

static DEFINE_SPINLOCK(power_off_lock);
static void ak37_power_off(void)
{
	unsigned long flags;
	/*
	 * Do a real power off by polling WAKEUP pin to low
	 */
	mdelay(1);
	spin_lock_irqsave(&power_off_lock, flags);
#ifdef CONFIG_AK37_REBOOT_POLICY
	if (ak_gpio_getpin(ak37_bat_info.ac_gpio.pindata.pin)
				== ak37_bat_info.ac_gpio.active)
	{
		printk(KERN_EMERG "Reboot and enter charge status.\n");
		ak_rtc_set_wpin(0);
		while(1);
	}
	else
	{
		printk(KERN_EMERG "Power down.\n");
		ak_rtc_set_wpin(0);
		while(1);
	}
#else
	printk(KERN_EMERG "Power down.\n");
	ak_rtc_set_wpin(0);
	while(1);
#endif

	spin_unlock_irqrestore(&power_off_lock, flags);
}

void wdt_enable(void);
void wdt_keepalive(unsigned int heartbeat);

static void ak37_reset(void)
{
/*
#ifdef CONFIG_AK37_REBOOT_POLICY
	printk("System reboot.\n");
	//ak_reboot_sys_by_wtd();
	ak37_reboot_sys_by_soft();
#else
	printk("System reset.\n");
	//ak_reboot_sys_by_wtd();
	ak37_reboot_sys_by_soft();
#endif
*/
	/* reset SPI */
	rSOFT_RESET_CON |= (1<<4);
	udelay(500);
	rSOFT_RESET_CON &= ~(1<<4);
	printk("System reset.\n");
	udelay(500);
#if defined CONFIG_AK37C_WATCHDOG
	wdt_enable();
	wdt_keepalive(10);
#endif
}

static void __init ak37_evt_machine_init(void)
{
    adc1_init();
	//ak_rtc_set_wpin(1);
	pm_power_off = ak37_power_off;
	ak37_arch_reset = ak37_reset;

	//ak37_usb_otg_hcd_device.dev.platform_data = &akotghc_plat_data;


	spi_register_board_info(ak37_spi_board_dev, ARRAY_SIZE(ak37_spi_board_dev));
	ak37_spi1_device.dev.platform_data = &ak37_spi1_info;


	// set battery platform_data
	ak37_battery_power.dev.platform_data = &ak37_bat_info;


	/* register platform devices */
	platform_add_devices(ak37_evt_platform_devices,
		ARRAY_SIZE(ak37_evt_platform_devices));



	/* Initialize L2 buffer */
	l2_init();

	ak_set_sharepin_lock_table(drv_shpin_lock,
		ARRAY_SIZE(drv_shpin_lock), sharepin_lock_array);

	/* register for cp2528  i2c1  hecong 2016/04/05 */
	i2c_register_board_info(1, i2c_key1, ARRAY_SIZE(i2c_key1));


}

MACHINE_START(AK37XXC, "BOARD_AK37C_V1.0")

/* Maintainer: */
	.phys_io = 0x20000000,
	.io_pg_offst = ((0xF0200000) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.init_irq = ak37_init_irq,
	.map_io = ak37_map_io,
	.init_machine = ak37_evt_machine_init,
	.timer = &ak37_timer,
MACHINE_END

