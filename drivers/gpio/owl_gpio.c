
#include <common.h>
#include <errno.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <fdtdec.h>

#include <asm/arch/gpio.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/actions_reg_owl.h>

DECLARE_GLOBAL_DATA_PTR;

struct owl_gpio_regs {
	unsigned int base;
	unsigned int length;
	unsigned count;
};

struct owl_gpios {
        struct owl_gpio_regs *reg;
};

struct owl_gpio_platdata {
	const char *bank_name;
	int   gpio_base;
	int   gpio_count;
};


#define GPIO_REG_BASE               (GPIO_MFP_PWM_BASE)

// Note:  gpio starts from 0 

#define GPIO_BANK(gpio)             ((gpio) / OWL_GPIO_PER_BANK)
#define GPIO_IN_BANK(gpio)          ((gpio) % OWL_GPIO_PER_BANK)
#define GPIO_BIT(gpio)              (1 << (GPIO_IN_BANK(gpio)))

#define GPIO_REG_OUTEN(gpio)	(GPIO_REG_BASE + (GPIO_BANK(gpio)) * 0xc + 0x0)
#define GPIO_REG_INEN(gpio)	(GPIO_REG_BASE + (GPIO_BANK(gpio)) * 0xc + 0x4)
#define GPIO_REG_DAT(gpio)	(GPIO_REG_BASE + (GPIO_BANK(gpio)) * 0xc + 0x8)

#define DEVCLKEN_GPIO			(0x1 << 18)

int owl_gpio_get_value(unsigned gpio)
{
	return (readl(GPIO_REG_DAT(gpio)) & GPIO_BIT(gpio) ? 1 : 0);
}

int owl_gpio_set_value(unsigned gpio, int value)
{
	if (value)
		setbits_le32(GPIO_REG_DAT(gpio), GPIO_BIT(gpio));
	else
		clrbits_le32(GPIO_REG_DAT(gpio), GPIO_BIT(gpio));

	return 0;
}

int owl_gpio_direction_input(unsigned gpio)
{
	clrbits_le32(GPIO_REG_OUTEN(gpio), GPIO_BIT(gpio));
	setbits_le32(GPIO_REG_INEN(gpio), GPIO_BIT(gpio));

	owl_gpio_reg_dump(gpio); //TS_

	return 0;
}
int owl_gpio_clr(unsigned gpio)
{
	clrbits_le32(GPIO_REG_OUTEN(gpio), GPIO_BIT(gpio));
	clrbits_le32(GPIO_REG_INEN(gpio), GPIO_BIT(gpio));
	return 0;
}


int owl_gpio_direction_output(unsigned gpio, int value)
{
	clrbits_le32(GPIO_REG_INEN(gpio), GPIO_BIT(gpio));
	setbits_le32(GPIO_REG_OUTEN(gpio), GPIO_BIT(gpio));

	owl_gpio_set_value(gpio, value);

	owl_gpio_reg_dump(gpio); //TS_

	return 0;
}

struct gpiochip_ops owl_gpiochip_ops = {
	.direction_input = owl_gpio_direction_input,
	.direction_output = owl_gpio_direction_output,
	.get_value = owl_gpio_get_value,
	.set_value = owl_gpio_set_value,
};


static void gpio_cfg_init(void)
{
	int node, i, j, k;
	char sname[32];
	unsigned int tmp[3];
	const uint32_t *cell;
	int len, iolen, item_num, gpio;

	node = fdt_path_offset(gd->fdt_blob, "/gpio_int");
	if ( node < 0 ) {
		printf("get node gpio_init fail\n");
		return ;
	}
	iolen = sizeof(uint32_t)*3;

//TS_ Enable D28,D29,D30,D31 GPIOs
	writel(0x000000f0, CPU_PWR_CTL);

	//for ( i = 0 ; i < 7; i++ ) { //GPIOA-GPIOG
	for ( i = 0 ; i < OWL_GPIO_BANKS; i++ ) { //TS
		sprintf(sname, "initgpio_%c", 'A'+i);
		len = 0;
		cell = fdt_getprop(gd->fdt_blob, node, sname, &len);
		if (!cell || len < iolen ) {
			printf("GPIO ERROR: get %s, len=%d fail\n", sname, len);
			continue;
		}
		item_num = len/iolen;
		for ( j= 0; j < item_num; j++) {
			for ( k = 0 ; k < 3; k++ ) {
				tmp[k] = fdt32_to_cpu(cell[j*3+k]);
			}
			if ( tmp[0] >= OWL_GPIO_PER_BANK ) {
				printf("GPIO ERROR: %s, gpio num=%d >= 32 fail\n", sname, tmp[0]);
				continue;
			}
			gpio = tmp[0]+ i * OWL_GPIO_PER_BANK; //internal 
			printf("GPIO_INIT: GPIO%c_%2d, gpio=%d,mode=%d,val=%d\n",'A'+i,tmp[0], gpio, tmp[1], tmp[2]);
			if ( tmp[1] == 1 ) // 1: input, 0: output
				owl_gpio_clr(gpio);
			else if ( tmp[1] == 0 )
				owl_gpio_direction_output(gpio, tmp[2]);
			else
				owl_gpio_direction_input(gpio);
                        //owl_gpio_reg_dump(gpio); //TS_

		}		

	}
	
}

int owl_gpio_init(void)
{
	int dev_node = 0;
	int ret;

	dev_node = fdtdec_next_compatible(gd->fdt_blob,
			0, COMPAT_ACTIONS_OWL_GPIO);
	if (dev_node <= 0) {
		debug("Can't get owl gpio device node\n");
		return -1;
	}

/***enable gpio module*************/
	setbits_le32(CMU_DEVCLKEN0, DEVCLKEN_GPIO);
/*****************************/

	ret = gpiochip_add(OWL_GPIOID_MASTER_IC,
			dev_node, &owl_gpiochip_ops);
	if (ret)
		return -1;
	gpio_cfg_init();
	return 0;
}

int owl_gpio_reg_dump_all()
{
	int i, dummy_gpio;
	/* Dump the regs */
	for (i=0; i< OWL_GPIO_BANKS; i++) {
		dummy_gpio = 1 + i*OWL_GPIO_PER_BANK;
		owl_gpio_reg_dump(dummy_gpio);
	}

	return 0;
}

int owl_gpio_reg_dump(unsigned int gpio)
{
	printf("BANK:%1c\n", 'A'+GPIO_BANK(gpio) );
	printf("OUTEN:0X%8X=0X%8X  INEN:0X%8X=0X%8X  DAT:0X%8X=0X%8X", GPIO_REG_OUTEN(gpio), readl(GPIO_REG_OUTEN(gpio)), GPIO_REG_INEN(gpio), readl(GPIO_REG_INEN(gpio)), GPIO_REG_DAT(gpio), readl(GPIO_REG_DAT(gpio)) );
	printf("\n");
	printf("\n");

	return 0;
}

//TS_
#ifdef CONFIG_DM_GPIO

static int owl_dm_gpio_direction_input(struct udevice *dev, unsigned gpio)
{
	owl_gpio_direction_input(gpio);

	return 0;
}

static int owl_dm_gpio_direction_output(struct udevice *dev, unsigned gpio, int value)
{
	owl_gpio_direction_output(gpio, value);

	return 0;
}

static int owl_dm_gpio_get_value(struct udevice *dev, unsigned gpio)
{
	return owl_gpio_get_value(gpio);
}

static int owl_dm_gpio_set_value(struct udevice *dev, unsigned gpio, int value)
{
	owl_gpio_set_value(gpio, value);

	return 0;
}

static int owl_dm_gpio_get_function(struct udevice *dev, unsigned gpio)
{
	struct owl_gpios *gpios = dev_get_priv(dev);
	
	/* xxx */
	/* Dump the regs */
	owl_gpio_reg_dump(gpio);

	return 0;
}


//static struct dm_gpio_ops owl_dm_gpio_ops;
static const struct dm_gpio_ops owl_dm_gpio_ops = {
	.direction_input 	= owl_dm_gpio_direction_input,
	.direction_output	= owl_dm_gpio_direction_output,
	.get_value		= owl_dm_gpio_get_value,
	.set_value		= owl_dm_gpio_set_value,
	.get_function		= owl_dm_gpio_get_function,
};


static int owl_dm_gpio_bind(struct udevice *parent)
{
	struct owl_gpio_platdata *plat;
	struct udevice *dev;
	int bank;
	int ret;

printf("GPIO Bind starting ...\n");

	/* if this is a child device, there is nothing to do here */
	if (parent->platdata) {
		printf("GPIO %s: parent->platdata is not NULL\n", __func__);
		return 0;
	}

	
	//for (bank = 0; bank < OWL_GPIO_BANKS; bank++) {

	plat = calloc(1, sizeof(*plat));
	if (!plat) {
		printf("GPIO %s: Can not alloc Plat\n", __func__);
		return -ENOMEM;
	}
	plat->bank_name = "GPIO";
	plat->gpio_base = 0;
	plat->gpio_count = (OWL_GPIO_BANKS-1) * OWL_GPIO_PER_BANK + 4; // the last bank has only 4 gpios

	ret = device_bind(parent, parent->driver, plat->bank_name, plat, -1, &dev);

	if (ret) {
		printf("GPIO %s: ret=0X%X\n", __func__, ret);
		return ret;
	}
	dev->of_offset = parent->of_offset;
	printf("\nGPIO : OWL_GM_GPIO BIND OK\n");
	return 0;
	//}
}

static int owl_dm_gpio_probe(struct udevice *dev)
{
	struct owl_gpios *gpios = dev_get_priv(dev);
	struct owl_gpio_platdata *plat = dev_get_platdata(dev);
	struct gpio_dev_priv *uc_priv = dev->uclass_priv;

printf("GPIO Probe starting ...\n");

	if (plat) {
		uc_priv->bank_name = plat->bank_name;
		uc_priv->gpio_base = plat->gpio_base;
		uc_priv->gpio_count = plat->gpio_count;
printf("GPIO %s: bank_name=%s, gpio_base=%d, gpip_count=%d\n", __func__, uc_priv->bank_name, uc_priv->gpio_base, uc_priv->gpio_count);
		//gpios->reg = (struct owl_gpio_regs *)plat->base;
	}

	return 0;
}

static const struct udevice_id owl_gpio_ids[] = {
	{ .compatible = "Actions,atm7059a-gpio" },
};

U_BOOT_DRIVER(gpio_owl) = {
	.name	= "gpio_owl",
	.id	= UCLASS_GPIO,
	.ops	= &owl_dm_gpio_ops,
	.of_match = owl_gpio_ids,
	.probe	= owl_dm_gpio_probe,
	.bind	= owl_dm_gpio_bind,
//	.priv_auto_alloc_size = sizeof(struct owl_gpios),
};

#endif //CONFIG_DM_GPIO
