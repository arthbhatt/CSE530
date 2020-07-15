#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/spi/spi.h>

#include "print_helper.h"

static struct spi_device *led_matrix_device;

static struct spi_board_info led_matrix_info = 
{
	.modalias ="MAX7219" ,
	.bus_num = 1,
	.chip_select = 1,
	.mode=SPI_MODE_0,
	.max_speed_hz=6666666,  // a clock cycle: 0.15us
};

static int __init led_matrix_device_init(void)
{
    struct spi_master *master;
    int ret;

    master = spi_busnum_to_master(led_matrix_info.bus_num);
    if(master == 0)
    {
        EPRINTK("Unable to get bus number for master\n");
        return -ENODEV;
    }

	led_matrix_device = spi_new_device(master, &led_matrix_info);
	if(led_matrix_device == 0)
	{
		EPRINTK("Unable to add led matrix as spi device\n");
		return -ENODEV;
	}

	led_matrix_device->bits_per_word = 16; //TODO: Verify this. Also try 8

	ret = spi_setup(led_matrix_device);
	if(ret != 0)
	{
		EPRINTK("spi_setup() failed\n");
		return -EINVAL;
	}

	return 0;
}

static void led_matrix_spi_device_exit(void)
{
	spi_unregister_device(led_matrix_device);
}

module_init(led_matrix_device_init);
module_exit(led_matrix_spi_device_exit);
MODULE_LICENSE("GPL");