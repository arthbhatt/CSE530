
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/moduleparam.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "print_helper.h"

 struct spi_device *led_matrix_spi;
static struct spi_message led_matrix_m;
static unsigned char led_buf[2];
int linux_cs_pin = 15;

static struct spi_transfer led_matrix_t = {
			.tx_buf = led_buf,
			.rx_buf = 0,
			.len = 2,
			.cs_change = 1,
			.bits_per_word = 8,
			.speed_hz = 500000,
			 };

/* This function does a 16 bit transfer to led matrix */
void spi_data_transfer(unsigned char byte0, unsigned char byte1, int cs_pin)
{
    // TODO: set chip select pin
    int ret=0;
    led_buf[0] = byte0;
    led_buf[1] = byte1;

	spi_message_init(&led_matrix_m);
	spi_message_add_tail(&led_matrix_t, &led_matrix_m);
    gpio_set_value_cansleep(cs_pin, 0);
    ret = spi_sync(led_matrix_spi, &led_matrix_m);
	gpio_set_value_cansleep(cs_pin, 1);
	
    if(ret)
    {
        EPRINTK("SPI Transfer unsuccessful!\n");
    }
    return;
}

void led_matrix_cfg(int cs_pin)
{
    long int count = 100000;

    spi_data_transfer(0xF0, 0x00, cs_pin);
    spi_data_transfer(0x0C, 0x01, cs_pin);
    spi_data_transfer(0x0B, 0x07, cs_pin);
    spi_data_transfer(0x09, 0x00, cs_pin);
    spi_data_transfer(0x0A, 0x02, cs_pin);

    /* Lit all */
    spi_data_transfer(0x01, 0xFF, cs_pin);
    spi_data_transfer(0x02, 0xFF, cs_pin);
    spi_data_transfer(0x03, 0xFF, cs_pin);
    spi_data_transfer(0x04, 0xFF, cs_pin);
    spi_data_transfer(0x05, 0xFF, cs_pin);
    spi_data_transfer(0x06, 0xFF, cs_pin);
    spi_data_transfer(0x07, 0xFF, cs_pin);
    spi_data_transfer(0x08, 0xFF, cs_pin);
    
    while(count--);

    /* Blow all */
    spi_data_transfer(0x01, 0x00, cs_pin);
    spi_data_transfer(0x02, 0x00, cs_pin);
    spi_data_transfer(0x03, 0x00, cs_pin);
    spi_data_transfer(0x04, 0x00, cs_pin);
    spi_data_transfer(0x05, 0x00, cs_pin);
    spi_data_transfer(0x06, 0x00, cs_pin);
    spi_data_transfer(0x07, 0x00, cs_pin);
    spi_data_transfer(0x08, 0x00, cs_pin);
}

/* This probe function is called when a matching spi device is added */
static int led_matrix_probe(struct spi_device *spi)
{
    EPRINTK("Led Matrix Driver Probe Function called\n");

	led_matrix_spi = spi;
    return 0;
}

/* This function is called when the registered spi device is removed */
static int led_matrix_remove(struct spi_device *spi)
{
    EPRINTK("Led Matrix Driver removed\n");
	return 0;
}

/* Filling the spi device table with the same name as device */
static const struct spi_device_id device_table[] = 
{
    { "MAX7219", 0 },
    { }
};

/* Spi Driver Struct */
static struct spi_driver led_matrix_driver = 
{
	.driver = 
    {
		.name   =	"MAX7219",
		.owner  =   THIS_MODULE,
	},
	.probe      =	led_matrix_probe,
	.remove     =	led_matrix_remove,
	.id_table   =   device_table,

};

static int led_matrix_driver_init(void)
{
	int ret;
	
    /*Request the required pins*/
    /* MOSI pin cfg */
    gpio_request(24, "MOSI_SHIFT");
    gpio_request(44, "MOSI_MUX1");
	gpio_request(72, "MOSI_MUX2");
	
    /* MISO pin cfg */
    gpio_request(42, "MISO_SHIFT");

    /* SCK pin cfg */
    gpio_request(46, "SPI_SCK");
	gpio_request(30, "SCK_SHIFT");
	
	/* Currently hardcoded CS pin */ /* Shield pin 12 */
	//gpio_request(15, "CS_PIN");  

    /* Set the pin to proper values */
    /* Initiliase GPIO values */
    /* MOSI */
    gpio_set_value_cansleep(44, 1);
	gpio_set_value_cansleep(72, 0);
	gpio_set_value_cansleep(24, 0);

    /* MISO */
    gpio_set_value_cansleep(42, 0);

    /* SCK */
    gpio_set_value_cansleep(46, 1);
    gpio_set_value_cansleep(30, 0);
	
	/* CS */
	//gpio_set_value_cansleep(15, 1);

    ret = spi_register_driver(&led_matrix_driver);
	if (ret < 0) 
    {
        EPRINTK("Unable to register led matrix spi driver\n");
        return -1;
    }
	EPRINTK("Led Matrix Driver Registered\n");
	return 0;
}

static void led_matrix_driver_exit(void)
{
	spi_unregister_driver(&led_matrix_driver);
	EPRINTK("Led Matrix Driver Unregistered\n");

    gpio_free(44);
	gpio_free(72);
	gpio_free(46);
	gpio_free(24);
	gpio_free(42);
	gpio_free(30);
	gpio_free(15); // TODO: Change once CS is not hardcoded

	return;
}

static void config_cs_pin(char shield_cs_pin)
{
    struct pin_val linux_gpio, level_shift, pullup_config, pin_mux1, pin_mux2;
    int ret;

    /*
        * To be configured as OUTPUT
        * CS Pin - Find the matching Linux Pin
        * 				 Set the level shifting GPIO
        * 				 Set the proper pin-muxing
        */
    switch(shield_cs_pin)
    {
        case 0:
            linux_gpio 		= (struct pin_val){.pin = 11, .val = 0, .label="gpio11"};
            level_shift 	= (struct pin_val){.pin = 32, .val = 0, .label="gpio32"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 1:
            linux_gpio 		= (struct pin_val){.pin = 12, .val = 0, .label="gpio12"};
            level_shift 	= (struct pin_val){.pin = 28, .val = 0, .label="gpio28"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = 45, .val = 0, .label="gpio45"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;
        
        case 2:
            linux_gpio 		= (struct pin_val){.pin = 13, .val = 0, .label="gpio13"};
            level_shift 	= (struct pin_val){.pin = 34, .val = 0, .label="gpio34"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO77 to L, IO error encountered
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 3:
            linux_gpio 		= (struct pin_val){.pin = 14, .val = 0, .label="gpio14"};
            level_shift 	= (struct pin_val){.pin = 16, .val = 0, .label="gpio16"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO76 to L, IO error encountered
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO64 to L, IO error encountered
            break;

        case 4:
            linux_gpio 		= (struct pin_val){.pin = 6, .val = 0, .label="gpio6"};
            level_shift 	= (struct pin_val){.pin = 36, .val = 0, .label="gpio36"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 5:
            linux_gpio 		= (struct pin_val){.pin = 0, .val = 0, .label="gpio0"};
            level_shift 	= (struct pin_val){.pin = 18, .val = 0, .label="gpio18"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO66 to L, IO error encountered
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 6:
            linux_gpio 		= (struct pin_val){.pin = 1, .val = 0, .label="gpio1"};
            level_shift 	= (struct pin_val){.pin = 20, .val = 0, .label="gpio20"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO68 to L, IO error encountered
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 7:
            linux_gpio 		= (struct pin_val){.pin = 38, .val = 0, .label="gpio38"};
            level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 8:
            linux_gpio 		= (struct pin_val){.pin = 40, .val = 0, .label="gpio40"};
            level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 9:
            linux_gpio 		= (struct pin_val){.pin = 4, .val = 0, .label="gpio4"};
            level_shift 	= (struct pin_val){.pin = 22, .val = 0, .label="gpio22"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO70 to L, IO error encountered
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 10:
            linux_gpio 		= (struct pin_val){.pin = 10, .val = 0, .label="gpio10"};
            level_shift 	= (struct pin_val){.pin = 26, .val = 0, .label="gpio26"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO74 to L, IO error encountered
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 11:
            linux_gpio 		= (struct pin_val){.pin = 5, .val = 0, .label="gpio5"};
            level_shift 	= (struct pin_val){.pin = 24, .val = 0, .label="gpio24"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = 44, .val = 0, .label="gpio44"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO72 to L, IO error encountered
            break;

        case 12:
            linux_gpio 		= (struct pin_val){.pin = 15, .val = 0, .label="gpio15"};
            level_shift 	= (struct pin_val){.pin = 42, .val = 0, .label="gpio42"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;
            
        case 13:
            linux_gpio 		= (struct pin_val){.pin = 7, .val = 0, .label="gpio7"};
            level_shift 	= (struct pin_val){.pin = 30, .val = 0, .label="gpio30"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = 46, .val = 0, .label="gpio46"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 14:
            linux_gpio 		= (struct pin_val){.pin = 48, .val = 0, .label="gpio48"};
            level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 15:
            linux_gpio 		= (struct pin_val){.pin = 50, .val = 0, .label="gpio50"};
            level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 16:
            linux_gpio 		= (struct pin_val){.pin = 52, .val = 0, .label="gpio52"};
            level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 17:
            linux_gpio 		= (struct pin_val){.pin = 54, .val = 0, .label="gpio54"};
            level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            break;

        case 18: // Not working due to pin_mux2 config problem
            linux_gpio 		= (struct pin_val){.pin = 56, .val = 0, .label="gpio56"};
            level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = 60, .val = 1, .label="gpio60"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 1, .label="NULL"}; // Unable to set GPIO78 to L, IO error encountered
            break;

        case 19: // Not working due to pin_mux2 config problem
            linux_gpio 		= (struct pin_val){.pin = 58, .val = 0, .label="gpio58"};
            level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
            pin_mux1 		= (struct pin_val){.pin = 60, .val = 1, .label="gpio60"};
            pin_mux2 		= (struct pin_val){.pin = -1, .val = 1, .label="NULL"}; // Unable to set GPIO79 to L, IO error encountered
            break;

        case -1: // Initial configuration may contain this case (sysfs)
            DPRINTK("CS pin is not set\n");
            break;

        default:
            EPRINTK("Invalid CS Pin Selection\n");
            return;
    }

    linux_cs_pin = linux_gpio.pin;
    DPRINTK("Configuring CS pins\n");
    
	/* Acquiring required pins */
	if(linux_gpio.pin != -1)
	{
		ret = gpio_request(linux_gpio.pin, linux_gpio.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", linux_gpio.label, ret);
	}

	if(level_shift.pin != -1)
	{
		ret = gpio_request(level_shift.pin, level_shift.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", level_shift.label, ret);
	}

	if(pullup_config.pin != -1)
	{
		ret = gpio_request(pullup_config.pin, pullup_config.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pullup_config.label, ret);
	}

	if(pin_mux1.pin != -1)
	{
		ret = gpio_request(pin_mux1.pin, pin_mux1.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pin_mux1.label, ret);
	}

	if(pin_mux2.pin != -1)
	{
		ret = gpio_request(pin_mux2.pin, pin_mux2.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pin_mux2.label, ret);
	}

	/* Configuring acquired pins */
	if(level_shift.pin != -1)
	{
		gpio_set_value_cansleep(level_shift.pin, level_shift.val);
	}

	if(pullup_config.pin != -1)
	{
		gpio_set_value_cansleep(pullup_config.pin, pullup_config.val);
	}

	if(pin_mux1.pin != -1)
	{
		gpio_set_value_cansleep(pin_mux1.pin, pin_mux1.val);
	}

	if(pin_mux2.pin != -1)
	{
		gpio_set_value_cansleep(pin_mux2.pin, pin_mux2.val);
	}
    
}

