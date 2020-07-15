#include <linux/gpio.h>
#include <linux/interrupt.h>
#include<linux/workqueue.h>

#include "kmodule.h"
#include "print_helper.h"

#define HELPER_NAME "hcsr_helper"

#define DEV_NAME_LEN 	20
#define BUFFER_LEN		5

/* IOCTL Macros */
#define CONFIG_PINS		1
#define SET_PARAMETERS	4

struct hcsr_work_data
{
	struct work_struct work;
};

/* pin_val structure: Used for pin configuration */
struct pin_val {
	int pin;
	int val;
	char label[20];
};

/* ioctl_params structure: Used to get multiple parameters in ioctl */
struct ioctl_params {
    unsigned int ioctl_param1;
    unsigned int ioctl_param2;
};

/* Distance Measurement Structure */
struct dist_measure
{
	unsigned long long time_stamp; // In us
	unsigned int distance; // In cm
};

/* per device structure */
struct hcsr_dev {
	/* sysfs members */
	int enable;

	char name[DEV_NAME_LEN];

	/* Measurement Configuration */
	int m; /* (m+2) samples will be collected and two outliers will be discarded from them */
	int delta; /* delta is the period between two consecutive distance measurements */

	/* Pin Configuration */
	int echo_pin;
	int trigger_pin;
	int echo_irq;

	unsigned int echo_acquired_pin_list[5];
	unsigned int echo_acquired_pin_list_len;
	unsigned int trigger_acquired_pin_list[5];
	unsigned int trigger_acquired_pin_list_len;

	/* Interrupt Configuration */
	int echo_interrupt_edge_config; /* 1: Rising; 0: Falling*/

	/* Configuration Check */
	int pin_config_flag;
	int measurement_config_flag;

	/* Distance measuring variables */
	unsigned long long time;
	struct dist_measure buffer[BUFFER_LEN];
	int buffer_pointer;
	int buffer_count;

	/* Synchronization variables */
    struct semaphore device_lock;   /* Device lock is used to allow only single access to hcsr at a time */
	struct semaphore measurement_flag_lock; /* This lock is used to lock access to ongoing_measurement_flag */
	struct semaphore irq_sync_lock; /* This semaphore is used for synchronization with echo pin interrupt handler*/
	struct semaphore buffer_lock;
	struct semaphore enable_lock;
	int ongoing_measurement_flag;

};

struct hcsr_dev *hcsr_devp;

/*
 *	Core Initialization Function ( for hcsr_dev )
 */

static int hcsr_dev_init(struct hcsr_dev *hcsr_devp)
{
	int ret = 0;

	DPRINTK("hcsr_dev_init() called\n");
	
	/* Initializing Pin Configurations */
	hcsr_devp->echo_pin = -1;
	hcsr_devp->trigger_pin = -1;
	hcsr_devp->echo_irq = -1;
	hcsr_devp->echo_acquired_pin_list_len = 0;
	hcsr_devp->trigger_acquired_pin_list_len = 0;

	/* sysfs configuration */
	hcsr_devp->enable = 1;

	/* Measurement Configurations */
	hcsr_devp->m = 10;
	hcsr_devp->delta = 60;

	/* Configuration Check */
	hcsr_devp->pin_config_flag = 0;
	hcsr_devp->measurement_config_flag = 0;

	/* Echo Interrupt Config */
	hcsr_devp->echo_interrupt_edge_config = 1;

	/* Synchronization variables */
	hcsr_devp->ongoing_measurement_flag = 0;

	/* Measurement Buffer Initialization */
	hcsr_devp->buffer_pointer = 0;
	hcsr_devp->buffer_count = 0;

	/* Initializing device lock */
	sema_init(&(hcsr_devp->device_lock), 1);
	sema_init(&(hcsr_devp->measurement_flag_lock), 1);
	sema_init(&(hcsr_devp->irq_sync_lock), 1);
	sema_init(&(hcsr_devp->buffer_lock), 1);
	sema_init(&(hcsr_devp->enable_lock), 1);

	return ret;
}


/*
 * IRQ Handler for Echo pin triggered interrupt
 */
static irq_handler_t echo_irq_handler(int irq, void *dev_id)
{
	struct hcsr_dev *hcsr_devp = dev_id;

	//DPRINTK("(%s)IRQ Handler called by %s\n", (hcsr_devp->echo_interrupt_edge_config)?"RISING":"FALLING", hcsr_devp->name);

	if(hcsr_devp->echo_interrupt_edge_config == 1)
	{
		/* Save start time */
		hcsr_devp->time = native_read_tsc();

		/* Changing the interrupt trigger */
		irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);
		hcsr_devp->echo_interrupt_edge_config = 0;
	}
	else if(hcsr_devp->echo_interrupt_edge_config == 0)
	{
		/* Get the end time */
		hcsr_devp->time = native_read_tsc() - hcsr_devp->time;

		/* Changing the interrupt trigger */
		irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
		hcsr_devp->echo_interrupt_edge_config = 2;
		
		/* Now the measurement worker can move ahead */
		up(&(hcsr_devp->irq_sync_lock));
	}

	return (irq_handler_t) IRQ_HANDLED;
}

/*
 * Set Trigger Pin Function
 */
int set_trigger_pin(struct hcsr_dev *hcsr_devp)
{
	int ret = 0, gpio;

	if(hcsr_devp->trigger_acquired_pin_list_len == 0)
	{
		EPRINTK("No trigger pin is configured for this device");
		return -ENXIO; // No trigger pin is configured
	}

	gpio = hcsr_devp->trigger_acquired_pin_list[0];

	//DPRINTK("Setting gpio%d to ON\n", gpio);
	ret = gpio_direction_output(gpio, 1);
	if(ret)
	{
		EPRINTK("Error setting direction of gpio%d (Error Code: %d)\n", gpio, ret);
		return ret;
	}

	return ret;
}

/*
 * Clear Trigger Pin Function
 */
int clear_trigger_pin(struct hcsr_dev *hcsr_devp)
{
	int ret = 0, gpio;

	if(hcsr_devp->trigger_acquired_pin_list_len == 0)
	{
		EPRINTK("No trigger pin is configured for this device");
		return -ENXIO; // No trigger pin is configured
	}

	gpio = hcsr_devp->trigger_acquired_pin_list[0];

	//DPRINTK("Setting gpio%d to OFF\n", gpio);
	ret = gpio_direction_output(gpio, 0);
	if(ret)
	{
		EPRINTK("Error setting direction of gpio%d (Error Code: %d)\n", gpio, ret);
		return ret;
	}

	return ret;
}

/*
 * Blink Function
 */
int blink(struct hcsr_dev *hcsr_devp)
{
	int ret = 0;
	ret = set_trigger_pin(hcsr_devp);
	if(ret)
	{
		return ret;
	}

	msleep(1000);

	ret = clear_trigger_pin(hcsr_devp);
	if(ret)
	{
		return ret;
	}
	return ret;
}

/*
 * Free Trigger Pins Function
 */
void free_trigger_pins(struct hcsr_dev *hcsr_devp)
{
	int iter;
	for(iter = 0; iter<hcsr_devp->trigger_acquired_pin_list_len; iter++)
	{
		DPRINTK("Freeing trigger pin: %d\n", hcsr_devp->trigger_acquired_pin_list[iter]);
		gpio_free(hcsr_devp->trigger_acquired_pin_list[iter]);
	}
	hcsr_devp->trigger_acquired_pin_list_len = 0;
}

/*
 * Free Echo Pins Function
 */
void free_echo_pins(struct hcsr_dev *hcsr_devp)
{
	int iter;

	/* Free echo irq */
	if(hcsr_devp->echo_irq >=0)
	{
		DPRINTK("Freeing irq from echo pin\n");
		free_irq(hcsr_devp->echo_irq, hcsr_devp);
	}

	for(iter = 0; iter<hcsr_devp->echo_acquired_pin_list_len; iter++)
	{
		DPRINTK("Freeing echo pin: %d\n", hcsr_devp->echo_acquired_pin_list[iter]);
		gpio_free(hcsr_devp->echo_acquired_pin_list[iter]);
	}
	hcsr_devp->echo_acquired_pin_list_len = 0;
}


/*
 * Configure Trigger Pins Function
 */
int trigger_configure_pins(struct hcsr_dev *hcsr_devp, struct pin_val linux_gpio, struct pin_val level_shift, struct pin_val pullup_config, struct pin_val pin_mux1, struct pin_val pin_mux2)
{
	int ret=0;

	/* Acquiring required pins */
	if(linux_gpio.pin != -1)
	{
		ret = gpio_request(linux_gpio.pin, linux_gpio.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", linux_gpio.label, ret);
		else
			hcsr_devp->trigger_acquired_pin_list[hcsr_devp->trigger_acquired_pin_list_len++] = linux_gpio.pin;
	}

	if(level_shift.pin != -1)
	{
		ret = gpio_request(level_shift.pin, level_shift.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", level_shift.label, ret);
		else
			hcsr_devp->trigger_acquired_pin_list[hcsr_devp->trigger_acquired_pin_list_len++] = level_shift.pin;
	}

	if(pullup_config.pin != -1)
	{
		ret = gpio_request(pullup_config.pin, pullup_config.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pullup_config.label, ret);
		else
			hcsr_devp->trigger_acquired_pin_list[hcsr_devp->trigger_acquired_pin_list_len++] = pullup_config.pin;
	}

	if(pin_mux1.pin != -1)
	{
		ret = gpio_request(pin_mux1.pin, pin_mux1.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pin_mux1.label, ret);
		else
			hcsr_devp->trigger_acquired_pin_list[hcsr_devp->trigger_acquired_pin_list_len++] = pin_mux1.pin;
	}

	if(pin_mux2.pin != -1)
	{
		ret = gpio_request(pin_mux2.pin, pin_mux2.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pin_mux2.label, ret);
		else
			hcsr_devp->trigger_acquired_pin_list[hcsr_devp->trigger_acquired_pin_list_len++] = pin_mux2.pin;
	}

	/* Configuring acquired pins */
	if(level_shift.pin != -1)
	{
		ret = gpio_direction_output(level_shift.pin, level_shift.val);
		if(ret)
			EPRINTK("Error setting direction of %s (Error Code: %d)\n", level_shift.label, ret);
	}

	if(pullup_config.pin != -1)
	{
		ret = gpio_direction_output(pullup_config.pin, pullup_config.val);
		if(ret)
			EPRINTK("Error setting direction of %s (Error Code: %d)\n", pullup_config.label, ret);
	}

	if(pin_mux1.pin != -1)
	{
		ret = gpio_direction_output(pin_mux1.pin, pin_mux1.val);
		if(ret)
			EPRINTK("Error setting direction of %s (Error Code: %d)\n", pin_mux1.label, ret);
	}

	if(pin_mux2.pin != -1)
	{
		ret = gpio_direction_output(pin_mux2.pin, pin_mux2.val);
		if(ret)
			EPRINTK("Error setting direction of %s (Error Code: %d)\n", pin_mux2.label, ret);
	}

	ret = clear_trigger_pin(hcsr_devp);
	if(ret)
	{
		return ret;
	}

	if(ret != 0)
	{
		free_trigger_pins(hcsr_devp);
	}
	return ret;
}

/*
 * Configure Echo Pins Function
 */
int echo_configure_pins(struct hcsr_dev *hcsr_devp, struct pin_val linux_gpio, struct pin_val level_shift, struct pin_val pullup_config, struct pin_val pin_mux1, struct pin_val pin_mux2)
{
	int ret=0;

	/* Acquiring required pins */
	if(linux_gpio.pin != -1)
	{
		ret = gpio_request(linux_gpio.pin, linux_gpio.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", linux_gpio.label, ret);
		else
			hcsr_devp->echo_acquired_pin_list[hcsr_devp->echo_acquired_pin_list_len++] = linux_gpio.pin;
	}
	
	if(level_shift.pin != -1)
	{
		ret = gpio_request(level_shift.pin, level_shift.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", level_shift.label, ret);
		else
			hcsr_devp->echo_acquired_pin_list[hcsr_devp->echo_acquired_pin_list_len++] = level_shift.pin;
	}
	
	if(pullup_config.pin != -1)
	{
		ret = gpio_request(pullup_config.pin, pullup_config.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pullup_config.label, ret);
		else
			hcsr_devp->echo_acquired_pin_list[hcsr_devp->echo_acquired_pin_list_len++] = pullup_config.pin;
	}
	
	if(pin_mux1.pin != -1)
	{
		ret = gpio_request(pin_mux1.pin, pin_mux1.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pin_mux1.label, ret);
		else
			hcsr_devp->echo_acquired_pin_list[hcsr_devp->echo_acquired_pin_list_len++] = pin_mux1.pin;
	}
	
	if(pin_mux2.pin != -1)
	{
		ret = gpio_request(pin_mux2.pin, pin_mux2.label); // Linux Pin
		if(ret)
			EPRINTK("Error obtaining %s (Error Code: %d)\n", pin_mux2.label, ret);
		else
			hcsr_devp->echo_acquired_pin_list[hcsr_devp->echo_acquired_pin_list_len++] = pin_mux2.pin;
	}
	
	/* Configuring acquired pins */
	if(level_shift.pin != -1)
	{
		ret = gpio_direction_output(level_shift.pin, level_shift.val);
		if(ret)
			EPRINTK("Error setting direction of %s (Error Code: %d)\n", level_shift.label, ret);
	}
	
	if(pullup_config.pin != -1)
	{
		ret = gpio_direction_output(pullup_config.pin, pullup_config.val);
		if(ret)
			EPRINTK("Error setting direction of %s (Error Code: %d)\n", pullup_config.label, ret);
	}
	
	if(pin_mux1.pin != -1)
	{
		ret = gpio_direction_output(pin_mux1.pin, pin_mux1.val);
		if(ret)
			EPRINTK("Error setting direction of %s (Error Code: %d)\n", pin_mux1.label, ret);
	}
	
	if(pin_mux2.pin != -1)
	{
		ret = gpio_direction_output(pin_mux2.pin, pin_mux2.val);
		if(ret)
			EPRINTK("Error setting direction of %s (Error Code: %d)\n", pin_mux2.label, ret);
	}
	
	ret = gpio_direction_input(linux_gpio.pin);
	if(ret)
		EPRINTK("Error setting direction of %s (Error Code: %d)\n", linux_gpio.label, ret);
	
	/* Binding an interrupt handler to echo pin */
	hcsr_devp->echo_irq = gpio_to_irq(linux_gpio.pin);
	if(hcsr_devp->echo_irq < 0)
	{
		EPRINTK("Errot obtaining irq number associated with echo pin %d (Error Code: %d)\n", linux_gpio.pin, ret);
	}
	
	ret = request_irq(hcsr_devp->echo_irq,(irq_handler_t) echo_irq_handler, IRQF_TRIGGER_RISING, "echo_rise", hcsr_devp);
	if(ret)
	{
		EPRINTK("Error setting IRQ Handler (Error Code: %d)\n", ret);
	}
	
	if(ret != 0)
	{
		free_echo_pins(hcsr_devp);
	}
	
	return ret;
}


/*
 * Core Ioctl Driver
 */
static int _hcsr_driver_ioctl(struct hcsr_dev *hcsr_devp, unsigned int ioctl_num, struct ioctl_params params)
{
	struct pin_val linux_gpio, level_shift, pullup_config, pin_mux1, pin_mux2;
	int ret = 0;
	int trig_unconfig = 0, echo_unconfig = 0;

	down(&(hcsr_devp->device_lock));
	
	switch(ioctl_num)
	{
		case CONFIG_PINS:
			/* params.ioctl_param1 will be the trigger pin */
			/* params.ioctl_param2 will be the echo pin */

			if(!(params.ioctl_param2 >= 0 && params.ioctl_param2 <= 20))
			{
				up(&(hcsr_devp->device_lock));
				return -EINVAL;
			}
			if(!(params.ioctl_param1 >= 0 && params.ioctl_param1 <= 20))
			{
				up(&(hcsr_devp->device_lock));
				return -EINVAL;
			}
			else if(params.ioctl_param1 == params.ioctl_param2)
			{
				up(&(hcsr_devp->device_lock));
				return -EINVAL;
			}

			if(ret)
			{
				EPRINTK("Either echo_pin or trigger_pin or both are already in use\n");
				up(&(hcsr_devp->device_lock));
				return ret;
			}

			hcsr_devp->trigger_pin = params.ioctl_param1;
			hcsr_devp->echo_pin = params.ioctl_param2;

			/* Free previously acquired trigger pins */
			free_trigger_pins(hcsr_devp);

			/* Free previously acquired echo pins */
			free_echo_pins(hcsr_devp);

			/*
			 * To be configured as OUTPUT
			 * Trigger Pin - Find the matching Linux Pin
			 * 				 Set the level shifting GPIO
			 * 				 Set the proper pin-muxing
			 */
			switch(hcsr_devp->trigger_pin)
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
					DPRINTK("Trigger pin is not set\n");
					trig_unconfig = 1;
					break;

				default:
					EPRINTK("Invalid Trigger Pin Selection\n");
					up(&(hcsr_devp->device_lock));
					return -EINVAL;
			}

			DPRINTK("Configuring Trigger pins\n");
			if(!trig_unconfig)
			{	
				ret = trigger_configure_pins(hcsr_devp, linux_gpio,	level_shift, pullup_config, pin_mux1, pin_mux2);
				if(ret)
				{
					EPRINTK("trigger_configure_pins() failed (Error Code: %d)\n", ret);
					free_trigger_pins(hcsr_devp);
					up(&(hcsr_devp->device_lock));
					return ret;
				}
			}
			
			/* Testing */
			//DPRINTK("Testing configured Trigger pins\n");
			//ret = blink(hcsr_devp);
			//if(ret)
			//{
			//	EPRINTK("blink() failed (Error Code: %d)\n", ret);
			//	return ret;
			//}

			/*
			 * To be configured as INPUT (with Interrupt)
			 * Echo Pin - Find the matching Linux Pin
			 * 			  Set the level shifting GPIO
			 *			  Set the Pull-up/Pull-down
			 *			  Set the proper pin-muxing
		     *			  Set the proper interrupt mode
			 */ 
			switch(hcsr_devp->echo_pin)
			{
				case 0:
					linux_gpio 		= (struct pin_val){.pin = 11, .val = 0, .label="gpio11"};
					level_shift 	= (struct pin_val){.pin = 32, .val = 1, .label="gpio32"};
					pullup_config 	= (struct pin_val){.pin = 33, .val = 0, .label="gpio33"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 1:
					linux_gpio 		= (struct pin_val){.pin = 12, .val = 0, .label="gpio12"};
					level_shift 	= (struct pin_val){.pin = 28, .val = 1, .label="gpio28"};
					pullup_config 	= (struct pin_val){.pin = 29, .val = 0, .label="gpio29"};
					pin_mux1 		= (struct pin_val){.pin = 45, .val = 0, .label="gpio45"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 2:
					linux_gpio 		= (struct pin_val){.pin = 13, .val = 0, .label="gpio13"};
					level_shift 	= (struct pin_val){.pin = 34, .val = 1, .label="gpio34"};
					pullup_config 	= (struct pin_val){.pin = 35, .val = 0, .label="gpio35"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO77 to L, IO error encountered
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 3:
					linux_gpio 		= (struct pin_val){.pin = 14, .val = 0, .label="gpio14"};
					level_shift 	= (struct pin_val){.pin = 16, .val = 1, .label="gpio16"};
					pullup_config 	= (struct pin_val){.pin = 17, .val = 0, .label="gpio17"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO76 to L, IO error encountered
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO64 to L, IO error encountered
					break;

				case 4:
					linux_gpio 		= (struct pin_val){.pin = 6, .val = 0, .label="gpio6"};
					level_shift 	= (struct pin_val){.pin = 36, .val = 1, .label="gpio36"};
					pullup_config 	= (struct pin_val){.pin = 37, .val = 0, .label="gpio37"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 5: // Not working, whenever interrupt occurs at this pin, the board goes whoosh
					linux_gpio 		= (struct pin_val){.pin = 0, .val = 0, .label="gpio0"};
					level_shift 	= (struct pin_val){.pin = 18, .val = 1, .label="gpio18"};
					pullup_config 	= (struct pin_val){.pin = 19, .val = 0, .label="gpio19"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO66 to L, IO error encountered
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 6:
					linux_gpio 		= (struct pin_val){.pin = 1, .val = 0, .label="gpio1"};
					level_shift 	= (struct pin_val){.pin = 20, .val = 1, .label="gpio20"};
					pullup_config 	= (struct pin_val){.pin = 21, .val = 0, .label="gpio21"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO68 to L, IO error encountered
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 9: // Not working, whenever interrupt occurs at this pin, the board goes whoosh
					linux_gpio 		= (struct pin_val){.pin = 4, .val = 0, .label="gpio4"};
					level_shift 	= (struct pin_val){.pin = 22, .val = 1, .label="gpio22"};
					pullup_config 	= (struct pin_val){.pin = 23, .val = 0, .label="gpio23"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO70 to L, IO error encountered
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 10:
					linux_gpio 		= (struct pin_val){.pin = 10, .val = 0, .label="gpio10"};
					level_shift 	= (struct pin_val){.pin = 26, .val = 1, .label="gpio26"};
					pullup_config 	= (struct pin_val){.pin = 27, .val = 0, .label="gpio27"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO74 to L, IO error encountered
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				// TODO: Check echo pins from here onwards
				case 11:
					linux_gpio 		= (struct pin_val){.pin = 5, .val = 0, .label="gpio5"};
					level_shift 	= (struct pin_val){.pin = 24, .val = 1, .label="gpio24"};
					pullup_config 	= (struct pin_val){.pin = 25, .val = 0, .label="gpio25"};
					pin_mux1 		= (struct pin_val){.pin = 44, .val = 0, .label="gpio44"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"}; // Unable to set GPIO72 to L, IO error encountered
					break;

				case 12:
					linux_gpio 		= (struct pin_val){.pin = 15, .val = 0, .label="gpio15"};
					level_shift 	= (struct pin_val){.pin = 42, .val = 1, .label="gpio42"};
					pullup_config 	= (struct pin_val){.pin = 43, .val = 0, .label="gpio43"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 13:
					linux_gpio 		= (struct pin_val){.pin = 7, .val = 0, .label="gpio7"};
					level_shift 	= (struct pin_val){.pin = 30, .val = 1, .label="gpio30"};
					pullup_config 	= (struct pin_val){.pin = 31, .val = 0, .label="gpio31"};
					pin_mux1 		= (struct pin_val){.pin = 46, .val = 0, .label="gpio46"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 14:
					linux_gpio 		= (struct pin_val){.pin = 48, .val = 0, .label="gpio48"};
					level_shift 	= (struct pin_val){.pin = -1, .val = 1, .label="NULL"};
					pullup_config 	= (struct pin_val){.pin = 49, .val = 0, .label="gpio49"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 15:
					linux_gpio 		= (struct pin_val){.pin = 50, .val = 0, .label="gpio50"};
					level_shift 	= (struct pin_val){.pin = -1, .val = 1, .label="NULL"};
					pullup_config 	= (struct pin_val){.pin = 51, .val = 0, .label="gpio51"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 16:
					linux_gpio 		= (struct pin_val){.pin = 52, .val = 0, .label="gpio52"};
					level_shift 	= (struct pin_val){.pin = -1, .val = 1, .label="NULL"};
					pullup_config 	= (struct pin_val){.pin = 53, .val = 0, .label="gpio53"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 17:
					linux_gpio 		= (struct pin_val){.pin = 54, .val = 0, .label="gpio54"};
					level_shift 	= (struct pin_val){.pin = -1, .val = 1, .label="NULL"};
					pullup_config 	= (struct pin_val){.pin = 55, .val = 0, .label="gpio55"};
					pin_mux1 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pin_mux2 		= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					break;

				case 18:
					linux_gpio 		= (struct pin_val){.pin = 56, .val = 0, .label="gpio56"};
					level_shift 	= (struct pin_val){.pin = -1, .val = 1, .label="NULL"};
					pullup_config 	= (struct pin_val){.pin = 57, .val = 0, .label="gpio57"};
					pin_mux1 		= (struct pin_val){.pin = 60, .val = 1, .label="gpio60"};
					pin_mux2 		= (struct pin_val){.pin = 78, .val = 1, .label="gpio78"};
					break;

				case 19:
					linux_gpio 		= (struct pin_val){.pin = 58, .val = 0, .label="gpio58"};
					level_shift 	= (struct pin_val){.pin = -1, .val = 1, .label="NULL"};
					pullup_config 	= (struct pin_val){.pin = 59, .val = 0, .label="gpio59"};
					pin_mux1 		= (struct pin_val){.pin = 60, .val = 1, .label="gpio60"};
					pin_mux2 		= (struct pin_val){.pin = 79, .val = 1, .label="gpio79"};
					break;

				case -1:
					DPRINTK("Echo pin is not configured\n");
					echo_unconfig = 1;
					break;

				default:
					EPRINTK("Invalid Echo Pin Selection\n");
					free_trigger_pins(hcsr_devp);
					up(&(hcsr_devp->device_lock));
					return -EINVAL;
			}

			DPRINTK("Configuring Echo pins\n");
			if(!echo_unconfig)
			{
				ret = echo_configure_pins(hcsr_devp, linux_gpio, level_shift, pullup_config, pin_mux1, pin_mux2);
				if(ret)
				{
					EPRINTK("echo_configure_pins() failed (Error Code: %d)\n", ret);
					free_trigger_pins(hcsr_devp);
					up(&(hcsr_devp->device_lock));
					return ret;
				}
			}			


			hcsr_devp->pin_config_flag = 1;
			DPRINTK("Pin configuration done\n");
			break; // for CONFIG_PINS

		case SET_PARAMETERS:
			DPRINTK("Setting measurement parameters\n");

			/* params.ioctl_param1 will be the samples per measurement */
			/* params.ioctl_param2 will be the sampling period */
			
			if(params.ioctl_param1 < 1)
			{
				EPRINTK("m is the number of samples to be taken, and so it cannot be less than 1\n");
				up(&(hcsr_devp->device_lock));
				return -EINVAL;
			}

			if(params.ioctl_param2 < 60)
			{
				EPRINTK("Delta is the sampling period in milliseconds and cannot be negative\n");
				up(&(hcsr_devp->device_lock));
				return -EINVAL;
			}
			
			hcsr_devp->m = params.ioctl_param1;
			hcsr_devp->delta = params.ioctl_param2;

			DPRINTK("M = %d\n", hcsr_devp->m);
			DPRINTK("Delta = %d\n", hcsr_devp->delta);

			hcsr_devp->measurement_config_flag = 1;
			break;

		default:
			DPRINTK("Invalid ioctl_num\n");
			up(&(hcsr_devp->device_lock));
			return -EINVAL;
	}
	up(&(hcsr_devp->device_lock));
	return 0;
}
