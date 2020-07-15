/* ----------------------------------------------- DRIVER hcsr --------------------------------------------------
CSE530: Embedded Operating System Internals
Assignment 2

Driver made to create and manage HCSR-04 Devices.
Author: Arth Bhatt(ASU ID: 1215361875)
 ----------------------------------------------------------------------------------------------------------------*/

// TODO: Move misc device registration at the end of init function. Check if it caused any bugs or not.
// TODO: Shift the echo pin interrupt config from alternating RISING/FALLING to BOTH_EDGE_TRIGGERED
// TODO: for gpio > 64, use gpio_set_value instead of gpio_direction_output
// TODO: Free struct work_data allocated for queueing work
// TODO: Write the test cases in gtest and submit that list
// TODO: Check every file if it has any memory leakage or not

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>

#include<linux/init.h>
#include<linux/moduleparam.h>

#include<linux/miscdevice.h>
#include<linux/errno.h>
#include<linux/semaphore.h>

#include<linux/string.h>
#include<linux/gpio.h>
#include<linux/delay.h>
#include<linux/interrupt.h>
#include<linux/workqueue.h>

#define DRIVER_NAME					"hcsr_drv"
#define DEVICE_NAME                 "HCSR_"  // device name to be created and registered
#define MAX_DEVICES					20
#define DEV_NAME_LEN				20
#define BUFFER_LEN					5

/* IOCTL Macros */
#define CONFIG_PINS                  1
#define SET_PARAMETERS				 4

//TODO: Convert this back to DEBUG and ERROR
#define D
#define E

#if defined(D)
 	#define DPRINTK(fmt, args...) printk("D: %s:%d:%s(): " fmt, DRIVER_NAME, __LINE__, __func__, ##args)
#else
	#define DPRINTK(fmt, args...)
#endif

#if defined(E)
 	#define EPRINTK(fmt, args...) printk("E: %s:%d:%s(): " fmt, DRIVER_NAME, __LINE__, __func__, ##args)
#else
	#define EPRINTK(fmt, args...)
#endif

struct work_data
{
	struct work_struct work;
	struct hcsr_dev *hcsr_devp;
};

/* Distance Measurement Structure */
struct dist_measure
{
	unsigned long long time_stamp; // In us
	unsigned int distance; // In cm
};

/* per device structure */
struct hcsr_dev {
	struct miscdevice *mdev;               /* The miscdevice structure */

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
	int ongoing_measurement_flag;

};

/* ioctl_params structure: Used to get multiple parameters in ioctl */
struct ioctl_params {
    unsigned int ioctl_param1;
    unsigned int ioctl_param2;
 };

/* pin_val structure: Used for pin configuration */
 struct pin_val {
	 int pin;
	 int val;
	 char label[20];
 };

static struct hcsr_dev *hcsr_dev_list[MAX_DEVICES], *hcsr_devp;
static struct miscdevice mdev_global[MAX_DEVICES];
static struct workqueue_struct *wq;
//static struct class *hcsr_dev_class;          /* Tie with the device model */
static struct semaphore device_list_lock;

static int device_count = 2;
module_param(device_count, int, 0000);


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
		if(pin_mux1.pin >= 64)
		{
			gpio_set_value(pin_mux1.pin, pin_mux1.val);
		}
		else
		{
			ret = gpio_direction_output(pin_mux1.pin, pin_mux1.val);
			if(ret)
				EPRINTK("Error setting direction of %s (Error Code: %d)\n", pin_mux1.label, ret);
		}
	}

	if(pin_mux2.pin != -1)
	{
		if(pin_mux2.pin >= 64)
		{
			gpio_set_value(pin_mux2.pin, pin_mux2.val);
		}
		else
		{
			ret = gpio_direction_output(pin_mux2.pin, pin_mux2.val);
			if(ret)
				EPRINTK("Error setting direction of %s (Error Code: %d)\n", pin_mux2.label, ret);
		}
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
		if(pin_mux1.pin >= 64)
		{
			gpio_set_value(pin_mux1.pin, pin_mux1.val);
		}
		else
		{
			ret = gpio_direction_output(pin_mux1.pin, pin_mux1.val);
			if(ret)
				EPRINTK("Error setting direction of %s (Error Code: %d)\n", pin_mux1.label, ret);
		}
	}

	if(pin_mux2.pin != -1)
	{
		if(pin_mux2.pin >= 64)
		{
			gpio_set_value(pin_mux2.pin, pin_mux2.val);
		}
		else
		{
			ret = gpio_direction_output(pin_mux2.pin, pin_mux2.val);
			if(ret)
				EPRINTK("Error setting direction of %s (Error Code: %d)\n", pin_mux2.label, ret);
		}
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
 * Worker Function: Used to Measure distance
 * This function is not thread-safe. It has to be done where the function is called
 */
static void measure(struct work_struct *work)
{
	struct work_data *data = (struct work_data*) work;
	struct hcsr_dev *hcsr_devp = data->hcsr_devp;
	int ret=0, measurement_count;
	unsigned int max_dist, min_dist, dist, avg_dist;
	DPRINTK("Worker function called\n");

	avg_dist = 0;
	max_dist = 0;
	min_dist = 10000;

	/* Now we start taking a new measurement */
	// TODO: Use timer interrupt to accurately turn off trigger pin
	/*Other irq configurations*/

	for(measurement_count = 0; measurement_count<(hcsr_devp->m+2); measurement_count++)
	{
		hcsr_devp->echo_interrupt_edge_config = 1;
		down(&(hcsr_devp->irq_sync_lock));

		ret = set_trigger_pin(hcsr_devp);
		if(ret)
		{
			DPRINTK("Error setting trigger pin(Error Code: %d)\n", ret);
		}

		udelay(10);

		ret = clear_trigger_pin(hcsr_devp);
		if(ret)
		{
			DPRINTK("Error clearing trigger pin(Error Code: %d)\n", ret);
		}

		down(&(hcsr_devp->irq_sync_lock));
		up(&(hcsr_devp->irq_sync_lock));

		dist = (unsigned int)div_u64(hcsr_devp->time, 23529);

		if(dist > max_dist)
			max_dist = dist;

		if(dist < min_dist)
			min_dist = dist;

		avg_dist += dist;

		// TODO: Adjust the time elapsed in taking a measurement
		msleep(hcsr_devp->delta);
	}

	/* Remove the outliers */
	avg_dist -= max_dist;
	avg_dist -= min_dist;

	/* Average the distance */
	avg_dist = div_u64((unsigned long long)avg_dist, \
																	(unsigned long long)hcsr_devp->m);

	/* Acquire buffer lock */
	down(&(hcsr_devp->buffer_lock));

	/* Store the distance in the buffer */
	hcsr_devp->buffer[hcsr_devp->buffer_pointer].distance = avg_dist;

	/* Time Stamp the measurement */
	hcsr_devp->buffer[hcsr_devp->buffer_pointer].time_stamp = div_u64(native_read_tsc(), 400);

	DPRINTK("Distance = %u cm (timestamp: %llu us)\n",  hcsr_devp->buffer[hcsr_devp->buffer_pointer].distance, \
														hcsr_devp->buffer[hcsr_devp->buffer_pointer].time_stamp);

	/* Increment the buffer pointer by 1 */
	hcsr_devp->buffer_pointer += 1;
	hcsr_devp->buffer_pointer %= BUFFER_LEN;
	if(hcsr_devp->buffer_count < BUFFER_LEN)
		hcsr_devp->buffer_count++;

	DPRINTK("Buffer Count = %d\n", hcsr_devp->buffer_count);

	/* Release buffer lock */
	up(&(hcsr_devp->buffer_lock));

	/* Clear ongoing measurement flag */
	down(&(hcsr_devp->measurement_flag_lock));
	hcsr_devp->ongoing_measurement_flag = 0;
	up(&(hcsr_devp->measurement_flag_lock));

	up(&(hcsr_devp->device_lock));
}

/*
* Open hcsr driver
*/
int hcsr_driver_open(struct inode *inode, struct file *file)
{
	struct hcsr_dev *hcsr_devp;
	int minor = iminor(inode), iter = 0;

	/* Loop through the per-device structure list and find the one with matching minor number */
	file->private_data = NULL;
	for(iter=0; iter<device_count; iter++)
	{
		if(hcsr_dev_list[iter]->mdev->minor == minor)
		{
			file->private_data = hcsr_dev_list[iter];
			hcsr_devp = hcsr_dev_list[iter];
			break;
		}
	}

	if(file->private_data == NULL)
	{
		EPRINTK("Fatal Error: Device not found\n");
		return -ENXIO;
	}
	/* Easy access to cmos_devp from rest of the entry points */
	DPRINTK("%s is openning \n", hcsr_devp->name);

	return 0;
}

/*
 * Release hcsr driver
 */
int hcsr_driver_release(struct inode *inode, struct file *file)
{
	//struct hcsr_dev *hcsr_devp = file->private_data;

    /* Remove the file->private data pointer to the per-device struct */
    //file->private_data = NULL;

	//DPRINTK("%s is closing\n", hcsr_devp->name);

	return 0;
}

/*
 * Write to hcsr driver
 */
ssize_t hcsr_driver_write(struct file *file, const char *buf,
           size_t count, loff_t *ppos)
{
	struct hcsr_dev *hcsr_devp = file->private_data;
	int ret=0;
	int clear_buf_cmd = 0;
	struct work_data *data;
	int bytes_read;

    DPRINTK("%s is being written to\n", hcsr_devp->name);

	bytes_read = copy_from_user(&clear_buf_cmd, (int*)buf, count);
	if(bytes_read != 0)
	{
		EPRINTK("Some error is performing copy_from_user(Error Code: %d)\n", bytes_read);
		return -EINVAL;
	}

	if(!hcsr_devp->pin_config_flag)
	{
		EPRINTK("Pin Configuration is not done\n");
		return -EFAULT;
	}
	// else if(!hcsr_devp->measurement_config_flag)
	// {
	// 	EPRINTK("Measurement Configuration (m and delta) is not done\n");
	// 	return -EFAULT;
	// }

	/* Checking for ongoing measurement */
	down(&(hcsr_devp->measurement_flag_lock));
	if(hcsr_devp->ongoing_measurement_flag)
	{
		EPRINTK("Ongoing measurement detected! Returning...\n");
		up(&(hcsr_devp->measurement_flag_lock));
		return -EINVAL;
	}
	else /* All set to take the measurement */
	{
		/* Obtain the device lock */
		down(&(hcsr_devp->device_lock));
		
		/* Setting the ongoing measurement flag */
		hcsr_devp->ongoing_measurement_flag = 1;
	}
	up(&(hcsr_devp->measurement_flag_lock));

	/* No ongoing measurement found, proceeding with the measurement */
	if(clear_buf_cmd)
	{
		//TODO: Clear Per-Device Buffer
		DPRINTK("Clearing per-device buffer\n");
		hcsr_devp->buffer_pointer = 0;
		hcsr_devp->buffer_count = 0;
	}

	/* Queue Measurement work in the workqueue */
	data = kmalloc(sizeof(struct work_data), GFP_KERNEL);
	data->hcsr_devp = hcsr_devp;
	INIT_WORK(&data->work, measure);
	queue_work(wq, &data->work);

	return ret;
}
EXPORT_SYMBOL_GPL(hcsr_driver_write);

/*
 * Read to hcsr driver
 */
ssize_t hcsr_driver_read(struct file *file, char *buf,
           size_t count, loff_t *ppos)
{
	struct hcsr_dev *hcsr_devp = file->private_data;
    struct dist_measure dist_data;
	struct work_data *data;
	int bytes_read;
	int irq_end_flag = 0;

	DPRINTK("%s is being read from\n", hcsr_devp->name);

	/* Acquire the buffer lock */
	down(&(hcsr_devp->buffer_lock));
	DPRINTK("Buffer Count = %d\n", hcsr_devp->buffer_count);

	if(hcsr_devp->buffer_count == 0) /* Buffer is empty; So fill or wait-for-it-to-fill */
	{
		DPRINTK("Buffer is empty\n");

		up(&(hcsr_devp->buffer_lock));
		down(&(hcsr_devp->measurement_flag_lock));

		/* Ongoing Measurement, so we wait for it to complete */
		if(hcsr_devp->ongoing_measurement_flag)
		{
			DPRINTK("Ongoing Measurement, so we wait\n");
			up(&(hcsr_devp->measurement_flag_lock));

			/* Wait for irq to finish */
			irq_end_flag = 0;
			while(!irq_end_flag)
			{
				msleep(1);

				down(&(hcsr_devp->measurement_flag_lock));
				if(hcsr_devp->ongoing_measurement_flag == 0)
				{
					irq_end_flag = 1;
				}
				up(&(hcsr_devp->measurement_flag_lock));
			}
		}

		/* No ongoing measurement, so we start a new measurement */
		else
		{
			DPRINTK("Starting a new measurement\n");

			/* Obtain the device lock */
			down(&(hcsr_devp->device_lock));

			/* Setting the ongoing measurement flag */
			hcsr_devp->ongoing_measurement_flag = 1;

			up(&(hcsr_devp->measurement_flag_lock));

			/* Queue Measurement work in the workqueue */
			data = kmalloc(sizeof(struct work_data), GFP_KERNEL);
			data->hcsr_devp = hcsr_devp;
			INIT_WORK(&data->work, measure);
			queue_work(wq, &data->work);

			/* Wait for irq to finish */
			irq_end_flag = 0;
			while(!irq_end_flag)
			{
				msleep(1);

				down(&(hcsr_devp->measurement_flag_lock));
				if(hcsr_devp->ongoing_measurement_flag == 0)
				{
					irq_end_flag = 1;
				}
				up(&(hcsr_devp->measurement_flag_lock));
			}
		}
	}
	else
	{
		DPRINTK("Buffer is not empty\n");
		up(&(hcsr_devp->buffer_lock));
	}
	

	/* Obtain the buffer lock */
	down(&(hcsr_devp->buffer_lock));

	dist_data.distance = hcsr_devp->buffer[(hcsr_devp->buffer_pointer - hcsr_devp->buffer_count) % BUFFER_LEN].distance;
	dist_data.time_stamp = hcsr_devp->buffer[(hcsr_devp->buffer_pointer - hcsr_devp->buffer_count) % BUFFER_LEN].time_stamp;
	hcsr_devp->buffer_count -= 1;

	/* Release the buffer lock */
	up(&(hcsr_devp->buffer_lock));

	/* Copy the distance data to the userspace buffer */
	bytes_read = copy_to_user(buf, &dist_data, sizeof(struct dist_measure));
	if(bytes_read != 0)
	{
		EPRINTK("Unable to send complete data to the userspace!\n");
		return bytes_read;
	}

	return 0;

}
EXPORT_SYMBOL_GPL(hcsr_driver_read);

/*
 * Ioctl to hcsr driver
*/
long hcsr_driver_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
    struct hcsr_dev *hcsr_devp = file->private_data;
	struct ioctl_params params;
	struct pin_val linux_gpio, level_shift, pullup_config, pin_mux1, pin_mux2;
	int ret=0, dev_id;

    DPRINTK("%s is being ioctl'ed\n", hcsr_devp->name);
    
	copy_from_user(&params, (struct ioctl_params*)ioctl_param, sizeof(struct ioctl_params));
	
	down(&(hcsr_devp->device_lock));

	switch(ioctl_num)
	{
		case CONFIG_PINS:
			/* params.ioctl_param1 will be the trigger pin */
			/* params.ioctl_param2 will be the echo pin */

			if(params.ioctl_param2 < 0)
			{
				up(&(hcsr_devp->device_lock));
				return -EINVAL;
			}
			else if(params.ioctl_param1 < 0)
			{
				up(&(hcsr_devp->device_lock));
				return -EINVAL;
			}
			else if(params.ioctl_param1 == params.ioctl_param2)
			{
				up(&(hcsr_devp->device_lock));
				return -EINVAL;
			}

			/* Check whether the pins given are already in use or not */
			down(&device_list_lock);
			for(dev_id = 0; dev_id<device_count; dev_id++)
			{
				if(hcsr_dev_list[dev_id]->trigger_pin == params.ioctl_param1)
					ret = -EBUSY;
				
				if(hcsr_dev_list[dev_id]->echo_pin == params.ioctl_param2)
					ret = -EBUSY;
			}
			up(&device_list_lock);

			if(ret)
			{
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
					pin_mux2 		= (struct pin_val){.pin = 78, .val = 1, .label="NULL"}; // Unable to set GPIO78 to H, IO error encountered
					break;

				case 19: // Not working due to pin_mux2 config problem
					linux_gpio 		= (struct pin_val){.pin = 58, .val = 0, .label="gpio58"};
					level_shift 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pullup_config 	= (struct pin_val){.pin = -1, .val = 0, .label="NULL"};
					pin_mux1 		= (struct pin_val){.pin = 60, .val = 1, .label="gpio60"};
					pin_mux2 		= (struct pin_val){.pin = 79, .val = 1, .label="NULL"}; // Unable to set GPIO79 to H, IO error encountered
					break;

				default:
					EPRINTK("Invalid Trigger Pin Selection\n");
					up(&(hcsr_devp->device_lock));
					return -EINVAL;
			}

			DPRINTK("Configuring Trigger pins\n");
			ret = trigger_configure_pins(hcsr_devp, linux_gpio,	level_shift, pullup_config, pin_mux1, pin_mux2);
			if(ret)
			{
				EPRINTK("trigger_configure_pins() failed (Error Code: %d)\n", ret);
				free_trigger_pins(hcsr_devp);
				up(&(hcsr_devp->device_lock));
				return ret;
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

				default:
					EPRINTK("Invalid Echo Pin Selection\n");
					free_trigger_pins(hcsr_devp);
					up(&(hcsr_devp->device_lock));
					return -EINVAL;
			}

			DPRINTK("Configuring Echo pins\n");
			ret = echo_configure_pins(hcsr_devp, linux_gpio, level_shift, pullup_config, pin_mux1, pin_mux2);
			if(ret)
			{
				EPRINTK("echo_configure_pins() failed (Error Code: %d)\n", ret);
				free_trigger_pins(hcsr_devp);
				up(&(hcsr_devp->device_lock));
				return ret;
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
EXPORT_SYMBOL_GPL(hcsr_driver_ioctl);

/* File operations structure. Defined in linux/fs.h */
static struct file_operations hcsr_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= hcsr_driver_open,        /* Open method */
    .release	= hcsr_driver_release,     /* Release method */
    .write		= hcsr_driver_write,       /* Write method */
    .read		= hcsr_driver_read,        /* Read method */
    .unlocked_ioctl      = hcsr_driver_ioctl,       /* Ioctl method*/
};

/*
 * Driver Initialization
 */
int __init hcsr_driver_init(void)
{
    //struct hcsr_dev *hcsr_devp;
	int ret;
	char dev_name[15];
	int dev_id;

	DPRINTK("%d device(s) will be created\n", device_count);

	/* Device Count Validation */
	if(device_count<1)
		device_count = 1;
	if(device_count>MAX_DEVICES)
		device_count = MAX_DEVICES;

	sema_init(&device_list_lock, 1);

	for(dev_id=0; dev_id<device_count; dev_id++)
	{
		/* Generating device name */
		sprintf(dev_name, "%s%d",DEVICE_NAME, dev_id);

		hcsr_devp = kmalloc(sizeof(struct hcsr_dev), GFP_KERNEL);
		if (!hcsr_devp) {
			EPRINTK("Bad Kmalloc!\n"); return -ENOMEM;
		}

		hcsr_devp->mdev = &(mdev_global[dev_id]);

		strcpy(hcsr_devp->name, dev_name);

		// Add to the hcsr_dev_list for future reference
		hcsr_dev_list[dev_id] =  hcsr_devp;
 
		// Initialize the miscdevice structure
		mdev_global[dev_id].minor  =   MISC_DYNAMIC_MINOR;
        mdev_global[dev_id].name   =   hcsr_devp->name;
        mdev_global[dev_id].fops   =   &hcsr_fops;

		/* Initializing Pin Configurations */
		hcsr_devp->echo_pin = -1;
		hcsr_devp->trigger_pin = -1;
		hcsr_devp->echo_irq = -1;
		hcsr_devp->echo_acquired_pin_list_len = 0;
		hcsr_devp->trigger_acquired_pin_list_len = 0;

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

		// Register the miscdevice
		ret = misc_register(&(mdev_global[dev_id]));
		if (ret) {
			EPRINTK("Bad mdev!\n");
			return ret;
		}

        EPRINTK("%s misc_chr_dev created.\n", dev_name);
	}

	/* Workqueue Initialization */
	wq = create_workqueue("wq_dist_measure");

	return 0;
}
/* Driver Exit */
void __exit hcsr_driver_exit(void)
{
    struct hcsr_dev *hcsr_devp;
	int dev_id;
	
	/* Destroy device */
	for(dev_id = 0; dev_id<device_count; dev_id++)
	{
		hcsr_devp = hcsr_dev_list[dev_id];

		/* Free previously acquired trigger pins */
		free_trigger_pins(hcsr_devp);

		/* Free previously acquired echo pins */
		free_echo_pins(hcsr_devp);

        misc_deregister(hcsr_devp->mdev);
		kfree(hcsr_devp);
	}

	flush_workqueue(wq);
	destroy_workqueue(wq);

	EPRINTK("hcsr driver removed.\n");
}

module_init(hcsr_driver_init);
module_exit(hcsr_driver_exit);
MODULE_LICENSE("GPL v2");
