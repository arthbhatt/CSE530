/* ----------------------------------------------- DRIVER hcsr --------------------------------------------------
CSE530: Embedded Operating System Internals
Assignment 2

Driver made to create and manage HCSR-04 Devices.
Author: Arth Bhatt(ASU ID: 1215361875)
 ----------------------------------------------------------------------------------------------------------------*/


//TODO: Complete the device id table
//TODO: Sometimes, the driver crashes when a device in registered
//TODO: This driver has to be inserted before device is inserted
//TODO: Unable to create symlinks as /sys/class/HCSR_n

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "hcsr_of_device.h"

/* IOCTL Macros */
#define CONFIG_PINS                  1
#define SET_PARAMETERS				 4


struct work_data
{
	struct work_struct work;
	struct hcsr_dev *hcsr_devp;
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

static struct hcsr_dev *hcsr_dev_list[MAX_DEVICES];
static struct miscdevice mdev_global[MAX_DEVICES];
static struct class *hcsr_class;          /* Tie with the device model */
static struct workqueue_struct *wq;
static int number_of_devices = 0;
static struct semaphore device_list_lock;

//static struct class_interface hcsr_interface;


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

	down(&(hcsr_devp->enable_lock));
	
	if(!(hcsr_devp->enable))
		return;

	up(&(hcsr_devp->enable_lock));
	
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
	struct hcsr_dev *hcsr_devp = hcsr_dev_list[0];
	int minor = iminor(inode), iter = 0;

	/* Loop through the per-device structure list and find the one with matching minor number */
	file->private_data = NULL;
	for(iter=0; iter<=number_of_devices; iter++)
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
 * Core Write Function
 */
static int _hcsr_driver_write(struct hcsr_dev *hcsr_devp, int clear_buf_cmd)
{
	struct work_data *data;

	if(!hcsr_devp->pin_config_flag)
	{
		EPRINTK("Pin Configuration is not done\n");
		return -EFAULT;
	}

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
	int bytes_read;

    DPRINTK("%s is being written to\n", hcsr_devp->name);

	bytes_read = copy_from_user(&clear_buf_cmd, (int*)buf, count);
	if(bytes_read != 0)
	{
		EPRINTK("Some error is performing copy_from_user(Error Code: %d)\n", bytes_read);
		return -EINVAL;
	}

	ret = _hcsr_driver_write(hcsr_devp, clear_buf_cmd);

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
 * Core Ioctl Driver
 */
static int _hcsr_driver_ioctl(struct hcsr_dev *hcsr_devp, unsigned int ioctl_num, struct ioctl_params params)
{
	struct pin_val linux_gpio, level_shift, pullup_config, pin_mux1, pin_mux2;
	int ret = 0, dev_id;
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
			
			/* Check whether the pins given are already in use or not */
			down(&device_list_lock);

			for(dev_id = 0; dev_id<number_of_devices; dev_id++)
			{
				if(hcsr_devp == hcsr_dev_list[dev_id])
					continue;

				if(hcsr_dev_list[dev_id]->trigger_pin != -1)
					if(hcsr_dev_list[dev_id]->trigger_pin == params.ioctl_param1)
						ret = -EBUSY;
				
				if(hcsr_dev_list[dev_id]->echo_pin != -1)
					if(hcsr_dev_list[dev_id]->echo_pin == params.ioctl_param2)
						ret = -EBUSY;
			}
			up(&device_list_lock);

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

/*
 * Ioctl to hcsr driver
*/
long hcsr_driver_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
    struct hcsr_dev *hcsr_devp = file->private_data;
	struct ioctl_params params;
	int ret=0;

    DPRINTK("%s is being ioctl'ed\n", hcsr_devp->name);
    
	copy_from_user(&params, (struct ioctl_params*)ioctl_param, sizeof(struct ioctl_params));
	
	ret = (long) _hcsr_driver_ioctl(hcsr_devp, ioctl_num, params);

	return ret;
}
EXPORT_SYMBOL_GPL(hcsr_driver_ioctl);

static ssize_t  trigger_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int ret = 0;

	DPRINTK("trigger_show() called for %s\n", hcsr_devp->name);

	down(&(hcsr_devp->device_lock));
	ret = snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->trigger_pin);
	DPRINTK("trigger_pin = %d\n", hcsr_devp->trigger_pin);
	up(&(hcsr_devp->device_lock));

	if(ret >= PAGE_SIZE)
		EPRINTK("trigger_pin value truncated when copying\n");
	
	return ret;
}

static ssize_t trigger_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	struct ioctl_params params;
	int ret = 0;
	int arg = 0; // enable value

	DPRINTK("trigger_store() called for %s\n", hcsr_devp->name);

	sscanf(buf, "%d", &arg);

	down(&(hcsr_devp->device_lock));
	
	if(hcsr_devp->trigger_pin != arg)
	{
		params.ioctl_param1 = arg;
		params.ioctl_param2 = hcsr_devp->echo_pin;

		up(&(hcsr_devp->device_lock));

		ret = _hcsr_driver_ioctl(hcsr_devp, CONFIG_PINS, params);
		if(ret)
		{
			EPRINTK("Some error occured while measuring distance(Error Code: %d)\n", ret);
		}

		down(&(hcsr_devp->device_lock));
	}

	DPRINTK("trigger_pin = %d\n", hcsr_devp->trigger_pin);
	up(&(hcsr_devp->device_lock));

	return PAGE_SIZE;
}

static ssize_t  echo_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int ret = 0;

	DPRINTK("echo_show() called for %s\n", hcsr_devp->name);

	down(&(hcsr_devp->device_lock));
	ret = snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->echo_pin);
	DPRINTK("echo_pin = %d\n", hcsr_devp->echo_pin);
	up(&(hcsr_devp->device_lock));

	if(ret >= PAGE_SIZE)
		EPRINTK("echo_pin value truncated when copying\n");
	
	return ret;
}

static ssize_t echo_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	struct ioctl_params params;
	int ret = 0;
	int arg = 0; // enable value

	DPRINTK("echo_store() called for %s\n", hcsr_devp->name);

	sscanf(buf, "%d", &arg);

	down(&(hcsr_devp->device_lock));
	
	if(hcsr_devp->echo_pin != arg)
	{
		params.ioctl_param1 = hcsr_devp->trigger_pin;
		params.ioctl_param2 = arg;

		up(&(hcsr_devp->device_lock));

		ret = _hcsr_driver_ioctl(hcsr_devp, CONFIG_PINS, params);
		if(ret)
		{
			EPRINTK("Some error occured while measuring distance(Error Code: %d)\n", ret);
		}

		down(&(hcsr_devp->device_lock));
	}

	DPRINTK("echo_pin = %d\n", hcsr_devp->echo_pin);
	up(&(hcsr_devp->device_lock));

	return PAGE_SIZE;
}

static ssize_t  number_samples_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int ret = 0;

	DPRINTK("Number_samples_show() called for %s\n", hcsr_devp->name);

	down(&(hcsr_devp->device_lock));
	ret = snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->m);
	DPRINTK("m = %d\n", hcsr_devp->m);
	up(&(hcsr_devp->device_lock));

	if(ret >= PAGE_SIZE)
		EPRINTK("Number of samples value truncated when copying\n");
	
	return ret;
}

static ssize_t number_samples_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	struct ioctl_params params;
	int ret = 0;
	int arg = 0; // enable value

	DPRINTK("number_samples_store() called for %s\n", hcsr_devp->name);

	sscanf(buf, "%d", &arg);

	down(&(hcsr_devp->device_lock));
	
	if(hcsr_devp->m != arg)
	{
		params.ioctl_param1 = arg;
		params.ioctl_param2 = hcsr_devp->delta;

		up(&(hcsr_devp->device_lock));

		ret = _hcsr_driver_ioctl(hcsr_devp, SET_PARAMETERS, params);
		if(ret)
		{
			EPRINTK("Some error occured while measuring distance(Error Code: %d)\n", ret);
		}

		down(&(hcsr_devp->device_lock));
	}

	DPRINTK("m = %d\n", hcsr_devp->m);
	up(&(hcsr_devp->device_lock));

	return PAGE_SIZE;
}

static ssize_t  sampling_period_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int ret = 0;

	DPRINTK("sampling_period_show() called for %s\n", hcsr_devp->name);

	down(&(hcsr_devp->device_lock));
	ret = snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->delta);
	DPRINTK("delta = %d\n", hcsr_devp->delta);
	up(&(hcsr_devp->device_lock));

	if(ret >= PAGE_SIZE)
		EPRINTK("delta value truncated when copying\n");
	
	return ret;
}

static ssize_t sampling_period_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	struct ioctl_params params;
	int ret = 0;
	int arg = 0; // enable value

	DPRINTK("sampling_period_store() called for %s\n", hcsr_devp->name);

	sscanf(buf, "%d", &arg);

	down(&(hcsr_devp->device_lock));
	
	if(hcsr_devp->delta != arg)
	{
		params.ioctl_param1 = hcsr_devp->m;
		params.ioctl_param2 = arg;

		up(&(hcsr_devp->device_lock));

		ret = _hcsr_driver_ioctl(hcsr_devp, SET_PARAMETERS, params);
		if(ret)
		{
			EPRINTK("Some error occured while measuring distance(Error Code: %d)\n", ret);
		}

		down(&(hcsr_devp->device_lock));
	}

	DPRINTK("delta = %d\n", hcsr_devp->delta);
	up(&(hcsr_devp->device_lock));

	return PAGE_SIZE;
}

static ssize_t  enable_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int ret = 0;

	DPRINTK("enable_show() called for %s\n", hcsr_devp->name);

	down(&(hcsr_devp->buffer_lock));
	ret = snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->enable);
	DPRINTK("enable = %d\n", hcsr_devp->enable);
	up(&(hcsr_devp->buffer_lock));

	if(ret >= PAGE_SIZE)
		EPRINTK("enable value truncated when copying\n");
	
	return ret;
}

static ssize_t enable_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int ret = 0;
	int arg = 0; // enable value

	DPRINTK("enable_store() called for %s\n", hcsr_devp->name);

	sscanf(buf, "%d", &arg);

	down(&(hcsr_devp->enable_lock));
	hcsr_devp->enable = arg;

	if(hcsr_devp->enable)
	{
		ret = _hcsr_driver_write(hcsr_devp, 0);
		if(ret)
		{
			EPRINTK("Some error occured while measuring distance(Error Code: %d)\n", ret);
		}
	}

	DPRINTK("enable = %d\n", hcsr_devp->enable);
	up(&(hcsr_devp->enable_lock));

	return PAGE_SIZE;
}

static ssize_t  distance_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct hcsr_dev *hcsr_devp = dev_get_drvdata(dev);
	int ret = 0;

	DPRINTK("distance_show() called for %s\n", hcsr_devp->name);

	down(&(hcsr_devp->buffer_lock));
	ret = snprintf(buf, PAGE_SIZE, "%d\n", hcsr_devp->buffer[(hcsr_devp->buffer_pointer - 1)%BUFFER_LEN].distance);
	DPRINTK("distance = %d\n", hcsr_devp->buffer[(hcsr_devp->buffer_pointer - 1)%BUFFER_LEN].distance);
	up(&(hcsr_devp->buffer_lock));

	if(ret >= PAGE_SIZE)
		EPRINTK("distance value truncated when copying\n");
	
	return ret;
}

static const struct platform_device_id P_id_table[] = {
         { "HCSR_0", 0 },
         { "HCSR_1", 0 },
         { "HCSR_2", 0 },
		 { "HCSR_3", 0 },
		 { "HCSR_4", 0 },
		 { "HCSR_5", 0 },
		 { "HCSR_6", 0 },
		 { "HCSR_7", 0 },
		 { "HCSR_8", 0 },
		 { "HCSR_9", 0 },
		 { "HCSR_10", 0 },
		 { "HCSR_11", 0 },
		 { "HCSR_12", 0 },
		 { "HCSR_13", 0 },
		 { "HCSR_14", 0 },
		 { "HCSR_15", 0 },
		 { "HCSR_16", 0 },
		 { "HCSR_17", 0 },
		 { "HCSR_18", 0 },
		 { "HCSR_19", 0 },

	 { },
};

/* File operations structure. Defined in linux/fs.h */
static struct file_operations hcsr_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= hcsr_driver_open,        /* Open method */
    .release	= hcsr_driver_release,     /* Release method */
    .write		= hcsr_driver_write,       /* Write method */
    .read		= hcsr_driver_read,        /* Read method */
    .unlocked_ioctl      = hcsr_driver_ioctl,       /* Ioctl method*/
};

static DEVICE_ATTR(trigger, S_IRUSR | S_IWUSR, trigger_show, trigger_store);
static DEVICE_ATTR(echo, S_IRUSR | S_IWUSR, echo_show, echo_store);
static DEVICE_ATTR(number_samples, S_IRUSR | S_IWUSR, number_samples_show, number_samples_store);
static DEVICE_ATTR(sampling_period, S_IRUSR | S_IWUSR, sampling_period_show, sampling_period_store);
static DEVICE_ATTR(enable, S_IRUSR | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(distance, S_IRUSR , distance_show, NULL);

/*
 *	Core Initialization Function ( for hcsr_dev )
 */

static int hcsr_dev_init(struct hcsr_dev *hcsr_devp)
{
	int ret = 0;

	DPRINTK("hcsr_dev_init() called\n");

	/* Some one-time initialization */
	if(!number_of_devices)
	{
		wq = create_workqueue("wq_dist_measure");
		sema_init(&device_list_lock, 1);
		hcsr_class = class_create(THIS_MODULE, CLASS_NAME);
		if(!hcsr_class)
		{
			EPRINTK("Unable to create device classn\n");
			return -EFAULT;
		}
	}

	/* Add to the hcsr_dev_list for future reference */
	hcsr_dev_list[number_of_devices] =  hcsr_devp;

	/* Attach an element of static miscdevice list to per-device struct */
	hcsr_devp->mdev = &(mdev_global[number_of_devices]);

	/* Initialize the miscdevice structure */
	hcsr_devp->mdev->minor  =   MISC_DYNAMIC_MINOR;
	hcsr_devp->mdev->name   =   hcsr_devp->name;
	hcsr_devp->mdev->fops   =   &hcsr_fops;
		
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

	/* Register the miscdevice */
	ret = misc_register(hcsr_devp->mdev);
	if (ret) {
		EPRINTK("Bad mdev!\n");
		return ret;
	}

	/* Storing hcsr_devp to access it in show and store functions */
	dev_set_drvdata(hcsr_devp->mdev->this_device,(void *) hcsr_devp);

	/* Now we create sysfs attribute files */
	ret = device_create_file(hcsr_devp->mdev->this_device, &dev_attr_trigger);
	if(ret)
	{
		EPRINTK("Cannot create trigger attribute\n");
		return ret;
	}

	ret = device_create_file(hcsr_devp->mdev->this_device, &dev_attr_echo);
	if(ret)
	{
		EPRINTK("Cannot create echo attribute\n");
		return ret;
	}

	ret = device_create_file(hcsr_devp->mdev->this_device, &dev_attr_number_samples);
	if(ret)
	{
		EPRINTK("Cannot create number_samples attribute\n");
		return ret;
	}

	ret = device_create_file(hcsr_devp->mdev->this_device, &dev_attr_sampling_period);
	if(ret)
	{
		EPRINTK("Cannot create sampling_period attribute\n");
		return ret;
	}

	ret = device_create_file(hcsr_devp->mdev->this_device, &dev_attr_enable);
	if(ret)
	{
		EPRINTK("Cannot create enalbe attribute\n");
		return ret;
	}

	ret = device_create_file(hcsr_devp->mdev->this_device, &dev_attr_distance);
	if(ret)
	{
		EPRINTK("Cannot create distance attribute\n");
		return ret;
	}

	return ret;
}

/*
 *	Core Exit Function ( for hcsr_dev )
 */
static int hcsr_dev_exit(struct hcsr_dev *hcsr_devp)
{
	DPRINTK("hcsr_dev exit function called\n");
	
	// /* Free previously acquired trigger pins */
	// free_trigger_pins(hcsr_devp);

	// /* Free previously acquired echo pins */
	// free_echo_pins(hcsr_devp);

	// /* Remove sysfs interface */
	// device_destroy(hcsr_class, MKDEV(10, hcsr_devp->mdev->minor));

	/* Unregister the misc device */
	misc_deregister(hcsr_devp->mdev);

	/* If this is the last device then remove the device class */
	if(!number_of_devices)
	{
		class_unregister(hcsr_class);
		class_destroy(hcsr_class);

		flush_workqueue(wq);
		destroy_workqueue(wq);
	}	
	
	kfree(hcsr_devp);

	return 0;
}

static int P_driver_probe(struct platform_device *dev_found)
{
	struct hcsr_dev *hcsr_devp;
	int ret = 0;

	hcsr_devp = container_of(dev_found, struct hcsr_dev, plf_dev);
	EPRINTK("Probe Function called for device %s\n", hcsr_devp->name);		

	ret = hcsr_dev_init(hcsr_devp);
	if(ret)
	{
		EPRINTK("Error Initialization hcsr_dev(Error Code: %d)\n", ret);
		return ret;
	}

	number_of_devices++;

	return ret;
};

static int P_driver_remove(struct platform_device *pdev)
{
	struct hcsr_dev *hcsr_devp;

	hcsr_devp = container_of(pdev, struct hcsr_dev, plf_dev);
	DPRINTK("%s will be removed\n", hcsr_devp->name);

	hcsr_dev_exit(hcsr_devp);

	return 0;
};

static struct platform_driver P_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= P_driver_probe,
	.remove		= P_driver_remove,
	.id_table	= P_id_table,
};

module_platform_driver(P_driver);
MODULE_LICENSE("GPL");
