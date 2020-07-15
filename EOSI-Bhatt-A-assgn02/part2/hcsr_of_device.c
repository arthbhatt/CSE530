/* ----------------------------------------------- DRIVER hcsr --------------------------------------------------
CSE530: Embedded Operating System Internals
Assignment 2

Driver made to create and manage HCSR-04 Devices.
Author: Arth Bhatt(ASU ID: 1215361875)
 ----------------------------------------------------------------------------------------------------------------*/


/*
 * A sample program to show the binding of platform driver and device.
 */

//TODO: Kernel bug occurs with invalid opcode when we try to remove this module

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "hcsr_of_device.h"

static int device_count = 2;
module_param(device_count, int, 0000);

static struct platform_device *plf_dev_global[MAX_DEVICES];

static void P_device_release(struct device *dev)
{
	printk("platform device released\n");
}

/**
 * register the device when module is initiated
 */

static int p_device_init(void)
{
	struct hcsr_dev *hcsr_devp;
	int ret = 0;
	char dev_name[15];
	int dev_id;

	DPRINTK("%d device(s) will be created\n", device_count);

	/* Device Count Validation */
	if(device_count<1)
		device_count = 1;
	if(device_count>MAX_DEVICES)
		device_count = MAX_DEVICES;

	for(dev_id=0; dev_id<device_count; dev_id++)
	{
		/* Generating device name */
		sprintf(dev_name, "%s%d",DEVICE_NAME, dev_id);

		hcsr_devp = kmalloc(sizeof(struct hcsr_dev), GFP_KERNEL);
		if (!hcsr_devp) {
			EPRINTK("Bad Kmalloc!\n"); return -ENOMEM;
		}

		strcpy(hcsr_devp->name, dev_name);

		/* Attach an element of static platform_device list to per-device struct */
		//hcsr_devp->plf_dev = &(plf_dev_global[dev_id]);
		plf_dev_global[dev_id] = &hcsr_devp->plf_dev;

		hcsr_devp->plf_dev.name = dev_name;
		hcsr_devp->plf_dev.id = -1;
		hcsr_devp->plf_dev.dev = (struct device){.release = P_device_release};

		DPRINTK("Now registering platform device %s\n", hcsr_devp->name);
		/* Register platform device */
		platform_device_register(&hcsr_devp->plf_dev);

		EPRINTK("Platform device %s is registered in init \n", hcsr_devp->name);
	}
	return ret;
}

static void p_device_exit(void)
{
	int dev_id;

	for(dev_id=0; dev_id<device_count; dev_id++)
	{
		DPRINTK("Removing platform device\n");

		/* Unregister platform device */
		platform_device_unregister(plf_dev_global[dev_id]);
	}
}

module_init(p_device_init);
module_exit(p_device_exit);
MODULE_LICENSE("GPL");
