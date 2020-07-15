/* ----------------------------------------------- DRIVER hcsr --------------------------------------------------
CSE530: Embedded Operating System Internals
Assignment 2

Driver made to create and manage HCSR-04 Devices.
Author: Arth Bhatt(ASU ID: 1215361875)
 ----------------------------------------------------------------------------------------------------------------*/


#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include<linux/errno.h>
#include<linux/semaphore.h>
#include<linux/string.h>
#include<linux/gpio.h>
#include<linux/delay.h>
#include<linux/interrupt.h>
#include<linux/workqueue.h>


#ifndef __SAMPLE_PLATFORM_H__


  
#define __SAMPLE_PLATFORM_H__

#define CLASS_NAME		"HCSR"
#define DRIVER_NAME		"HCSR_of_driver"
#define DEVICE_NAME		"HCSR_"

#define MAX_DEVICES		20
#define DEV_NAME_LEN	20
#define BUFFER_LEN		5

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



/* Distance Measurement Structure */
struct dist_measure
{
	unsigned long long time_stamp; // In us
	unsigned int distance; // In cm
};

/* per device structure */
struct hcsr_dev {
	struct platform_device 	plf_dev;	   /* The Platform device structure*/
	struct miscdevice *mdev;               /* The miscdevice structure */
	//struct device *sysfs_dev;

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

#endif /* __GPIO_FUNC_H__ */
