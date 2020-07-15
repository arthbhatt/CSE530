#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/timer.h>
#include <linux/export.h>
#include <net/genetlink.h>
#include<linux/delay.h>
#include<linux/interrupt.h>
#include<linux/workqueue.h>

#include "kmodule.h"
#include "print_helper.h"
#include "hcsr_helper.c"
#include "led_matrix_helper.c"

#define DRIVER_NAME		            "kmodule"

struct led_work_data
{
	struct work_struct work;
	char led_pattern[GENL_KMODULE_ATTR_LED_PATTERN_MAX];
};

static struct genl_family genl_kmodule_family;
static struct workqueue_struct *hcsr_wq;
static struct workqueue_struct *led_wq;

static void send_distance(unsigned int group, uint8_t distance_measure);

static void update_led(struct work_struct *work)
{
    struct led_work_data *data = (struct led_work_data*) work;
    int row_led;
    
    DPRINTK("\tPattern: \n");
    for(row_led=1; row_led<=8; row_led++)
    {
        //DPRINTK("\t\t"BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(data->led_pattern[row_led-1]));
        spi_data_transfer(row_led, data->led_pattern[row_led-1], linux_cs_pin);
    }
}

static void measure(struct work_struct *work)
{
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

    send_distance(GENL_KMODULE_MCGRP0, hcsr_devp->buffer[hcsr_devp->buffer_pointer].distance);
}


static void send_distance(unsigned int group, uint8_t distance_measure)
{   
    void *hdr;
    int res, flags = GFP_ATOMIC;
    char msg[GENL_KMODULE_ATTR_HCSR_READING_MAX];
    struct sk_buff* skb = genlmsg_new(NLMSG_DEFAULT_SIZE, flags);

    DPRINTK("Sending Distance to user\n");

    if (!skb) {
        EPRINTK("OOM!!\n");
        return;
    }

    hdr = genlmsg_put(skb, 0, 0, &genl_kmodule_family, flags, GENL_KMODULE_C_MSG);
    if (!hdr) {
        EPRINTK("Unknown err !\n");
        goto nlmsg_fail;
    }

    msg[0] = distance_measure;

    // TODO: Add distance here
    res = nla_put_string(skb, GENL_KMODULE_ATTR_HCSR_READING, msg);
    if (res) {
        EPRINTK("Error in nla_put_string()\n");
        DPRINTK("Ignore this print: %s\n", genl_kmodule_mcgrp_names[group]);
        goto nlmsg_fail;
    }

    genlmsg_end(skb, hdr);
    genlmsg_multicast(&genl_kmodule_family, skb, 0, group, flags);
    return;

nlmsg_fail:
    genlmsg_cancel(skb, hdr);
    nlmsg_free(skb);
    return;
}


static int genl_kmodule_rx_msg(struct sk_buff* skb, struct genl_info* info)
{
    char *msg_data;
    struct hcsr_work_data *hcsr_data;
    struct led_work_data *led_data;
    struct ioctl_params params;
    int led_row;

    if (info->attrs[GENL_KMODULE_ATTR_PIN_CFG]) 
    {
        msg_data = (char*)nla_data(info->attrs[GENL_KMODULE_ATTR_PIN_CFG]);

        DPRINTK("GENL_KMODULE_ATTR_PIN_CFG from %u: \n", info->snd_portid);
        DPRINTK("\tcs_pin: %u\n", msg_data[0]);
        DPRINTK("\techo_pin: %u\n", msg_data[1]);
        DPRINTK("\ttrigger_pin: %u\n", msg_data[2]);

        /* Configuring pins related to hcsr */
        params.ioctl_param1 = msg_data[2]; // Trigger pin
        params.ioctl_param2 = msg_data[1]; // Echo pin
        _hcsr_driver_ioctl(hcsr_devp, CONFIG_PINS, params);
        blink(hcsr_devp);

        config_cs_pin(msg_data[0]);
        led_matrix_cfg(linux_cs_pin);
    }
    else if (info->attrs[GENL_KMODULE_ATTR_HCSR_TRIGGER]) 
    {
        msg_data = (char*)nla_data(info->attrs[GENL_KMODULE_ATTR_HCSR_TRIGGER]);

        DPRINTK("GENL_KMODULE_ATTR_HCSR_TRIGGER from %u: \n", info->snd_portid);
        DPRINTK("\ttrigger: %u\n", msg_data[0]);
    
        /* Queue Measurement work in hcsr workqueue */
        hcsr_data = kmalloc(sizeof(struct hcsr_work_data), GFP_KERNEL);
        INIT_WORK(&hcsr_data->work,  measure);
        queue_work(hcsr_wq, &hcsr_data->work);
    }
    else if (info->attrs[GENL_KMODULE_ATTR_LED_PATTERN]) 
    {
        msg_data = (char*)nla_data(info->attrs[GENL_KMODULE_ATTR_LED_PATTERN]);

        DPRINTK("GENL_KMODULE_ATTR_LED_PATTERN from %u: \n", info->snd_portid);
        
        /* Queue Measurement work in led workqueue */
        led_data = kmalloc(sizeof(struct led_work_data), GFP_KERNEL);
        INIT_WORK(&led_data->work,  update_led);
        for(led_row=0; led_row<8; led_row++)
        {
            led_data->led_pattern[led_row] = msg_data[led_row];
        }
        queue_work(led_wq, &led_data->work);
    }
    else
    {
        EPRINTK("Empty message from %d!\n", info->snd_portid);
        return -EINVAL;
    }

    return 0;
}

static const struct genl_ops genl_kmodule_ops[] = {
    {
        .cmd = GENL_KMODULE_C_MSG,
        .policy = genl_kmodule_policy,
        .doit = genl_kmodule_rx_msg,
        .dumpit = NULL,
    },
};

static const struct genl_multicast_group genl_kmodule_mcgrps[] = {
    [GENL_KMODULE_MCGRP0] = { .name = GENL_KMODULE_MCGRP0_NAME, },
    [GENL_KMODULE_MCGRP1] = { .name = GENL_KMODULE_MCGRP1_NAME, },
    [GENL_KMODULE_MCGRP2] = { .name = GENL_KMODULE_MCGRP2_NAME, },
};

static struct genl_family genl_kmodule_family = {
    .name = GENL_KMODULE_FAMILY_NAME,
    .version = 1,
    .maxattr = GENL_KMODULE_ATTR_MAX,
    .netnsok = false,
    .module = THIS_MODULE,
    .ops = genl_kmodule_ops, // define
    .n_ops = ARRAY_SIZE(genl_kmodule_ops),
    .mcgrps = genl_kmodule_mcgrps, // define
    .n_mcgrps = ARRAY_SIZE(genl_kmodule_mcgrps),
};

static int __init kmodule_init(void)
{
    struct ioctl_params params;
    int ret;

    EPRINTK("Initializing Kmodule\n");
    
    ret = genl_register_family(&genl_kmodule_family);
    if(ret != 0)
    {
        EPRINTK("Some error occured while registering netlink family\n");
        return -EINVAL;
    }

    /* Workqueue Initialization */
	hcsr_wq = create_workqueue("wq_dist_measure");
    led_wq = create_workqueue("led_pattern_update");

    /* Initialize HCSR */
    hcsr_devp = kmalloc(sizeof(struct hcsr_dev), GFP_KERNEL);
    if(!hcsr_devp)
    {
        EPRINTK("Bad Kmalloc!\n");
        return -ENOMEM;
    }

    hcsr_dev_init(hcsr_devp);

    params.ioctl_param1 = 5;// Number of Samples
    params.ioctl_param2 = 100;// Delay between two samples in ms
    _hcsr_driver_ioctl(hcsr_devp, SET_PARAMETERS, params);

    /* Initialize LED Matrix */
    led_matrix_driver_init();

    return 0;
}

static void kmodule_exit(void)
{
    // TODO: Free configured pins

    DPRINTK("Flushing hcsr_wq\n");
    flush_workqueue(hcsr_wq);
    DPRINTK("Destroying hcsr_wq\n");
	destroy_workqueue(hcsr_wq);

    DPRINTK("Flushing led_wq\n");
    flush_workqueue(led_wq);
	DPRINTK("Destroying led_wq\n");
    destroy_workqueue(led_wq);

    DPRINTK("Freeing Trigger Pins\n");
    free_trigger_pins(hcsr_devp);

    DPRINTK("Freeing Echo Pins\n");
    free_echo_pins(hcsr_devp);

    kfree(hcsr_devp);

    led_matrix_driver_exit();

    DPRINTK("Unregister Genl Family\n");
    genl_unregister_family(&genl_kmodule_family);
    EPRINTK("Exiting Kmodule\n");
}

module_init(kmodule_init);
module_exit(kmodule_exit);
MODULE_AUTHOR("Arth Bhatt <aubhatt@asu.edu>");
MODULE_LICENSE("GPL");