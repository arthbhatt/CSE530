#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netlink/msg.h>
#include <netlink/attr.h> 

#include "kmodule.h"

#ifndef GRADING
#define MAX7219_CS_PIN 10
#define HCSR04_TRIGGER_PIN 0
#define HCSR04_ECHO_PIN 1
#endif

// TODO: Name the groups as TO_KERNEL and FROM_KERNEL
// TODO: Define macros for PINs and add main.c as APP in Makefile

static char led_pattern1[] = {  0xFF,   // 1 1 1 1 1 1 1 1
                                0x81,   // 1 0 0 0 0 0 0 1
                                0xBD,   // 1 0 1 1 1 1 0 1
                                0xA5,   // 1 0 1 0 0 1 0 1
                                0xA5,   // 1 0 1 0 0 1 0 1
                                0xBD,   // 1 0 1 1 1 1 0 1
                                0x81,   // 1 0 0 0 0 0 0 1 
                                0xFF};  // 1 1 1 1 1 1 1 1

static char led_pattern2[] = {  0xFF,   // 0 0 0 0 0 0 0 0
                                0xFF,   // 0 1 1 1 1 1 1 0
                                0xFF,   // 0 1 0 0 0 0 1 0
                                0xFF,   // 0 1 0 1 1 0 1 0
                                0xFF,   // 0 1 0 1 1 0 1 0
                                0xFF,   // 0 1 0 0 0 0 1 0
                                0xFF,   // 0 1 1 1 1 1 1 0 
                                0xFF};  // 0 0 0 0 0 0 0 0


static unsigned int mcgroups=1;		/* Mask of groups */
static pthread_mutex_t distance_lock;
uint8_t distance_measure;

static void prep_nl_sock(struct nl_sock** nlsock)
{
	int family_id, grp_id;
	unsigned int bit = 0;
	
	*nlsock = nl_socket_alloc();
	if(!*nlsock) {
		printf("Unable to alloc nl socket!\n");
		exit(EXIT_FAILURE);
	}

	/* disable seq checks on multicast sockets */
	nl_socket_disable_seq_check(*nlsock);
	nl_socket_disable_auto_ack(*nlsock);

	/* connect to genl */
	if (genl_connect(*nlsock)) {
		printf("Unable to connect to genl!\n");
		goto exit_err;
	}

	/* resolve the generic nl family id*/
	family_id = genl_ctrl_resolve(*nlsock, GENL_KMODULE_FAMILY_NAME);
	if(family_id < 0){
		printf("Unable to resolve family name!\n");
		goto exit_err;
	}

	if (!mcgroups)
		return;

	while (bit < sizeof(unsigned int)) {
		if (!(mcgroups & (1 << bit)))
			goto next;

		grp_id = genl_ctrl_resolve_grp(*nlsock, GENL_KMODULE_FAMILY_NAME,
				genl_kmodule_mcgrp_names[bit]);
        printf("Adding group\n");
		if (grp_id < 0)	{
			printf("Unable to resolve group name for %u!\n",
				(1 << bit));
            goto exit_err;
		}
		if (nl_socket_add_membership(*nlsock, grp_id)) {
			printf("Unable to join group %u!\n", 
				(1 << bit));
            goto exit_err;
		}
next:
		bit++;
	}

    return;

exit_err:
    nl_socket_free(*nlsock); // this call closes the socket as well
    exit(EXIT_FAILURE);
}

static int send_msg(struct nl_sock *sock, int attr_type, char msg_payload[])
{
    struct nl_msg* msg;
	int family_id, err = 0;

    do
    {
        family_id = genl_ctrl_resolve(sock, GENL_KMODULE_FAMILY_NAME);
    } while (family_id<0);
    
	msg = nlmsg_alloc();
	if (!msg) {
		printf("failed to allocate netlink message\n");
		exit(EXIT_FAILURE);
	}

	if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_id, 0, 
		NLM_F_REQUEST, GENL_KMODULE_C_MSG, 0)) {
		printf("failed to put nl hdr!\n");
		err = -ENOMEM;
		goto out;
	}

	err = nla_put_string(msg, attr_type, msg_payload);
	if (err) {
		printf("failed to put nl string!\n");
		goto out;
	}

	err = nl_send_auto(sock, msg);
	if (err < 0) {
		printf("failed to send nl message!\n");
	}

out:
	nlmsg_free(msg);
	return err;
}

static int skip_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}

static int print_rx_msg(struct nl_msg *msg, void* arg)
{
	struct nlattr *attr[GENL_KMODULE_ATTR_MAX+1];
    char *msg_data;

	genlmsg_parse(nlmsg_hdr(msg), 0, attr, 
			GENL_KMODULE_ATTR_MAX, genl_kmodule_policy);

	if (!attr[GENL_KMODULE_ATTR_HCSR_READING]) {
		printf("Kernel sent empty message!!\n");
		return NL_OK;
	}

    msg_data = nla_get_string(attr[GENL_KMODULE_ATTR_HCSR_READING]);

    pthread_mutex_lock(&distance_lock);
    distance_measure = (uint8_t)msg_data[0];
    printf("Distance measure = %u\n", distance_measure);
    pthread_mutex_unlock(&distance_lock);

	return NL_OK;
}


void led_ctrl(void)
{
    struct nl_sock* nlsock2 = NULL;
    int ret;

    /* Prepare the Netlink socket */
    prep_nl_sock(&nlsock2);

    while(1)
    {    
        printf("Sending led pattern1\n");
        ret = send_msg(nlsock2, GENL_KMODULE_ATTR_LED_PATTERN, led_pattern1);
        if(ret<0)
        {
            printf("Error occurred while sending msg for led pattern\n");
            return;
        }

        pthread_mutex_lock(&distance_lock);
        usleep( (20+2*distance_measure) *1000);
        pthread_mutex_unlock(&distance_lock);

        printf("Sending led pattern2\n");
        ret = send_msg(nlsock2, GENL_KMODULE_ATTR_LED_PATTERN, led_pattern2);
        if(ret<0)
        {
            printf("Error occurred while sending msg for led pattern\n");
            return;
        }

        pthread_mutex_lock(&distance_lock);
        usleep(distance_measure*1000);
        pthread_mutex_unlock(&distance_lock);
    }
}

int main()
{
    struct nl_sock* nlsock = NULL;
	struct nl_cb *cb = NULL;
    int ret;
    pthread_t led_ctrl_id;

    pthread_mutex_init(&distance_lock, NULL);

    char pin_cfg[GENL_KMODULE_ATTR_PIN_CFG_MAX];
    char trigger[GENL_KMODULE_ATTR_HCSR_TRIGGER_MAX];

    // TODO: Add validity check for pins

    pin_cfg[0] = MAX7219_CS_PIN; // cs_pin
    pin_cfg[1] = HCSR04_ECHO_PIN; // echo_pin
    pin_cfg[2] = HCSR04_TRIGGER_PIN; // trigger_pin
    /* Done parsing the arguments */

    trigger[0] = 1;

    /* Prepare the Netlink socket */
    prep_nl_sock(&nlsock);

    /* Prepare callback functions to be called when msg is recvd */
	cb = nl_cb_alloc(NL_CB_DEFAULT);
	nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, skip_seq_check, NULL);
	nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, print_rx_msg, NULL);

    /* Configure Pins */
    printf("Configuring pins\n");
    ret = send_msg(nlsock, GENL_KMODULE_ATTR_PIN_CFG, pin_cfg);
    if(ret<0)
    {
        return -1;
    }

    pthread_create(&led_ctrl_id, NULL, (void *)led_ctrl, NULL);

    /* Main loop */
    while(1)
    {
        printf("Sending trigger message\n");

        ret = send_msg(nlsock, GENL_KMODULE_ATTR_HCSR_TRIGGER, trigger);
        if(ret<0)
        {
            return -1;
        }

        ret = nl_recvmsgs(nlsock, cb);
        if(ret != 0)
        {
            printf("Some error occured while receiving msg\n");
            return -1;
        }
    }

    nl_cb_put(cb);
    nl_socket_free(nlsock);
    return 0;
}