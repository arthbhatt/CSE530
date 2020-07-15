
#ifndef GENL_TEST_H
#define GENL_TEST_H

#include <linux/netlink.h>

#ifndef __KERNEL__
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#endif

// The following two macros were arrived at after seeing a solution on stack overflow for converting byte to binary sequence

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c\n"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? 'o' : '_'), \
  (byte & 0x40 ? 'o' : '_'), \
  (byte & 0x20 ? 'o' : '_'), \
  (byte & 0x10 ? 'o' : '_'), \
  (byte & 0x08 ? 'o' : '_'), \
  (byte & 0x04 ? 'o' : '_'), \
  (byte & 0x02 ? 'o' : '_'), \
  (byte & 0x01 ? 'o' : '_') 

// Stack Overflow code end. This code is not crucial to the assignment and is just use to get a pretty printk output

#define GENL_KMODULE_FAMILY_NAME		    "genl_kmodule"
#define GENL_KMODULE_MCGRP0_NAME		    "genl_mcgrp0"
#define GENL_KMODULE_MCGRP1_NAME		    "genl_mcgrp1"
#define GENL_KMODULE_MCGRP2_NAME		    "genl_mcgrp2"

#define GENL_KMODULE_ATTR_PIN_CFG_MAX	    256//3+1
#define GENL_KMODULE_ATTR_HCSR_TRIGGER_MAX	256//1+1
#define GENL_KMODULE_ATTR_LED_PATTERN_MAX	256//8+1
#define GENL_KMODULE_ATTR_HCSR_READING_MAX	256//1+1

/*
    Represent different msg types using different attributes of type string

    PIN_CFG: [ cs_pin, echo_pin, trigger_pin ]
    HCSR_TRIGGER: [ trigger ]
    LED_PATTERN: [ l1, l2, l3, l4, l5, l6, l7, l8 ]
    HCSR_READING: [ distance ]
*/

enum {
	GENL_KMODULE_C_UNSPEC,		/* Must NOT use element 0 */
	GENL_KMODULE_C_MSG,
};

enum genl_kmodule_multicast_groups {
	GENL_KMODULE_MCGRP0,
	GENL_KMODULE_MCGRP1,
	GENL_KMODULE_MCGRP2,
};
#define GENL_KMODULE_MCGRP_MAX		3

static char* genl_kmodule_mcgrp_names[GENL_KMODULE_MCGRP_MAX] = {
	GENL_KMODULE_MCGRP0_NAME,
	GENL_KMODULE_MCGRP1_NAME,
	GENL_KMODULE_MCGRP2_NAME,
};

enum genl_kmodule_attrs {
	GENL_KMODULE_ATTR_UNSPEC,		/* Must NOT use element 0 */

	GENL_KMODULE_ATTR_PIN_CFG,
    GENL_KMODULE_ATTR_HCSR_TRIGGER,
    GENL_KMODULE_ATTR_LED_PATTERN,
    GENL_KMODULE_ATTR_HCSR_READING,

	__GENL_KMODULE_ATTR__MAX,
};
#define GENL_KMODULE_ATTR_MAX (__GENL_KMODULE_ATTR__MAX - 1)

static struct nla_policy genl_kmodule_policy[GENL_KMODULE_ATTR_MAX+1] = {
	[GENL_KMODULE_ATTR_PIN_CFG] = {
		.type = NLA_STRING,
#ifdef __KERNEL__
		.len = GENL_KMODULE_ATTR_PIN_CFG_MAX
#else
		.maxlen = GENL_KMODULE_ATTR_PIN_CFG_MAX
#endif
	},

    [GENL_KMODULE_ATTR_HCSR_TRIGGER] = {
		.type = NLA_STRING,
#ifdef __KERNEL__
		.len = GENL_KMODULE_ATTR_HCSR_TRIGGER_MAX
#else
		.maxlen = GENL_KMODULE_ATTR_HCSR_TRIGGER_MAX
#endif
	},

    [GENL_KMODULE_ATTR_LED_PATTERN] = {
		.type = NLA_STRING,
#ifdef __KERNEL__
		.len = GENL_KMODULE_ATTR_LED_PATTERN_MAX
#else
		.maxlen = GENL_KMODULE_ATTR_LED_PATTERN_MAX
#endif
	},

    [GENL_KMODULE_ATTR_HCSR_READING] = {
		.type = NLA_STRING,
#ifdef __KERNEL__
		.len = GENL_KMODULE_ATTR_HCSR_READING_MAX
#else
		.maxlen = GENL_KMODULE_ATTR_HCSR_READING_MAX
#endif
	},
};

#endif
