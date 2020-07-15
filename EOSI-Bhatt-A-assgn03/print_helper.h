#include <linux/string.h>


#define HELPER_NAME "hcsr_helper"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define D
#define E

#if defined(D)
 	#define DPRINTK(fmt, args...) printk("D: %s:%d:%s(): " fmt, __FILENAME__, __LINE__, __func__, ##args)
#else
	#define DPRINTK(fmt, args...)
#endif

#if defined(E)
 	#define EPRINTK(fmt, args...) printk("E: %s:%d:%s(): " fmt, __FILENAME__, __LINE__, __func__, ##args)
#else
	#define EPRINTK(fmt, args...)
#endif