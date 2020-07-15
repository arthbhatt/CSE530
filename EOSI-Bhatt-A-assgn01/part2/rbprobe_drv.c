/* ----------------------------------------------- DRIVER rbprobe --------------------------------------------------
CSE530: Embedded Operating System Internals
Assignment 1

Driver made to register and unregister kprobes pointing at read and write functions in rbt530 driver.
Author: Arth Bhatt(ASU ID: 1215361875)
 ----------------------------------------------------------------------------------------------------------------*/

//TODO: Traverse count is not correct incase of read_handler
//TODO: Add locks mainly in read, write and handler functions

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

#include<linux/rbtree.h>
#include<linux/errno.h>
#include<linux/semaphore.h>
#include<linux/kprobes.h>

#define DRIVER_NAME					"rbprobe_drv"
#define DEVICE_NAME                 "rbprobe_dev"  // device name to be created and registered

#define RBT_WRITE_SYM_NAME          "rbt530_driver_write"
#define RBT_READ_SYM_NAME           "kprobe_send"//"rbt530_driver_read" //

#define RB_OBJ_LIST_LEN             10

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


/* Tree-Node Object structure */
typedef struct tree_node{
	struct rb_node rb; // rb_node object. This needs to be embedded in our structure to manage this rb_tree using kernel implementation

	int key;  // Used to arrange the nodes as an rb_tree
	int data; // Arbitrary data
} tree_node_t;

/* RB-Tree Object structure */
typedef struct rb_object{
	int key;  // Used to arrange the nodes as an rb_tree
	int data; // Arbitrary data
} rb_object_t;


/* Trace data structure */
typedef struct trace_data{
    kprobe_opcode_t addr;
    int pid;
    unsigned long long time_stamp; // x86 TSC
    rb_object_t traversed_obj_list[RB_OBJ_LIST_LEN];
    int traversed_obj_count;
} trace_data_t;

/* kprobe command structure */
typedef struct kprobe_cmd{
    int probe_select; // 0: Probe Read; 1: Probe write
	int register_flag;  // 1: Register kprobe; 0: Unregister kprobe
	int offset; // offset of the desired instruction from the function symbol
} kprobe_cmd_t;

/* kprobe_data structure */
typedef struct kprobe_data{
    struct kprobe *probe;
    trace_data_t tr_data;
    struct semaphore probe_lock;
} kprobe_data_t;

/* per device structure */
struct rbprobe_dev {
	struct cdev cdev;               /* The cdev structure */
    char name[20];                  /* Used to store probe driver name */
    
    kprobe_data_t probe_obj;
} *rbprobe_devp_global;

/* rbt530_dev per-device structure (Used while probing) */
struct rbt530_dev {
	struct cdev cdev;               /* The cdev structure */
	char name[20];                  /* Name of device*/
	struct rb_root tree_root;       /* Root of rbt530 */

    /* Following members are used for traversing and reading rbt530 data */
    int left;                       /* Flag denoting whether the read pointer is at the start of in-order */
    int right;                      /* Flag denoting whether the read pointer is at the end of in-order */
    int traversal_order;            /* Order of Traversal: 0->Ascending 1->Descending*/
    struct rb_node *next_node;      /* Ptr to the next node to be read */

    struct semaphore device_lock;   /* Device lock is used to allow only single access to rbt530 at a time */
};

static trace_data_t tr_data_global;
static struct kprobe *probe_write;      /* Probe for write function of rbt530 driver */
static struct kprobe *probe_read;       /* Probe for read function of rbt530 driver*/
static dev_t rbprobe_dev_number;      /* Allotted device number */
struct class *rbprobe_dev_class;          /* Tie with the device model */
static struct device *rbprobe_dev_device;
static struct semaphore global_lock;    /* Only one user can use this driver at a time */

/*
 * Function to traverse rb-tree (Function is not thread safe by itself, the caller has to take care of it)
 */
int rb_traverse(struct rb_root *root, int key, rb_object_t traversed_obj_list[RB_OBJ_LIST_LEN])
{
    int count = 0;
    struct rb_node *node = root->rb_node;
    tree_node_t *node_container;

    //node_container = rb_entry(node, tree_node_t, rb);
    //DPRINTK("Traversing RB-Tree(Key: %d, Root Key: %d)\n", key, node_container->key );

    while(node)
    {
        node_container = rb_entry(node, tree_node_t, rb);

        traversed_obj_list[count%RB_OBJ_LIST_LEN].key = node_container->key;
        traversed_obj_list[count%RB_OBJ_LIST_LEN].data = node_container->data;
        count++;
        
        if(key < node_container->key) // Go Left
        {
            node = node->rb_left;
        }
        else if(key > node_container->key) // Go Right
        {
            node = node->rb_right;
        }
        else if(key == node_container->key)
        {
            DPRINTK("Traverse Count: %d\n", count);
            return count;
        }
    }
    DPRINTK("Traverse Count: %d\n", count);
    return count;
}

/*Handler Functions*/
/*
 * probe_write pre-handler
 */
int write_handler_pre(struct kprobe *p, struct pt_regs *regs)
{
    /* Obtaining write function parameters using regs */
    struct file *file = (struct file *)(regs->ax);
    char *buf = (char *)(regs->dx);
    int count = (int)(regs->cx);

    /* Objects for tree traversal */
    struct rbt530_dev *rbt530_devp = file->private_data;
    rb_object_t *seek_rb_node;

    int ret;

    down(&global_lock);

    DPRINTK("Write pre_handler called\n");

    seek_rb_node = kmalloc(sizeof(rb_object_t), GFP_KERNEL);
    ret = copy_from_user(seek_rb_node, buf, count);
    if(ret != 0) /* Not all data was copied from user space to kernel space */
    {
        EPRINTK("Unable to get complete data from user space\n");
    	up(&global_lock);
        return ret;
    }

    DPRINTK("Rbt Dev name: %s\n", rbt530_devp->name);    

    /* Collecting trace data */
    tr_data_global.addr = *(p->addr); // Getting the address of the kprobe
    tr_data_global.pid = current->pid; // Getting the pid
    tr_data_global.time_stamp = native_read_tsc(); // Getting time-stamp (x86 TSC)
    tr_data_global.traversed_obj_count = rb_traverse(&(rbt530_devp->tree_root), seek_rb_node->key, tr_data_global.traversed_obj_list);

    DPRINTK("addr = %u\n", tr_data_global.addr);
    DPRINTK("pid = %d\n", tr_data_global.pid);
    DPRINTK("time = %llu\n", tr_data_global.time_stamp);
    DPRINTK("traverse-count = %d\n", tr_data_global.traversed_obj_count);

    //dump_stack();
    up(&global_lock);
    return 0;
}

/*
 * probe_write post-handler
 */
void write_handler_post(struct kprobe *p, struct pt_regs *regs, unsigned long flags)
{
    down(&global_lock);
    DPRINTK("Write post_handler called\n");
    up(&global_lock);
    return;
}

/*
 * probe_write fault-handler
 */
int write_handler_fault(struct kprobe *p, struct pt_regs *regs, int trapnr)
{
    down(&global_lock);
    DPRINTK("Write fault_handler called\n");
    up(&global_lock);
    return 0;
}

/*
 * probe_read pre-handler
 */
int read_handler_pre(struct kprobe *p, struct pt_regs *regs)
{
    /* Obtaining write function parameters using regs */
    struct file *file = (struct file *)(regs->ax);
    rb_object_t *seek_rb_node = (rb_object_t *)(regs->dx);

    /* Objects for tree traversal */
    struct rbt530_dev *rbt530_devp = file->private_data;

    down(&global_lock);

    DPRINTK("Read pre_handler called\n");

    DPRINTK("Rbt Dev name: %s\n", rbt530_devp->name);    

    /* Collecting trace data */
    tr_data_global.addr = *(p->addr); // Getting the address of the kprobe
    tr_data_global.pid = current->pid; // Getting the pid
    tr_data_global.time_stamp = native_read_tsc(); // Getting time-stamp (x86 TSC)
    tr_data_global.traversed_obj_count = rb_traverse(&(rbt530_devp->tree_root), seek_rb_node->key, tr_data_global.traversed_obj_list);

    DPRINTK("addr = %u\n", tr_data_global.addr);
    DPRINTK("pid = %d\n", tr_data_global.pid);
    DPRINTK("time = %llu\n", tr_data_global.time_stamp);
    DPRINTK("traverse-count = %d\n", tr_data_global.traversed_obj_count);

    //dump_stack();
    up(&global_lock);
    return 0;
}

/*
 * probe_read post-handler
 */
void read_handler_post(struct kprobe *p, struct pt_regs *regs, unsigned long flags)
{
    down(&global_lock);
    DPRINTK("Read post_handler called\n");
    up(&global_lock);
    return;
}

/*
 * probe_read fault-handler
 */
int read_handler_fault(struct kprobe *p, struct pt_regs *regs, int trapnr)
{
    down(&global_lock);
    DPRINTK("Read fault_handler called\n");
    up(&global_lock);
    return 0;
}

/* Driver Functions */
/*
 * Open rbprobe driver
 */
int rbprobe_driver_open(struct inode *inode, struct file *file)
{
	struct rbprobe_dev *rbprobe_devp;

    down(&global_lock);

	/* Get the per-device structure that contains this cdev */
	rbprobe_devp = container_of(inode->i_cdev, struct rbprobe_dev, cdev);

	/* Easy access to cmos_devp from rest of the entry points */
	file->private_data = rbprobe_devp;
	DPRINTK("%s is openning \n", rbprobe_devp->name);

    up(&global_lock);
	return 0;
}

/*
 * Release rbprobe driver
 */
int rbprobe_driver_release(struct inode *inode, struct file *file)
{
	struct rbprobe_dev *rbprobe_devp = file->private_data;
    
    down(&global_lock);
    
    DPRINTK("%s is closing\n", rbprobe_devp->name);
    
    up(&global_lock);
	return 0;
}

/*
 * Write to rbprobe driver
 */
ssize_t rbprobe_driver_write(struct file *file, const char *buf,
           size_t count, loff_t *ppos)
{
	struct rbprobe_dev *rbprobe_devp = file->private_data;
    kprobe_cmd_t *kprobe_cmd_obj;
    int ret;

    down(&global_lock);

    DPRINTK("%s is being written to\n", rbprobe_devp->name);
    DPRINTK("READ_SYMBOL_ADDR = %lu\n", kallsyms_lookup_name(RBT_READ_SYM_NAME));

	/* First access the structure object kprobe_cmd_obj given by the user */
    kprobe_cmd_obj = kmalloc(sizeof(kprobe_cmd_t), GFP_KERNEL);
    ret = copy_from_user(kprobe_cmd_obj, buf, count);
    if(ret != 0) /* Not all data was copied from user space to kernel space */
    {
        EPRINTK("Unable to get complete data from user space\n");
    	up(&global_lock);
        return ret;
    }

	DPRINTK("New Node Info: probe_select = %d; register_flag = %d; Offset = %d\n",  kprobe_cmd_obj->probe_select, 
                                                                                    kprobe_cmd_obj->register_flag, 
                                                                                    kprobe_cmd_obj->offset);

    switch (kprobe_cmd_obj->probe_select)
    {
    case 0: // Register probe for rbt530_drv read function
        if(kprobe_cmd_obj->register_flag == 1)
        {
            DPRINTK("Registering read_probe\n");

            probe_read = kmalloc(sizeof(struct kprobe), GFP_KERNEL);

            /* Configuring read kprobe */
            probe_read->pre_handler = read_handler_pre;
            probe_read->post_handler = read_handler_post;
            probe_read->fault_handler = read_handler_fault;
            probe_read->symbol_name = RBT_READ_SYM_NAME;
            probe_read->offset = kprobe_cmd_obj->offset;

            /* Register probe_read */
            ret = register_kprobe(probe_read);
            if(ret != 0)
            {
                EPRINTK("Unable to register kprobe probe_read!\n");
                DPRINTK("Error code: %d\n", ret);
                up(&global_lock);
                return ret;
            }

            //(rbprobe_devp->probe_obj).probe = probe_read;
            //(rbprobe_devp->probe_obj).tr_data.traversed_obj_count = 0;
        }
        else
        {
            DPRINTK("Unregistering read_probe\n");

            /* Unegister probe_read */
            //probe_read = (rbprobe_devp->probe_obj).probe;
            //(rbprobe_devp->probe_obj).tr_data.traversed_obj_count = 0;
            unregister_kprobe(probe_read);
            kfree(probe_read);
        }
        break;

    case 1: // Register probe for rbt530_drv write function
        if(kprobe_cmd_obj->register_flag == 1)
        {
            DPRINTK("Registering write_probe\n");

            probe_write = kmalloc(sizeof(struct kprobe), GFP_KERNEL);

            /* Configuring read kprobe */
            probe_write->pre_handler = write_handler_pre;
            probe_write->post_handler = write_handler_post;
            probe_write->fault_handler = write_handler_fault;
            probe_write->symbol_name = RBT_WRITE_SYM_NAME;
            probe_write->offset = kprobe_cmd_obj->offset;

            /* Register probe_read */
            ret = register_kprobe(probe_write);
            if(ret != 0)
            {
                EPRINTK("Unable to register kprobe probe_write!\n");
                DPRINTK("Error code: %d\n", ret);
                up(&global_lock);
                return ret;
            }
        }
        else
        {
            DPRINTK("Unregistering write_probe\n");

            /* Unegister probe_read */
            unregister_kprobe(probe_write);
            kfree(probe_write);
        }
        break;

    default:
        EPRINTK("Invalid register_flag passed\n");
        break;
    }
    up(&global_lock);
	return 0;
}

/*
 * Read to rbprobe driver
 */
ssize_t rbprobe_driver_read(struct file *file, char *buf,
           size_t count, loff_t *ppos)
{
    /* Get the per-device structure */
	struct rbprobe_dev *rbprobe_devp = file->private_data;
    int bytes_read;

    down(&global_lock);

    DPRINTK("%s is being read from\n", rbprobe_devp->name);

    /* Copy the trace data to the user buffer */
    bytes_read = copy_to_user(buf, &tr_data_global, sizeof(trace_data_t));
    if(bytes_read != 0)
    {
        EPRINTK("Unable to send complete data to the user space!\n");
        up(&global_lock);
        return bytes_read;
    }
    up(&global_lock);
	return 0;

}


/* File operations structure. Defined in linux/fs.h */
static struct file_operations rbprobe_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= rbprobe_driver_open,        /* Open method */
    .release	= rbprobe_driver_release,     /* Release method */
    .write		= rbprobe_driver_write,       /* Write method */
    .read		= rbprobe_driver_read,        /* Read method */
};

/*
 * Driver Initialization
 */
int __init rbprobe_driver_init(void)
{
	int ret;
	char dev_name[15];

	/* Request dynamic allocation of a device major number */
	if (alloc_chrdev_region(&rbprobe_dev_number, 0, 1, DEVICE_NAME) < 0) {
			EPRINTK(KERN_DEBUG "Can't register device!\n"); return -1;
	}

	/* Populate sysfs entries */
	rbprobe_dev_class = class_create(THIS_MODULE, DEVICE_NAME);

    /* Generating device name */
    sprintf(dev_name, "%s",DEVICE_NAME);
    EPRINTK("%s chr_dev created.\n", dev_name);

    /* Allocate memory for the per-device structure object */
    rbprobe_devp_global = kmalloc(sizeof(struct rbprobe_dev), GFP_KERNEL);

    if (!rbprobe_devp_global) {
        EPRINTK("Bad Kmalloc!\n"); return -ENOMEM;
    }

    /* Request I/O region */
    sprintf(rbprobe_devp_global->name, dev_name);

    /* Connect the file operations with the cdev */
    cdev_init(&rbprobe_devp_global->cdev, &rbprobe_fops);
    rbprobe_devp_global->cdev.owner = THIS_MODULE;

    /* Connect the major/minor number to the cdev */
    ret = cdev_add(&rbprobe_devp_global->cdev, MKDEV(MAJOR(rbprobe_dev_number), 0), 1);

    if (ret) {
        EPRINTK("Bad cdev!\n");
        return ret;
    }

    /* Send uevents to udev, so it'll create /dev nodes */
    rbprobe_dev_device = device_create(rbprobe_dev_class, NULL, MKDEV(MAJOR(rbprobe_dev_number), 0), NULL, dev_name);

    sema_init(&global_lock, 1);

    DPRINTK("Tree created (corresponding to %s)\n", rbprobe_devp_global->name);

	return 0;
}

/* Driver Exit */
void __exit rbprobe_driver_exit(void)
{
    /* Freeing up the space taken by tree nodes */
    // TODO: Delete every node and set the tree_node to RB_ROOT(NULL)

	// device_remove_file(rbprobe_dev_device, &dev_attr_xxx);
	/* Release the major number */
	unregister_chrdev_region((rbprobe_dev_number), 1);

	/* Destroy device */

    device_destroy (rbprobe_dev_class, MKDEV(MAJOR(rbprobe_dev_number), 0));
    cdev_del(&rbprobe_devp_global->cdev);
    kfree(rbprobe_devp_global);

	/* Destroy driver_class */
	class_destroy(rbprobe_dev_class);

	EPRINTK("rbprobe driver removed.\n");
}

module_init(rbprobe_driver_init);
module_exit(rbprobe_driver_exit);
MODULE_LICENSE("GPL v2");
