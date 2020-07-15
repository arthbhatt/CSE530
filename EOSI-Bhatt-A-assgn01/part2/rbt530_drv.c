/* ----------------------------------------------- DRIVER rbt530 --------------------------------------------------
CSE530: Embedded Operating System Internals
Assignment 1

Driver made to create and manage Red-Black Trees.
Author: Arth Bhatt(ASU ID: 1215361875)
 ----------------------------------------------------------------------------------------------------------------*/

//TODO: Add device name in every DPRINTK
//TODO: Remove all tree elements before exiting
//TODO: Remove static user_name which is not being used

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

#define DRIVER_NAME					"rbt530_drv"
#define DEVICE_NAME                 "rbt530_dev"  // device name to be created and registered
#define MAX_DEVICES					10

/* IOCTL Macros */
#define READ_ORDER                  1

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

/* RB-Tree Object structure */
typedef struct rb_object{
	int key;  // Used to arrange the nodes as an rb_tree
	int data; // Arbitrary data
} rb_object_t;

/* Tree-Node Object structure */
typedef struct tree_node{
	struct rb_node rb; // rb_node object. This needs to be embedded in our structure to manage this rb_tree using kernel implementation

	int key;  // Used to arrange the nodes as an rb_tree
	int data; // Arbitrary data
} tree_node_t;

/* per device structure */
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
} *rbt530_devp_global, *rbt530_dev_list[MAX_DEVICES];

static dev_t rbt530_dev_number;      /* Allotted device number */
struct class *rbt530_dev_class;          /* Tie with the device model */
static struct device *rbt530_dev_device;

static char *user_name = "Dear Arth";  /* the default user name, can be replaced if a new name is attached in insmod command */
static int device_count = 2;
module_param(user_name,charp,0000);	/* to get parameter from load.sh script to greet the user */
module_param(device_count, int, 0000);

/*
 * Send trace data to kprobe
 */
void kprobe_send(struct file *file, rb_object_t *seek_rb_node, int size)
{
    int a,b;
    for(a=0; a<10; a++)
    {
        b = a;
        printk(" ");
    }

    DPRINTK("kprobe_send() called\n");
    return;
}
/*
* Open rbt530 driver
*/
int rbt530_driver_open(struct inode *inode, struct file *file)
{
	struct rbt530_dev *rbt530_devp;

	/* Get the per-device structure that contains this cdev */
	rbt530_devp = container_of(inode->i_cdev, struct rbt530_dev, cdev);

	/* Easy access to cmos_devp from rest of the entry points */
	file->private_data = rbt530_devp;
	DPRINTK("%s is openning \n", rbt530_devp->name);

	return 0;
}

/*
 * Release rbt530 driver
 */
int rbt530_driver_release(struct inode *inode, struct file *file)
{
	struct rbt530_dev *rbt530_devp = file->private_data;
	DPRINTK("%s is closing\n", rbt530_devp->name);

	return 0;
}

/*
 * Write to rbt530 driver
 */
ssize_t rbt530_driver_write(struct file *file, const char *buf,
           size_t count, loff_t *ppos)
{
	struct rbt530_dev *rbt530_devp = file->private_data;
    struct rb_root *root = &(rbt530_devp->tree_root);
	struct rb_node **new, *parent = NULL;
    u32 key;
    rb_object_t *new_rb_node;
    tree_node_t *new_tree_node;
	int ret;

    DPRINTK("%s is being written to\n", rbt530_devp->name);

	/* First access the structure object rb_object_t */
    new_rb_node = kmalloc(sizeof(rb_object_t), GFP_KERNEL);
    ret = copy_from_user(new_rb_node, buf, count);
    if(ret != 0) /* Not all data was copied from user space to kernel space */
    {
        EPRINTK("Unable to get complete data from user space\n");
    	return ret;
    }

	DPRINTK("New Node Info: Key = %d; Data = %d\n", new_rb_node->key, new_rb_node->data);

    /* Copying data from rb_object_t to tree_node_t object */
    new_tree_node = kmalloc(sizeof(tree_node_t), GFP_KERNEL);
    new_tree_node->key = new_rb_node->key;
    new_tree_node->data = new_rb_node->data;
    kfree(new_rb_node);

    down(&(rbt530_devp->device_lock)); /* Start of critical region */

    /* Write Cases */
	// If data != 0 :
	//		If key does not exist in tree:	Create node
	//		Else: Modify that value
	// Else: Delete that value (What if the key doesn't exist)

    new = &root->rb_node;
	key = new_tree_node->key;

	while (*new)
	{
		parent = *new;
        if (key == rb_entry(parent, tree_node_t, rb)->key)
        {
            if (new_tree_node->data == 0) // If data field is 0, then we delete the node at key
            {
                if(rbt530_devp->next_node == parent)
                {
                    /* If traversal_order=0 then move back */
                    if(rbt530_devp->traversal_order == 0)
                    {
                        if(rb_prev(rbt530_devp->next_node) != NULL)
                        {
                            rbt530_devp->next_node = rb_prev(rbt530_devp->next_node);
                        }
                        else /* Case when the first element is being deleted and next_node points to it */
                        {
                            rbt530_devp->next_node = NULL;
                            rbt530_devp->left = 1;
                            rbt530_devp->right = 0;
                        }
                    }
                    /* If traversal_order=1 then move fwd */
                    else if(rbt530_devp->traversal_order == 1)
                    {
                        if(rb_next(rbt530_devp->next_node) != NULL)
                        {
                            rbt530_devp->next_node = rb_next(rbt530_devp->next_node);
                        }
                        else /* Case when the last element is being deleted and next_node points to it */
                        {
                            rbt530_devp->next_node = NULL;
                            rbt530_devp->left = 0;
                            rbt530_devp->right = 1;
                        }
                    }
                }

                //TODO: Use parent instead of &rb_entry(parent, tree_node_t, rb)
                rb_erase(&rb_entry(parent, tree_node_t, rb)->rb, root);
                kfree(rb_entry(parent, tree_node_t, rb));
                DPRINTK("Node removed at key=%d\n", key);
            }
            else // Modify the data field of the node at given key
            {
                rb_replace_node(&rb_entry(parent, tree_node_t, rb)->rb, &new_tree_node->rb, root);
                DPRINTK("Node replaced at key=%d\n", key);
            }
            up(&(rbt530_devp->device_lock)); /* End of critical region */
            return 0;
        }
        /* Moving through RB-Tree */
		else if (key < rb_entry(parent, tree_node_t, rb)->key)
			new = &parent->rb_left;
		else
			new = &parent->rb_right;
	}

    if(new_tree_node->data != 0) // If the data field if non-zero, then insert the given data at given key
    {
        rb_link_node(&new_tree_node->rb, parent, new);
        rb_insert_color(&new_tree_node->rb, root);
        DPRINTK("New Node created at key=%d\n", key);
    }

    up(&(rbt530_devp->device_lock)); /* End of critical region */

	return 0;
}
EXPORT_SYMBOL_GPL(rbt530_driver_write);

/*
 * Read to rbt530 driver
 */
ssize_t rbt530_driver_read(struct file *file, char *buf,
           size_t count, loff_t *ppos)
{
	struct rbt530_dev *rbt530_devp = file->private_data;
    int bytes_read;
    rb_object_t *read_rb_node;

    DPRINTK("%s is being read from\n", rbt530_devp->name);

    down(&(rbt530_devp->device_lock)); /* Start of critical region */

    /* Retrieving the next_node according to traversal_order */
    if(rbt530_devp->traversal_order == 0) /* Ascending Traversal */
    {
        if(rbt530_devp->left == 1)
        {
            rbt530_devp->next_node = rb_first(&(rbt530_devp->tree_root));
            if(rbt530_devp->next_node != NULL)
                rbt530_devp->left = 0;
            else
            {
                up(&(rbt530_devp->device_lock)); /* End of critical region */
                return -EINVAL;
            }
        }
        else
        {
            if(rb_next(rbt530_devp->next_node) == NULL)
            {
                rbt530_devp->right = 1;
                up(&(rbt530_devp->device_lock)); /* End of critical region */
                return -EINVAL;
            }
            else
            {
                rbt530_devp->next_node = rb_next(rbt530_devp->next_node);
            }
        }
    }
    else if(rbt530_devp->traversal_order == 1) /* Descending Traversal */
    {
        if(rbt530_devp->right == 1)
        {
            rbt530_devp->next_node = rb_last(&(rbt530_devp->tree_root));
            if(rbt530_devp->next_node != NULL)
                rbt530_devp->right = 0;
            else
            {
                up(&(rbt530_devp->device_lock)); /* End of critical region */
                return -EINVAL;
            }
        }
        else
        {
            if(rb_prev(rbt530_devp->next_node) == NULL)
            {
                rbt530_devp->left = 1;
                up(&(rbt530_devp->device_lock)); /* End of critical region */
                return -EINVAL;
            }
            else
            {
                rbt530_devp->next_node = rb_prev(rbt530_devp->next_node);
            }
        }
    }
    else
    {
        EPRINTK("Invalid Traversal Order set!\n");
        up(&(rbt530_devp->device_lock)); /* End of critical region */
        return -EINVAL;
    }

    /* If the node ptr is pointing to NULL, then return an error value */
    if(rbt530_devp->next_node == NULL)
    {
        EPRINTK("Next Node pointing to NULL!");
        up(&(rbt530_devp->device_lock)); /* End of critical region */
        return -EINVAL;
    }
    /* Copy the data and key values to rb_object_t */
    read_rb_node = kmalloc(sizeof(rb_object_t), GFP_KERNEL);
    read_rb_node->key = rb_entry(rbt530_devp->next_node, tree_node_t, rb)->key;
    read_rb_node->data = rb_entry(rbt530_devp->next_node, tree_node_t, rb)->data;

    kprobe_send(file, read_rb_node, sizeof(read_rb_node));

    DPRINTK("next_node->key = %d\n", read_rb_node->key);
    DPRINTK("next_node->data = %d\n", read_rb_node->data);

    up(&(rbt530_devp->device_lock)); /* End of critical region */

    /* Copying the rb_object_t to user space buffer */
    bytes_read = copy_to_user(buf, read_rb_node, sizeof(rb_object_t));
    kfree(read_rb_node);
    if(bytes_read != 0)
    {
        EPRINTK("Unable to send complete data to the user space!\n");
        return bytes_read;
    }

	return 0;

}
EXPORT_SYMBOL_GPL(rbt530_driver_read);

/*
 * Ioctl to rbt530 driver
*/
long rbt530_driver_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
    struct rbt530_dev *rbt530_devp = file->private_data;

    DPRINTK("%s is being ioctl'ed\n", rbt530_devp->name);

    down(&(rbt530_devp->device_lock)); /* Start of critical region */
    switch(ioctl_num)
    {
        case READ_ORDER:
            if(ioctl_param == 0)    /* Set traversal order as ascending */
            {
                rbt530_devp->traversal_order = 0;
                DPRINTK("Traversal Order is set to 0(Ascending)\n");
            }
            else if(ioctl_param == 1)    /* Set traversal order as descending */
            {
                rbt530_devp->traversal_order = 1;
                DPRINTK("Traversal Order is set to 1(Descending)\n");
            }
            else
            {
                EPRINTK("Invalid Value of (READ_ORDER)ioctl_param (neither 1 nor 0)\n");
                up(&(rbt530_devp->device_lock)); /* End of critical region */
                return -EINVAL;
            }
            break;
        default:
            DPRINTK("Invalid ioctl_num passed\n");
            break;
    }

    DPRINTK("ioctl_num = %d\n", ioctl_num);
    DPRINTK("ioctl_param = %lu\n", ioctl_param);
    up(&(rbt530_devp->device_lock)); /* End of critical region */
    return 0;
}

/* File operations structure. Defined in linux/fs.h */
static struct file_operations rbt530_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= rbt530_driver_open,        /* Open method */
    .release	= rbt530_driver_release,     /* Release method */
    .write		= rbt530_driver_write,       /* Write method */
    .read		= rbt530_driver_read,        /* Read method */
    .unlocked_ioctl      = rbt530_driver_ioctl,       /* Ioctl method*/
};

/*
 * Driver Initialization
 */
int __init rbt530_driver_init(void)
{
	int ret;
	char dev_name[15];
	int dev_id;

	DPRINTK("%d device(s) will be created\n", device_count);

	/* Request dynamic allocation of a device major number */
	if (alloc_chrdev_region(&rbt530_dev_number, 0, device_count, DEVICE_NAME) < 0) {
			EPRINTK(KERN_DEBUG "Can't register device!\n"); return -1;
	}

	/* Populate sysfs entries */
	rbt530_dev_class = class_create(THIS_MODULE, DEVICE_NAME);

	/* Device Count Validation */
	if(device_count<1)
		device_count = 1;
	if(device_count>MAX_DEVICES)
		device_count = MAX_DEVICES;

	for(dev_id=1; dev_id<=device_count; dev_id++)
	{
		/* Generating device name */
		sprintf(dev_name, "%s%d",DEVICE_NAME, dev_id);
		EPRINTK("%s chr_dev created.\n", dev_name);

		rbt530_devp_global = kmalloc(sizeof(struct rbt530_dev), GFP_KERNEL);

		if (!rbt530_devp_global) {
			EPRINTK("Bad Kmalloc!\n"); return -ENOMEM;
		}
		rbt530_dev_list[dev_id-1] =  rbt530_devp_global;

		/* Request I/O region */
		sprintf(rbt530_devp_global->name, dev_name);

		/* Connect the file operations with the cdev */
		cdev_init(&rbt530_devp_global->cdev, &rbt530_fops);
		rbt530_devp_global->cdev.owner = THIS_MODULE;

		/* Connect the major/minor number to the cdev */
		ret = cdev_add(&rbt530_devp_global->cdev, MKDEV(MAJOR(rbt530_dev_number), dev_id-1), device_count);

		if (ret) {
			EPRINTK("Bad cdev!\n");
			return ret;
		}

		/* Send uevents to udev, so it'll create /dev nodes */
		rbt530_dev_device = device_create(rbt530_dev_class, NULL, MKDEV(MAJOR(rbt530_dev_number), dev_id-1), NULL, dev_name);

		/* Initializing Tree */
		rbt530_devp_global->tree_root = RB_ROOT;
        rbt530_devp_global->traversal_order = 0;
        rbt530_devp_global->left = 1;
        rbt530_devp_global->right = 0;

        /* Initializing device lock */
        sema_init(&(rbt530_devp_global->device_lock), 1);
		DPRINTK("Tree created (corresponding to %s)\n", rbt530_devp_global->name);
	}
	return 0;
}
/* Driver Exit */
void __exit rbt530_driver_exit(void)
{
	int dev_id;

    /* Freeing up the space taken by tree nodes */
    // TODO: Delete every node and set the tree_node to RB_ROOT(NULL)

	// device_remove_file(rbt530_dev_device, &dev_attr_xxx);
	/* Release the major number */
	unregister_chrdev_region((rbt530_dev_number), device_count);

	/* Destroy device */
	for(dev_id = 1; dev_id<=device_count; dev_id++)
	{
		rbt530_devp_global = rbt530_dev_list[dev_id-1];
		device_destroy (rbt530_dev_class, MKDEV(MAJOR(rbt530_dev_number), dev_id-1));
		cdev_del(&rbt530_devp_global->cdev);
		kfree(rbt530_devp_global);
	}

	/* Destroy driver_class */
	class_destroy(rbt530_dev_class);

	EPRINTK("rbt530 driver removed.\n");
}

module_init(rbt530_driver_init);
module_exit(rbt530_driver_exit);
MODULE_LICENSE("GPL v2");
