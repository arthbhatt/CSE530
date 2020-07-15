/* ----------------------------------------------- Module dump_stack ---------------------------------------------
CSE530: Embedded Operating System Internals
Assignment 4

Driver made to create and manage dump_stack Kprobes.
Author: Arth Bhatt(ASU ID: 1215361875)
Reference: Kprobe example code given on canvas
 ----------------------------------------------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kprobes.h>
#include <linux/sched.h>
#include <linux/export.h>
#include <linux/syscalls.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/semaphore.h>
#include <linux/init.h>

typedef int dumpmode_t;

/*
TODOs:
-> Remove kprobes when the owner is killed or exited
*/

/*
TEST:
-> Dumpmodes code, 0, 1, any other value
-> Prevent kprobe duplications
*/

/* Actual Code Insert Start */
#define SYMBOL_NAME "do_fork"

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


struct kprobe_list_node{
	unsigned int dumpid;
	pid_t owner_pid;
	pid_t parent_pid;
	dumpmode_t mode;
	struct kprobe kp;

	struct kprobe_list_node *next;
};
static struct kprobe_list_node *kprobe_list_head;

static struct semaphore list_lock;

struct kprobe *kp;

/*For each probe you need to allocate a kprobe structure*/
//static struct kprobe *kp_ptr, kp_static;

/*kprobe pre_handler: called just before the probed instruction is executed*/
int handler_pre(struct kprobe *p, struct pt_regs *regs)
{
	struct kprobe_list_node *node;
	
	//DPRINTK("pre_handler called\n");
	
	node = container_of(p, struct kprobe_list_node, kp);
	
	if(node->mode == 0)
	{
		if(node->owner_pid != current->pid)
			return 0;
	}
	else if(node->mode == 1)
	{
		if(current->parent == NULL)
			return 0;

		if(node->parent_pid != current->parent->pid)
			return 0;
	}
	
	dump_stack();
	return 0;
}

/*kprobe post_handler: called after the probed instruction is executed*/
void handler_post(struct kprobe *p, struct pt_regs *regs, unsigned long flags)
{
	//DPRINTK("post_handler called\n");
}

/* fault_handler: this is called if an exception is generated for any
 * instruction within the pre- or post-handler, or when Kprobes
 * single-steps the probed instruction.
 */
int handler_fault(struct kprobe *p, struct pt_regs *regs, int trapnr)
{
	DPRINTK("fault_handler: p->addr=0x%p, trap #%dn",
		p->addr, trapnr);
	/* Return 0 because we don't handle the fault. */
	return 0;
}

int kprobe_insert(struct kprobe_list_node *node, char *symbolname)
{
	int ret;
	//node->kp = kzalloc(sizeof(struct kprobe), GFP_KERNEL);
	(node->kp).pre_handler = handler_pre;
	(node->kp).post_handler = handler_post;
	(node->kp).fault_handler = handler_fault;
	(node->kp).addr = (kprobe_opcode_t*) kallsyms_lookup_name(symbolname);
	//kp.symbol_name = SYMBOL_NAME;

	ret = register_kprobe(&(node->kp));
	if (ret < 0) {
		EPRINTK("register_kprobe failed, returned %d\n", ret);
		return ret;
	}

	//node->kp = kp;
	DPRINTK("kprobe registered\n");
	return 0;
}

void kprobe_remove(struct kprobe *kp)
{
	unregister_kprobe(kp);
}

/* Actual Code Insert End */


SYSCALL_DEFINE2(insdump, const char __user *, symbolname, dumpmode_t, mode)
{
    /* Actual Code Insert Start */
    
	#ifndef CONFIG_DYNAMIC_DUMP_STACK
		return -ENOSYS;
	#endif

    /* Create new node for kprobe_list */
	struct kprobe_list_node *node = kzalloc(sizeof(struct kprobe_list_node), GFP_KERNEL);
	struct kprobe_list_node *iter_node, *prev_node;
	char *ksymbolname;
	int ret;

	ksymbolname=kmalloc(256, GFP_KERNEL);
	strncpy_from_user(ksymbolname, symbolname, 255);
	
	DPRINTK("insdump called from (pid:%d, tgid:%d)\n", current->pid, current->tgid);

	/* Save the owner's pid */
	node->owner_pid = current->pid;

	if(current->parent != NULL)
	{
		DPRINTK("Parent pid: %d\n", current->parent->pid);
		node->parent_pid = current->parent->pid;
	}
	else
	{
		DPRINTK("No parent found for the current process\n");
		node->parent_pid = -1;
	}
	/* Save the dumpmode of kprobe */
	node->mode = mode;

	/* Insert the kprobe */
	ret = kprobe_insert(node, ksymbolname);
	if(ret != 0)
	{
		EPRINTK("kprobe_insert() failed with error code: %d\n", ret);
		return ret;
	}
	/* Insert the created node in kprobe_list */
	/*
		I have tried to implement a variant on linked list for maintaining dumpid(s).
		The difference being that it is not a stack neither a queue.
		
		Insertion:
			->Iterate through the list starting from the head.
			->Check the difference between the dumpid of the current node
				and the previous node.
				if it is not 1:
					then it means that an element has been deleted and its dumpid is free.
					     so, we assign it to the new kprobe element.
				else:
					we have reached the end and all the dumpid(s) are consecutive.
					this means that there are no gaps in the assigned dumpid(s).
					     so, take the last element's dumpid, increment by 1 and assign it to the new element.
			->There is one more case which is possible, that is
				The first element of the list was deleted, making the head point to an element with dumpid not equal to 1
					so, we insert new element at the start of the list and update the list head to point to it.
	*/
	down(&list_lock);
	/* No element present in the list */
	if(kprobe_list_head == NULL)
	{
		kprobe_list_head = node; // Actual insertion
		node->dumpid = 1;
	}
	else
	{
		iter_node = kprobe_list_head;

		/* List Head's dumpid is not 1. So, we insert element at the start of list */
		if(iter_node->dumpid > 1)
		{
			node->dumpid = 1;
			node->next = kprobe_list_head;
			kprobe_list_head = node; // Actual insertion
		}
		else
		{
			prev_node = iter_node;
			iter_node = iter_node->next;

			/* Navigation to the desired point in the list */
			DPRINTK("Insdump Finding empty space: \n");
			while(iter_node)
			{
				DPRINTK("\tdumpid: %d\n", iter_node->dumpid);
				if(iter_node->dumpid != (prev_node->dumpid + 1) )
				{
					break;
				}
				prev_node = iter_node;
				iter_node = iter_node->next;
			}
			
			/* Main insertion */
			node->dumpid = prev_node->dumpid + 1;
			node->next = prev_node->next;
			prev_node->next = node; // Actual insertion
		}
	}	
	up(&list_lock);
	DPRINTK("dumpid assigned = %d\n", node->dumpid);

    /* Actual Code Insert End */
    
    return node->dumpid;
}

SYSCALL_DEFINE1(rmdump, unsigned int, dumpid)
{   
    /* Actual Code Insert Start */

	#ifndef CONFIG_DYNAMIC_DUMP_STACK
		return -ENOSYS;
	#endif

    struct kprobe_list_node *iter_node, *temp_node;

	DPRINTK("rmdump called from (pid:%d, tgid:%d)\n", current->pid, current->tgid);

	down(&list_lock);
	iter_node = kprobe_list_head;

	if(kprobe_list_head == NULL)
	{
		EPRINTK("Kprobe list is empty!\n");
		EPRINTK("Kprobe does not exist for the given dumpid(%d)\n", dumpid);
		up(&list_lock);
		return -EINVAL;
	}

	if(kprobe_list_head->dumpid == dumpid)
	{
		if(kprobe_list_head->owner_pid != current->pid)
		{
			EPRINTK("Some imposter trying to remove dump_stack operatio. Denied!!!\n");
			up(&list_lock);
			return -EPERM;
		}
		
		// Actual removal
		DPRINTK("Removing kprobe with given dumpid(%d)\n", dumpid);
		kprobe_remove(&(kprobe_list_head->kp));
		kprobe_list_head = kprobe_list_head->next;
		kfree(iter_node);
		up(&list_lock);
		return 0;
	}
	else
	{
		DPRINTK("Finding kprobe with given dumpid:\n");
		while(iter_node->next)
		{
			DPRINTK("\tdumpid: %d\n", iter_node->next->dumpid);
			if(iter_node->next->dumpid == dumpid) /* Found our kprobe */
			{
				if(iter_node->next->owner_pid != current->pid)
				{
					EPRINTK("Some imposter trying to remove dump_stack operatio. Denied!!!\n");
					up(&list_lock);
					return -EPERM;
				}

				/* Updating the linked list */
				temp_node = iter_node->next;
				iter_node->next = iter_node->next->next;
				
				/* Actual removal */
				DPRINTK("Removing kprobe with given dumpid(%d)\n", dumpid);
				kprobe_remove(&(temp_node->kp));
				kfree(temp_node);
				up(&list_lock);
				return 0;
			}
			iter_node = iter_node->next;
		}
	}
	up(&list_lock);
	EPRINTK("Kprobe does not exist for the given dumpid(%d)\n", dumpid);
	return -EINVAL;

    /* Actual Code Insert End */
}



void clndump(pid_t pid)
{
	#ifndef CONFIG_DYNAMIC_DUMP_STACK
		return;
	#endif

	struct kprobe_list_node *iter_node, *temp_node;

	if(kprobe_list_head != NULL)
	{
		//DPRINTK("clndump called from (pid:%d, tgid:%d)\n", current->pid, current->tgid);

		down(&list_lock);
		//DPRINTK("clndump start for pid:%d\n", pid);
		iter_node = kprobe_list_head;

		while(kprobe_list_head->owner_pid == pid)
		{	
			// Actual removal
			DPRINTK("Removing kprobe_head with pid(%d)\n", pid);
			//DPRINTK("node_pid: %d\n", kprobe_list_head->owner_pid);
			kprobe_remove(&(kprobe_list_head->kp));
			kprobe_list_head = kprobe_list_head->next;
			kfree(iter_node);
			iter_node = kprobe_list_head;
			if(kprobe_list_head == NULL)
			{
				up(&list_lock);
				return;
			}
		}
		
		iter_node = kprobe_list_head;
		{
			//DPRINTK("Finding kprobe with given dumpid:\n");
			while(iter_node->next != NULL)
			{
				//DPRINTK("\tdumpid: %d\n", iter_node->next->dumpid);
				if(iter_node->next->owner_pid == pid) /* Found our kprobe */
				{
					/* Updating the linked list */
					temp_node = iter_node->next;
					iter_node->next = iter_node->next->next;
					
					/* Actual removal */
					DPRINTK("Removing kprobe with pid(%d)\n", pid);
					kprobe_remove(&(temp_node->kp));
					//DPRINTK("kprobe removed\n");
					kfree(temp_node);
				}
				else
				{
					iter_node = iter_node->next;
				}
				if(iter_node->next == NULL)
				{
					up(&list_lock);
					return;
				}
				//DPRINTK("IterNode pid = %d\n", iter_node->owner_pid);
			}
		}
		//DPRINTK("clndump over for pid:%d\n", pid);
		up(&list_lock);
	
		return;

		/* Actual Code Insert End */

	}
}
EXPORT_SYMBOL_GPL(clndump);

static int __init init_dump(void)
{
	#ifndef CONFIG_DYNAMIC_DUMP_STACK
		return 0;
	#endif

	sema_init(&list_lock, 1);	
	return 0;
}
late_initcall(init_dump);