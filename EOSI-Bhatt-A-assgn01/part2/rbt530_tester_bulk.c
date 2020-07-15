/*
 * A test program for /dev/rbt530_dev
*/

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*#include <sys/time.h>
#include <linux/rtc.h>*/
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>  

#define DEVICE_NAME "/dev/rbt530_dev"

#define DEVICE_NUM			2
#define THREADS_PER_DEVICE	2

#define READ_ORDER			1
#define FINAL_WRITE_COUNT	50

/* RB-Tree Object structure */
typedef struct rb_object{
	int key;  // Used to arrange the nodes as an rb_tree
	int data; // Arbitrary data
} rb_object_t;

struct
{
	int dev_id;
	int fd;
	int write_count;
	pthread_mutex_t lock_wc;
} dev_write_lock_list[DEVICE_NUM];

int generateRandom(int lower, int upper)
{
	int num = (rand() % (upper-lower + 1)) + lower;
	return num;
}


void rbt_interact(void *dev_id_void)
{
	int dev_id = *(int *)dev_id_void;
	int write_count = 0;
	rb_object_t package;
	int res;
	int choice;
	int traversal_order;

	while(1)
	{
		usleep(generateRandom(100, 1000));

		pthread_mutex_lock(&(dev_write_lock_list[dev_id].lock_wc));

		write_count = dev_write_lock_list[dev_id].write_count;
		if(dev_write_lock_list[dev_id].write_count < 500)
		{
			dev_write_lock_list[dev_id].write_count++;
		}

		pthread_mutex_unlock(&(dev_write_lock_list[dev_id].lock_wc));

		if(write_count < 50)
		{
			memset(&package, 0, sizeof(package));
			package.key = write_count;
			package.data = generateRandom(1, 100);

			printf("Write[%d]: K:%d D:%d\n", dev_id, package.key, package.data);

			res = write(dev_write_lock_list[dev_id].fd, &package, sizeof(package));
			if(res != 0)
			{
				printf("Can not write to the device file.\n");
				return;
			}
		}
		else if(write_count < 150)
		{
			choice = generateRandom(1,3);
			switch(choice)
			{
				case 1:	/* Write */
					memset(&package, 0, sizeof(package));
					package.key = generateRandom(1, 100);
					package.data = generateRandom(0, 100);

					printf("Write[%d]: K:%d D:%d\n", dev_id, package.key, package.data);

					res = write(dev_write_lock_list[dev_id].fd, &package, sizeof(package));
					if(res != 0)
					{
						printf("Can not write to the device file.\n");
						return;
					}
					break;

				case 2:	/* Read */
					memset(&package, 0, sizeof(package));

					printf("Read[%d]: ", dev_id);

					res = read(dev_write_lock_list[dev_id].fd, &package, sizeof(package));
					if(res != 0)
					{
						printf("Could not read from the device file.\n");
						printf("Error Code: %d\n", errno);
						printf("EINVAL: %d\n", EINVAL);
					}

					printf("K:%d D:%d\n", package.key, package.data);

					break;

				case 3:	/* Change Order (IOCTL) */
					traversal_order = generateRandom(0,1);
					printf("Ioctl[%d]: Traversal_order:%d\n", dev_id, traversal_order);
					res = ioctl(dev_write_lock_list[dev_id].fd, READ_ORDER, traversal_order);
					if(res != 0)
					{
						printf("Could not read from the device file.\n");
						printf("Error Code: %d\n", errno);
						printf("EINVAL: %d\n", EINVAL);
						return;
					}
					break;
				default:
					printf("Don't know why we are here!\n");
			}
		}
		else
		{
			pthread_mutex_lock(&(dev_write_lock_list[dev_id].lock_wc));
			if(dev_write_lock_list[dev_id].write_count < 3000)
			{
				dev_write_lock_list[dev_id].write_count = 3000;

				traversal_order = 1;
				res = ioctl(dev_write_lock_list[dev_id].fd, READ_ORDER, traversal_order);
				if(res != 0)
				{
					printf("Could not read from the device file.\n");
					printf("Error Code: %d\n", errno);
					printf("EINVAL: %d\n", EINVAL);
					return;
				}

				while(!res)
				{
					res = read(dev_write_lock_list[dev_id].fd, &package, sizeof(package));
				}

				traversal_order = 0;
				res = ioctl(dev_write_lock_list[dev_id].fd, READ_ORDER, traversal_order);
				if(res != 0)
				{
					printf("Could not read from the device file.\n");
					printf("Error Code: %d\n", errno);
					printf("EINVAL: %d\n", EINVAL);
					return;
				}

				printf("RB Object Dump:\n");
				while(!res)
				{
					res = read(dev_write_lock_list[dev_id].fd, &package, sizeof(package));
					printf("K:%d D:%d\n", package.key, package.data);
				}


			}
			pthread_mutex_unlock(&(dev_write_lock_list[dev_id].lock_wc));
			return;
		}
	}
	return;
}

int main() /* rbt_tester device_num */
{
	int fd;
	int dev_id, thread_count;
	char device_name[20];
	pthread_t thread_id;

	srand(time(0));

	for(dev_id = 0; dev_id<DEVICE_NUM; dev_id++)
	{
		sprintf(device_name,"%s%d",DEVICE_NAME, dev_id+1);
		printf("Opening %s\n", device_name);

		fd = open(device_name, O_RDWR);
		if (fd < 0 )
		{
			printf("Can not open device file.\n");
			return 1;
		}

		dev_write_lock_list[dev_id].dev_id = dev_id;
		dev_write_lock_list[dev_id].fd = fd;
		dev_write_lock_list[dev_id].write_count = 0;
		pthread_mutex_init(&(dev_write_lock_list[dev_id].lock_wc), NULL);

		for(thread_count=0; thread_count<THREADS_PER_DEVICE; thread_count++)
		{
			pthread_create(&thread_id, NULL, (void *)rbt_interact, &(dev_write_lock_list[dev_id].dev_id)); // TODO: Change the second param of pthread_create to specify the priority of the thread
		}
	}

	while(1); // TODO: Add a join instead of while(1)

	return 0;
}
