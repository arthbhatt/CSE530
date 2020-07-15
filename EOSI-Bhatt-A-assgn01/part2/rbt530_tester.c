/*   A test program for /dev/rbt530_dev
		To run the program, enter "kbuf_tester show" to show the current contend of the buffer.
				enter "kbuf_tester write <input_string> to append the <input_string> into the buffer

*/

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*#include <sys/time.h>
#include <linux/ioctl.h>
#include <linux/rtc.h>*/
#include <time.h>
#include <errno.h>

#define RB_OBJ_LIST_LEN             10


/* kprobe command structure */
typedef struct kprobe_cmd{
    int probe_select; // 0: Probe Read; 1: Probe write
	int register_flag;  // 1: Register kprobe; 0: Unregister kprobe
	int offset; // offset of the desired instruction from the function symbol
} kprobe_cmd_t;

/* RB-Tree Object structure */
typedef struct rb_object{
	int key;  // Used to arrange the nodes as an rb_tree
	int data; // Arbitrary data
} rb_object_t;

/* Trace data structure */
typedef struct trace_data{
    unsigned char addr;
    int pid;
    unsigned long long time_stamp; // x86 TSC
    rb_object_t traversed_obj_list[RB_OBJ_LIST_LEN];
    int traversed_obj_count;
} trace_data_t;

int main(int argc, char **argv)
{
	int fd, res;
	rb_object_t package;
	kprobe_cmd_t kprobe_cmd_obj;
	int i = 0;
	unsigned int ioctl_num;
	unsigned long ioctl_param;
	trace_data_t read_tr_data;
	

	if(argc == 1)
	{
		return 1;
	}

	/* open devices */
	printf("Opening device: %s\n", argv[1]);
	fd = open(argv[1], O_RDWR);
	if (fd < 0 )
	{
		printf("Can not open device file.\n");
		return 0;
	}
	else if(strcmp(argv[1], "/dev/rbprobe_dev") == 0)
	{
		printf("Writing kprobe\n");
		if(strcmp("write", argv[2]) == 0)
		{
			memset(&kprobe_cmd_obj, 0, sizeof(kprobe_cmd_obj));
			printf("argc = %d\n",argc);
			if(argc == 6)
			{
				kprobe_cmd_obj.probe_select = atoi(argv[3]);
				kprobe_cmd_obj.register_flag = atoi(argv[4]);
				kprobe_cmd_obj.offset = atoi(argv[5]);
			}
			else
			{
				printf("Wrong number of arguments.\n");
				return 1;
			}

			printf("Probe Select = %d\n", kprobe_cmd_obj.probe_select);
			printf("Register Flag = %d\n", kprobe_cmd_obj.register_flag);
			printf("Offset = %d\n", kprobe_cmd_obj.offset);

			res = write(fd, &kprobe_cmd_obj, sizeof(kprobe_cmd_obj));
			if(res != 0)
			{
				printf("Can not write to the device file.\n");
				return 1;
			}
		}
		else if(strcmp("read", argv[2]) == 0)
		{
			memset(&read_tr_data, 0, sizeof(trace_data_t));

			res = read(fd, &read_tr_data, sizeof(trace_data_t));
			if(res != 0)
			{
				printf("Could not read from the device file.\n");
				printf("Error Code: %d\n", errno);
				printf("EINVAL: %d\n", EINVAL);
				return 1;
			}
			printf("Addr = %u\n", read_tr_data.addr);
			printf("pid = %d\n", read_tr_data.pid);
			printf("time-stamp = %llu\n", read_tr_data.time_stamp);
			printf("traverse-count = %d\n", read_tr_data.traversed_obj_count);
		}

	}
	else
	{
		if(strcmp("write", argv[2]) == 0)
		{
			memset(&package, 0, sizeof(package));
			if(argc == 5)
			{
				package.key = atoi(argv[3]);
				package.data = atoi(argv[4]);
			}
			else
			{
				printf("Wrong number of arguments.\n");
				return 1;
			}

			printf("Key = %d\n", package.key);
			printf("Data = %d\n", package.data);
			//printf("'%s'\n", buff);
			res = write(fd, &package, sizeof(package));
			if(res != 0)
			{
				printf("Can not write to the device file.\n");
				return 1;
			}
		}

		else if(strcmp("read", argv[2]) == 0)
		{
			memset(&package, 0, sizeof(package));

			res = read(fd, &package, sizeof(package));
			if(res != 0)
			{
				printf("Could not read from the device file.\n");
				printf("Error Code: %d\n", errno);
				printf("EINVAL: %d\n", EINVAL);
				return 1;
			}
			printf("Key = %d\n", package.key);
			printf("Data = %d\n", package.data);
		}

		else if(strcmp("ioctl", argv[2]) == 0)
		{
			if(argc == 5)
			{
				ioctl_num = atoi(argv[3]);
				ioctl_param = atoi(argv[4]);
			}
			else
			{
				printf("Wrong number of arguments.\n");
				return 1;
			}

			printf("ioctl_num = %d\n", ioctl_num);
			printf("ioctl_param = %d\n", ioctl_param);

			res = ioctl(fd, ioctl_num, ioctl_param);
			if(res != 0)
			{
				printf("Could not read from the device file.\n");
				printf("Error Code: %d\n", errno);
				printf("EINVAL: %d\n", EINVAL);
				return 1;
			}
		}

		//scanf("%d", &i);

		/* close devices */
		close(fd);
	}
	return 0;
}
