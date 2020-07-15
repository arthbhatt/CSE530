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
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

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
	kprobe_cmd_t kprobe_cmd_obj;
	int i = 0;
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

            if(read_tr_data.traversed_obj_count > 10)
            {
                printf("traversed_obj_list overflowed (traversed_obj_count is greater than 10)\n");
                read_tr_data.traversed_obj_count = 10;
            }
            
            for(i=0; i<read_tr_data.traversed_obj_count; i++)
            {
                printf("Traversed Node 1: K = %d; d = %d\n", read_tr_data.traversed_obj_list[i].key, read_tr_data.traversed_obj_list[i].data);
            }
            
		}

	}

    close(fd);
	return 0;
}
