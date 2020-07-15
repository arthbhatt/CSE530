// TODO: Write the test cases for runnig this test program in gtest and include that in the assignment submission files

/*   A test program for /dev/hcsr_dev
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

/* IOCTL Macros */
#define CONFIG_PINS                  1
#define SET_PARAMETERS				 2

struct ioctl_params {
         unsigned int ioctl_param1;
         unsigned int ioctl_param2;
 };

 /* Distance Measurement Structure */
struct dist_measure
{
	unsigned long long time_stamp; // In us
	unsigned int distance; // In cm
};

int main(int argc, char **argv)
{
	int fd, res;
	int clear_buf_cmd;
	unsigned int ioctl_num;
	struct ioctl_params params;
	struct dist_measure dist_data;

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
	else
	{
		if(strcmp("write", argv[2]) == 0)
		{
			printf("Writing to device\n");
			if(argc == 4)
			{
				clear_buf_cmd = atoi(argv[3]);
				res = write(fd, &clear_buf_cmd, sizeof(clear_buf_cmd));
				if(res != 0)
				{
					printf("Error occurred during write(Error Code: %d)\n", res);
					return 1;
				}
			}
			else
			{
				printf("Wrong number of arguments.\n");
				return 1;
			}
		}

		else if(strcmp("read", argv[2]) == 0)
		{
			printf("Reading from device\n");

			res = read(fd, &dist_data, sizeof(struct dist_measure));
			if(res != 0)
			{
				printf("Error occured during read(Error Code: %d)\n", res);
				return 1;
			}

			printf("Distance: %d cm; Time_Stamp: %llu us\n", dist_data.distance, dist_data.time_stamp);
		}

		else if(strcmp("ioctl", argv[2]) == 0)
		{
			if(argc == 6)
			{
				ioctl_num = atoi(argv[3]);
				params.ioctl_param1 = atoi(argv[4]);
				params.ioctl_param2 = atoi(argv[5]);
			}
			else
			{
				printf("Wrong number of arguments.\n");
				return 1;
			}

			printf("ioctl_num = %d\n", ioctl_num);
			printf("ioctl_param1 = %d\n", params.ioctl_param1);
			printf("ioctl_param2 = %d\n", params.ioctl_param2);

			res = ioctl(fd, ioctl_num, &params);
			if(res != 0)
			{
				printf("Could not read from the device file.\n");
				printf("Error Code: %d\n", errno);
				return 1;
			}
		}

		//scanf("%d", &i);

		/* close devices */
		close(fd);
	}
	return 0;
}
