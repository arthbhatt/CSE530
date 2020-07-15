#define _GNU_SOURCE
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <stdio.h>
#include <pthread.h>

/*
 * Put your syscall number here.
 */
#define insdump 359
#define rmdump  360

typedef int dumpmode_t;

#define SYMBOL(x) SYMBOL##x
#define SYMBOL1 "do_fork"
#define SYMBOL2 "do_fork"
#define SYMBOL3 "do_fork"
#define SYMBOL4 "do_fork"

void *insdump_op(void *arg)
{
	int res;
	char **argv = (char**)arg;
	pthread_t thread1;
	printf("calling insdump()\n");
	res = syscall(insdump, SYMBOL1, atoi(argv[2]));
	printf("System call returned %ld.\n", res);
	//pthread_create(&thread1, NULL, insdump_op, (void*) argv);
}

void *rmdump_op(void *arg)
{
	int res;
	char **argv = (char**)arg;
	printf("calling rmdump()\n");
	res = syscall(rmdump, (unsigned int)atoi(argv[2]));
	printf("System call returned %ld.\n", res);

}

int main(int argc, char **argv)
{
	long res;
	int choice;
	pthread_t thread1;
	printf("%s\n", SYMBOL(1));
	if(argc < 2)
	{
		printf("Very few arguments\n");
	}

	choice = atoi(argv[1]);

	//fork();


	if(choice == 1)
	{
		//pthread_create(&thread1, NULL, insdump_op, (void*) argv);
		printf("calling insdump()\n");
		res = syscall(insdump, SYMBOL1, atoi(argv[2]));
	}
	else
	{
		//pthread_create(&thread1, NULL, rmdump_op, (void*) argv);
		printf("calling rmdump()\n");
		res = syscall(rmdump, (unsigned int)atoi(argv[2]));
	}
	printf("System call returned %ld.\n", res);

	return res;
}
