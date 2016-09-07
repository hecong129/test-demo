#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>


/**
 * @ test write "10000" times hotplug 
 * @{ */
/**  @} */

void main(int argc, char *argv[])
{
	int err=0;
    if(argc != 3)
    {
        printf("please set write path : ./write /mnt/sdcard/xxxx  10000");
    }
	printf("sd hotplug test here\n");
    int fp = open(argv[1], O_RDWR  | O_CREAT);
    char txt[1024 * 4];
    int i;
    for(i = 0; i < 1024*4; i ++)
    {
        txt[i] = 0x01;
    }

    for(i = 0; i < atoi(argv[2]); i ++)
    {
        err = write(fp, txt, 1024*4);
		if(err < 0)
		{
			fprintf(stderr,"err:%d  count:%d \n", err,i);
			return ;
		}
    }
    close(fp);

}
