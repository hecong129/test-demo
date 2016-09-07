/*===============================================================
*   Copyright @ hecong129@gmail  All rights reserved.
*   
*   文件名称：test_atsha204.c
*   创 建 者：何聪		
*   创建日期：2016年07月04日
*   描    述：
*
================================================================*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

/**
 * @file test_atsha204.c
 * 函数: main 加密芯片驱动测试代码,
 * 应用层应该在特殊操作时触发写操作(写入密码),
 * 在特定条件下 触发read操作(读取密码) ,  "密码未写入时读取返回为非0值"
 * @author 何聪:
 * @version v1.0
 * @date 2016-07-04
 */
int main()
{

	int fd = 0;
	unsigned char buf;
	int n;
	int test;

	fd = open("/dev/atsha204",O_RDWR);
#if 0
	printf("fd = %d \n", fd);
	n = write(fd,&buf,1);
	
	if(n < 0)
	{
		perror("Fail to Write\n");
		return -1;
	}

	n = read(fd,buf,1);
	if(n < 0)
	{
		perror("Fail to Write\n");
		return -1;
	}
#endif	
	while (1)
	{
		static short i=0;
		sleep(5);
	//	sleep(i++);
	//	if(i==10000)
	//		i=0;
		printf("input value i:%d\n",i);
		scanf("%d",&test);
		
		if(1)
		{
			n = read(fd,&buf,1);
			if(n < 0)
			{
				printf("Fail to read\n");
				//return -1;
			}
			else
			{
				printf("n=%d buf= %d " ,n,buf);
			}	
		}
	}
}	
