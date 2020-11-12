
/* 1.open the video device
   2.close the video device */


#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/time.h>

#include <linux/videodev2.h>
#include "videodev2-fsr172x.h"

#ifndef V4L2_BUF_FLAG_ERROR
#define V4L2_BUF_FLAG_ERROR	0x0040
#endif

#define ARRAY_SIZE(a)	(sizeof(a)/sizeof((a)[0]))

struct device
{
	int fd;
}dev;


static int video_open(struct device *dev,const char *devname, int no_query)
{
	dev->fd=open(devname, O_RDWR);
	printf("Bhagya : video device opened\n");
	if(dev->fd < 0)
	{
		printf("Error opening device %s: %d\n",devname,errno);
		return dev->fd;
	}
	return 0;
}


static void video_close(struct device *dev)
{
	close(dev->fd);
	printf("Bhagya : video device closed\n");
}


int main()
{
		
	char *dev_name="/dev/video0";
	/* open the video device */
	printf("Bhagya : opening video device\n");
	video_open(&dev,dev_name,0);
		

	/* close the video device */
	printf("Bhagya : closing video device \n");
	video_close(&dev);


	return 0;

}


