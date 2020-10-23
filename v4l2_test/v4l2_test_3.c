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
	enum v4l2_buf_type type;
}dev;

/* open the device taking arguments as devname,file descriptor,third argument as zero */
static int video_open(struct device *dev,const char *devname, int no_query)
{
	struct v4l2_capability cap;
	/**
	  * struct v4l2_capability - Describes V4L2 device caps returned by VIDIOC_QUERYCAP
	  *
	  * @driver:	   name of the driver module (e.g. "bttv")
	  * @card:	   name of the card (e.g. "Hauppauge WinTV")
	  * @bus_info:	   name of the bus (e.g. "PCI:" + pci_name(pci_dev) )
	  * @version:	   KERNEL_VERSION
	  * @capabilities: capabilities of the physical device as a whole
	  * @device_caps:  capabilities accessed via this particular device (node)
	  * @reserved:	   reserved fields for future extensions
	  */
	int ret;

	dev->fd=open(devname, O_RDWR);
	//printf("Bhagya : video device opened\n");
	if(dev->fd < 0)
	{
		printf("Error opening device %s: %d\n",devname,errno);
		return dev->fd;
	}

	if (!no_query) {
		printf("Bhagya : querying device  capabilities \n");
		ret = ioctl(dev->fd, VIDIOC_QUERYCAP, &cap);
		if (ret < 0) {
			printf("Error opening device %s: unable to query "
				"device.\n", devname);
			close(dev->fd);
			return ret;
		}

		if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
		{
			dev->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			printf("Bhagya: dev type is video capture and line no = %d\n",__LINE__);
		}
		else if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)
		{
			dev->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			printf("Bhagya: dev type is video capture and line no = %d\n",__LINE__);
		}
		else {
			printf("Error opening device %s: neither video capture "
				"nor video output supported.\n", devname);
			close(dev->fd);
			return -EINVAL;
		}

		printf("Device %s opened: %s (%s).\n", devname, cap.card, cap.bus_info);
	} else {
		dev->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		printf("Device %s opened.\n", devname);
	}

	return 0;
}

static int video_set_format(struct device *dev, unsigned int w, unsigned int h, unsigned int format)
{
	printf("Bhagya: setting video format \n");
	struct v4l2_format fmt;
	int ret;

	memset(&fmt, 0, sizeof fmt);
	fmt.type = dev->type;
	fmt.fmt.pix.width = w;
	fmt.fmt.pix.height = h;
	fmt.fmt.pix.pixelformat = format;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;

	ret = ioctl(dev->fd, VIDIOC_S_FMT, &fmt);
	if (ret < 0) {
		printf("Unable to set format: %s (%d).\n", strerror(errno),
			errno);
		return ret;
	}

	printf("Video format set: width: %u height: %u buffer size: %u\n",
		fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
	return 0;
}


static int video_set_framerate(struct device *dev, struct v4l2_fract *time_per_frame)
{
	printf("Bhagya: func_name = %s and line no = %d\n",__func__,__LINE__);
	struct v4l2_streamparm parm;
	int ret;

	memset(&parm, 0, sizeof parm);
	parm.type = dev->type;

	ret = ioctl(dev->fd, VIDIOC_G_PARM, &parm);
	if (ret < 0) {
		printf("Unable to get frame rate: %d.\n", errno);
		return ret;
	}

	printf("Current frame rate: %u/%u\n",
		parm.parm.capture.timeperframe.numerator,
		parm.parm.capture.timeperframe.denominator);

	printf("Setting frame rate to: %u/%u\n",
		time_per_frame->numerator,
		time_per_frame->denominator);

	parm.parm.capture.timeperframe.numerator = time_per_frame->numerator;
	parm.parm.capture.timeperframe.denominator = time_per_frame->denominator;

	ret = ioctl(dev->fd, VIDIOC_S_PARM, &parm);
	if (ret < 0) {
		printf("Unable to set frame rate: %d.\n", errno);
		return ret;
	}

	ret = ioctl(dev->fd, VIDIOC_G_PARM, &parm);
	if (ret < 0) {
		printf("Unable to get frame rate: %d.\n", errno);
		return ret;
	}

	printf("Frame rate set: %u/%u\n",
		parm.parm.capture.timeperframe.numerator,
		parm.parm.capture.timeperframe.denominator);
	return 0;
}



/* closing the video device taking argument as file descriptor of the device*/
static void video_close(struct device *dev)
{
	close(dev->fd);
	printf("Bhagya : video device closed\n");
}


int main()
{
	int ret;
	char *dev_name="/dev/video0";
	unsigned int pixelformat = V4L2_PIX_FMT_YUYV;
	unsigned int width = 640;
	unsigned int height = 480;
	struct v4l2_fract time_per_frame = {1,30};

	/* open the video device */
	printf("Bhagya : opening video device\n");
	video_open(&dev,dev_name,0);

	/* Set the format */	
		ret=video_set_format(&dev, width, height, pixelformat); 
		printf("Bhagya:setting video format\n");
		if(ret < 0){
			printf("Bhagya:closing video device (video_set_format)\n");
			video_close(&dev);
			return 1;
		}

	/* Set the frame rate. */
		ret=video_set_framerate(&dev, &time_per_frame);
		if(ret < 0){
			printf("Bhagya:closing video device (video_set_framerate)\n");
			video_close(&dev);
			return 1;
		}
		

	/* close the video device */
	printf("Bhagya : closing video device \n");
	video_close(&dev);


	return 0;

}

