
/* 1.open the video device
   2.querying the video device capabilities
   3.set the format
   4.set the framerate
   5.allocate  and map buffers
   6.stream on
   7.stream off
   8.free buffers
   9.close the video device */




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

#define V4L_BUFFERS_DEFAULT	6
#define V4L_BUFFERS_MAX	32

/* buffer structure containing data members as size and memory */
struct buffer
{
	unsigned int size;
	void *mem;
};


struct device
{
	int fd;
	enum v4l2_buf_type type;
	enum v4l2_memory memtype;
	unsigned int nbufs;
	struct buffer *buffers;

	unsigned int width;
	unsigned int height;
	unsigned int bytesperline;
	unsigned int imagesize;

	void *pattern;
	
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
		memset(&cap, 0, sizeof cap);
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
			printf("Bhagya: dev type is video output and line no = %d\n",__LINE__);
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

static int video_alloc_buffers(struct device *dev, int nbufs, unsigned int offset)
{
	printf("Bhagya: func_name = %s and line no = %d\n",__func__,__LINE__);
	struct v4l2_requestbuffers rb;
	struct v4l2_buffer buf;
	int page_size;
	struct buffer *buffers;
	unsigned int i;
	int ret;

	memset(&rb, 0, sizeof rb);
	rb.count = nbufs;
	rb.type = dev->type;
	rb.memory = dev->memtype;
	printf("Bhagya: allocate buffers: VIDIOC_REQBUFS \n");
	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		printf("Unable to request buffers: %d.\n", errno);
		return ret;
	}

	printf("%u buffers requested.\n", rb.count);
	printf("Bhagya: no.of buffers requested is %u \n",rb.count);

	buffers = malloc(rb.count * sizeof buffers[0]);
	if (buffers == NULL)
		return -ENOMEM;

	page_size = getpagesize();

	/* Map the buffers. */
	for (i = 0; i < rb.count; ++i) {
		memset(&buf, 0, sizeof buf);
		buf.index = i;
		buf.type = dev->type;
		buf.memory = dev->memtype;
		printf("Bhagya: map buffers: VIDIOC_QUERYBUF \n");
		ret = ioctl(dev->fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			printf("Unable to query buffer %u (%d).\n", i, errno);
			return ret;
		}
		printf("length: %u offset: %u\n", buf.length, buf.m.offset);

		switch (dev->memtype) {
		case V4L2_MEMORY_MMAP:
			printf("Bhagya: memory type is V4L2_MEMORY_MMAP \n");
			buffers[i].mem = mmap(0, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, dev->fd, buf.m.offset);
			if (buffers[i].mem == MAP_FAILED) {
				printf("Unable to map buffer %u (%d)\n", i, errno);
				return ret;
			}
			buffers[i].size = buf.length;
			printf("Buffer %u mapped at address %p.\n", i, buffers[i].mem);
			break;

		case V4L2_MEMORY_USERPTR:
			printf("Bhagya: memory type isV4L2_MEMORY_USERPTR \n");
			ret = posix_memalign(&buffers[i].mem, page_size, buf.length + offset);
			if (ret < 0) {
				printf("Unable to allocate buffer %u (%d)\n", i, ret);
				return -ENOMEM;
			}
			buffers[i].mem += offset;
			buffers[i].size = buf.length;
			printf("Buffer %u allocated at address %p.\n", i, buffers[i].mem);
			break;

		default:
			break;
		}
	}

	dev->buffers = buffers;
	dev->nbufs = rb.count;
	return 0;
}

static int video_queue_buffer(struct device *dev, int index)
{
	printf("Bhagya: func_name = %s and line no = %d\n",__func__,__LINE__);
	struct v4l2_buffer buf;
	int ret;

	memset(&buf, 0, sizeof buf);
	buf.index = index;
	buf.type = dev->type;
	buf.memory = dev->memtype;
	buf.length = dev->buffers[index].size;
	if (dev->memtype == V4L2_MEMORY_USERPTR)
		buf.m.userptr = (unsigned long)dev->buffers[index].mem;

	if (dev->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		printf("Bhagya : dev type: V4L2_BUF_TYPE_VIDEO_OUTPUT \n");
		buf.bytesused = buf.length;
		memcpy(dev->buffers[buf.index].mem, dev->pattern, buf.bytesused);
	} else
		memset(dev->buffers[buf.index].mem, 0x55, buf.length);
	printf("Bhagya: queue buffers: VIDIOC_QBUF \n");
	ret = ioctl(dev->fd, VIDIOC_QBUF, &buf);
	if (ret < 0)
		printf("Unable to queue buffer (%d).\n", errno);

	return ret;
}

static int video_load_test_pattern(struct device *dev, const char *filename)
{
	printf("Bhagya: func_name = %s and line no = %d\n",__func__,__LINE__);
	unsigned int size = dev->buffers[0].size;
	unsigned int x, y;
	uint8_t *data;
	int ret;
	int fd;

	/* Load or generate the test pattern */
	dev->pattern = malloc(size);
	if (dev->pattern == NULL)
		return -ENOMEM;

	if (filename == NULL) {
		if (dev->bytesperline == 0) {
			printf("Compressed format detect and no test pattern filename given.\n"
				"The test pattern can't be generated automatically.\n");
			return -EINVAL;
		}

		data = dev->pattern;

		for (y = 0; y < dev->height; ++y) {
			for (x = 0; x < dev->bytesperline; ++x)
				*data++ = x + y;
		}

		return 0;
	}

	fd = open(filename, O_RDONLY);
	if (fd == -1) {
		printf("Unable to open test pattern file '%s': %s.\n", filename,
			strerror(errno));
		return -errno;
	}

	ret = read(fd, dev->pattern, size);
	close(fd);

	if (ret != (int)size) {
		printf("Test pattern file size %u doesn't match image size %u\n",
			ret, size);
		return -EINVAL;
	}

	return 0;
}

static int video_prepare_capture(struct device *dev, int nbufs, unsigned int offset,
				 const char *filename)
{
	printf("Bhagya: funtion_name : %s line_no : %d\n", __func__,__LINE__);

	unsigned int i;
	int ret;

	/* Allocate and map buffers. */
	printf("Bhagya : Allocate and map buffers.\n");
	if ((ret = video_alloc_buffers(dev, nbufs, offset)) < 0)
		return ret;

	if (dev->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		printf("Bhagya: dev type is video output and line no = %d\n",__LINE__);
		ret = video_load_test_pattern(dev, filename);
		if (ret < 0)
			return ret;
	}

	/* Queue the buffers. */
	printf("Bhagya :Queue the buffers.\n");
	for (i = 0; i < dev->nbufs; ++i) {
		ret = video_queue_buffer(dev, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int video_enable(struct device *dev, int enable)
{
	printf("Bhagya: func_name = %s and line no = %d\n",__func__,__LINE__);
	int type = dev->type;
	int ret;

	ret = ioctl(dev->fd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type);
	if (ret < 0) {
		printf("Unable to %s streaming: %d.\n", enable ? "start" : "stop",
			errno);
		return ret;
	}

	return 0;
}

static int video_free_buffers(struct device *dev)
{
	printf("Bhagya: func_name = %s and line no = %d\n",__func__,__LINE__);
	struct v4l2_requestbuffers rb;
	unsigned int i;
	int ret;

	if (dev->nbufs == 0)
		return 0;

	if (dev->memtype == V4L2_MEMORY_MMAP) {
		for (i = 0; i < dev->nbufs; ++i) {
			ret = munmap(dev->buffers[i].mem, dev->buffers[i].size);
			if (ret < 0) {
				printf("Unable to unmap buffer %u (%d)\n", i, errno);
				return ret;
			}
		}
	}

	memset(&rb, 0, sizeof rb);
	rb.count = 0;
	rb.type = dev->type;
	rb.memory = dev->memtype;
	printf("Bhagya: free buffers: VIDIOC_REQBUFS \n");
	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		printf("Unable to release buffers: %d.\n", errno);
		return ret;
	}

	printf("%u buffers released.\n", dev->nbufs);

	free(dev->buffers);
	dev->nbufs = 0;
	dev->buffers = NULL;

	return 0;
}

static int video_do_capture(struct device *dev, unsigned int nframes,
	unsigned int skip, unsigned int delay, const char *filename_prefix,
	int do_requeue_last)
{
	
	char *filename = NULL;
	struct timeval start = { 0, 0 };
	struct timeval end, ts;
	struct v4l2_buffer buf;
	unsigned int size;
	unsigned int i;
	FILE *file;
	double bps;
	double fps;
	int ret;


	if (filename_prefix != NULL) {
		filename = malloc(strlen(filename_prefix) + 12);
		if (filename == NULL)
			return -ENOMEM;
	}

	/* Start streaming. */
	printf("Bhagya : start streaming \n");
	video_enable(dev, 1);

	size = 0;

	for (i = 0; i < nframes; ++i) {
		/* Dequeue a buffer. */
		printf("Bhagya : Dequeue buffer \n");
		memset(&buf, 0, sizeof buf);
		buf.type = dev->type;
		buf.memory = dev->memtype;
		ret = ioctl(dev->fd, VIDIOC_DQBUF, &buf);
		if (ret < 0) {
			if (errno != EIO) {
				printf("Unable to dequeue buffer (%d).\n", errno);
				goto done;
			}
			buf.type = dev->type;
			buf.memory = dev->memtype;
			if (dev->memtype == V4L2_MEMORY_USERPTR)
				buf.m.userptr = (unsigned long)dev->buffers[i].mem;
		}

		if (dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		    dev->imagesize != 0	&& buf.bytesused != dev->imagesize)
			printf("Warning: bytes used %u != image size %u\n",
			       buf.bytesused, dev->imagesize);

		size += buf.bytesused;

		gettimeofday(&ts, NULL);
		printf("%u (%u) [%c] %u %u bytes %ld.%06ld %ld.%06ld\n", i, buf.index,
			(buf.flags & V4L2_BUF_FLAG_ERROR) ? 'E' : '-',
			buf.sequence, buf.bytesused, buf.timestamp.tv_sec,
			buf.timestamp.tv_usec, ts.tv_sec, ts.tv_usec);

		if (i == 0)
			start = ts;

		printf("Bhagya : save the image\n");
		/* Save the image. */
		if (dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE && filename_prefix && !skip) {
			sprintf(filename, "%s-%06u.bin", filename_prefix, i);
			file = fopen(filename, "wb");
			if (file != NULL) {
				ret = fwrite(dev->buffers[buf.index].mem, buf.bytesused, 1, file);
				fclose(file);
			}
		}
		if (skip)
			--skip;
		
		/* Requeue the buffer. */
		printf("Bhagya : requeue buffer\n");
		if (delay > 0)
			usleep(delay * 1000);

		fflush(stdout);

		if (i == nframes - dev->nbufs && !do_requeue_last)
			continue;

		ret = video_queue_buffer(dev, buf.index);
		if (ret < 0) {
			printf("Unable to requeue buffer (%d).\n", errno);
			goto done;
		}
	}
	gettimeofday(&end, NULL);

	/* Stop streaming. */
	printf("Bhagya: stop streaming \n");
	video_enable(dev, 0);

	if (nframes == 0) {
		printf("No frames captured.\n");
		goto done;
	}

	if (end.tv_sec == start.tv_sec && end.tv_usec == start.tv_usec)
		goto done;

	end.tv_sec -= start.tv_sec;
	end.tv_usec -= start.tv_usec;
	if (end.tv_usec < 0) {
		end.tv_sec--;
		end.tv_usec += 1000000;
	}

	bps = size/(end.tv_usec+1000000.0*end.tv_sec)*1000000.0;
	fps = (i-1)/(end.tv_usec+1000000.0*end.tv_sec)*1000000.0;

	printf("Captured %u frames in %lu.%06lu seconds (%f fps, %f B/s).\n",
		i-1, end.tv_sec, end.tv_usec, fps, bps);

done:
	free(filename);
	printf("Bhagya:free buffers\n");
	return video_free_buffers(dev);
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
	enum v4l2_memory memtype = V4L2_MEMORY_MMAP;
	unsigned int nbufs = V4L_BUFFERS_DEFAULT;
	unsigned int userptr_offset = 0;
	const char *filename = "/dev/shm/capture.output";
	unsigned int delay = 0, nframes = 10;
	unsigned int skip=0;
	int do_requeue_last = 0;

	/* open the video device */
	printf("Bhagya : opening video device\n");
	ret = video_open(&dev,dev_name,0);
	if (ret < 0)
			return 1;

	dev.memtype = memtype;


	/* Set the format */	
		printf("Bhagya:setting video format\n");
		ret=video_set_format(&dev, width, height, pixelformat); 
		
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

	/* allocate and map the buffers */
		printf("Bhagya :calling video_prepare_capture \n");
		ret=video_prepare_capture(&dev, nbufs, userptr_offset, filename);
		if(ret<0){
			printf("Bhagya :closing video device (video_prepare_capture)\n");		
			video_close(&dev);
			return 1;
		}

	/* stream on and stream off */
	printf("shashi:video_do_capture\n");
	ret=video_do_capture(&dev, nframes, skip, delay, filename, do_requeue_last);
	if(ret < 0){
	printf("shashi:video_do_capture:video_close\n");
		video_close(&dev);
		return 1;
	}

	/* close the video device */
	printf("Bhagya : closing video device \n");
	video_close(&dev);


	return 0;

}

