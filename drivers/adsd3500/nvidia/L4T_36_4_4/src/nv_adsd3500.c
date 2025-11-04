// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Analog Devices ADSD3500 chip.
 *
 * Copyright (C) 2022 Analog Devices, All Rights Reserved.
 *
 */

#include "adsd3500_regs.h"

#include <linux/bitfield.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pwm.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/proc_fs.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>

#include "adsd3500_mode_tbls.h"

#define ADSD3500_DEFAULT_MODE		ADSD3500_MODE_512x512_30FPS
#define ADSD3500_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SRGGB12_1X12
#define ADSD3500_DEFAULT_WIDTH		512
#define ADSD3500_DEFAULT_HEIGHT		512
#define ADSD3500_DEFAULT_CLK_FREQ	104250000
#define ADSD3500_DEFAULT_FPS            10

static bool fw_load = true;
module_param(fw_load, bool, 0644);
MODULE_PARM_DESC(fw_load, "Boolean enabling/disbaling firmware loading by driver");

struct adsd3500_mode_info {
	uint32_t width;
	uint32_t height;
	uint32_t pixel_rate;
	uint32_t code;
	uint32_t link_freq_idx;
};

struct adsd3500_config_info {
	uint8_t nr_depth_bits;
	uint8_t nr_ab_bits;
	uint8_t nr_confidence_bits;
	uint8_t nr_mipi_lanes;
	bool use_vc;
};

struct adsd3500 {
	struct i2c_client 		*i2c_client;
	struct regmap 			*regmap;
	struct device 			*dev;
	struct v4l2_subdev 		*sd;
	struct media_pad 		pad;
	struct v4l2_mbus_framefmt 	fmt;
	struct v4l2_rect 		crop;

	const struct adsd3500_mode_info *current_mode;
	struct adsd3500_config_info 	current_config;

	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;

	struct v4l2_ctrl_handler 	ctrl_handler;

	const struct firmware  		*main_fw;
	struct mutex 			lock;
	bool 				streaming;
	s64 				framerate;
	u8 				curr_sync_mode;
	int 				gpio;
	int				irq;
	int                             signalnum;

	struct v4l2_ctrl 		*ctrls[15];
	struct pwm_device 		*pwm_fsync;
	struct proc_dir_entry 		*proc_dir;
	struct proc_dir_entry 		*proc_file;
	struct task_struct 		*task;
};

#define V4L2_CID_ADSD3500_OPERATING_MODE  	(V4L2_CID_USER_ADITOF_BASE + 0)
#define V4L2_CID_ADSD3500_CHIP_CONFIG 		(V4L2_CID_USER_ADITOF_BASE + 1)
#define V4L2_CID_ADSD3500_DEPTH_BITS 		(V4L2_CID_USER_ADITOF_BASE + 2)
#define V4L2_CID_ADSD3500_AB_BITS 		(V4L2_CID_USER_ADITOF_BASE + 3)
#define V4L2_CID_ADSD3500_CONFIDENCE_BITS	(V4L2_CID_USER_ADITOF_BASE + 4)
#define V4L2_CID_ADSD3500_AB_AVG 		(V4L2_CID_USER_ADITOF_BASE + 5)
#define V4L2_CID_ADSD3500_DEPTH_EN 		(V4L2_CID_USER_ADITOF_BASE + 6)
#define V4L2_CID_ADSD3500_FSYNC_TRIGGER 	(V4L2_CID_USER_ADITOF_BASE + 7)
#define V4L2_CID_ADSD3500_LOAD_FIRMWARE 	(V4L2_CID_USER_ADITOF_BASE + 8)

ssize_t adsd3500_proc_read(struct file *file, char __user *buff, size_t count, loff_t *offset);
ssize_t adsd3500_proc_write(struct file *file, const char __user *buff, size_t count, loff_t *offset);
static int adsd3500_set_fsync_trigger(struct adsd3500 *adsd3500, s32 val);
static int adsd3500_load_firmware(struct v4l2_subdev *sd);

static const struct reg_sequence adsd3500_powerup_setting[] = {
};

static const struct reg_sequence adsd3500_powerdown_setting[] = {
};

static const struct reg_sequence adsd3500_standby_setting[] = {
};

static const s64 link_freq_tbl[] = {
	732000000,
	1250000000,
};

static int adsd3500_proc_open(struct inode *inode, struct file *file)
{
	struct adsd3500 *adsd3500;

	adsd3500 = PDE_DATA(inode);
	if (!adsd3500)
		return -ENODEV;

	file->private_data = adsd3500;

	dev_dbg(adsd3500->dev, "Entered procfs file open\n");

	return 0;
}

static int adsd3500_proc_release(struct inode *inode, struct file *file)
{
	struct adsd3500 *adsd3500;
	struct task_struct *release_task = get_current();

	adsd3500 = PDE_DATA(inode);
	if (!adsd3500)
		return -ENODEV;

	file->private_data = adsd3500;

	dev_dbg(adsd3500->dev, "Entered procfs file close\n");
	if(release_task == adsd3500->task) {
		adsd3500->task = NULL;
	}

	return 0;
}

ssize_t adsd3500_proc_read(struct file *file, char __user *buff, size_t count, loff_t *offset){

	struct adsd3500 *adsd3500;
	unsigned int read_val;
	unsigned int len;
	int ret;
	char data[16];

	adsd3500 = PDE_DATA(file_inode(file));
	if (!adsd3500)
		return -ENODEV;

	if (!adsd3500->regmap) {
		dev_err(adsd3500->dev, "regmap not initialized\n");
		return -EFAULT;
	}

	dev_dbg(adsd3500->dev, "Entered procfs file read\n");
	ret = regmap_read(adsd3500->regmap, GET_IMAGER_STATUS_CMD, &read_val);
	if (ret < 0) {
		dev_err(adsd3500->dev, "Read of get status cmd failed %d.\n", ret);
		len = snprintf(data, sizeof(data), "Read failed\n");
	}
	else{
		dev_dbg(adsd3500->dev, "Read the error status: %.4X\n", read_val);
		len = snprintf(data, sizeof(data), "0x%.4X\n",read_val);
	}
	return simple_read_from_buffer(buff, count, offset, data, len);

}

ssize_t adsd3500_proc_write(struct file *file, const char __user *buff, size_t count, loff_t *offset){

	struct adsd3500 *adsd3500;

	adsd3500 = PDE_DATA(file_inode(file));
	if (!adsd3500)
		return -ENODEV;

	dev_dbg(adsd3500->dev, "Entered procfs file write\n");

	return count;
}

static long adsd3500_proc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct adsd3500 *adsd3500;

	adsd3500 = PDE_DATA(file_inode(file));
	if (!adsd3500)
		return -ENODEV;

	dev_dbg(adsd3500->dev, "Entered procfs ioctl\n");
	if (cmd == USER_TASK) {
		dev_dbg(adsd3500->dev, "Registered user task\n");
		adsd3500->task = get_current();
		adsd3500->signalnum = SIGETX;
	}

	return 0;
}

static const struct proc_ops adsd3500_proc_ops = {
	.proc_open   	= adsd3500_proc_open,
	.proc_read 	= adsd3500_proc_read,
	.proc_write  	= adsd3500_proc_write,
	.proc_ioctl 	= adsd3500_proc_ioctl,
	.proc_release	= adsd3500_proc_release,
};

static irqreturn_t adsd3500_irq_handler(int irq,void *priv)
{

	struct adsd3500 *adsd3500 = (struct adsd3500 *) priv;

	dev_dbg(adsd3500-> dev, "Entered ADSD3500 IRQ handler\n");

	if (adsd3500->task != NULL) {
		dev_dbg(adsd3500->dev, "Sending signal to app\n");
		if(send_sig_info(SIGETX, SEND_SIG_PRIV, adsd3500->task) < 0) {
			dev_err(adsd3500->dev, "Unable to send signal\n");
		}
	}

	return IRQ_HANDLED;

}

/* Elements of the structure must be ordered ascending by width & height */
static const struct adsd3500_mode_info adsd3500_mode_info_data[] = {

	/* --- RAW 8 MEDIA_BUS_FMT_SBGGR8_1X8 --- */

	{	/* RAW8 8BPP ADSD3100 QMP RESOLUTION */
		.width = 512,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 16BPP ADSD3100 QMP RESOLUTION */
		.width = 1024,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{       /* RAW8 16BPP Phase 12BPP AB ADSD3100 - DUAL MP - (1024x1 1024x4) */
		.width = 1024,
		.height = 4096,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 20BPP ADSD3100 QMP RESOLUTION */
		.width = 1280,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 24BPP ADSD3100 QMP RESOLUTION */
		.width = 1536,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 28BPP ADSD3100 QMP RESOLUTION */
		.width = 1792,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 32BPP ADSD3100 QMP RESOLUTION */
		.width = 2048,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{       /* RAW8 16BPP Phase 12BPP AB ADSD3100 - DUAL MP - (1024x2 1024x2)  */
		.width = 2048,
		.height = 2048,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 36BPP ADSD3100 QMP RESOLUTION */
		.width = 2304,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 40BPP ADSD3100 QMP RESOLUTION */
		.width = 2560,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{   /* RAW8 8BPP ADSD3030 VGA RESOLUTION */
		.width = 512,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 16BPP ADSD3030 VGA RESOLUTION */
		.width = 1024,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 24BPP ADSD3030 VGA RESOLUTION */
		.width = 1536,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 32BPP ADSD3030 VGA RESOLUTION */
		.width = 2048,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 40BPP ADSD3030 VGA RESOLUTION */
		.width = 2560,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{   /* RAW8 8BPP ADSD3030 QVGA RESOLUTION */
		.width =  256,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 16BPP ADSD3030 QVGA RESOLUTION */
		.width =  512,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 24BPP ADSD3030 QVGA RESOLUTION */
		.width =  768,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 32BPP ADSD3030 QVGA RESOLUTION */
		.width = 1024,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 40BPP ADSD3030 QVGA RESOLUTION */
		.width = 1280,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 12BPP * 3 phase + 16BPP * 1 AB SuperFrame (1024x2 1024x3.25)*/
		.width = 2048,
		.height = 3328,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 12BPP * 3 phase + 16BPP * 1 AB Superframe (512x2 512x3.25) */
		.width = 1024,
		.height = 1664,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	/* RAW8 ADSD3030 VGA 12BPP * 3 phase + 16BPP * 1 AB  SuperFrame (512x2 640x3.25)*/
		.width = 1024,
		.height = 2080,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 12BPP * 3 phase + 16BPP * 1 AB  SuperFrame (256x2 320x3.25)*/
		.width = 512,
		.height = 1040,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 12BPP * 2 phase + 16BPP * 1 AB SuperFrame (1024x2 1024x2.5)*/
		.width = 2048,
		.height = 2560,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	/* RAW8 ADSD3100 QMP 12BPP * 2 phase + 16BPP * 1 AB SuperFrame (512x2 512x2.5)*/
		.width = 1024,
		.height = 1280,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 12BPP * 2 phase + 16BPP * 1 AB SuperFrame (512x2 640x2.5)*/
		.width = 1024,
		.height = 1600,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 12BPP * 2 phase + 16BPP * 1 AB SuperFrame (256x2 320x2.5)*/
		.width = 512,
		.height = 800,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 16BPP * 3 phase + 16BPP * 1 AB SuperFrame (1024x2 1024x4)*/
		.width = 2048,
		.height = 4096,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 16BPP * 3 phase + 16BPP * 1 AB SuperFrame (512x2 512x4)*/
		.width = 1024,
		.height = 2048,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 16BPP * 3 phase + 16BPP * 1 AB SuperFrame (512x2 640x4)*/
		.width = 1024,
		.height = 2560,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 16BPP * 3 phase + 16BPP * 1 AB SuperFrame (256x2 320x4)*/
		.width = 512,
		.height = 1280,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 16BPP * 2 phase + 16BPP * 1 AB SuperFrame (1024x2 1024x3)*/
		.width = 2048,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 16BPP * 2 phase + 16BPP * 1 AB SuperFrame (512x2 512x3)*/
		.width = 1024,
		.height = 1536,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 16BPP * 2 phase + 16BPP * 1 AB SuperFrame (512x2 640x3)*/
		.width = 1024,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 16BPP * 2 phase + 16BPP * 1 AB SuperFrame (256x2 320x3)*/
		.width = 512,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 12BPP * 3 phase + 16BPP * 3 AB Interleaved (1024x3.5 1024x3) */
		.width = 3584,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 12BPP * 3 phase + 16BPP * 3 AB Interleaved (512x3.5 512x3) */
		.width = 1792,
		.height = 1536,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 12BPP * 3 phase + 16BPP * 3 AB Interleaved (512x3.5 640x3) */
		.width = 1792,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 12BPP * 3 phase + 16BPP * 3 AB Interleaved (256x3.5 320x3) */
		.width = 896,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 12BPP * 2 phase + 16BPP * 2 AB Interleaved (1024x3.5 1024x2) */
		.width = 3584,
		.height = 2048,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 12BPP * 2 phase + 16BPP * 2 AB Interleaved (512x3.5 512x2) */
		.width = 1792,
		.height = 1024,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 12BPP * 2 phase + 16BPP * 2 AB Interleaved (512x3.5 640x2) */
		.width = 1792,
		.height = 1280,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 12BPP * 2 phase + 16BPP * 2 AB Interleaved (256x3.5 320x2) */
		.width = 896,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 16BPP * 3 phase + 16BPP * 3 AB Interleaved (1024x4 1024x3) */
		.width = 4096,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 16BPP * 3 phase + 16BPP * 3 AB Interleaved (512x4 512x3) */
		.width = 2048,
		.height = 1536,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 16BPP * 3 phase + 16BPP * 3 AB Interleaved (512x4 640x3) */
		.width = 2048,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 16BPP * 3 phase + 16BPP * 3 AB Interleaved (256x4 320x3) */
		.width = 1024,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	/* --- RAW 8 MEDIA_BUS_FMT_SRGGB8_1X8  Miscellaneous --- */

	{ 	//RAW8 MP 16BPP * 3 phase + 16BPP * 3 AB
		.width = 2048,
		.height = 6144,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW8 QMP 16BPP * 3 phase + 16BPP * 3 AB
		.width = 3072,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW8 VGA 16BPP * 3 phase + 16BPP * 3 AB
		.width = 3072,
		.height = 3840,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW8 QVGA 16BPP * 3 phase + 16BPP * 1 AB
		.width = 512,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW8 QVGA 16BPP * 3 phase + 16BPP * 3 AB
		.width = 1536,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	/* --- RAW 12 MEDIA_BUS_FMT_SRGGB12_1X12 --- */

	{   /* RAW12 12BPP ADSD3100 MP - 9 subframes - (1024x2 1024x4.5) */
		.width = 2048,
		.height = 4608,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3100 QMP - 9 subframes - (512x2 512x4.5) */
		.width = 1024,
		.height = 2304,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 VGA - 9 subframes - (512x2 640x4.5) */
		.width = 1024,
		.height = 2880,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 QVGA - 9 subframes - (256x2 320x4.5) */
		.width = 512,
		.height = 1440,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3100 MP - 6 subframes - (1024x2 1024x3) */
		.width = 2048,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3100 QMP - 6 subframes - (512x2 512x3) */
		.width = 1024,
		.height = 1536,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 VGA - 6 subframes - (512x2 640x3) */
		.width = 1024,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 QVGA - 6 subframes - (256x2 320x3) */
		.width = 512,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	/* --- RAW 12 MEDIA_BUS_FMT_SRGGB12_1X12  Miscellaneous --- */

	{ 	//RAW12 ADSD3100 QMP 12BPP Depth only/ AB Only
		.width = 512,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 ADSD3030 VGA 12BPP Depth Only/ AB Only
		.width = 512,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 12BPP AB test
		.width = 2048,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	//RAW12 1 Phase / Frame
		.width = 1024,
		.height = 1024,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	//RAW12 12BPP * 3 phase
		.width = 1024,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 12BPP * 3 phase
		.width = 1024,
		.height = 4096,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 12BPP * 3 phase + 12BPP * 3 AB
		.width = 2048,
		.height = 5376,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 12BPP * 2 phase + 12BPP * 2 AB
		.width = 2048,
		.height = 3584,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
};

static int adsd3500_set_frame_rate(struct adsd3500 *priv, s64 val);

static bool adsd3500_regmap_accessible_reg(struct device *dev, unsigned int reg)
{
	if (reg % 2)
		return 0;

	switch (reg) {
		case GET_CHIP_ID_CMD:
		case GET_IMAGER_MODE_CMD:
		case GET_IMAGER_AB_INVLD_TRSHLD:
		case GET_IMAGER_CONFIDENCE_TRSHLD:
		case GET_IMAGER_JBLF_STATE:
		case GET_IMAGER_JBLF_FILT_SIZE:
		case GET_FRAMERATE_CMD:
		case GET_IMAGER_STATUS_CMD:
			return 1;
		default:
			return 0;
	}
}

static const struct regmap_config adsd3500_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_NONE,
	.readable_reg = adsd3500_regmap_accessible_reg,
};

static int _adsd3500_power_on(struct camera_common_data *s_data)
{
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	struct v4l2_ctrl *operating_mode = adsd3500->ctrls[1];
	struct v4l2_ctrl *ab_avg = adsd3500->ctrls[2];
	struct v4l2_ctrl *depth_en = adsd3500->ctrls[3];
	struct adsd3500_config_info config = adsd3500->current_config;
	unsigned int write_cmd, write_val = 0;
	int ret;

	dev_dbg(adsd3500->dev, "Entered addicmos_power_on\n");

	write_cmd = SET_IMAGER_MODE_CMD | SET_IMAGER_MODE(operating_mode->val);

	write_val |= SET_IMAGER_MODE_DEPTH_EN(depth_en->val);
	write_val |= SET_IMAGER_MODE_DEPTH_BITS(config.nr_depth_bits ? 6 - config.nr_depth_bits: 0);

	write_val |= SET_IMAGER_MODE_AB_EN(config.nr_ab_bits ? 1 : 0);
	write_val |= SET_IMAGER_MODE_AB_BITS(config.nr_ab_bits ? 6 - config.nr_ab_bits : 0);

	write_val |= SET_IMAGER_MODE_CONF_BITS(config.nr_confidence_bits);

	write_val |= SET_IMAGER_MODE_VC_EN(!config.use_vc);
	write_val |= SET_IMAGER_MODE_AB_AVG_EN(ab_avg->val);
	write_val |= SET_IMAGER_MODE_MIPI_LANES_NR(config.nr_mipi_lanes);

	ret = regmap_write(adsd3500->regmap, write_cmd, write_val);
	if (ret) {
		dev_err(adsd3500->dev, "Could not set mode register\n");
		return ret;
	}

	return 0;
}

static int adsd3500_power_off(struct camera_common_data *s_data)
{
	return 0;
}

static int adsd3500_power_on(struct camera_common_data *s_data)
{
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int adsd3500_g_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	unsigned int read_val;
	int ret;

	reg->size = 2;
	ret = regmap_read(adsd3500->regmap, reg->reg, &read_val);
	reg->val = read_val;

	return ret;
}

static int adsd3500_s_register(struct v4l2_subdev *sd,
			       const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;

	return regmap_write(adsd3500->regmap, reg->reg, reg->val);
}
#endif

static int adsd3500_bpp_config(struct adsd3500 *priv,
				    struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_ADSD3500_DEPTH_BITS:
		priv->current_config.nr_depth_bits = ctrl->val;
		break;
	case V4L2_CID_ADSD3500_AB_BITS:
		priv->current_config.nr_ab_bits = ctrl->val;
		break;
	case V4L2_CID_ADSD3500_CONFIDENCE_BITS:
		priv->current_config.nr_confidence_bits = ctrl->val;
		break;
	default:
		break;
	}

	return 0;
}

static int adsd3500_chip_config(struct adsd3500 *adsd3500,
				    struct v4l2_ctrl *ctrl)
{
	struct device *dev = adsd3500->dev;
	struct i2c_client *client = adsd3500->i2c_client;
	uint16_t pld_size;
	uint8_t r_w, *data;
	int ret;

	r_w = *ctrl->p_new.p_u8;
	pld_size = (uint16_t)(*(ctrl->p_new.p_u8 + 1) << 8 | *(ctrl->p_new.p_u8 + 2));
	data = ctrl->p_new.p_u8 + 3;

	dev_dbg(dev, "Entered adsd3500_chip_config. R/W: %d, PLD_SIZE: %d\n", r_w, pld_size);

	if ((pld_size > 4096) || (pld_size < 2))
		return -EINVAL;

	if (r_w) {
		ret = i2c_master_send(client, data, pld_size);
		if (ret < 0) {
			dev_warn(dev, "Write burst transfer failed\n");
			return -EIO;
		}
	} else {
		ret = i2c_master_recv(client, data, pld_size);
		if (ret < 0) {
			dev_warn(dev, "Read burst transfer failed\n");
			return -EIO;
		}
	}
	memset(ctrl->p_new.p_u8, 0xFF, 1);
	return 0;
}

static int adsd3500_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct adsd3500 *priv = container_of(ctrl->handler, struct adsd3500, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	switch (ctrl->id) {
	default:
		dev_err(dev, "%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int adsd3500_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct adsd3500 *adsd3500 = container_of(ctrl->handler,
						 struct adsd3500, ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
	case V4L2_CID_ADSD3500_OPERATING_MODE:
	case V4L2_CID_ADSD3500_AB_AVG:
	case V4L2_CID_ADSD3500_DEPTH_EN:
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_LINK_FREQ:
		break;
	case V4L2_CID_ADSD3500_CHIP_CONFIG:
		ret = adsd3500_chip_config(adsd3500, ctrl);
		break;
	case V4L2_CID_ADSD3500_DEPTH_BITS:
	case V4L2_CID_ADSD3500_AB_BITS:
	case V4L2_CID_ADSD3500_CONFIDENCE_BITS:
		ret = adsd3500_bpp_config(adsd3500, ctrl);
		break;
	case V4L2_CID_ADSD3500_FSYNC_TRIGGER:
		ret = adsd3500_set_fsync_trigger(adsd3500, ctrl->val);
		break;
	case V4L2_CID_ADSD3500_LOAD_FIRMWARE:
		ret = adsd3500_load_firmware(adsd3500->sd);
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		ret = adsd3500_set_frame_rate(adsd3500, *ctrl->p_new.p_s64);
		break;
	default:
		dev_err(adsd3500->dev, "%s > Unhandled: %x  param=%x\n",
			__func__, ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops adsd3500_ctrl_ops = {
	.g_volatile_ctrl = adsd3500_g_volatile_ctrl,
	.s_ctrl = adsd3500_s_ctrl,
};

static const s64 nr_bits_qmenu[] = {
	0, 4, 8, 10, 12, 14, 16
};

static const struct v4l2_ctrl_config adsd3500_ctrls[] = {
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= TEGRA_CAMERA_CID_SENSOR_MODE_ID,
		.name		= "Sensor Mode",
		.type		= V4L2_CTRL_TYPE_INTEGER64,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
		.min		= 0,
		.max		= 0xFF,
		.def		= 0x0,
		.step 		= 1,
	},
	{
		/* Should always be second control in list*/
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_OPERATING_MODE,
		.name		= "Operating Mode",
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.def		= 7,
		.min		= 0,
		.max		= 10,
		.step		= 1,
	},
	{
		/* Should always be third control in list*/
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_AB_AVG,
		.name		= "AB Averaging",
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.def		= 1,
		.min		= 0,
		.max		= 1,
		.step		= 1,
	},
	{
		/* Should always be fourth control in list*/
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_DEPTH_EN,
		.name		= "Depth enable",
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.def		= 1,
		.min		= 0,
		.max		= 1,
		.step		= 1,
	},
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_CHIP_CONFIG,
		.name		= "Chip Config",
		.type		= V4L2_CTRL_TYPE_U8,
		.def		= 0x00,
		.min		= 0x00,
		.max		= 0xFF,
		.step		= 1,
		.dims		= { 4099 }
	},
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_DEPTH_BITS,
		.name		= "Phase / Depth Bits",
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.def		= 0,
		.min		= 0,
		.max		= 6,
		.menu_skip_mask = 0x02,
		.qmenu_int	= nr_bits_qmenu,
	},
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_AB_BITS,
		.name		= "AB Bits",
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.def		= 0,
		.min		= 0,
		.max		= 6,
		.menu_skip_mask = 0x02,
		.qmenu_int	= nr_bits_qmenu,
	},
	{
		.ops		= &adsd3500_ctrl_ops,
		.id		= V4L2_CID_ADSD3500_CONFIDENCE_BITS,
		.name		= "Confidence Bits",
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.def		= 0,
		.min		= 0,
		.max		= 2,
		.qmenu_int	= nr_bits_qmenu,
	},
	{
		.ops            = &adsd3500_ctrl_ops,
		.id             = V4L2_CID_ADSD3500_FSYNC_TRIGGER,
		.name           = "Fsync Trigger",
		.type           = V4L2_CTRL_TYPE_INTEGER,
		.def            = 1,
		.min            = 0,
		.max            = 1,
		.step           = 1,
	},
	{
		.ops    	= &adsd3500_ctrl_ops,
		.id     	= V4L2_CID_ADSD3500_LOAD_FIRMWARE,
		.name   	= "Load Firmware",
		.type   	= V4L2_CTRL_TYPE_INTEGER,
		.def    	= 0,
		.min    	= 0,
		.max    	= 1,
		.step   	= 1,
	},
	{
		.ops 		= &adsd3500_ctrl_ops,
		.id 		= TEGRA_CAMERA_CID_FRAME_RATE,
		.name 		= "Frame Rate",
		.type 		= V4L2_CTRL_TYPE_INTEGER64,
		.flags 		= V4L2_CTRL_FLAG_SLIDER,
		.min 		= 1,
		.max 		= 90,
		.def 		= 10,
		.step		= 1,
	},
};

static int adsd3500_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;

	switch (code->index) {
	case 0:
		code->code = MEDIA_BUS_FMT_SRGGB8_1X8;
		break;
	case 1:
		code->code = MEDIA_BUS_FMT_SRGGB12_1X12;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int adsd3500_enum_frame_size(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	int i, j = 0;

	if (fse->index >= ARRAY_SIZE(adsd3500_mode_info_data))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(adsd3500_mode_info_data); i++)
	{
		if(adsd3500_mode_info_data[i].code == fse->code)
			j++;
		if (j > fse->index)
			break;
	}

	if (i < ARRAY_SIZE(adsd3500_mode_info_data)) {
		fse->min_width = adsd3500_mode_info_data[i].width;
		fse->max_width = adsd3500_mode_info_data[i].width;
		fse->min_height = adsd3500_mode_info_data[i].height;
		fse->max_height = adsd3500_mode_info_data[i].height;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int adsd3500_get_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int adsd3500_set_format(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
	struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	int ret;

	dev_dbg(dev, "set_fmt: %x %dx%d %d\n",
		format->format.code, format->format.width,
		format->format.height, format->which);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static int adsd3500_start_streaming(struct adsd3500 *adsd3500)
{
	int ret;

	if(adsd3500->pwm_fsync != NULL && adsd3500->curr_sync_mode == PWM_TRIGGER){
		ret = pwm_enable(adsd3500->pwm_fsync);
		if (ret){
			dev_err(adsd3500->dev, "Could not enable FSYNC PWM\n");
			return ret;
		}
	}

	ret = regmap_write(adsd3500->regmap, STREAM_ON_CMD, STREAM_ON_VAL);
	if (ret < 0)
		dev_err(adsd3500->dev, "Write of STREAM-ON command failed.\n");

	return ret;
}

static int adsd3500_stop_streaming(struct adsd3500 *adsd3500)
{
	int ret;

	if(adsd3500->pwm_fsync != NULL && adsd3500->curr_sync_mode == PWM_TRIGGER){
		pwm_disable(adsd3500->pwm_fsync);
		msleep(1000 / adsd3500->framerate);
	}

	ret = regmap_write(adsd3500->regmap, STREAM_OFF_CMD, STREAM_OFF_VAL);
	if (ret < 0)
		dev_err(adsd3500->dev, "Write of STREAM-OFF command failed.\n");

	return ret;
}

static int adsd3500_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	int ret = 0;

	dev_dbg(dev, "s_stream: %d\n", enable);

	mutex_lock(&adsd3500->lock);
	if (adsd3500->streaming == enable) {
		mutex_unlock(&adsd3500->lock);
		return 0;
	}

	if (enable) {
		_adsd3500_power_on(s_data);
		ret = adsd3500_start_streaming(adsd3500);
		if (ret)
			goto err_unlock;
	} else {
		adsd3500_stop_streaming(adsd3500);
	}

	adsd3500->streaming = enable;
	mutex_unlock(&adsd3500->lock);

	return ret;

err_unlock:
	mutex_unlock(&adsd3500->lock);

	return ret;
}

static int adsd3500_set_frame_rate(struct adsd3500 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;
	struct pwm_state state;
	int ret;

	priv->framerate = val;

	if(priv->pwm_fsync != NULL){
		pwm_init_state(priv->pwm_fsync, &state);
		state.period = DIV_ROUND_UP(1 * NSEC_PER_SEC, priv->framerate);
		pwm_set_relative_duty_cycle(&state, 50, 100);
		ret = pwm_apply_state(priv->pwm_fsync, &state);
	}

	ret = regmap_write(priv->regmap, SET_FRAMERATE_CMD, val);
	if (ret < 0)
		dev_err(dev, "Set FRAMERATE COMMAND failed.\n");

	dev_dbg(dev, "Set frame rate to %lld\n", val);

	return ret;
}

static int adsd3500_configure_interrupt(struct adsd3500 *adsd3500){

	struct device *dev = adsd3500->dev;
	struct i2c_client *client = adsd3500->i2c_client;
	struct device_node *np = client->dev.of_node;
	int gpio, ret = 0;

	dev_dbg(dev, "Entered: %s\n", __func__);
	gpio = of_get_named_gpio(np, "interrupt-gpios", 0);
	if (!gpio_is_valid(gpio)){
		dev_err(&client->dev, "interrupt-gpios not found %d\n", gpio);
		return ret;
	}
	adsd3500->gpio = gpio;

	ret = gpio_request(adsd3500->gpio, "adsd3500_irq");
	if (ret < 0){
		dev_err(&client->dev, "Unable to get adsd3500 gpio\n");
		return ret;
	}

	ret = gpio_direction_input(adsd3500->gpio);
	if (ret < 0) {
		dev_err(&client->dev,"Unable to set adsd3500 gpio as input\n");
		return ret;
	}

	ret = gpio_to_irq(adsd3500->gpio);
	if (ret < 0){
		dev_err(&client->dev, "Unable to map GPIO adsd3500 gpio to an IRQ\n");
		return ret;
	}
	adsd3500->irq = ret;

	ret = request_irq(adsd3500->irq, adsd3500_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "adsd3500", adsd3500);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request adsd3500 irq\n");
		return ret;
	}

	return ret;

}

static int adsd3500_set_fsync_trigger(struct adsd3500 *adsd3500, s32 val){

	struct device *dev = adsd3500->dev;
	struct i2c_client *client = adsd3500->i2c_client;
	struct pwm_state state;
	int ret;

	if(adsd3500->pwm_fsync == NULL){
		dev_warn(dev, "Failed to get the pwm device\n");
		return -ENODEV;
	}

	dev_dbg(dev, "Entered: %s Value: %d\n", __func__, val);
	adsd3500->curr_sync_mode = val;

	if(adsd3500->curr_sync_mode == PWM_TRIGGER){
		dev_dbg(dev, "Enable frame sync using pwm trigger\n");
		if(IS_ERR(adsd3500->pwm_fsync)){
			dev_err(&client->dev, "Failed to get the pwm device\n");
			goto error;
		}
		free_irq(adsd3500->irq, adsd3500);
		gpio_free(adsd3500->gpio);

		pwm_init_state(adsd3500->pwm_fsync, &state);
		state.polarity = PWM_POLARITY_NORMAL;
		ret = pwm_apply_state(adsd3500->pwm_fsync, &state);
		if(ret){
			dev_err(&client->dev, "Failed to change the PWM state polarity\n");
		}

		ret = adsd3500_set_frame_rate(adsd3500, adsd3500->framerate);
		if(ret){
			dev_err(&client->dev, "Failed to set the frame rate\n");
		}

		ret = regmap_write(adsd3500->regmap, ENABLE_FSYNC_TRIGGER, FSYNC_HIZ);
		if (ret < 0){
			dev_err(adsd3500->dev, "Write of enable fsync external trigger command failed.\n");
		}
	}
	else if(adsd3500->curr_sync_mode == INTR_TRIGGER){
		dev_dbg(dev, "Enable interrupt trigger\n");
		pwm_init_state(adsd3500->pwm_fsync, &state);
		state.polarity = PWM_POLARITY_INVERSED;

		ret = pwm_apply_state(adsd3500->pwm_fsync, &state);
		if(ret) {
			dev_err(&client->dev, "Failed to change the PWM state polarity\n");
		}

		ret = adsd3500_configure_interrupt(adsd3500);
		if(ret < 0){
			return ret;
		}

		ret = regmap_write(adsd3500->regmap, ENABLE_FSYNC_TRIGGER, FSYNC_START);
		if (ret < 0){
			dev_err(adsd3500->dev, "Write of enable fsync internal trigger command failed.\n");
		}
	}
	else{
		dev_err(dev, "Invalid sync mode %d\n", adsd3500->curr_sync_mode);
	}

	return 0;

error:
	return -ENXIO;

}

static int adsd3500_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static int adsd3500_ctrls_init(struct adsd3500 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(adsd3500_ctrls);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&adsd3500_ctrls[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				adsd3500_ctrls[i].name);
			continue;
		}

		if (adsd3500_ctrls[i].type == V4L2_CTRL_TYPE_STRING &&
			adsd3500_ctrls[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				adsd3500_ctrls[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->sd->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	//Initialize by default to 4 (RAW12, 12 bpp)
	v4l2_ctrl_s_ctrl(priv->ctrls[5], 4);

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static int adsd3500_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops adsd3500_subdev_internal_ops = {
	.open = adsd3500_open,
};

const static struct of_device_id adsd3500_of_match[] = {
	{ .compatible = "adi,adsd3500" },
	{ /* sentinel */ }
};

static int adsd3500_procfs_init(struct adsd3500 *priv){

	priv->proc_dir = proc_mkdir("adsd3500", NULL);
	if (!priv->proc_dir)
		return -ENOMEM;

	priv->proc_file = proc_create_data("value", 0666, priv->proc_dir, &adsd3500_proc_ops, priv);
	if (!priv->proc_file) {
		proc_remove(priv->proc_dir);
		return -ENOMEM;
	}

	return 0;
}

static struct camera_common_pdata *adsd3500_parse_dt(struct i2c_client *client,
				struct camera_common_data *s_data)
{
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	struct pwm_state state;
	int err;
	const char *str;

	if (!np)
		return NULL;

	board_priv_pdata = devm_kzalloc(&client->dev,
			sizeof(*board_priv_pdata), GFP_KERNEL);

	adsd3500->current_config.use_vc = of_property_read_bool(np, "adi,use-vc");
	if (adsd3500->current_config.use_vc)
		dev_dbg(&client->dev, "Virtual Channel mode activated\n");

	err = of_property_read_string(np, "use_sensor_mode_id", &str);
	if (!err) {
		if (!strcmp(str, "true"))
			s_data->use_sensor_mode_id = true;
		else
			s_data->use_sensor_mode_id = false;
	}

	adsd3500->pwm_fsync = devm_pwm_get(&client->dev, NULL);
	if(IS_ERR(adsd3500->pwm_fsync)){
		dev_warn(&client->dev, "Failed to get the pwm device\n");
		adsd3500->pwm_fsync = NULL;
		return board_priv_pdata;
	}

	adsd3500->framerate = ADSD3500_DEFAULT_FPS;
	pwm_init_state(adsd3500->pwm_fsync, &state);
	state.period = DIV_ROUND_UP(1 * NSEC_PER_SEC, adsd3500->framerate);
	pwm_set_relative_duty_cycle(&state, 50, 100);
	err = pwm_apply_state(adsd3500->pwm_fsync, &state);
	if(err){
		dev_err(&client->dev, "PWM init failed %d\n", err);
		goto error;
	}

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

static int adsd3500_read_ack(struct v4l2_subdev *sd){

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	uint8_t read_val[4];
	int ret;

	ret = i2c_master_recv(client, read_val, sizeof(read_val));
	if (ret < 0) {
		dev_err(adsd3500->dev, "Read ACK cmd failed.\n");
		return -EIO;
	}
	else{
	// Verify the acknowledgement success command op-code from the received response
		if (read_val[0] != 0x0B) {
			dev_err(adsd3500->dev, "ACK ERROR response d0: %02X  d1: %02X d2: %02X d3: %02X\n", read_val[0], read_val[1], read_val[2], read_val[3]);
			return -ENXIO;
		}
		dev_dbg(adsd3500->dev, "ACK Received\n");
	}

	return 0;
}

static int adsd3500_send_host_boot_data(struct v4l2_subdev *sd, uint8_t* data, int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	int n_segments = 0, prev_seg_end = 0, curr_segment= 0;
	int position = 0, location = 0;
	int segment_start[100], segment_end[100], segment_size[100];
	int header_start[100], header_end[100];
	uint8_t header_packet[16];
	uint8_t* data_packet = NULL;
	int ret;

	dev_dbg(adsd3500->dev, "Entered: %s\n",__func__);

	data_packet = (uint8_t *)kvzalloc(PAYLOAD_MAX_CHUNK_SIZE * sizeof(uint8_t),GFP_KERNEL);

	while (1) {
		segment_size[n_segments] = (data[prev_seg_end+2]<<8)|data[prev_seg_end+1];
		segment_start[n_segments] = prev_seg_end + HEADER_SIZE_IN_BYTES;
		segment_end[n_segments] = segment_start[n_segments] + segment_size[n_segments];
		header_start[n_segments] = segment_start[n_segments] - HEADER_SIZE_IN_BYTES;
		header_end[n_segments] = segment_start[n_segments] - 1;
		prev_seg_end = segment_end[n_segments];
		n_segments++;
		if (prev_seg_end >= size) {
			break;
		}
	}

	dev_info(adsd3500->dev, "No of headers =%02d\n", n_segments);

	while (curr_segment < n_segments){
		location = header_start[curr_segment];
		memcpy(header_packet, &data[location], HEADER_SIZE_IN_BYTES);
		dev_dbg(adsd3500->dev,"Current Segment = %d header_start = %d\n Size= %04X", curr_segment + 1, header_start[curr_segment], segment_size[curr_segment]);
		dev_dbg(adsd3500->dev, "Send Header data\n");
		ret = i2c_master_send(client, header_packet, HEADER_SIZE_IN_BYTES);
		if (ret < 0) {
			dev_err(adsd3500->dev, "Failed to write the header packet of segment %d\n", curr_segment + 1);
			return -EIO;
		}
		// Check for the RESET command op-code and skip the read acknowledgement
		if (header_packet[03] == 0x55) {
			dev_info(adsd3500->dev, "Firmware transfer Compelete\n");
			break;
		}
		ret = adsd3500_read_ack(sd);
		if(ret < 0) {
			dev_err(adsd3500->dev, "Failed to read the acknowledgement header packet segment: %d\n", curr_segment + 1);
			return -ENXIO;
		}
		msleep(5);

		if(segment_size[curr_segment] != 0){
			location = segment_start[curr_segment];
			memcpy(data_packet, &data[location], segment_size[curr_segment]);
			dev_dbg(adsd3500->dev, "Send Payload data\n");
			ret = i2c_master_send(client, data_packet, segment_size[curr_segment]);
			if (ret < 0) {
				dev_err(adsd3500->dev, "Failed to write the payload data of segment %d\n",curr_segment + 1);
				return -EIO;
			}
			msleep(5);
			ret = adsd3500_read_ack(sd);
			if(ret < 0) {
				dev_err(adsd3500->dev, "Failed to read the acknowledgement payload data segment: %d\n", curr_segment + 1);
				return -ENXIO;
			}
		}
		msleep(1);

		memset(header_packet, 0, sizeof(header_packet));
		memset(data_packet, 0, PAYLOAD_MAX_CHUNK_SIZE * sizeof(uint8_t));
		position=0;
		curr_segment++;
	}

	kfree(data_packet);

	return 0;
}

static int adsd3500_parse_fw(struct v4l2_subdev *sd)
{

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	const struct firmware *fw = adsd3500->main_fw;
	uint8_t* data_fw_3500 = NULL;
	size_t data_fw_3500_size = 0;
	int ret;

	dev_dbg(adsd3500->dev, "Entered: %s\n",__func__);

	data_fw_3500_size = fw->size;
	dev_info(adsd3500->dev, "Firmware size = %ld\n",data_fw_3500_size);
	data_fw_3500 = (uint8_t *)kvzalloc(data_fw_3500_size * sizeof(uint8_t),GFP_KERNEL);
	if(!data_fw_3500){
		dev_err(adsd3500->dev, "Failed to allocate memory for FW data\n");
		return -ENOMEM;
	}

	memcpy(data_fw_3500, fw->data, data_fw_3500_size);
	dev_dbg(adsd3500->dev, "FW data 1:%02X  2:%02X  3:%02X  4:%02X\n",data_fw_3500[0], data_fw_3500[1], data_fw_3500[2], data_fw_3500[3]);
	ret = adsd3500_send_host_boot_data(sd, data_fw_3500, data_fw_3500_size);
	if(ret != 0){
		dev_err(adsd3500->dev, "Failed to send the host boot firmware\n");
		kfree(data_fw_3500);
		return ret;
	};

	kfree(data_fw_3500);

	return 0;
}

static int adsd3500_load_firmware(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct adsd3500 *adsd3500 = (struct adsd3500 *)s_data->priv;
	int ret;

	dev_dbg(adsd3500->dev, "Entered: %s\n",__func__);

	if(fw_load){
		dev_dbg(adsd3500->dev, "Request ADSD3500 firmware file\n");
		ret = request_firmware(&adsd3500->main_fw, ADSD3500_FIRMWARE, adsd3500->dev);
		if(ret < 0) {
			dev_err(adsd3500->dev, "Failed to read firmware\n");
			goto release_firmware;
		}
		else {
			ret = adsd3500_parse_fw(sd);
			if(ret < 0){
				dev_err(adsd3500->dev, "Failed to parse the firmware\n");
				goto release_firmware;
				return ret;
			}
		}
	}

	release_firmware(adsd3500->main_fw);

	return 0;

release_firmware:
	release_firmware(adsd3500->main_fw);
	return ret;

}

MODULE_DEVICE_TABLE(of, adsd3500_of_match);

static struct camera_common_sensor_ops adsd3500_common_ops = {
	.power_off = adsd3500_power_off,
	.power_on = adsd3500_power_on,
};

static const struct v4l2_subdev_core_ops adsd3500_core_ops = {
	.s_power = camera_common_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = adsd3500_g_register,
	.s_register = adsd3500_s_register,
#endif
};

static const struct v4l2_subdev_video_ops adsd3500_video_ops = {
	.s_stream = adsd3500_s_stream,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	.g_mbus_config	= camera_common_g_mbus_config,
#endif
};

static const struct v4l2_subdev_pad_ops adsd3500_subdev_pad_ops = {
	.enum_mbus_code = adsd3500_enum_mbus_code,
	.enum_frame_size = adsd3500_enum_frame_size,
	.get_fmt = adsd3500_get_format,
	.set_fmt = adsd3500_set_format,
	.enum_frame_interval = camera_common_enum_frameintervals,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	.get_mbus_config	= camera_common_get_mbus_config,
#endif
};

static const struct v4l2_subdev_ops adsd3500_subdev_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.core = &adsd3500_core_ops,
#endif
	.video = &adsd3500_video_ops,
	.pad = &adsd3500_subdev_pad_ops,
};

static const struct media_entity_operations adsd3500_subdev_entity_ops = {
	.link_setup = adsd3500_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
static int adsd3500_probe(struct i2c_client *client)
#else
static int adsd3500_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
#endif
{
	struct camera_common_data *common_data;
	struct adsd3500 *priv;
	int ret;

	dev_info(&client->dev, "probing adsd3500 v4l2 sensor\n");

	common_data = devm_kzalloc(&client->dev, sizeof(struct camera_common_data),
							   GFP_KERNEL);

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct adsd3500) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(adsd3500_ctrls),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}
	common_data->priv = (void *)priv;

	priv->regmap = devm_regmap_init_i2c(client,
						 &adsd3500_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev, "Error initializing I2C regmap\n");
		return PTR_ERR(priv->regmap);
	}

	if (client->dev.of_node)
		priv->pdata = adsd3500_parse_dt(client, common_data);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops = &adsd3500_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &adsd3500_frmfmt[0];
	common_data->colorfmt = camera_common_find_datafmt(
					  ADSD3500_DEFAULT_DATAFMT);
	common_data->numctrls = 0;
	common_data->numfmts = ARRAY_SIZE(adsd3500_frmfmt);
	common_data->def_mode = ADSD3500_DEFAULT_MODE;
	common_data->def_width = ADSD3500_DEFAULT_WIDTH;
	common_data->def_height = ADSD3500_DEFAULT_HEIGHT;
	common_data->def_clk_freq = ADSD3500_DEFAULT_CLK_FREQ;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;

	priv->dev = &client->dev;
	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->sd = &common_data->subdev;
	priv->sd->dev = &client->dev;
	priv->sd->fwnode = dev_fwnode(priv->dev);
	priv->s_data->dev = &client->dev;
	priv->framerate = ADSD3500_DEFAULT_FPS;
	mutex_init(&priv->lock);

	ret = camera_common_initialize(common_data, "adsd3500");
	if (ret) {
		dev_err(&client->dev, "Failed to initialize adsd3500.\n");
		return ret;
	}

	priv->current_config.nr_mipi_lanes = common_data->numlanes;
	dev_dbg(&client->dev, "Lanes nr: %u\n", priv->current_config.nr_mipi_lanes);

	v4l2_i2c_subdev_init(priv->sd, client, &adsd3500_subdev_ops);
	//v4l2_set_subdevdata(priv->sd, priv);

	ret= adsd3500_procfs_init(priv);
	if(ret < 0){
		dev_err(&client->dev, "Failed to initialize procfs.\n");
		return ret;
	}

	ret = adsd3500_ctrls_init(priv);
	if (ret < 0){
		dev_err(&client->dev, "Failed to initialize v4l2 ctrls\n");
		return ret;
	}

	if(priv->pwm_fsync != NULL){
		ret = adsd3500_set_fsync_trigger(priv, INTR_TRIGGER);
		if(ret < 0)
			dev_err(&client->dev, "Failed to initalize interrupt\n");
	}

	priv->sd->internal_ops = &adsd3500_subdev_internal_ops;
	priv->sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->sd->entity.ops = &adsd3500_subdev_entity_ops;
	ret = tegra_media_entity_init(&priv->sd->entity, 1,
				&priv->pad, true, true);
	if (ret) {
		dev_err(&client->dev, "unable to init media entity\n");
		return ret;
	}
#endif

#if defined(CONFIG_V4L2_ASYNC)
	ret = v4l2_async_register_subdev(priv->sd);
	if (ret) {
		dev_err(&client->dev, "could not register v4l2 device\n");
		return ret;
	}
#else
	dev_err(&client->dev, "CONFIG_V4L2_ASYNC not enabled!\n");
	return -ENOTSUPP;
#endif

	dev_info(&client->dev, "probe ADSD3500 success\n");

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void
adsd3500_remove(struct i2c_client *client)
#else
static int adsd3500_remove(struct i2c_client *client)
#endif
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct adsd3500 *priv;

	if (!s_data)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		return;
#else
	return -EINVAL;
#endif

	priv = (struct adsd3500 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->sd);
	media_entity_cleanup(&priv->sd->entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	camera_common_cleanup(priv->s_data);
	mutex_destroy(&priv->lock);
	proc_remove(priv->proc_file);
	proc_remove(priv->proc_dir);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static const struct i2c_device_id adsd3500_id[] = {
	{ "adsd3500", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, adsd3500_id);

static struct i2c_driver adsd3500_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(adsd3500_of_match),
		.name		= "adsd3500",
	},
	.probe			= adsd3500_probe,
	.remove			= adsd3500_remove,
	.id_table 		= adsd3500_id,
};

module_i2c_driver(adsd3500_i2c_driver);

MODULE_DESCRIPTION("Analog Devices ADSD3500 Driver");
MODULE_AUTHOR("Bogdan Togorean");
MODULE_AUTHOR("Sivasubramaniyan Padmanaban");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
