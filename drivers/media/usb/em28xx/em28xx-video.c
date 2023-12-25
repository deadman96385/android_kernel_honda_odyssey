/*
   em28xx-video.c - driver for Empia EM2800/EM2820/2840 USB
		    video capture devices

   Copyright (C) 2005 Ludovico Cavedon <cavedon@sssup.it>
		      Markus Rechberger <mrechberger@gmail.com>
		      Mauro Carvalho Chehab <mchehab@infradead.org>
		      Sascha Sommer <saschasommer@freenet.de>
   Copyright (C) 2012 Frank Schäfer <fschaefer.oss@googlemail.com>

	Some parts based on SN9C10x PC Camera Controllers GPL driver made
		by Luca Risolia <luca.risolia@studio.unibo.it>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/bitmap.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "em28xx.h"
#include "em28xx-v4l.h"
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-clk.h>
#include <media/msp3400.h>
#include <media/tuner.h>

#define DRIVER_AUTHOR "Ludovico Cavedon <cavedon@sssup.it>, " \
		      "Markus Rechberger <mrechberger@gmail.com>, " \
		      "Mauro Carvalho Chehab <mchehab@infradead.org>, " \
		      "Sascha Sommer <saschasommer@freenet.de>"

static unsigned int isoc_debug;
module_param(isoc_debug, int, 0644);
MODULE_PARM_DESC(isoc_debug, "enable debug messages [isoc transfers]");

static unsigned int disable_vbi;
module_param(disable_vbi, int, 0644);
MODULE_PARM_DESC(disable_vbi, "disable vbi support");

static int alt;
module_param(alt, int, 0644);
MODULE_PARM_DESC(alt, "alternate setting to use for video endpoint");

#define em28xx_videodbg(fmt, arg...) do {\
	if (video_debug) \
		printk(KERN_INFO "%s %s :"fmt, \
			 dev->name, __func__ , ##arg); } while (0)

#define em28xx_isocdbg(fmt, arg...) \
do {\
	if (isoc_debug) { \
		printk(KERN_INFO "%s %s :"fmt, \
			 dev->name, __func__ , ##arg); \
	} \
  } while (0)

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC " - v4l2 interface");
MODULE_LICENSE("GPL");
MODULE_VERSION(EM28XX_VERSION);


#define EM25XX_FRMDATAHDR_BYTE1			0x02
#define EM25XX_FRMDATAHDR_BYTE2_STILL_IMAGE	0x20
#define EM25XX_FRMDATAHDR_BYTE2_FRAME_END	0x02
#define EM25XX_FRMDATAHDR_BYTE2_FRAME_ID	0x01
#define EM25XX_FRMDATAHDR_BYTE2_MASK	(EM25XX_FRMDATAHDR_BYTE2_STILL_IMAGE | \
					 EM25XX_FRMDATAHDR_BYTE2_FRAME_END |   \
					 EM25XX_FRMDATAHDR_BYTE2_FRAME_ID)


static unsigned int video_nr[] = {[0 ... (EM28XX_MAXBOARDS - 1)] = -1U };
static unsigned int vbi_nr[]   = {[0 ... (EM28XX_MAXBOARDS - 1)] = -1U };
static unsigned int radio_nr[] = {[0 ... (EM28XX_MAXBOARDS - 1)] = -1U };

module_param_array(video_nr, int, NULL, 0444);
module_param_array(vbi_nr, int, NULL, 0444);
module_param_array(radio_nr, int, NULL, 0444);
MODULE_PARM_DESC(video_nr, "video device numbers");
MODULE_PARM_DESC(vbi_nr,   "vbi device numbers");
MODULE_PARM_DESC(radio_nr, "radio device numbers");

static unsigned int video_debug;
module_param(video_debug, int, 0644);
MODULE_PARM_DESC(video_debug, "enable debug messages [video]");

/* supported video standards */
static struct em28xx_fmt format[] = {
	{
		.name     = "16 bpp YUY2, 4:2:2, packed",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.depth    = 16,
		.reg	  = EM28XX_OUTFMT_YUV422_Y0UY1V,
	}, {
		.name     = "16 bpp RGB 565, LE",
		.fourcc   = V4L2_PIX_FMT_RGB565,
		.depth    = 16,
		.reg      = EM28XX_OUTFMT_RGB_16_656,
	}, {
		.name     = "8 bpp Bayer BGBG..GRGR",
		.fourcc   = V4L2_PIX_FMT_SBGGR8,
		.depth    = 8,
		.reg      = EM28XX_OUTFMT_RGB_8_BGBG,
	}, {
		.name     = "8 bpp Bayer GRGR..BGBG",
		.fourcc   = V4L2_PIX_FMT_SGRBG8,
		.depth    = 8,
		.reg      = EM28XX_OUTFMT_RGB_8_GRGR,
	}, {
		.name     = "8 bpp Bayer GBGB..RGRG",
		.fourcc   = V4L2_PIX_FMT_SGBRG8,
		.depth    = 8,
		.reg      = EM28XX_OUTFMT_RGB_8_GBGB,
	}, {
		.name     = "12 bpp YUV411",
		.fourcc   = V4L2_PIX_FMT_YUV411P,
		.depth    = 12,
		.reg      = EM28XX_OUTFMT_YUV411,
	},
};

static int em28xx_vbi_supported(struct em28xx *dev)
{
	/* Modprobe option to manually disable */
	if (disable_vbi == 1)
		return 0;

	if (dev->board.is_webcam)
		return 0;

	/* FIXME: check subdevices for VBI support */

	if (dev->chip_id == CHIP_ID_EM2860 ||
	    dev->chip_id == CHIP_ID_EM2883)
		return 1;

	/* Version of em28xx that does not support VBI */
	return 0;
}

/*
 * em28xx_wake_i2c()
 * configure i2c attached devices
 */
static void em28xx_wake_i2c(struct em28xx *dev)
{
	v4l2_device_call_all(&dev->v4l2_dev, 0, core,  reset, 0);
	v4l2_device_call_all(&dev->v4l2_dev, 0, video, s_routing,
			INPUT(dev->ctl_input)->vmux, 0, 0);
	v4l2_device_call_all(&dev->v4l2_dev, 0, video, s_stream, 0);
}

static int em28xx_colorlevels_set_default(struct em28xx *dev)
{
	em28xx_write_reg(dev, EM28XX_R20_YGAIN, CONTRAST_DEFAULT);
	em28xx_write_reg(dev, EM28XX_R21_YOFFSET, BRIGHTNESS_DEFAULT);
	em28xx_write_reg(dev, EM28XX_R22_UVGAIN, SATURATION_DEFAULT);
	em28xx_write_reg(dev, EM28XX_R23_UOFFSET, BLUE_BALANCE_DEFAULT);
	em28xx_write_reg(dev, EM28XX_R24_VOFFSET, RED_BALANCE_DEFAULT);
	em28xx_write_reg(dev, EM28XX_R25_SHARPNESS, SHARPNESS_DEFAULT);

	em28xx_write_reg(dev, EM28XX_R14_GAMMA, 0x20);
	em28xx_write_reg(dev, EM28XX_R15_RGAIN, 0x20);
	em28xx_write_reg(dev, EM28XX_R16_GGAIN, 0x20);
	em28xx_write_reg(dev, EM28XX_R17_BGAIN, 0x20);
	em28xx_write_reg(dev, EM28XX_R18_ROFFSET, 0x00);
	em28xx_write_reg(dev, EM28XX_R19_GOFFSET, 0x00);
	return em28xx_write_reg(dev, EM28XX_R1A_BOFFSET, 0x00);
}

static int em28xx_set_outfmt(struct em28xx *dev)
{
	int ret;
	u8 fmt, vinctrl;

	fmt = dev->format->reg;
	if (!dev->is_em25xx)
		fmt |= 0x20;
	/*
	 * NOTE: it's not clear if this is really needed !
	 * The datasheets say bit 5 is a reserved bit and devices seem to work
	 * fine without it. But the Windows driver sets it for em2710/50+em28xx
	 * devices and we've always been setting it, too.
	 *
	 * em2765 (em25xx, em276x/7x/8x) devices do NOT work with this bit set,
	 * it's likely used for an additional (compressed ?) format there.
	 */
	ret = em28xx_write_reg(dev, EM28XX_R27_OUTFMT, fmt);
	if (ret < 0)
		return ret;

	ret = em28xx_write_reg(dev, EM28XX_R10_VINMODE, dev->vinmode);
	if (ret < 0)
		return ret;

	vinctrl = dev->vinctl;
	if (em28xx_vbi_supported(dev) == 1) {
		vinctrl |= EM28XX_VINCTRL_VBI_RAW;
		em28xx_write_reg(dev, EM28XX_R34_VBI_START_H, 0x00);
		em28xx_write_reg(dev, EM28XX_R36_VBI_WIDTH, dev->vbi_width/4);
		em28xx_write_reg(dev, EM28XX_R37_VBI_HEIGHT, dev->vbi_height);
		if (dev->norm & V4L2_STD_525_60) {
			/* NTSC */
			em28xx_write_reg(dev, EM28XX_R35_VBI_START_V, 0x09);
		} else if (dev->norm & V4L2_STD_625_50) {
			/* PAL */
			em28xx_write_reg(dev, EM28XX_R35_VBI_START_V, 0x07);
		}
	}

	return em28xx_write_reg(dev, EM28XX_R11_VINCTRL, vinctrl);
}

static int em28xx_accumulator_set(struct em28xx *dev, u8 xmin, u8 xmax,
				  u8 ymin, u8 ymax)
{
	em28xx_videodbg("em28xx Scale: (%d,%d)-(%d,%d)\n",
			xmin, ymin, xmax, ymax);

	em28xx_write_regs(dev, EM28XX_R28_XMIN, &xmin, 1);
	em28xx_write_regs(dev, EM28XX_R29_XMAX, &xmax, 1);
	em28xx_write_regs(dev, EM28XX_R2A_YMIN, &ymin, 1);
	return em28xx_write_regs(dev, EM28XX_R2B_YMAX, &ymax, 1);
}

static void em28xx_capture_area_set(struct em28xx *dev, u8 hstart, u8 vstart,
				   u16 width, u16 height)
{
	u8 cwidth = width >> 2;
	u8 cheight = height >> 2;
	u8 overflow = (height >> 9 & 0x02) | (width >> 10 & 0x01);
	/* NOTE: size limit: 2047x1023 = 2MPix */

	em28xx_videodbg("capture area set to (%d,%d): %dx%d\n",
		       hstart, vstart,
		       ((overflow & 2) << 9 | cwidth << 2),
		       ((overflow & 1) << 10 | cheight << 2));

	em28xx_write_regs(dev, EM28XX_R1C_HSTART, &hstart, 1);
	em28xx_write_regs(dev, EM28XX_R1D_VSTART, &vstart, 1);
	em28xx_write_regs(dev, EM28XX_R1E_CWIDTH, &cwidth, 1);
	em28xx_write_regs(dev, EM28XX_R1F_CHEIGHT, &cheight, 1);
	em28xx_write_regs(dev, EM28XX_R1B_OFLOW, &overflow, 1);

	/* FIXME: function/meaning of these registers ? */
	/* FIXME: align width+height to multiples of 4 ?! */
	if (dev->is_em25xx) {
		em28xx_write_reg(dev, 0x34, width >> 4);
		em28xx_write_reg(dev, 0x35, height >> 4);
	}
}

static int em28xx_scaler_set(struct em28xx *dev, u16 h, u16 v)
{
	u8 mode;
	/* the em2800 scaler only supports scaling down to 50% */

	if (dev->board.is_em2800) {
		mode = (v ? 0x20 : 0x00) | (h ? 0x10 : 0x00);
	} else {
		u8 buf[2];

		buf[0] = h;
		buf[1] = h >> 8;
		em28xx_write_regs(dev, EM28XX_R30_HSCALELOW, (char *)buf, 2);

		buf[0] = v;
		buf[1] = v >> 8;
		em28xx_write_regs(dev, EM28XX_R32_VSCALELOW, (char *)buf, 2);
		/* it seems that both H and V scalers must be active
		   to work correctly */
		mode = (h || v) ? 0x30 : 0x00;
	}
	return em28xx_write_reg_bits(dev, EM28XX_R26_COMPR, mode, 0x30);
}

/* FIXME: this only function read values from dev */
static int em28xx_resolution_set(struct em28xx *dev)
{
	int width, height;
	width = norm_maxw(dev);
	height = norm_maxh(dev);

	/* Properly setup VBI */
	dev->vbi_width = 720;
	if (dev->norm & V4L2_STD_525_60)
		dev->vbi_height = 12;
	else
		dev->vbi_height = 18;

	em28xx_set_outfmt(dev);

	em28xx_accumulator_set(dev, 1, (width - 4) >> 2, 1, (height - 4) >> 2);

	/* If we don't set the start position to 2 in VBI mode, we end up
	   with line 20/21 being YUYV encoded instead of being in 8-bit
	   greyscale.  The core of the issue is that line 21 (and line 23 for
	   PAL WSS) are inside of active video region, and as a result they
	   get the pixelformatting associated with that area.  So by cropping
	   it out, we end up with the same format as the rest of the VBI
	   region */
	if (em28xx_vbi_supported(dev) == 1)
		em28xx_capture_area_set(dev, 0, 2, width, height);
	else
		em28xx_capture_area_set(dev, 0, 0, width, height);

	return em28xx_scaler_set(dev, dev->hscale, dev->vscale);
}

/* Set USB alternate setting for analog video */
static int em28xx_set_alternate(struct em28xx *dev)
{
	int errCode;
	int i;
	unsigned int min_pkt_size = dev->width * 2 + 4;

	/* NOTE: for isoc transfers, only alt settings > 0 are allowed
		 bulk transfers seem to work only with alt=0 ! */
	dev->alt = 0;
	if ((alt > 0) && (alt < dev->num_alt)) {
		em28xx_videodbg("alternate forced to %d\n", dev->alt);
		dev->alt = alt;
		goto set_alt;
	}
	if (dev->analog_xfer_bulk)
		goto set_alt;

	/* When image size is bigger than a certain value,
	   the frame size should be increased, otherwise, only
	   green screen will be received.
	 */
	if (dev->width * 2 * dev->height > 720 * 240 * 2)
		min_pkt_size *= 2;

	for (i = 0; i < dev->num_alt; i++) {
		/* stop when the selected alt setting offers enough bandwidth */
		if (dev->alt_max_pkt_size_isoc[i] >= min_pkt_size) {
			dev->alt = i;
			break;
		/* otherwise make sure that we end up with the maximum bandwidth
		   because the min_pkt_size equation might be wrong...
		*/
		} else if (dev->alt_max_pkt_size_isoc[i] >
			   dev->alt_max_pkt_size_isoc[dev->alt])
			dev->alt = i;
	}

set_alt:
	/* NOTE: for bulk transfers, we need to call usb_set_interface()
	 * even if the previous settings were the same. Otherwise streaming
	 * fails with all urbs having status = -EOVERFLOW ! */
	if (dev->analog_xfer_bulk) {
		dev->max_pkt_size = 512; /* USB 2.0 spec */
		dev->packet_multiplier = EM28XX_BULK_PACKET_MULTIPLIER;
	} else { /* isoc */
		em28xx_videodbg("minimum isoc packet size: %u (alt=%d)\n",
			       min_pkt_size, dev->alt);
		dev->max_pkt_size =
				  dev->alt_max_pkt_size_isoc[dev->alt];
		dev->packet_multiplier = EM28XX_NUM_ISOC_PACKETS;
	}
	em28xx_videodbg("setting alternate %d with wMaxPacketSize=%u\n",
		       dev->alt, dev->max_pkt_size);
	errCode = usb_set_interface(dev->udev, dev->ifnum, dev->alt);
	if (errCode < 0) {
		em28xx_errdev("cannot change alternate number to %d (error=%i)\n",
			      dev->alt, errCode);
		return errCode;
	}
	return 0;
}

/* ------------------------------------------------------------------
	DMA and thread functions
   ------------------------------------------------------------------*/

/*
 * Finish the current buffer
 */
static inline void finish_buffer(struct em28xx *dev,
				 struct em28xx_buffer *buf)
{
	em28xx_isocdbg("[%p/%d] wakeup\n", buf, buf->top_field);

	buf->vb.v4l2_buf.sequence = dev->field_count++;
	buf->vb.v4l2_buf.field = V4L2_FIELD_INTERLACED;
	v4l2_get_timestamp(&buf->vb.v4l2_buf.timestamp);

	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
}

/*
 * Copy picture data from USB buffer to videobuf buffer
 */
static void em28xx_copy_video(struct em28xx *dev,
			      struct em28xx_buffer *buf,
			      unsigned char *usb_buf,
			      unsigned long len)
{
	void *fieldstart, *startwrite, *startread;
	int  linesdone, currlinedone, offset, lencopy, remain;
	int bytesperline = dev->width << 1;

	if (buf->pos + len > buf->length)
		len = buf->length - buf->pos;

	startread = usb_buf;
	remain = len;

	if (dev->progressive || buf->top_field)
		fieldstart = buf->vb_buf;
	else /* interlaced mode, even nr. of lines */
		fieldstart = buf->vb_buf + bytesperline;

	linesdone = buf->pos / bytesperline;
	currlinedone = buf->pos % bytesperline;

	if (dev->progressive)
		offset = linesdone * bytesperline + currlinedone;
	else
		offset = linesdone * bytesperline * 2 + currlinedone;

	startwrite = fieldstart + offset;
	lencopy = bytesperline - currlinedone;
	lencopy = lencopy > remain ? remain : lencopy;

	if ((char *)startwrite + lencopy > (char *)buf->vb_buf + buf->length) {
		em28xx_isocdbg("Overflow of %zi bytes past buffer end (1)\n",
			      ((char *)startwrite + lencopy) -
			      ((char *)buf->vb_buf + buf->length));
		remain = (char *)buf->vb_buf + buf->length -
			 (char *)startwrite;
		lencopy = remain;
	}
	if (lencopy <= 0)
		return;
	memcpy(startwrite, startread, lencopy);

	remain -= lencopy;

	while (remain > 0) {
		if (dev->progressive)
			startwrite += lencopy;
		else
			startwrite += lencopy + bytesperline;
		startread += lencopy;
		if (bytesperline > remain)
			lencopy = remain;
		else
			lencopy = bytesperline;

		if ((char *)startwrite + lencopy > (char *)buf->vb_buf +
		    buf->length) {
			em28xx_isocdbg("Overflow of %zi bytes past buffer end"
				       "(2)\n",
				       ((char *)startwrite + lencopy) -
				       ((char *)buf->vb_buf + buf->length));
			lencopy = remain = (char *)buf->vb_buf + buf->length -
				(char *)startwrite;
		}
		if (lencopy <= 0)
			break;

		memcpy(startwrite, startread, lencopy);

		remain -= lencopy;
	}

	buf->pos += len;
}

/*
 * Copy VBI data from USB buffer to videobuf buffer
 */
static void em28xx_copy_vbi(struct em28xx *dev,
			    struct em28xx_buffer *buf,
			    unsigned char *usb_buf,
			    unsigned long len)
{
	unsigned int offset;

	if (buf->pos + len > buf->length)
		len = buf->length - buf->pos;

	offset = buf->pos;
	/* Make sure the bottom field populates the second half of the frame */
	if (buf->top_field == 0)
		offset += dev->vbi_width * dev->vbi_height;

	memcpy(buf->vb_buf + offset, usb_buf, len);
	buf->pos += len;
}

static inline void print_err_status(struct em28xx *dev,
				     int packet, int status)
{
	char *errmsg = "Unknown";

	switch (status) {
	case -ENOENT:
		errmsg = "unlinked synchronuously";
		break;
	case -ECONNRESET:
		errmsg = "unlinked asynchronuously";
		break;
	case -ENOSR:
		errmsg = "Buffer error (overrun)";
		break;
	case -EPIPE:
		errmsg = "Stalled (device not responding)";
		break;
	case -EOVERFLOW:
		errmsg = "Babble (bad cable?)";
		break;
	case -EPROTO:
		errmsg = "Bit-stuff error (bad cable?)";
		break;
	case -EILSEQ:
		errmsg = "CRC/Timeout (could be anything)";
		break;
	case -ETIME:
		errmsg = "Device does not respond";
		break;
	}
	if (packet < 0) {
		em28xx_isocdbg("URB status %d [%s].\n",	status, errmsg);
	} else {
		em28xx_isocdbg("URB packet %d, status %d [%s].\n",
			       packet, status, errmsg);
	}
}

/*
 * get the next available buffer from dma queue
 */
static inline struct em28xx_buffer *get_next_buf(struct em28xx *dev,
						 struct em28xx_dmaqueue *dma_q)
{
	struct em28xx_buffer *buf;

	if (list_empty(&dma_q->active)) {
		em28xx_isocdbg("No active queue to serve\n");
		return NULL;
	}

	/* Get the next buffer */
	buf = list_entry(dma_q->active.next, struct em28xx_buffer, list);
	/* Cleans up buffer - Useful for testing for frame/URB loss */
	list_del(&buf->list);
	buf->pos = 0;
	buf->vb_buf = buf->mem;

	return buf;
}

/*
 * Finish the current buffer if completed and prepare for the next field
 */
static struct em28xx_buffer *
finish_field_prepare_next(struct em28xx *dev,
			  struct em28xx_buffer *buf,
			  struct em28xx_dmaqueue *dma_q)
{
	if (dev->progressive || dev->top_field) { /* Brand new frame */
		if (buf != NULL)
			finish_buffer(dev, buf);
		buf = get_next_buf(dev, dma_q);
	}
	if (buf != NULL) {
		buf->top_field = dev->top_field;
		buf->pos = 0;
	}

	return buf;
}

/*
 * Process data packet according to the em2710/em2750/em28xx frame data format
 */
static inline void process_frame_data_em28xx(struct em28xx *dev,
					     unsigned char *data_pkt,
					     unsigned int  data_len)
{
	struct em28xx_buffer    *buf = dev->usb_ctl.vid_buf;
	struct em28xx_buffer    *vbi_buf = dev->usb_ctl.vbi_buf;
	struct em28xx_dmaqueue  *dma_q = &dev->vidq;
	struct em28xx_dmaqueue  *vbi_dma_q = &dev->vbiq;

	/* capture type 0 = vbi start
	   capture type 1 = vbi in progress
	   capture type 2 = video start
	   capture type 3 = video in progress */
	if (data_len >= 4) {
		/* NOTE: Headers are always 4 bytes and
		 * never split across packets */
		if (data_pkt[0] == 0x88 && data_pkt[1] == 0x88 &&
		    data_pkt[2] == 0x88 && data_pkt[3] == 0x88) {
			/* Continuation */
			data_pkt += 4;
			data_len -= 4;
		} else if (data_pkt[0] == 0x33 && data_pkt[1] == 0x95) {
			/* Field start (VBI mode) */
			dev->capture_type = 0;
			dev->vbi_read = 0;
			em28xx_isocdbg("VBI START HEADER !!!\n");
			dev->top_field = !(data_pkt[2] & 1);
			data_pkt += 4;
			data_len -= 4;
		} else if (data_pkt[0] == 0x22 && data_pkt[1] == 0x5a) {
			/* Field start (VBI disabled) */
			dev->capture_type = 2;
			em28xx_isocdbg("VIDEO START HEADER !!!\n");
			dev->top_field = !(data_pkt[2] & 1);
			data_pkt += 4;
			data_len -= 4;
		}
	}
	/* NOTE: With bulk transfers, intermediate data packets
	 * have no continuation header */

	if (dev->capture_type == 0) {
		vbi_buf = finish_field_prepare_next(dev, vbi_buf, vbi_dma_q);
		dev->usb_ctl.vbi_buf = vbi_buf;
		dev->capture_type = 1;
	}

	if (dev->capture_type == 1) {
		int vbi_size = dev->vbi_width * dev->vbi_height;
		int vbi_data_len = ((dev->vbi_read + data_len) > vbi_size) ?
				   (vbi_size - dev->vbi_read) : data_len;

		/* Copy VBI data */
		if (vbi_buf != NULL)
			em28xx_copy_vbi(dev, vbi_buf, data_pkt, vbi_data_len);
		dev->vbi_read += vbi_data_len;

		if (vbi_data_len < data_len) {
			/* Continue with copying video data */
			dev->capture_type = 2;
			data_pkt += vbi_data_len;
			data_len -= vbi_data_len;
		}
	}

	if (dev->capture_type == 2) {
		buf = finish_field_prepare_next(dev, buf, dma_q);
		dev->usb_ctl.vid_buf = buf;
		dev->capture_type = 3;
	}

	if (dev->capture_type == 3 && buf != NULL && data_len > 0)
		em28xx_copy_video(dev, buf, data_pkt, data_len);
}

/*
 * Process data packet according to the em25xx/em276x/7x/8x frame data format
 */
static inline void process_frame_data_em25xx(struct em28xx *dev,
					     unsigned char *data_pkt,
					     unsigned int  data_len)
{
	struct em28xx_buffer    *buf = dev->usb_ctl.vid_buf;
	struct em28xx_dmaqueue  *dmaq = &dev->vidq;
	bool frame_end = 0;

	/* Check for header */
	/* NOTE: at least with bulk transfers, only the first packet
	 * has a header and has always set the FRAME_END bit         */
	if (data_len >= 2) {	/* em25xx header is only 2 bytes long */
		if ((data_pkt[0] == EM25XX_FRMDATAHDR_BYTE1) &&
		    ((data_pkt[1] & ~EM25XX_FRMDATAHDR_BYTE2_MASK) == 0x00)) {
			dev->top_field = !(data_pkt[1] &
					   EM25XX_FRMDATAHDR_BYTE2_FRAME_ID);
			frame_end = data_pkt[1] &
				    EM25XX_FRMDATAHDR_BYTE2_FRAME_END;
			data_pkt += 2;
			data_len -= 2;
		}

		/* Finish field and prepare next (BULK only) */
		if (dev->analog_xfer_bulk && frame_end) {
			buf = finish_field_prepare_next(dev, buf, dmaq);
			dev->usb_ctl.vid_buf = buf;
		}
		/* NOTE: in ISOC mode when a new frame starts and buf==NULL,
		 * we COULD already prepare a buffer here to avoid skipping the
		 * first frame.
		 */
	}

	/* Copy data */
	if (buf != NULL && data_len > 0)
		em28xx_copy_video(dev, buf, data_pkt, data_len);

	/* Finish frame (ISOC only) => avoids lag of 1 frame */
	if (!dev->analog_xfer_bulk && frame_end) {
		buf = finish_field_prepare_next(dev, buf, dmaq);
		dev->usb_ctl.vid_buf = buf;
	}

	/* NOTE: Tested with USB bulk transfers only !
	 * The wording in the datasheet suggests that isoc might work different.
	 * The current code assumes that with isoc transfers each packet has a
	 * header like with the other em28xx devices.
	 */
	/* NOTE: Support for interlaced mode is pure theory. It has not been
	 * tested and it is unknown if these devices actually support it. */
	/* NOTE: No VBI support yet (these chips likely do not support VBI). */
}

/* Processes and copies the URB data content (video and VBI data) */
static inline int em28xx_urb_data_copy(struct em28xx *dev, struct urb *urb)
{
	int xfer_bulk, num_packets, i;
	unsigned char *usb_data_pkt;
	unsigned int usb_data_len;

	if (!dev)
		return 0;

	if (dev->disconnected)
		return 0;

	if (urb->status < 0)
		print_err_status(dev, -1, urb->status);

	xfer_bulk = usb_pipebulk(urb->pipe);

	if (xfer_bulk) /* bulk */
		num_packets = 1;
	else /* isoc */
		num_packets = urb->number_of_packets;

	for (i = 0; i < num_packets; i++) {
		if (xfer_bulk) { /* bulk */
			usb_data_len = urb->actual_length;

			usb_data_pkt = urb->transfer_buffer;
		} else { /* isoc */
			if (urb->iso_frame_desc[i].status < 0) {
				print_err_status(dev, i,
						 urb->iso_frame_desc[i].status);
				if (urb->iso_frame_desc[i].status != -EPROTO)
					continue;
			}

			usb_data_len = urb->iso_frame_desc[i].actual_length;
			if (usb_data_len > dev->max_pkt_size) {
				em28xx_isocdbg("packet bigger than packet size");
				continue;
			}

			usb_data_pkt = urb->transfer_buffer +
				       urb->iso_frame_desc[i].offset;
		}

		if (usb_data_len == 0) {
			/* NOTE: happens very often with isoc transfers */
			/* em28xx_usbdbg("packet %d is empty",i); - spammy */
			continue;
		}

		if (dev->is_em25xx)
			process_frame_data_em25xx(dev,
						  usb_data_pkt, usb_data_len);
		else
			process_frame_data_em28xx(dev,
						  usb_data_pkt, usb_data_len);

	}
	return 1;
}


static int get_ressource(enum v4l2_buf_type f_type)
{
	switch (f_type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return EM28XX_RESOURCE_VIDEO;
	case V4L2_BUF_TYPE_VBI_CAPTURE:
		return EM28XX_RESOURCE_VBI;
	default:
		BUG();
		return 0;
	}
}

/* Usage lock check functions */
static int res_get(struct em28xx *dev, enum v4l2_buf_type f_type)
{
	int res_type = get_ressource(f_type);

	/* is it free? */
	if (dev->resources & res_type) {
		/* no, someone else uses it */
		return -EBUSY;
	}

	/* it's free, grab it */
	dev->resources |= res_type;
	em28xx_videodbg("res: get %d\n", res_type);
	return 0;
}

static void res_free(struct em28xx *dev, enum v4l2_buf_type f_type)
{
	int res_type = get_ressource(f_type);

	dev->resources &= ~res_type;
	em28xx_videodbg("res: put %d\n", res_type);
}

/* ------------------------------------------------------------------
	Videobuf2 operations
   ------------------------------------------------------------------*/

static int queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		       unsigned int *nbuffers, unsigned int *nplanes,
		       unsigned int sizes[], void *alloc_ctxs[])
{
	struct em28xx *dev = vb2_get_drv_priv(vq);
	unsigned long size;

	if (fmt)
		size = fmt->fmt.pix.sizeimage;
	else
		size = (dev->width * dev->height * dev->format->depth + 7) >> 3;

	if (size == 0)
		return -EINVAL;

	if (0 == *nbuffers)
		*nbuffers = 32;

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int
buffer_prepare(struct vb2_buffer *vb)
{
	struct em28xx        *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct em28xx_buffer *buf = container_of(vb, struct em28xx_buffer, vb);
	unsigned long size;

	em28xx_videodbg("%s, field=%d\n", __func__, vb->v4l2_buf.field);

	size = (dev->width * dev->height * dev->format->depth + 7) >> 3;

	if (vb2_plane_size(vb, 0) < size) {
		em28xx_videodbg("%s data will not fit into plane (%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	vb2_set_plane_payload(&buf->vb, 0, size);

	return 0;
}

int em28xx_start_analog_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct em28xx *dev = vb2_get_drv_priv(vq);
	struct v4l2_frequency f;
	int rc = 0;

	em28xx_videodbg("%s\n", __func__);

	/* Make sure streaming is not already in progress for this type
	   of filehandle (e.g. video, vbi) */
	rc = res_get(dev, vq->type);
	if (rc)
		return rc;

	if (dev->streaming_users == 0) {
		/* First active streaming user, so allocate all the URBs */

		/* Allocate the USB bandwidth */
		em28xx_set_alternate(dev);

		/* Needed, since GPIO might have disabled power of
		   some i2c device
		*/
		em28xx_wake_i2c(dev);

		dev->capture_type = -1;
		rc = em28xx_init_usb_xfer(dev, EM28XX_ANALOG_MODE,
					  dev->analog_xfer_bulk,
					  EM28XX_NUM_BUFS,
					  dev->max_pkt_size,
					  dev->packet_multiplier,
					  em28xx_urb_data_copy);
		if (rc < 0)
			return rc;

		/*
		 * djh: it's not clear whether this code is still needed.  I'm
		 * leaving it in here for now entirely out of concern for
		 * backward compatibility (the old code did it)
		 */

		/* Ask tuner to go to analog or radio mode */
		memset(&f, 0, sizeof(f));
		f.frequency = dev->ctl_freq;
		if (vq->owner && vq->owner->vdev->vfl_type == VFL_TYPE_RADIO)
			f.type = V4L2_TUNER_RADIO;
		else
			f.type = V4L2_TUNER_ANALOG_TV;
		v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_frequency, &f);
	}

	dev->streaming_users++;

	return rc;
}

static int em28xx_stop_streaming(struct vb2_queue *vq)
{
	struct em28xx *dev = vb2_get_drv_priv(vq);
	struct em28xx_dmaqueue *vidq = &dev->vidq;
	unsigned long flags = 0;

	em28xx_videodbg("%s\n", __func__);

	res_free(dev, vq->type);

	if (dev->streaming_users-- == 1) {
		/* Last active user, so shutdown all the URBS */
		em28xx_uninit_usb_xfer(dev, EM28XX_ANALOG_MODE);
	}

	spin_lock_irqsave(&dev->slock, flags);
	if (dev->usb_ctl.vid_buf != NULL) {
		vb2_buffer_done(&dev->usb_ctl.vid_buf->vb, VB2_BUF_STATE_ERROR);
		dev->usb_ctl.vid_buf = NULL;
	}
	while (!list_empty(&vidq->active)) {
		struct em28xx_buffer *buf;
		buf = list_entry(vidq->active.next, struct em28xx_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&dev->slock, flags);

	return 0;
}

int em28xx_stop_vbi_streaming(struct vb2_queue *vq)
{
	struct em28xx *dev = vb2_get_drv_priv(vq);
	struct em28xx_dmaqueue *vbiq = &dev->vbiq;
	unsigned long flags = 0;

	em28xx_videodbg("%s\n", __func__);

	res_free(dev, vq->type);

	if (dev->streaming_users-- == 1) {
		/* Last active user, so shutdown all the URBS */
		em28xx_uninit_usb_xfer(dev, EM28XX_ANALOG_MODE);
	}

	spin_lock_irqsave(&dev->slock, flags);
	if (dev->usb_ctl.vbi_buf != NULL) {
		vb2_buffer_done(&dev->usb_ctl.vbi_buf->vb, VB2_BUF_STATE_ERROR);
		dev->usb_ctl.vbi_buf = NULL;
	}
	while (!list_empty(&vbiq->active)) {
		struct em28xx_buffer *buf;
		buf = list_entry(vbiq->active.next, struct em28xx_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&dev->slock, flags);

	return 0;
}

static void
buffer_queue(struct vb2_buffer *vb)
{
	struct em28xx *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct em28xx_buffer *buf = container_of(vb, struct em28xx_buffer, vb);
	struct em28xx_dmaqueue *vidq = &dev->vidq;
	unsigned long flags = 0;

	em28xx_videodbg("%s\n", __func__);
	buf->mem = vb2_plane_vaddr(vb, 0);
	buf->length = vb2_plane_size(vb, 0);

	spin_lock_irqsave(&dev->slock, flags);
	list_add_tail(&buf->list, &vidq->active);
	spin_unlock_irqrestore(&dev->slock, flags);
}

static struct vb2_ops em28xx_video_qops = {
	.queue_setup    = queue_setup,
	.buf_prepare    = buffer_prepare,
	.buf_queue      = buffer_queue,
	.start_streaming = em28xx_start_analog_streaming,
	.stop_streaming = em28xx_stop_streaming,
	.wait_prepare   = vb2_ops_wait_prepare,
	.wait_finish    = vb2_ops_wait_finish,
};

static int em28xx_vb2_setup(struct em28xx *dev)
{
	int rc;
	struct vb2_queue *q;

	/* Setup Videobuf2 for Video capture */
	q = &dev->vb_vidq;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_READ | VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->drv_priv = dev;
	q->buf_struct_size = sizeof(struct em28xx_buffer);
	q->ops = &em28xx_video_qops;
	q->mem_ops = &vb2_vmalloc_memops;

	rc = vb2_queue_init(q);
	if (rc < 0)
		return rc;

	/* Setup Videobuf2 for VBI capture */
	q = &dev->vb_vbiq;
	q->type = V4L2_BUF_TYPE_VBI_CAPTURE;
	q->io_modes = VB2_READ | VB2_MMAP | VB2_USERPTR;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->drv_priv = dev;
	q->buf_struct_size = sizeof(struct em28xx_buffer);
	q->ops = &em28xx_vbi_qops;
	q->mem_ops = &vb2_vmalloc_memops;

	rc = vb2_queue_init(q);
	if (rc < 0)
		return rc;

	return 0;
}

/*********************  v4l2 interface  **************************************/

static void video_mux(struct em28xx *dev, int index)
{
	dev->ctl_input = index;
	dev->ctl_ainput = INPUT(index)->amux;
	dev->ctl_aoutput = INPUT(index)->aout;

	if (!dev->ctl_aoutput)
		dev->ctl_aoutput = EM28XX_AOUT_MASTER;

	v4l2_device_call_all(&dev->v4l2_dev, 0, video, s_routing,
			INPUT(index)->vmux, 0, 0);

	if (dev->board.has_msp34xx) {
		if (dev->i2s_speed) {
			v4l2_device_call_all(&dev->v4l2_dev, 0, audio,
				s_i2s_clock_freq, dev->i2s_speed);
		}
		/* Note: this is msp3400 specific */
		v4l2_device_call_all(&dev->v4l2_dev, 0, audio, s_routing,
			 dev->ctl_ainput, MSP_OUTPUT(MSP_SC_IN_DSP_SCART1), 0);
	}

	if (dev->board.adecoder != EM28XX_NOADECODER) {
		v4l2_device_call_all(&dev->v4l2_dev, 0, audio, s_routing,
			dev->ctl_ainput, dev->ctl_aoutput, 0);
	}

	em28xx_audio_analog_set(dev);
}

static void em28xx_ctrl_notify(struct v4l2_ctrl *ctrl, void *priv)
{
	struct em28xx *dev = priv;

	/*
	 * In the case of non-AC97 volume controls, we still need
	 * to do some setups at em28xx, in order to mute/unmute
	 * and to adjust audio volume. However, the value ranges
	 * should be checked by the corresponding V4L subdriver.
	 */
	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		dev->mute = ctrl->val;
		em28xx_audio_analog_set(dev);
		break;
	case V4L2_CID_AUDIO_VOLUME:
		dev->volume = ctrl->val;
		em28xx_audio_analog_set(dev);
		break;
	}
}

static int em28xx_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct em28xx *dev = container_of(ctrl->handler, struct em28xx, ctrl_handler);
	int ret = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		dev->mute = ctrl->val;
		ret = em28xx_audio_analog_set(dev);
		break;
	case V4L2_CID_AUDIO_VOLUME:
		dev->volume = ctrl->val;
		ret = em28xx_audio_analog_set(dev);
		break;
	case V4L2_CID_CONTRAST:
		ret = em28xx_write_reg(dev, EM28XX_R20_YGAIN, ctrl->val);
		break;
	case V4L2_CID_BRIGHTNESS:
		ret = em28xx_write_reg(dev, EM28XX_R21_YOFFSET, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = em28xx_write_reg(dev, EM28XX_R22_UVGAIN, ctrl->val);
		break;
	case V4L2_CID_BLUE_BALANCE:
		ret = em28xx_write_reg(dev, EM28XX_R23_UOFFSET, ctrl->val);
		break;
	case V4L2_CID_RED_BALANCE:
		ret = em28xx_write_reg(dev, EM28XX_R24_VOFFSET, ctrl->val);
		break;
	case V4L2_CID_SHARPNESS:
		ret = em28xx_write_reg(dev, EM28XX_R25_SHARPNESS, ctrl->val);
		break;
	}

	return (ret < 0) ? ret : 0;
}

static const struct v4l2_ctrl_ops em28xx_ctrl_ops = {
	.s_ctrl = em28xx_s_ctrl,
};

static void size_to_scale(struct em28xx *dev,
			unsigned int width, unsigned int height,
			unsigned int *hscale, unsigned int *vscale)
{
	unsigned int          maxw = norm_maxw(dev);
	unsigned int          maxh = norm_maxh(dev);

	*hscale = (((unsigned long)maxw) << 12) / width - 4096L;
	if (*hscale > EM28XX_HVSCALE_MAX)
		*hscale = EM28XX_HVSCALE_MAX;

	*vscale = (((unsigned long)maxh) << 12) / height - 4096L;
	if (*vscale > EM28XX_HVSCALE_MAX)
		*vscale = EM28XX_HVSCALE_MAX;
}

static void scale_to_size(struct em28xx *dev,
			  unsigned int hscale, unsigned int vscale,
			  unsigned int *width, unsigned int *height)
{
	unsigned int          maxw = norm_maxw(dev);
	unsigned int          maxh = norm_maxh(dev);

	*width = (((unsigned long)maxw) << 12) / (hscale + 4096L);
	*height = (((unsigned long)maxh) << 12) / (vscale + 4096L);
}

/* ------------------------------------------------------------------
	IOCTL vidioc handling
   ------------------------------------------------------------------*/

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;

	f->fmt.pix.width = dev->width;
	f->fmt.pix.height = dev->height;
	f->fmt.pix.pixelformat = dev->format->fourcc;
	f->fmt.pix.bytesperline = (dev->width * dev->format->depth + 7) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline  * dev->height;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;

	/* FIXME: TOP? NONE? BOTTOM? ALTENATE? */
	if (dev->progressive)
		f->fmt.pix.field = V4L2_FIELD_NONE;
	else
		f->fmt.pix.field = dev->interlaced ?
			   V4L2_FIELD_INTERLACED : V4L2_FIELD_TOP;
	return 0;
}

static struct em28xx_fmt *format_by_fourcc(unsigned int fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(format); i++)
		if (format[i].fourcc == fourcc)
			return &format[i];

	return NULL;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct em28xx_fh      *fh    = priv;
	struct em28xx         *dev   = fh->dev;
	unsigned int          width  = f->fmt.pix.width;
	unsigned int          height = f->fmt.pix.height;
	unsigned int          maxw   = norm_maxw(dev);
	unsigned int          maxh   = norm_maxh(dev);
	unsigned int          hscale, vscale;
	struct em28xx_fmt     *fmt;

	fmt = format_by_fourcc(f->fmt.pix.pixelformat);
	if (!fmt) {
		em28xx_videodbg("Fourcc format (%08x) invalid.\n",
				f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	if (dev->board.is_em2800) {
		/* the em2800 can only scale down to 50% */
		height = height > (3 * maxh / 4) ? maxh : maxh / 2;
		width = width > (3 * maxw / 4) ? maxw : maxw / 2;
		/*
		 * MaxPacketSize for em2800 is too small to capture at full
		 * resolution use half of maxw as the scaler can only scale
		 * to 50%
		 */
		if (width == maxw && height == maxh)
			width /= 2;
	} else {
		/* width must even because of the YUYV format
		   height must be even because of interlacing */
		v4l_bound_align_image(&width, 48, maxw, 1, &height, 32, maxh,
				      1, 0);
	}

	size_to_scale(dev, width, height, &hscale, &vscale);
	scale_to_size(dev, hscale, vscale, &width, &height);

	f->fmt.pix.width = width;
	f->fmt.pix.height = height;
	f->fmt.pix.pixelformat = fmt->fourcc;
	f->fmt.pix.bytesperline = (width * fmt->depth + 7) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * height;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
	if (dev->progressive)
		f->fmt.pix.field = V4L2_FIELD_NONE;
	else
		f->fmt.pix.field = dev->interlaced ?
			   V4L2_FIELD_INTERLACED : V4L2_FIELD_TOP;
	f->fmt.pix.priv = 0;

	return 0;
}

static int em28xx_set_video_format(struct em28xx *dev, unsigned int fourcc,
				   unsigned width, unsigned height)
{
	struct em28xx_fmt     *fmt;

	fmt = format_by_fourcc(fourcc);
	if (!fmt)
		return -EINVAL;

	dev->format = fmt;
	dev->width  = width;
	dev->height = height;

	/* set new image size */
	size_to_scale(dev, dev->width, dev->height, &dev->hscale, &dev->vscale);

	em28xx_resolution_set(dev);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct em28xx *dev = video_drvdata(file);

	if (dev->streaming_users > 0)
		return -EBUSY;

	vidioc_try_fmt_vid_cap(file, priv, f);

	return em28xx_set_video_format(dev, f->fmt.pix.pixelformat,
				f->fmt.pix.width, f->fmt.pix.height);
}

static int vidioc_g_std(struct file *file, void *priv, v4l2_std_id *norm)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;

	*norm = dev->norm;

	return 0;
}

static int vidioc_querystd(struct file *file, void *priv, v4l2_std_id *norm)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;

	v4l2_device_call_all(&dev->v4l2_dev, 0, video, querystd, norm);

	return 0;
}

static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id norm)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;
	struct v4l2_format f;

	if (norm == dev->norm)
		return 0;

	if (dev->streaming_users > 0)
		return -EBUSY;

	dev->norm = norm;

	/* Adjusts width/height, if needed */
	f.fmt.pix.width = 720;
	f.fmt.pix.height = (norm & V4L2_STD_525_60) ? 480 : 576;
	vidioc_try_fmt_vid_cap(file, priv, &f);

	/* set new image size */
	dev->width = f.fmt.pix.width;
	dev->height = f.fmt.pix.height;
	size_to_scale(dev, dev->width, dev->height, &dev->hscale, &dev->vscale);

	em28xx_resolution_set(dev);
	v4l2_device_call_all(&dev->v4l2_dev, 0, core, s_std, dev->norm);

	return 0;
}

static int vidioc_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *p)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;
	int rc = 0;

	p->parm.capture.readbuffers = EM28XX_MIN_BUF;
	if (dev->board.is_webcam)
		rc = v4l2_device_call_until_err(&dev->v4l2_dev, 0,
						video, g_parm, p);
	else
		v4l2_video_std_frame_period(dev->norm,
						 &p->parm.capture.timeperframe);

	return rc;
}

static int vidioc_s_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *p)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;

	p->parm.capture.readbuffers = EM28XX_MIN_BUF;
	return v4l2_device_call_until_err(&dev->v4l2_dev, 0, video, s_parm, p);
}

static const char *iname[] = {
	[EM28XX_VMUX_COMPOSITE1] = "Composite1",
	[EM28XX_VMUX_COMPOSITE2] = "Composite2",
	[EM28XX_VMUX_COMPOSITE3] = "Composite3",
	[EM28XX_VMUX_COMPOSITE4] = "Composite4",
	[EM28XX_VMUX_SVIDEO]     = "S-Video",
	[EM28XX_VMUX_TELEVISION] = "Television",
	[EM28XX_VMUX_CABLE]      = "Cable TV",
	[EM28XX_VMUX_DVB]        = "DVB",
	[EM28XX_VMUX_DEBUG]      = "for debug only",
};

static int vidioc_enum_input(struct file *file, void *priv,
				struct v4l2_input *i)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;
	unsigned int       n;

	n = i->index;
	if (n >= MAX_EM28XX_INPUT)
		return -EINVAL;
	if (0 == INPUT(n)->type)
		return -EINVAL;

	i->index = n;
	i->type = V4L2_INPUT_TYPE_CAMERA;

	strcpy(i->name, iname[INPUT(n)->type]);

	if ((EM28XX_VMUX_TELEVISION == INPUT(n)->type) ||
		(EM28XX_VMUX_CABLE == INPUT(n)->type))
		i->type = V4L2_INPUT_TYPE_TUNER;

	i->std = dev->vdev->tvnorms;
	/* webcams do not have the STD API */
	if (dev->board.is_webcam)
		i->capabilities = 0;

	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;

	*i = dev->ctl_input;

	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;

	if (i >= MAX_EM28XX_INPUT)
		return -EINVAL;
	if (0 == INPUT(i)->type)
		return -EINVAL;

	video_mux(dev, i);
	return 0;
}

static int vidioc_g_audio(struct file *file, void *priv, struct v4l2_audio *a)
{
	struct em28xx_fh   *fh    = priv;
	struct em28xx      *dev   = fh->dev;

	switch (a->index) {
	case EM28XX_AMUX_VIDEO:
		strcpy(a->name, "Television");
		break;
	case EM28XX_AMUX_LINE_IN:
		strcpy(a->name, "Line In");
		break;
	case EM28XX_AMUX_VIDEO2:
		strcpy(a->name, "Television alt");
		break;
	case EM28XX_AMUX_PHONE:
		strcpy(a->name, "Phone");
		break;
	case EM28XX_AMUX_MIC:
		strcpy(a->name, "Mic");
		break;
	case EM28XX_AMUX_CD:
		strcpy(a->name, "CD");
		break;
	case EM28XX_AMUX_AUX:
		strcpy(a->name, "Aux");
		break;
	case EM28XX_AMUX_PCM_OUT:
		strcpy(a->name, "PCM");
		break;
	default:
		return -EINVAL;
	}

	a->index = dev->ctl_ainput;
	a->capability = V4L2_AUDCAP_STEREO;

	return 0;
}

static int vidioc_s_audio(struct file *file, void *priv, const struct v4l2_audio *a)
{
	struct em28xx_fh   *fh  = priv;
	struct em28xx      *dev = fh->dev;

	if (a->index >= MAX_EM28XX_INPUT)
		return -EINVAL;
	if (0 == INPUT(a->index)->type)
		return -EINVAL;

	dev->ctl_ainput = INPUT(a->index)->amux;
	dev->ctl_aoutput = INPUT(a->index)->aout;

	if (!dev->ctl_aoutput)
		dev->ctl_aoutput = EM28XX_AOUT_MASTER;

	return 0;
}

static int vidioc_g_tuner(struct file *file, void *priv,
				struct v4l2_tuner *t)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;

	if (0 != t->index)
		return -EINVAL;

	strcpy(t->name, "Tuner");

	v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, g_tuner, t);
	return 0;
}

static int vidioc_s_tuner(struct file *file, void *priv,
				const struct v4l2_tuner *t)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;

	if (0 != t->index)
		return -EINVAL;

	v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_tuner, t);
	return 0;
}

static int vidioc_g_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;

	if (0 != f->tuner)
		return -EINVAL;

	f->frequency = dev->ctl_freq;
	return 0;
}

static int vidioc_s_frequency(struct file *file, void *priv,
				const struct v4l2_frequency *f)
{
	struct v4l2_frequency new_freq = *f;
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;

	if (0 != f->tuner)
		return -EINVAL;

	v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_frequency, f);
	v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, g_frequency, &new_freq);
	dev->ctl_freq = new_freq.frequency;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vidioc_g_chip_info(struct file *file, void *priv,
	       struct v4l2_dbg_chip_info *chip)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;

	if (chip->match.addr > 1)
		return -EINVAL;
	if (chip->match.addr == 1)
		strlcpy(chip->name, "ac97", sizeof(chip->name));
	else
		strlcpy(chip->name, dev->v4l2_dev.name, sizeof(chip->name));
	return 0;
}

static int em28xx_reg_len(int reg)
{
	switch (reg) {
	case EM28XX_R40_AC97LSB:
	case EM28XX_R30_HSCALELOW:
	case EM28XX_R32_VSCALELOW:
		return 2;
	default:
		return 1;
	}
}

static int vidioc_g_register(struct file *file, void *priv,
			     struct v4l2_dbg_register *reg)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;
	int ret;

	if (reg->match.addr > 1)
		return -EINVAL;
	if (reg->match.addr) {
		ret = em28xx_read_ac97(dev, reg->reg);
		if (ret < 0)
			return ret;

		reg->val = ret;
		reg->size = 1;
		return 0;
	}

	/* Match host */
	reg->size = em28xx_reg_len(reg->reg);
	if (reg->size == 1) {
		ret = em28xx_read_reg(dev, reg->reg);

		if (ret < 0)
			return ret;

		reg->val = ret;
	} else {
		__le16 val = 0;
		ret = dev->em28xx_read_reg_req_len(dev, USB_REQ_GET_STATUS,
						   reg->reg, (char *)&val, 2);
		if (ret < 0)
			return ret;

		reg->val = le16_to_cpu(val);
	}

	return 0;
}

static int vidioc_s_register(struct file *file, void *priv,
			     const struct v4l2_dbg_register *reg)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;
	__le16 buf;

	if (reg->match.addr > 1)
		return -EINVAL;
	if (reg->match.addr)
		return em28xx_write_ac97(dev, reg->reg, reg->val);

	/* Match host */
	buf = cpu_to_le16(reg->val);

	return em28xx_write_regs(dev, reg->reg, (char *)&buf,
			       em28xx_reg_len(reg->reg));
}
#endif


static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct video_device *vdev = video_devdata(file);
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;

	strlcpy(cap->driver, "em28xx", sizeof(cap->driver));
	strlcpy(cap->card, em28xx_boards[dev->model].name, sizeof(cap->card));
	usb_make_path(dev->udev, cap->bus_info, sizeof(cap->bus_info));

	if (vdev->vfl_type == VFL_TYPE_GRABBER)
		cap->device_caps = V4L2_CAP_READWRITE |
			V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	else if (vdev->vfl_type == VFL_TYPE_RADIO)
		cap->device_caps = V4L2_CAP_RADIO;
	else
		cap->device_caps = V4L2_CAP_READWRITE | V4L2_CAP_VBI_CAPTURE;

	if (dev->audio_mode.has_audio)
		cap->device_caps |= V4L2_CAP_AUDIO;

	if (dev->tuner_type != TUNER_ABSENT)
		cap->device_caps |= V4L2_CAP_TUNER;

	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS |
		V4L2_CAP_READWRITE | V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	if (dev->vbi_dev)
		cap->capabilities |= V4L2_CAP_VBI_CAPTURE;
	if (dev->radio_dev)
		cap->capabilities |= V4L2_CAP_RADIO;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	if (unlikely(f->index >= ARRAY_SIZE(format)))
		return -EINVAL;

	strlcpy(f->description, format[f->index].name, sizeof(f->description));
	f->pixelformat = format[f->index].fourcc;

	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;
	struct em28xx_fmt     *fmt;
	unsigned int	      maxw = norm_maxw(dev);
	unsigned int	      maxh = norm_maxh(dev);

	fmt = format_by_fourcc(fsize->pixel_format);
	if (!fmt) {
		em28xx_videodbg("Fourcc format (%08x) invalid.\n",
				fsize->pixel_format);
		return -EINVAL;
	}

	if (dev->board.is_em2800) {
		if (fsize->index > 1)
			return -EINVAL;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = maxw / (1 + fsize->index);
		fsize->discrete.height = maxh / (1 + fsize->index);
		return 0;
	}

	if (fsize->index != 0)
		return -EINVAL;

	/* Report a continuous range */
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	scale_to_size(dev, EM28XX_HVSCALE_MAX, EM28XX_HVSCALE_MAX,
		      &fsize->stepwise.min_width, &fsize->stepwise.min_height);
	if (fsize->stepwise.min_width < 48)
		fsize->stepwise.min_width = 48;
	if (fsize->stepwise.min_height < 38)
		fsize->stepwise.min_height = 38;
	fsize->stepwise.max_width = maxw;
	fsize->stepwise.max_height = maxh;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;
	return 0;
}

/* RAW VBI ioctls */

static int vidioc_g_fmt_vbi_cap(struct file *file, void *priv,
				struct v4l2_format *format)
{
	struct em28xx_fh      *fh  = priv;
	struct em28xx         *dev = fh->dev;

	format->fmt.vbi.samples_per_line = dev->vbi_width;
	format->fmt.vbi.sample_format = V4L2_PIX_FMT_GREY;
	format->fmt.vbi.offset = 0;
	format->fmt.vbi.flags = 0;
	format->fmt.vbi.sampling_rate = 6750000 * 4 / 2;
	format->fmt.vbi.count[0] = dev->vbi_height;
	format->fmt.vbi.count[1] = dev->vbi_height;
	memset(format->fmt.vbi.reserved, 0, sizeof(format->fmt.vbi.reserved));

	/* Varies by video standard (NTSC, PAL, etc.) */
	if (dev->norm & V4L2_STD_525_60) {
		/* NTSC */
		format->fmt.vbi.start[0] = 10;
		format->fmt.vbi.start[1] = 273;
	} else if (dev->norm & V4L2_STD_625_50) {
		/* PAL */
		format->fmt.vbi.start[0] = 6;
		format->fmt.vbi.start[1] = 318;
	}

	return 0;
}

/* ----------------------------------------------------------- */
/* RADIO ESPECIFIC IOCTLS                                      */
/* ----------------------------------------------------------- */

static int radio_g_tuner(struct file *file, void *priv,
			 struct v4l2_tuner *t)
{
	struct em28xx *dev = ((struct em28xx_fh *)priv)->dev;

	if (unlikely(t->index > 0))
		return -EINVAL;

	strcpy(t->name, "Radio");

	v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, g_tuner, t);

	return 0;
}

static int radio_s_tuner(struct file *file, void *priv,
			 const struct v4l2_tuner *t)
{
	struct em28xx *dev = ((struct em28xx_fh *)priv)->dev;

	if (0 != t->index)
		return -EINVAL;

	v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_tuner, t);

	return 0;
}

/*
 * em28xx_v4l2_open()
 * inits the device and starts isoc transfer
 */
static int em28xx_v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct em28xx *dev = video_drvdata(filp);
	enum v4l2_buf_type fh_type = 0;
	struct em28xx_fh *fh;

	switch (vdev->vfl_type) {
	case VFL_TYPE_GRABBER:
		fh_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		break;
	case VFL_TYPE_VBI:
		fh_type = V4L2_BUF_TYPE_VBI_CAPTURE;
		break;
	case VFL_TYPE_RADIO:
		break;
	default:
		return -EINVAL;
	}

	em28xx_videodbg("open dev=%s type=%s users=%d\n",
			video_device_node_name(vdev), v4l2_type_names[fh_type],
			dev->users);


	if (mutex_lock_interruptible(&dev->lock))
		return -ERESTARTSYS;
	fh = kzalloc(sizeof(struct em28xx_fh), GFP_KERNEL);
	if (!fh) {
		em28xx_errdev("em28xx-video.c: Out of memory?!\n");
		mutex_unlock(&dev->lock);
		return -ENOMEM;
	}
	v4l2_fh_init(&fh->fh, vdev);
	fh->dev = dev;
	fh->type = fh_type;
	filp->private_data = fh;

	if (dev->users == 0) {
		em28xx_set_mode(dev, EM28XX_ANALOG_MODE);

		if (vdev->vfl_type != VFL_TYPE_RADIO)
			em28xx_resolution_set(dev);

		/*
		 * Needed, since GPIO might have disabled power
		 * of some i2c devices
		 */
		em28xx_wake_i2c(dev);
	}

	if (vdev->vfl_type == VFL_TYPE_RADIO) {
		em28xx_videodbg("video_open: setting radio device\n");
		v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_radio);
	}

	dev->users++;

	mutex_unlock(&dev->lock);
	v4l2_fh_add(&fh->fh);

	return 0;
}

/*
 * em28xx_v4l2_fini()
 * unregisters the v4l2,i2c and usb devices
 * called when the device gets disconected or at module unload
*/
static int em28xx_v4l2_fini(struct em28xx *dev)
{
	if (dev->is_audio_only) {
		/* Shouldn't initialize IR for this interface */
		return 0;
	}

	if (!dev->has_video) {
		/* This device does not support the v4l2 extension */
		return 0;
	}

	em28xx_info("Closing video extension\n");

	mutex_lock(&dev->lock);

	v4l2_device_disconnect(&dev->v4l2_dev);

	em28xx_uninit_usb_xfer(dev, EM28XX_ANALOG_MODE);

	if (dev->radio_dev) {
		em28xx_info("V4L2 device %s deregistered\n",
			    video_device_node_name(dev->radio_dev));
		video_unregister_device(dev->radio_dev);
	}
	if (dev->vbi_dev) {
		em28xx_info("V4L2 device %s deregistered\n",
			    video_device_node_name(dev->vbi_dev));
		video_unregister_device(dev->vbi_dev);
	}
	if (dev->vdev) {
		em28xx_info("V4L2 device %s deregistered\n",
			    video_device_node_name(dev->vdev));
		video_unregister_device(dev->vdev);
	}

	if (dev->clk) {
		v4l2_clk_unregister_fixed(dev->clk);
		dev->clk = NULL;
	}

	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	v4l2_device_unregister(&dev->v4l2_dev);

	if (dev->users)
		em28xx_warn("Device is open ! Memory deallocation is deferred on last close.\n");
	mutex_unlock(&dev->lock);

	return 0;
}

/*
 * em28xx_v4l2_close()
 * stops streaming and deallocates all resources allocated by the v4l2
 * calls and ioctls
 */
static int em28xx_v4l2_close(struct file *filp)
{
	struct em28xx_fh *fh  = filp->private_data;
	struct em28xx    *dev = fh->dev;
	int              errCode;

	em28xx_videodbg("users=%d\n", dev->users);

	vb2_fop_release(filp);
	mutex_lock(&dev->lock);

	if (dev->users == 1) {
		/* free the remaining resources if device is disconnected */
		if (dev->disconnected) {
			kfree(dev->alt_max_pkt_size_isoc);
			goto exit;
		}

		/* Save some power by putting tuner to sleep */
		v4l2_device_call_all(&dev->v4l2_dev, 0, core, s_power, 0);

		/* do this before setting alternate! */
		em28xx_set_mode(dev, EM28XX_SUSPEND);

		/* set alternate 0 */
		dev->alt = 0;
		em28xx_videodbg("setting alternate 0\n");
		errCode = usb_set_interface(dev->udev, 0, 0);
		if (errCode < 0) {
			em28xx_errdev("cannot change alternate number to "
					"0 (error=%i)\n", errCode);
		}
	}

exit:
	dev->users--;
	mutex_unlock(&dev->lock);
	return 0;
}

/*
 * em28xx_videodevice_release()
 * called when the last user of the video device exits and frees the memeory
 */
static void em28xx_videodevice_release(struct video_device *vdev)
{
	struct em28xx *dev = video_get_drvdata(vdev);

	video_device_release(vdev);
	if (vdev == dev->vdev)
		dev->vdev = NULL;
	else if (vdev == dev->vbi_dev)
		dev->vbi_dev = NULL;
	else if (vdev == dev->radio_dev)
		dev->radio_dev = NULL;
}

static const struct v4l2_file_operations em28xx_v4l_fops = {
	.owner         = THIS_MODULE,
	.open          = em28xx_v4l2_open,
	.release       = em28xx_v4l2_close,
	.read          = vb2_fop_read,
	.poll          = vb2_fop_poll,
	.mmap          = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct v4l2_ioctl_ops video_ioctl_ops = {
	.vidioc_querycap            = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap    = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap       = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap     = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap       = vidioc_s_fmt_vid_cap,
	.vidioc_g_fmt_vbi_cap       = vidioc_g_fmt_vbi_cap,
	.vidioc_try_fmt_vbi_cap     = vidioc_g_fmt_vbi_cap,
	.vidioc_s_fmt_vbi_cap       = vidioc_g_fmt_vbi_cap,
	.vidioc_enum_framesizes     = vidioc_enum_framesizes,
	.vidioc_g_audio             = vidioc_g_audio,
	.vidioc_s_audio             = vidioc_s_audio,

	.vidioc_reqbufs             = vb2_ioctl_reqbufs,
	.vidioc_create_bufs         = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf         = vb2_ioctl_prepare_buf,
	.vidioc_querybuf            = vb2_ioctl_querybuf,
	.vidioc_qbuf                = vb2_ioctl_qbuf,
	.vidioc_dqbuf               = vb2_ioctl_dqbuf,

	.vidioc_g_std               = vidioc_g_std,
	.vidioc_querystd            = vidioc_querystd,
	.vidioc_s_std               = vidioc_s_std,
	.vidioc_g_parm		    = vidioc_g_parm,
	.vidioc_s_parm		    = vidioc_s_parm,
	.vidioc_enum_input          = vidioc_enum_input,
	.vidioc_g_input             = vidioc_g_input,
	.vidioc_s_input             = vidioc_s_input,
	.vidioc_streamon            = vb2_ioctl_streamon,
	.vidioc_streamoff           = vb2_ioctl_streamoff,
	.vidioc_g_tuner             = vidioc_g_tuner,
	.vidioc_s_tuner             = vidioc_s_tuner,
	.vidioc_g_frequency         = vidioc_g_frequency,
	.vidioc_s_frequency         = vidioc_s_frequency,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_chip_info         = vidioc_g_chip_info,
	.vidioc_g_register          = vidioc_g_register,
	.vidioc_s_register          = vidioc_s_register,
#endif
};

static const struct video_device em28xx_video_template = {
	.fops		= &em28xx_v4l_fops,
	.ioctl_ops	= &video_ioctl_ops,
	.release	= em28xx_videodevice_release,
	.tvnorms	= V4L2_STD_ALL,
};

static const struct v4l2_file_operations radio_fops = {
	.owner         = THIS_MODULE,
	.open          = em28xx_v4l2_open,
	.release       = em28xx_v4l2_close,
	.unlocked_ioctl = video_ioctl2,
};

static const struct v4l2_ioctl_ops radio_ioctl_ops = {
	.vidioc_querycap      = vidioc_querycap,
	.vidioc_g_tuner       = radio_g_tuner,
	.vidioc_s_tuner       = radio_s_tuner,
	.vidioc_g_frequency   = vidioc_g_frequency,
	.vidioc_s_frequency   = vidioc_s_frequency,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_chip_info   = vidioc_g_chip_info,
	.vidioc_g_register    = vidioc_g_register,
	.vidioc_s_register    = vidioc_s_register,
#endif
};

static struct video_device em28xx_radio_template = {
	.fops		= &radio_fops,
	.ioctl_ops	= &radio_ioctl_ops,
	.release	= em28xx_videodevice_release,
};

/* I2C possible address to saa7115, tvp5150, msp3400, tvaudio */
static unsigned short saa711x_addrs[] = {
	0x4a >> 1, 0x48 >> 1,   /* SAA7111, SAA7111A and SAA7113 */
	0x42 >> 1, 0x40 >> 1,   /* SAA7114, SAA7115 and SAA7118 */
	I2C_CLIENT_END };

static unsigned short tvp5150_addrs[] = {
	0xb8 >> 1,
	0xba >> 1,
	I2C_CLIENT_END
};

static unsigned short msp3400_addrs[] = {
	0x80 >> 1,
	0x88 >> 1,
	I2C_CLIENT_END
};

/******************************** usb interface ******************************/

static struct video_device *em28xx_vdev_init(struct em28xx *dev,
					const struct video_device *template,
					const char *type_name)
{
	struct video_device *vfd;

	vfd = video_device_alloc();
	if (NULL == vfd)
		return NULL;

	*vfd		= *template;
	vfd->v4l2_dev	= &dev->v4l2_dev;
	vfd->debug	= video_debug;
	vfd->lock	= &dev->lock;
	set_bit(V4L2_FL_USE_FH_PRIO, &vfd->flags);
	if (dev->board.is_webcam)
		vfd->tvnorms = 0;

	snprintf(vfd->name, sizeof(vfd->name), "%s %s",
		 dev->name, type_name);

	video_set_drvdata(vfd, dev);
	return vfd;
}

static void em28xx_tuner_setup(struct em28xx *dev)
{
	struct tuner_setup           tun_setup;
	struct v4l2_frequency        f;

	if (dev->tuner_type == TUNER_ABSENT)
		return;

	memset(&tun_setup, 0, sizeof(tun_setup));

	tun_setup.mode_mask = T_ANALOG_TV | T_RADIO;
	tun_setup.tuner_callback = em28xx_tuner_callback;

	if (dev->board.radio.type) {
		tun_setup.type = dev->board.radio.type;
		tun_setup.addr = dev->board.radio_addr;

		v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_type_addr, &tun_setup);
	}

	if ((dev->tuner_type != TUNER_ABSENT) && (dev->tuner_type)) {
		tun_setup.type   = dev->tuner_type;
		tun_setup.addr   = dev->tuner_addr;

		v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_type_addr, &tun_setup);
	}

	if (dev->tda9887_conf) {
		struct v4l2_priv_tun_config tda9887_cfg;

		tda9887_cfg.tuner = TUNER_TDA9887;
		tda9887_cfg.priv = &dev->tda9887_conf;

		v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_config, &tda9887_cfg);
	}

	if (dev->tuner_type == TUNER_XC2028) {
		struct v4l2_priv_tun_config  xc2028_cfg;
		struct xc2028_ctrl           ctl;

		memset(&xc2028_cfg, 0, sizeof(xc2028_cfg));
		memset(&ctl, 0, sizeof(ctl));

		em28xx_setup_xc3028(dev, &ctl);

		xc2028_cfg.tuner = TUNER_XC2028;
		xc2028_cfg.priv  = &ctl;

		v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_config, &xc2028_cfg);
	}

	/* configure tuner */
	f.tuner = 0;
	f.type = V4L2_TUNER_ANALOG_TV;
	f.frequency = 9076;     /* just a magic number */
	dev->ctl_freq = f.frequency;
	v4l2_device_call_all(&dev->v4l2_dev, 0, tuner, s_frequency, &f);
}

static int em28xx_v4l2_init(struct em28xx *dev)
{
	u8 val;
	int ret;
	unsigned int maxw;
	struct v4l2_ctrl_handler *hdl = &dev->ctrl_handler;

	if (dev->is_audio_only) {
		/* Shouldn't initialize IR for this interface */
		return 0;
	}

	if (!dev->has_video) {
		/* This device does not support the v4l2 extension */
		return 0;
	}

	em28xx_info("Registering V4L2 extension\n");

	mutex_lock(&dev->lock);

	ret = v4l2_device_register(&dev->udev->dev, &dev->v4l2_dev);
	if (ret < 0) {
		em28xx_errdev("Call to v4l2_device_register() failed!\n");
		goto err;
	}

	v4l2_ctrl_handler_init(hdl, 8);
	dev->v4l2_dev.ctrl_handler = hdl;

	/*
	 * Default format, used for tvp5150 or saa711x output formats
	 */
	dev->vinmode = 0x10;
	dev->vinctl  = EM28XX_VINCTRL_INTERLACED |
		       EM28XX_VINCTRL_CCIR656_ENABLE;

	/* request some modules */

	if (dev->board.has_msp34xx)
		v4l2_i2c_new_subdev(&dev->v4l2_dev, &dev->i2c_adap[dev->def_i2c_bus],
			"msp3400", 0, msp3400_addrs);

	if (dev->board.decoder == EM28XX_SAA711X)
		v4l2_i2c_new_subdev(&dev->v4l2_dev, &dev->i2c_adap[dev->def_i2c_bus],
			"saa7115_auto", 0, saa711x_addrs);

	if (dev->board.decoder == EM28XX_TVP5150)
		v4l2_i2c_new_subdev(&dev->v4l2_dev, &dev->i2c_adap[dev->def_i2c_bus],
			"tvp5150", 0, tvp5150_addrs);

	if (dev->board.adecoder == EM28XX_TVAUDIO)
		v4l2_i2c_new_subdev(&dev->v4l2_dev, &dev->i2c_adap[dev->def_i2c_bus],
			"tvaudio", dev->board.tvaudio_addr, NULL);

	/* Initialize tuner and camera */

	if (dev->board.tuner_type != TUNER_ABSENT) {
		int has_demod = (dev->tda9887_conf & TDA9887_PRESENT);

		if (dev->board.radio.type)
			v4l2_i2c_new_subdev(&dev->v4l2_dev, &dev->i2c_adap[dev->def_i2c_bus],
				"tuner", dev->board.radio_addr, NULL);

		if (has_demod)
			v4l2_i2c_new_subdev(&dev->v4l2_dev,
				&dev->i2c_adap[dev->def_i2c_bus], "tuner",
				0, v4l2_i2c_tuner_addrs(ADDRS_DEMOD));
		if (dev->tuner_addr == 0) {
			enum v4l2_i2c_tuner_type type =
				has_demod ? ADDRS_TV_WITH_DEMOD : ADDRS_TV;
			struct v4l2_subdev *sd;

			sd = v4l2_i2c_new_subdev(&dev->v4l2_dev,
				&dev->i2c_adap[dev->def_i2c_bus], "tuner",
				0, v4l2_i2c_tuner_addrs(type));

			if (sd)
				dev->tuner_addr = v4l2_i2c_subdev_addr(sd);
		} else {
			v4l2_i2c_new_subdev(&dev->v4l2_dev, &dev->i2c_adap[dev->def_i2c_bus],
				"tuner", dev->tuner_addr, NULL);
		}
	}

	em28xx_tuner_setup(dev);
	em28xx_init_camera(dev);

	/* Configure audio */
	ret = em28xx_audio_setup(dev);
	if (ret < 0) {
		em28xx_errdev("%s: Error while setting audio - error [%d]!\n",
			__func__, ret);
		goto unregister_dev;
	}
	if (dev->audio_mode.ac97 != EM28XX_NO_AC97) {
		v4l2_ctrl_new_std(hdl, &em28xx_ctrl_ops,
			V4L2_CID_AUDIO_MUTE, 0, 1, 1, 1);
		v4l2_ctrl_new_std(hdl, &em28xx_ctrl_ops,
			V4L2_CID_AUDIO_VOLUME, 0, 0x1f, 1, 0x1f);
	} else {
		/* install the em28xx notify callback */
		v4l2_ctrl_notify(v4l2_ctrl_find(hdl, V4L2_CID_AUDIO_MUTE),
				em28xx_ctrl_notify, dev);
		v4l2_ctrl_notify(v4l2_ctrl_find(hdl, V4L2_CID_AUDIO_VOLUME),
				em28xx_ctrl_notify, dev);
	}

	/* wake i2c devices */
	em28xx_wake_i2c(dev);

	/* init video dma queues */
	INIT_LIST_HEAD(&dev->vidq.active);
	INIT_LIST_HEAD(&dev->vbiq.active);

	if (dev->board.has_msp34xx) {
		/* Send a reset to other chips via gpio */
		ret = em28xx_write_reg(dev, EM2820_R08_GPIO_CTRL, 0xf7);
		if (ret < 0) {
			em28xx_errdev("%s: em28xx_write_reg - msp34xx(1) failed! error [%d]\n",
				      __func__, ret);
			goto unregister_dev;
		}
		msleep(3);

		ret = em28xx_write_reg(dev, EM2820_R08_GPIO_CTRL, 0xff);
		if (ret < 0) {
			em28xx_errdev("%s: em28xx_write_reg - msp34xx(2) failed! error [%d]\n",
				      __func__, ret);
			goto unregister_dev;
		}
		msleep(3);
	}

	/* set default norm */
	dev->norm = V4L2_STD_PAL;
	v4l2_device_call_all(&dev->v4l2_dev, 0, core, s_std, dev->norm);
	dev->interlaced = EM28XX_INTERLACED_DEFAULT;

	/* Analog specific initialization */
	dev->format = &format[0];

	maxw = norm_maxw(dev);
	/* MaxPacketSize for em2800 is too small to capture at full resolution
	 * use half of maxw as the scaler can only scale to 50% */
	if (dev->board.is_em2800)
		maxw /= 2;

	em28xx_set_video_format(dev, format[0].fourcc,
				maxw, norm_maxh(dev));

	video_mux(dev, 0);

	/* Audio defaults */
	dev->mute = 1;
	dev->volume = 0x1f;

/*	em28xx_write_reg(dev, EM28XX_R0E_AUDIOSRC, 0xc0); audio register */
	val = (u8)em28xx_read_reg(dev, EM28XX_R0F_XCLK);
	em28xx_write_reg(dev, EM28XX_R0F_XCLK,
			 (EM28XX_XCLK_AUDIO_UNMUTE | val));

	em28xx_set_outfmt(dev);
	em28xx_compression_disable(dev);

	/* Add image controls */
	/* NOTE: at this point, the subdevices are already registered, so bridge
	 * controls are only added/enabled when no subdevice provides them */
	if (NULL == v4l2_ctrl_find(&dev->ctrl_handler, V4L2_CID_CONTRAST))
		v4l2_ctrl_new_std(&dev->ctrl_handler, &em28xx_ctrl_ops,
				  V4L2_CID_CONTRAST,
				  0, 0x1f, 1, CONTRAST_DEFAULT);
	if (NULL == v4l2_ctrl_find(&dev->ctrl_handler, V4L2_CID_BRIGHTNESS))
		v4l2_ctrl_new_std(&dev->ctrl_handler, &em28xx_ctrl_ops,
				  V4L2_CID_BRIGHTNESS,
				  -0x80, 0x7f, 1, BRIGHTNESS_DEFAULT);
	if (NULL == v4l2_ctrl_find(&dev->ctrl_handler, V4L2_CID_SATURATION))
		v4l2_ctrl_new_std(&dev->ctrl_handler, &em28xx_ctrl_ops,
				  V4L2_CID_SATURATION,
				  0, 0x1f, 1, SATURATION_DEFAULT);
	if (NULL == v4l2_ctrl_find(&dev->ctrl_handler, V4L2_CID_BLUE_BALANCE))
		v4l2_ctrl_new_std(&dev->ctrl_handler, &em28xx_ctrl_ops,
				  V4L2_CID_BLUE_BALANCE,
				  -0x30, 0x30, 1, BLUE_BALANCE_DEFAULT);
	if (NULL == v4l2_ctrl_find(&dev->ctrl_handler, V4L2_CID_RED_BALANCE))
		v4l2_ctrl_new_std(&dev->ctrl_handler, &em28xx_ctrl_ops,
				  V4L2_CID_RED_BALANCE,
				  -0x30, 0x30, 1, RED_BALANCE_DEFAULT);
	if (NULL == v4l2_ctrl_find(&dev->ctrl_handler, V4L2_CID_SHARPNESS))
		v4l2_ctrl_new_std(&dev->ctrl_handler, &em28xx_ctrl_ops,
				  V4L2_CID_SHARPNESS,
				  0, 0x0f, 1, SHARPNESS_DEFAULT);

	/* Reset image controls */
	em28xx_colorlevels_set_default(dev);
	v4l2_ctrl_handler_setup(&dev->ctrl_handler);
	ret = dev->ctrl_handler.error;
	if (ret)
		goto unregister_dev;

	/* allocate and fill video video_device struct */
	dev->vdev = em28xx_vdev_init(dev, &em28xx_video_template, "video");
	if (!dev->vdev) {
		em28xx_errdev("cannot allocate video_device.\n");
		ret = -ENODEV;
		goto unregister_dev;
	}
	dev->vdev->queue = &dev->vb_vidq;
	dev->vdev->queue->lock = &dev->vb_queue_lock;

	/* disable inapplicable ioctls */
	if (dev->board.is_webcam) {
		v4l2_disable_ioctl(dev->vdev, VIDIOC_QUERYSTD);
		v4l2_disable_ioctl(dev->vdev, VIDIOC_G_STD);
		v4l2_disable_ioctl(dev->vdev, VIDIOC_S_STD);
	} else {
		v4l2_disable_ioctl(dev->vdev, VIDIOC_S_PARM);
	}
	if (dev->tuner_type == TUNER_ABSENT) {
		v4l2_disable_ioctl(dev->vdev, VIDIOC_G_TUNER);
		v4l2_disable_ioctl(dev->vdev, VIDIOC_S_TUNER);
		v4l2_disable_ioctl(dev->vdev, VIDIOC_G_FREQUENCY);
		v4l2_disable_ioctl(dev->vdev, VIDIOC_S_FREQUENCY);
	}
	if (!dev->audio_mode.has_audio) {
		v4l2_disable_ioctl(dev->vdev, VIDIOC_G_AUDIO);
		v4l2_disable_ioctl(dev->vdev, VIDIOC_S_AUDIO);
	}

	/* register v4l2 video video_device */
	ret = video_register_device(dev->vdev, VFL_TYPE_GRABBER,
				       video_nr[dev->devno]);
	if (ret) {
		em28xx_errdev("unable to register video device (error=%i).\n",
			      ret);
		goto unregister_dev;
	}

	/* Allocate and fill vbi video_device struct */
	if (em28xx_vbi_supported(dev) == 1) {
		dev->vbi_dev = em28xx_vdev_init(dev, &em28xx_video_template,
						"vbi");

		dev->vbi_dev->queue = &dev->vb_vbiq;
		dev->vbi_dev->queue->lock = &dev->vb_vbi_queue_lock;

		/* disable inapplicable ioctls */
		v4l2_disable_ioctl(dev->vdev, VIDIOC_S_PARM);
		if (dev->tuner_type == TUNER_ABSENT) {
			v4l2_disable_ioctl(dev->vbi_dev, VIDIOC_G_TUNER);
			v4l2_disable_ioctl(dev->vbi_dev, VIDIOC_S_TUNER);
			v4l2_disable_ioctl(dev->vbi_dev, VIDIOC_G_FREQUENCY);
			v4l2_disable_ioctl(dev->vbi_dev, VIDIOC_S_FREQUENCY);
		}
		if (!dev->audio_mode.has_audio) {
			v4l2_disable_ioctl(dev->vbi_dev, VIDIOC_G_AUDIO);
			v4l2_disable_ioctl(dev->vbi_dev, VIDIOC_S_AUDIO);
		}

		/* register v4l2 vbi video_device */
		ret = video_register_device(dev->vbi_dev, VFL_TYPE_VBI,
					    vbi_nr[dev->devno]);
		if (ret < 0) {
			em28xx_errdev("unable to register vbi device\n");
			goto unregister_dev;
		}
	}

	if (em28xx_boards[dev->model].radio.type == EM28XX_RADIO) {
		dev->radio_dev = em28xx_vdev_init(dev, &em28xx_radio_template,
						  "radio");
		if (!dev->radio_dev) {
			em28xx_errdev("cannot allocate video_device.\n");
			ret = -ENODEV;
			goto unregister_dev;
		}
		ret = video_register_device(dev->radio_dev, VFL_TYPE_RADIO,
					    radio_nr[dev->devno]);
		if (ret < 0) {
			em28xx_errdev("can't register radio device\n");
			goto unregister_dev;
		}
		em28xx_info("Registered radio device as %s\n",
			    video_device_node_name(dev->radio_dev));
	}

	em28xx_info("V4L2 video device registered as %s\n",
		    video_device_node_name(dev->vdev));

	if (dev->vbi_dev)
		em28xx_info("V4L2 VBI device registered as %s\n",
			    video_device_node_name(dev->vbi_dev));

	/* Save some power by putting tuner to sleep */
	v4l2_device_call_all(&dev->v4l2_dev, 0, core, s_power, 0);

	/* initialize videobuf2 stuff */
	em28xx_vb2_setup(dev);

	em28xx_info("V4L2 extension successfully initialized\n");

	mutex_unlock(&dev->lock);
	return 0;

unregister_dev:
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	v4l2_device_unregister(&dev->v4l2_dev);
err:
	mutex_unlock(&dev->lock);
	return ret;
}

static struct em28xx_ops v4l2_ops = {
	.id   = EM28XX_V4L2,
	.name = "Em28xx v4l2 Extension",
	.init = em28xx_v4l2_init,
	.fini = em28xx_v4l2_fini,
};

static int __init em28xx_video_register(void)
{
	return em28xx_register_extension(&v4l2_ops);
}

static void __exit em28xx_video_unregister(void)
{
	em28xx_unregister_extension(&v4l2_ops);
}

module_init(em28xx_video_register);
module_exit(em28xx_video_unregister);
