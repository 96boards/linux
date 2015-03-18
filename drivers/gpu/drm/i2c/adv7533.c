/*
 * Analog Devices ADV7533 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

#include "adv7533.h"

struct adv7533 {
	struct i2c_client *i2c_main;
	struct i2c_client *i2c_edid;
	struct i2c_client *i2c_dsi;

	struct regmap *regmap_main;
	struct regmap *regmap_dsi;
	int dpms_mode;

	unsigned int current_edid_segment;
	uint8_t edid_buf[256];

	wait_queue_head_t wq;
	struct delayed_work hotplug_work;
	struct drm_encoder *encoder;

	struct edid *edid;
	struct gpio_desc *gpio_pd;

};

/* ADI recommended values for proper operation. */
static const struct reg_default adv7533_fixed_registers[] = {
	{ 0x16, 0x20 },
	{ 0x9a, 0xe0 },
	{ 0xba, 0x70 },
	{ 0xde, 0x82 },
	{ 0xe4, 0x40 },
	{ 0xe5, 0x80 },
};

static const struct reg_default adv7533_dsi_fixed_registers[] = {
	{ 0x15, 0x10 },
	{ 0x17, 0xd0 },
	{ 0x24, 0x20 },
	{ 0x57, 0x11 },
};

static const struct regmap_config adv7533_main_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
	.cache_type = REGCACHE_RBTREE,
};

static const struct regmap_config adv7533_dsi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
	.cache_type = REGCACHE_RBTREE,
};


static struct adv7533 *encoder_to_adv7533(struct drm_encoder *encoder)
{
	return to_encoder_slave(encoder)->slave_priv;
}

static const int edid_i2c_addr = 0x7e;
static const int packet_i2c_addr = 0x70;
static const int dsi_i2c_addr = 0x78;

static const struct of_device_id adv7533_of_ids[] = {
	{ .compatible = "adi,adv7533"},
	{ }
};
MODULE_DEVICE_TABLE(of, adv7533_of_ids);

/* -----------------------------------------------------------------------------
 * Hardware configuration
 */

static void adv7533_set_colormap(struct adv7533 *adv7533, bool enable,
				 const uint16_t *coeff,
				 unsigned int scaling_factor)
{
	unsigned int i;

	regmap_update_bits(adv7533->regmap_main, ADV7533_REG_CSC_UPPER(1),
			   ADV7533_CSC_UPDATE_MODE, ADV7533_CSC_UPDATE_MODE);

	if (enable) {
		for (i = 0; i < 12; ++i) {
			regmap_update_bits(adv7533->regmap_main,
					   ADV7533_REG_CSC_UPPER(i),
					   0x1f, coeff[i] >> 8);
			regmap_write(adv7533->regmap_main,
				     ADV7533_REG_CSC_LOWER(i),
				     coeff[i] & 0xff);
		}
	}

	if (enable)
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_CSC_UPPER(0),
				   0xe0, 0x80 | (scaling_factor << 5));
	else
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_CSC_UPPER(0),
				   0x80, 0x00);

	regmap_update_bits(adv7533->regmap_main, ADV7533_REG_CSC_UPPER(1),
			   ADV7533_CSC_UPDATE_MODE, 0);
}

static int adv7533_packet_enable(struct adv7533 *adv7533, unsigned int packet)
{
	if (packet & 0xff)
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_PACKET_ENABLE0,
				   packet, 0xff);

	if (packet & 0xff00) {
		packet >>= 8;
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_PACKET_ENABLE1,
				   packet, 0xff);
	}

	return 0;
}

static int adv7533_packet_disable(struct adv7533 *adv7533, unsigned int packet)
{
	if (packet & 0xff)
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_PACKET_ENABLE0,
				   packet, 0x00);

	if (packet & 0xff00) {
		packet >>= 8;
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_PACKET_ENABLE1,
				   packet, 0x00);
	}

	return 0;
}

/* Coefficients for adv7533 color space conversion */
static const uint16_t adv7533_csc_ycbcr_to_rgb[] = {
	0x0734, 0x04ad, 0x0000, 0x1c1b,
	0x1ddc, 0x04ad, 0x1f24, 0x0135,
	0x0000, 0x04ad, 0x087c, 0x1b77,
};

static void adv7533_set_config_csc(struct adv7533 *adv7533,
				   struct drm_connector *connector,
				   bool rgb)
{
	struct adv7533_video_config config;
	bool output_format_422, output_format_ycbcr;
	unsigned int mode;
	uint8_t infoframe[17];

	if (adv7533->edid)
		config.hdmi_mode = drm_detect_hdmi_monitor(adv7533->edid);
	else
		config.hdmi_mode = false;

	hdmi_avi_infoframe_init(&config.avi_infoframe);

	config.avi_infoframe.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;

	if (rgb) {
		config.csc_enable = false;
		config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
	} else {
		config.csc_scaling_factor = ADV7533_CSC_SCALING_4;
		config.csc_coefficents = adv7533_csc_ycbcr_to_rgb;

		if ((connector->display_info.color_formats &
		     DRM_COLOR_FORMAT_YCRCB422) &&
		    config.hdmi_mode) {
			config.csc_enable = false;
			config.avi_infoframe.colorspace =
				HDMI_COLORSPACE_YUV422;
		} else {
			config.csc_enable = true;
			config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
		}
	}

	if (config.hdmi_mode) {
		mode = ADV7533_HDMI_CFG_MODE_HDMI;

		switch (config.avi_infoframe.colorspace) {
		case HDMI_COLORSPACE_YUV444:
			output_format_422 = false;
			output_format_ycbcr = true;
			break;
		case HDMI_COLORSPACE_YUV422:
			output_format_422 = true;
			output_format_ycbcr = true;
			break;
		default:
			output_format_422 = false;
			output_format_ycbcr = false;
			break;
		}
	} else {
		mode = ADV7533_HDMI_CFG_MODE_DVI;
		output_format_422 = false;
		output_format_ycbcr = false;
	}

	DRM_INFO("HDMI: mode=%d,format_422=%d,format_ycbcr=%d\n",
			mode, output_format_422, output_format_ycbcr);
	adv7533_packet_disable(adv7533, ADV7533_PACKET_ENABLE_AVI_INFOFRAME);

	adv7533_set_colormap(adv7533, config.csc_enable,
			     config.csc_coefficents,
			     config.csc_scaling_factor);

	regmap_update_bits(adv7533->regmap_main, ADV7533_REG_VIDEO_INPUT_CFG1, 
			0x81, (output_format_422 << 7) | output_format_ycbcr);

	regmap_update_bits(adv7533->regmap_main, ADV7533_REG_HDCP_HDMI_CFG,
			   ADV7533_HDMI_CFG_MODE_MASK, mode);

	hdmi_avi_infoframe_pack(&config.avi_infoframe, infoframe,
				sizeof(infoframe));

	/* The AVI infoframe id is not configurable */
	regmap_bulk_write(adv7533->regmap_main, ADV7533_REG_AVI_INFOFRAME_VERSION,
			  infoframe + 1, sizeof(infoframe) - 1);

	adv7533_packet_enable(adv7533, ADV7533_PACKET_ENABLE_AVI_INFOFRAME);
}


/* -----------------------------------------------------------------------------
 * Interrupt and hotplug detection
 */

static bool adv7533_hpd(struct adv7533 *adv7533)
{
	unsigned int irq0;
	int ret;

	ret = regmap_read(adv7533->regmap_main, ADV7533_REG_INT(0), &irq0);
	if (ret < 0)
		return false;

	if (irq0 & ADV7533_INT0_HDP)
		return true;

	return false;
}

static void adv7533_hotplug_work_func(struct work_struct *work)
{
	struct adv7533 *adv7533;
	bool hpd_event_sent;

	DRM_INFO("HDMI: hpd work in\n");
	adv7533 = container_of(work, struct adv7533, hotplug_work.work);

	if (adv7533->encoder && adv7533_hpd(adv7533)) {
		hpd_event_sent = drm_helper_hpd_irq_event(adv7533->encoder->dev);
		DRM_INFO("HDMI: hpd_event_sent=%d\n", hpd_event_sent);
	}

	wake_up_all(&adv7533->wq);
	DRM_INFO("HDMI: hpd work out\n");
}

static irqreturn_t adv7533_irq_handler(int irq, void *devid)
{
	struct adv7533 *adv7533 = devid;

	mod_delayed_work(system_wq, &adv7533->hotplug_work,
			msecs_to_jiffies(1500));

	return IRQ_HANDLED;
}

static unsigned int adv7533_is_interrupt_pending(struct adv7533 *adv7533,
						 unsigned int irq)
{
	unsigned int irq0, irq1;
	unsigned int pending;
	int ret;

	ret = regmap_read(adv7533->regmap_main, ADV7533_REG_INT(0), &irq0);
	if (ret < 0)
		return 0;
	ret = regmap_read(adv7533->regmap_main, ADV7533_REG_INT(1), &irq1);
	if (ret < 0)
		return 0;

	pending = (irq1 << 8) | irq0;

	return pending & irq;
}

static int adv7533_wait_for_interrupt(struct adv7533 *adv7533, int irq,
				      int timeout)
{
	unsigned int pending;
	int ret;

	if (adv7533->i2c_main->irq) {
		ret = wait_event_interruptible_timeout(adv7533->wq,
				adv7533_is_interrupt_pending(adv7533, irq),
				msecs_to_jiffies(timeout));
		if (ret <= 0)
			return 0;
		pending = adv7533_is_interrupt_pending(adv7533, irq);
	} else {
		if (timeout < 25)
			timeout = 25;
		do {
			pending = adv7533_is_interrupt_pending(adv7533, irq);
			if (pending)
				break;
			msleep(25);
			timeout -= 25;
		} while (timeout >= 25);
	}

	return pending;
}

/* -----------------------------------------------------------------------------
 * EDID retrieval
 */

static int adv7533_get_edid_block(void *data, u8 *buf, unsigned int block,
				  size_t len)
{
	struct adv7533 *adv7533 = data;
	struct i2c_msg xfer[2];
	uint8_t offset;
	unsigned int i;
	int ret;

	if (len > 128)
		return -EINVAL;

	if (adv7533->current_edid_segment != block / 2) {
		unsigned int status;

		ret = regmap_read(adv7533->regmap_main, ADV7533_REG_DDC_STATUS,
				  &status);
		if (ret < 0)
			return ret;

		DRM_INFO("HDMI: status=0x%x, ret=%d\n", status, ret);
		if (status != 2) {
			/* Open EDID_READY and DDC_ERROR interrupts */
			regmap_update_bits(adv7533->regmap_main, ADV7533_REG_INT_ENABLE(0),
				ADV7533_INT0_EDID_READY, ADV7533_INT0_EDID_READY);
			regmap_update_bits(adv7533->regmap_main, ADV7533_REG_INT_ENABLE(1),
				ADV7533_INT1_DDC_ERROR, ADV7533_INT1_DDC_ERROR);
			regmap_write(adv7533->regmap_main, ADV7533_REG_EDID_SEGMENT,
				     block);
			ret = adv7533_wait_for_interrupt(adv7533,
					ADV7533_INT0_EDID_READY |
					(ADV7533_INT1_DDC_ERROR << 8), 200);

			DRM_INFO("HDMI: ret=0x%x\n", ret);
			if (!(ret & ADV7533_INT0_EDID_READY))
				return -EIO;
		}


		/* Break this apart, hopefully more I2C controllers will
		 * support 64 byte transfers than 256 byte transfers
		 */

		xfer[0].addr = adv7533->i2c_edid->addr;
		xfer[0].flags = 0;
		xfer[0].len = 1;
		xfer[0].buf = &offset;
		xfer[1].addr = adv7533->i2c_edid->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = 64;
		xfer[1].buf = adv7533->edid_buf;

		offset = 0;

		for (i = 0; i < 4; ++i) {
			ret = i2c_transfer(adv7533->i2c_edid->adapter, xfer,
					   ARRAY_SIZE(xfer));
			if (ret < 0)
				return ret;
			else if (ret != 2)
				return -EIO;

			xfer[1].buf += 64;
			offset += 64;
		}

		adv7533->current_edid_segment = block / 2;
	}

	if (block % 2 == 0)
		memcpy(buf, adv7533->edid_buf, len);
	else
		memcpy(buf, adv7533->edid_buf + 128, len);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Encoder operations
 */

static int adv7533_get_modes(struct drm_encoder *encoder,
			     struct drm_connector *connector)
{
	struct adv7533 *adv7533 = encoder_to_adv7533(encoder);
	struct edid *edid;
	unsigned int count;

	/* Reading the EDID only works if the device is powered */
	if (adv7533->dpms_mode != DRM_MODE_DPMS_ON) {
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_POWER,
				   ADV7533_POWER_POWER_DOWN, 0);
		adv7533->current_edid_segment = -1;
		msleep(200); /* wait for EDID finish reading */
	}

	edid = drm_do_get_edid(connector, adv7533_get_edid_block, adv7533);

	if (adv7533->dpms_mode != DRM_MODE_DPMS_ON)
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_POWER,
				   ADV7533_POWER_POWER_DOWN,
				   ADV7533_POWER_POWER_DOWN);

	adv7533->edid = edid;
	adv7533_set_config_csc(adv7533, connector, true);
	if (!edid) {
		DRM_ERROR("Reading edid block error!\n");
		return 0;
	}

	drm_mode_connector_update_edid_property(connector, edid);
	count = drm_add_edid_modes(connector, edid);

	kfree(adv7533->edid);
	adv7533->edid = NULL;
	return count;
}

static void adv7533_dsi_receiver_dpms(struct adv7533 *adv7533, int mode)
{
	switch (mode) {
	case DRM_MODE_DPMS_ON:
		regmap_write(adv7533->regmap_dsi, 0x03, 0x89);
		regmap_write(adv7533->regmap_dsi, 0x27, 0x0b); /* Timing generator off */
		regmap_write(adv7533->regmap_dsi, 0x1C, 0x30); /* 3 lanes */
		break;
	default:
		regmap_write(adv7533->regmap_dsi, 0x03, 0x0b);
		break;
	}
}

static void adv7533_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct adv7533 *adv7533 = encoder_to_adv7533(encoder);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		adv7533->current_edid_segment = -1;

		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_POWER,
				   ADV7533_POWER_POWER_DOWN, 0);
		/*
		 * Per spec it is allowed to pulse the HDP signal to indicate
		 * that the EDID information has changed. Some monitors do this
		 * when they wakeup from standby or are enabled. When the HDP
		 * goes low the adv7533 is reset and the outputs are disabled
		 * which might cause the monitor to go to standby again. To
		 * avoid this we ignore the HDP pin for the first few seconds
		 * after enabeling the output.
		 */
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_POWER2,
				   BIT(6), BIT(6));
		/* Most of the registers are reset during power down or
		 * when HPD is low
		 */
		regcache_sync(adv7533->regmap_main);
		break;
	default:
		/* TODO: setup additional power down modes */
		regmap_update_bits(adv7533->regmap_main, ADV7533_REG_POWER,
				   ADV7533_POWER_POWER_DOWN,
				   ADV7533_POWER_POWER_DOWN);
		regcache_mark_dirty(adv7533->regmap_main);
		break;
	}

	/* dsi receiver dpms */
	adv7533_dsi_receiver_dpms(adv7533, mode);
	adv7533->dpms_mode = mode;
}

static enum drm_connector_status
adv7533_encoder_detect(struct drm_encoder *encoder,
		       struct drm_connector *connector)
{
	struct adv7533 *adv7533 = encoder_to_adv7533(encoder);
	enum drm_connector_status status;
	unsigned int val;
	bool hpd;
	int ret;

	ret = regmap_read(adv7533->regmap_main, ADV7533_REG_STATUS, &val);
	if (ret < 0)
		return connector_status_disconnected;

	if (val & ADV7533_STATUS_HPD)
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	hpd = adv7533_hpd(adv7533);

	DRM_INFO("HDMI: status=%d,hpd=%d,dpms=%d\n",
		status, hpd, adv7533->dpms_mode);
	return status;
}

static int adv7533_encoder_mode_valid(struct drm_encoder *encoder,
				      struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	return MODE_OK;
}

static void adv7533_encoder_mode_set(struct drm_encoder *encoder,
				     struct drm_display_mode *mode,
				     struct drm_display_mode *adj_mode)
{
}

static struct drm_encoder_slave_funcs adv7533_encoder_funcs = {
	.dpms = adv7533_encoder_dpms,
	.mode_valid = adv7533_encoder_mode_valid,
	.mode_set = adv7533_encoder_mode_set,
	.detect = adv7533_encoder_detect,
	.get_modes = adv7533_get_modes,
};

static int adv7533_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct adv7533 *adv7533;
	struct device *dev = &i2c->dev;
	unsigned int val;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!dev->of_node)
		return -EINVAL;

	adv7533 = devm_kzalloc(dev, sizeof(*adv7533), GFP_KERNEL);
	if (!adv7533)
		return -ENOMEM;

	adv7533->dpms_mode = DRM_MODE_DPMS_OFF;

	/*
	 * The power down GPIO is optional. If present, toggle it from active to
	 * inactive to wake up the encoder.
	 */
	adv7533->gpio_pd = devm_gpiod_get_optional(dev, "pd", GPIOD_OUT_HIGH);
	if (IS_ERR(adv7533->gpio_pd))
		return PTR_ERR(adv7533->gpio_pd);

	if (adv7533->gpio_pd) {
		mdelay(5);
		gpiod_set_value_cansleep(adv7533->gpio_pd, 0);
	}

	adv7533->regmap_main = devm_regmap_init_i2c(i2c, &adv7533_main_regmap_config);
	if (IS_ERR(adv7533->regmap_main))
		return PTR_ERR(adv7533->regmap_main);

	ret = regmap_read(adv7533->regmap_main, ADV7533_REG_CHIP_REVISION, &val);
	if (ret)
		return ret;
	dev_dbg(dev, "Rev. %d\n", val);
	ret = regmap_register_patch(adv7533->regmap_main, adv7533_fixed_registers,
				    ARRAY_SIZE(adv7533_fixed_registers));
	 if (ret)
		return ret;

	regmap_write(adv7533->regmap_main, ADV7533_REG_EDID_I2C_ADDR, edid_i2c_addr);
	regmap_write(adv7533->regmap_main, ADV7533_REG_PACKET_I2C_ADDR,
		     packet_i2c_addr);
	regmap_write(adv7533->regmap_main, ADV7533_REG_CEC_I2C_ADDR, dsi_i2c_addr);
	adv7533_packet_disable(adv7533, 0xffff);

	adv7533->i2c_main = i2c;
	adv7533->i2c_edid = i2c_new_dummy(i2c->adapter, edid_i2c_addr >> 1);
	if (!adv7533->i2c_edid)
		return -ENOMEM;

	adv7533->i2c_dsi = i2c_new_dummy(i2c->adapter, dsi_i2c_addr >> 1);
	if (!adv7533->i2c_dsi) {
		ret = -ENOMEM;
		goto err_i2c_unregister_edid;
	}

	adv7533->regmap_dsi = devm_regmap_init_i2c(adv7533->i2c_dsi,
		&adv7533_dsi_regmap_config);
	if (IS_ERR(adv7533->regmap_dsi)) {
		ret = PTR_ERR(adv7533->regmap_dsi);
		goto err_i2c_unregister_dsi;
	}

	ret = regmap_register_patch(adv7533->regmap_dsi, adv7533_dsi_fixed_registers,
				    ARRAY_SIZE(adv7533_dsi_fixed_registers));
	if (ret)
		return ret;

	if (i2c->irq) {
		init_waitqueue_head(&adv7533->wq);

		INIT_DELAYED_WORK(&adv7533->hotplug_work, adv7533_hotplug_work_func);
		ret = devm_request_threaded_irq(dev, i2c->irq, NULL,
						adv7533_irq_handler,
						IRQF_ONESHOT, dev_name(dev),
						adv7533);
		if (ret)
			goto err_i2c_unregister_dsi;
	}

	/* CEC is unused for now */
	regmap_write(adv7533->regmap_main, ADV7533_REG_CEC_CTRL,
		     ADV7533_CEC_CTRL_POWER_DOWN);

	regmap_update_bits(adv7533->regmap_main, ADV7533_REG_POWER,
			   ADV7533_POWER_POWER_DOWN, ADV7533_POWER_POWER_DOWN);

	adv7533->current_edid_segment = -1;

	i2c_set_clientdata(i2c, adv7533);

	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;

err_i2c_unregister_dsi:
	i2c_unregister_device(adv7533->i2c_dsi);
err_i2c_unregister_edid:
	i2c_unregister_device(adv7533->i2c_edid);

	return ret;
}

static int adv7533_remove(struct i2c_client *i2c)
{
	struct adv7533 *adv7533 = i2c_get_clientdata(i2c);

	i2c_unregister_device(adv7533->i2c_dsi);
	i2c_unregister_device(adv7533->i2c_edid);

	return 0;
}

static int adv7533_encoder_init(struct i2c_client *i2c, struct drm_device *dev,
				struct drm_encoder_slave *encoder)
{

	struct adv7533 *adv7533 = i2c_get_clientdata(i2c);

	encoder->slave_priv = adv7533;
	encoder->slave_funcs = &adv7533_encoder_funcs;
	adv7533->encoder = &encoder->base;

	return 0;
}

static const struct i2c_device_id adv7533_i2c_ids[] = {
	{ "adv7533", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, adv7533_i2c_ids);

static struct drm_i2c_encoder_driver adv7533_driver = {
	.i2c_driver = {
		.driver = {
			.name = "adv7533",
			.of_match_table = adv7533_of_ids,
		},
		.id_table = adv7533_i2c_ids,
		.probe = adv7533_probe,
		.remove = adv7533_remove,
	},

	.encoder_init = adv7533_encoder_init,
};

static int __init adv7533_init(void)
{
	return drm_i2c_encoder_register(THIS_MODULE, &adv7533_driver);
}
module_init(adv7533_init);

static void __exit adv7533_exit(void)
{
	drm_i2c_encoder_unregister(&adv7533_driver);
}
module_exit(adv7533_exit);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("ADV7533 HDMI transmitter driver");
MODULE_LICENSE("GPL");
