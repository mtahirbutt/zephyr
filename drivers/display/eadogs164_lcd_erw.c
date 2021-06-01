/*
 * Copyright (c) 2020 Wilmers Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ea_eadogs164

#include <device.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <string.h>
#include <display/eadogs164_lcd.h>

LOG_MODULE_REGISTER(EADOGS164_LCD, CONFIG_DISPLAY_LOG_LEVEL);

struct eadogs164_display_data eadogs0_display_driver;
static const struct eadogs164_display_config eadogs0_display_cfg = {
	.bus_name = DT_INST_BUS_LABEL(0),
	.i2c_address = DT_INST_REG_ADDR(0),
};

/* global variable for holding orientation information
 * only top and bottom view are considered in this implementation
 */

static enum eadogs164_orientation disp_orientation;
static bool is_line_4;

/*
 *
 *  ********** ERW Backlight Functions ************
 *
 */

static struct backlight_pin eadogs164_backlight_pins[] = {
	BACKLIGHT_PIN(DT_INST_GPIO_LABEL(0, backlt_grn_gpios),
		      DT_INST_GPIO_PIN(0, backlt_grn_gpios),
		      DT_INST_GPIO_FLAGS(0, backlt_grn_gpios) | GPIO_OUTPUT),

	BACKLIGHT_PIN(DT_INST_GPIO_LABEL(0, backlt_red_gpios),
		      DT_INST_GPIO_PIN(0, backlt_red_gpios),
		      DT_INST_GPIO_FLAGS(0, backlt_red_gpios) | GPIO_OUTPUT),

	BACKLIGHT_PIN(DT_INST_GPIO_LABEL(0, backlt_white_gpios),
		      DT_INST_GPIO_PIN(0, backlt_white_gpios),
		      DT_INST_GPIO_FLAGS(0, backlt_white_gpios) | GPIO_OUTPUT),
};

static int eadogs164_pin_write(struct backlight_pin *bpin, uint32_t pin,
			       uint32_t value)
{
	if (pin >= PIN_LEN) {
		return -ENODEV;
	}

	return gpio_pin_set(bpin[pin].gpio_port_dev, bpin[pin].pin, value);
}

static int eadogs164_pin_config(struct backlight_pin *bpin, uint32_t pin,
				bool enable)
{
	if (pin >= PIN_LEN) {
		return -ENODEV;
	}

	return gpio_pin_configure(bpin[pin].gpio_port_dev, bpin[pin].pin,
				  enable ? bpin[pin].init_flags :
					   GPIO_INPUT);
}

static int eadogs164_pin_init(struct backlight_pin *bpin)
{
	int i, ret;

	for (i = 0; i < PIN_LEN; i++) {
		bpin[i].gpio_port_dev = device_get_binding(bpin[i].dev_name);
		if (!bpin[i].gpio_port_dev) {
			return -ENODEV;
		}

		ret = eadogs164_pin_config(bpin, i, true);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

int eadogs164_color_select(enum eadogs164_color_selection color)
{
	int rc;
	uint8_t pin_values[PIN_LEN] = {0, 0, 0};

	switch (color) {
	case EADOGS164_E:
		pin_values[0] = 1;
		pin_values[1] = 0;
		pin_values[2] = 0;
		break;
	case EADOGS164_R:
		pin_values[0] = 0;
		pin_values[1] = 1;
		pin_values[2] = 0;
		break;
	case EADOGS164_W:
		pin_values[0] = 0;
		pin_values[1] = 0;
		pin_values[2] = 1;
		break;
	case EADOGS164_ER:
		pin_values[0] = 1;
		pin_values[1] = 1;
		pin_values[2] = 0;
		break;
	case EADOGS164_EW:
		pin_values[0] = 1;
		pin_values[1] = 0;
		pin_values[2] = 1;
		break;
	case EADOGS164_RW:
		pin_values[0] = 0;
		pin_values[1] = 1;
		pin_values[2] = 1;
		break;
	case EADOGS164_ERW:
		pin_values[0] = 1;
		pin_values[1] = 1;
		pin_values[2] = 1;
		break;
	case EADOGS164_ALL_OFF:
		pin_values[0] = 0;
		pin_values[1] = 0;
		pin_values[2] = 0;
		break;
	default:
		break;
	}

	rc = eadogs164_pin_write(eadogs164_backlight_pins, 0, pin_values[0]);
	if (rc != 0)
		return -EINVAL;
	rc = eadogs164_pin_write(eadogs164_backlight_pins, 1, pin_values[1]);
	if (rc != 0)
		return -EINVAL;
	rc = eadogs164_pin_write(eadogs164_backlight_pins, 2, pin_values[2]);
	if (rc != 0)
		return -EINVAL;

	return 0;
}

/*
 *
 *  ********** Private Functions ************
 *
 */

static inline uint8_t eadogs164_i2c_address(const struct device *dev)
{
	const struct eadogs164_display_config *dcp = dev->config;

	return dcp->i2c_address;
}

static inline const struct device
			*eadogs164_i2c_device(const struct device *dev)
{
	const struct eadogs164_display_data *ddp = dev->data;

	return ddp->bus;
}

static inline int eadogs164_write_i2c(const struct device *dev, uint8_t *data,
				      size_t len, bool cmd)
{
	if (cmd) {	/* it is command*/
		data[0] = CONTROL_BYTE;
	} else {	/* it is data*/
		data[0] = DATA_BYTE;
	}

	if (i2c_write(eadogs164_i2c_device(dev), data, len,
		      eadogs164_i2c_address(dev)) != 0) {
		LOG_ERR("Not able to send %s: %d", (cmd == true)
					? "command" : "data", data[1]);
		return -EINVAL;
	}

	return 0;
}

static uint8_t get_address(const uint16_t x, const uint16_t y)
{
	uint8_t address = 0;

	switch (y) {
	case 0:
		address = x + 4;
		break;
	case 1:
		address = x + 0x24;
		break;
	case 2:
		address = x + 0x44;
		break;
	case 3:
		address = x + 0x64;
		break;
	default:
		break;
	}
	if (disp_orientation == BOTTOM) {
		/*	It is Bottom view for EADOGS164 Display  */
		address = address - 4;
	}
	return address;
}


/*
 *
 * ************* Public Functions ****************
 *
 */

int eadogs164_print(const struct device *dev, const uint16_t x,
				   const uint16_t y,
				   const uint8_t buf_size,
				   const void *buf)
{
	uint8_t address;
	int rc;
	uint8_t char_sent = 0;
	uint8_t current_x = x;
	uint8_t current_y = y;
	uint8_t cmd[2];

	while (char_sent != buf_size) {
		address = get_address(current_x, current_y);
		address |= 0x80;
		/* Set DDRAM address */
		cmd[1] = address;
		rc = eadogs164_write_i2c(dev, cmd, 2, true);
		if (rc != 0)
			return rc;

		if (x + (buf_size - char_sent) < 16) {
			/* write ASCII data into DDRAM */
			for (uint8_t i = char_sent; i < buf_size; i++) {
				cmd[1] = ((uint8_t *)(buf))[i];
				rc = eadogs164_write_i2c(dev, cmd, 2, false);
				if (rc != 0)
					return rc;
			}
			char_sent = buf_size;
		} else {
			/* write ASCII data into DDRAM */
			for (uint8_t i = char_sent;
			     i < char_sent + (16-x); i++) {
				cmd[1] = ((uint8_t *)(buf))[i];
				rc = eadogs164_write_i2c(dev, cmd, 2, false);
				if (rc != 0)
					return rc;
			}
			char_sent += (16 - x);
			current_x = 0;
			current_y++;
		}
	}
	return 0;
}

int eadogs164_set_contrast(const struct device *dev, const uint8_t contrast)
{
	uint8_t cmd[2];
	int rc;

	/* Set POWER CONTROL */
	cmd[1] = EALCD_CMD_POWER_CONTROL;
	cmd[1] = cmd[1] & (~(0x03));
	/* clear DB0 and DB1 bits where 0x03 is the mask for C4 and D5 */
	cmd[1] = cmd[1] | ((contrast & 0x30) >> 4);
	/* set DB0 and DB1 in destination */
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* now write DB0-DB3 */

	/* Set Contrast */
	cmd[1] = EALCD_CMD_CONTRAST_SET;
	cmd[1] = cmd[1] & (~(0x0F));
	/* clear DB0 and DB3 bits where 0x30 is the mask for DB4 and DB5 */
	cmd[1] = cmd[1] | (contrast & 0x0F);
	/* set DB4 and DB5 in destination */
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	return 0;
}

int eadogs164_set_orientation(const struct device *dev,
			const enum eadogs164_orientation orientation)
{
	disp_orientation = orientation;

	uint8_t cmd[2];
	int rc;

	cmd[1] = EALCD_CMD_FUNCTION_SET;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	if (disp_orientation == TOP) {
		/* it is top view */
		cmd[1] = EALCD_CMD_ENTRY_MODE_SET_TOP_VIEW;
	} else if (disp_orientation == BOTTOM) {
		/* it is bottom view */
		cmd[1] = EALCD_CMD_ENTRY_MODE_SET_BOTTOM_VIEW;
	} else {
		LOG_ERR("orientation not supported");
	}

	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;
	/* 8 bit data length extension bit, RE=0;IS=1 */

	cmd[1] = EALCD_CMD_FUNCTION_SET_3;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	return 0;
}

int eadogs164_cursor_on(const struct device *dev)
{
	uint8_t cmd[2];
	int rc;

	/* Set Blinking bit */
	cmd[1] = EALCD_CMD_DISPLAY_ON | 0x01;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	return 0;
}

int eadogs164_cursor_off(const struct device *dev)
{
	uint8_t cmd[2];
	int rc;

	/* clears blinking bit */
	cmd[1] = EALCD_CMD_DISPLAY_ON & 0xFE;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	return 0;
}

int eadogs164_clear(const struct device *dev)
{
	uint8_t cmd[2];
	int rc;

	/* clears blinking bit */
	cmd[1] = EALCD_CMD_SCREEN_CLEAR;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	return 0;
}

int eadogs164_set_line_4(const struct device *dev, bool line_4)
{
	uint8_t cmd[2];
	int rc;

	is_line_4 = line_4;
	cmd[1] = 0x3A;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;
	/* enable/disable NW bit */
	if (line_4)
		cmd[1] = 0x09;
	else
		cmd[1] =  0x1B;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;
	if (line_4) {
		cmd[1] = 0x38;
	} else
		cmd[1] = 0x3C;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	return 0;
}

/*
 *
 * ************* Initializations ****************
 *
 */

static int eadogs164_initialize(const struct device *dev)
{
	eadogs164_pin_init(eadogs164_backlight_pins);
	is_line_4 = true;

	struct eadogs164_display_data *data = dev->data;
	const struct eadogs164_display_config *cfg = dev->config;
	const struct device *i2c = device_get_binding(cfg->bus_name);

	if (i2c == NULL) {
		LOG_DBG("Failed to get pointer to %s device!", cfg->bus_name);
		return -EINVAL;
	}
	data->bus = i2c;

	if (!cfg->i2c_address) {
		LOG_DBG("No I2C address");
		return -EINVAL;
	}
	data->dev = dev;

	/* Start Up Time for LCD */
	k_sleep(K_MSEC(10));

	uint8_t cmd[2];
	int rc;

	/* 8 bit data length extension Bit RE=1; REV=0 */
	cmd[1] = EALCD_CMD_FUNCTION_SET;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* 4 Line Display */
	cmd[1] = EALCD_CMD_EXTENDED_FUNCTION_SET;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* Bottom View */
	cmd[1] = EALCD_CMD_ENTRY_MODE_SET_BOTTOM_VIEW;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	disp_orientation = BOTTOM;

	/* BS1 = 1 */
	cmd[1] = EALCD_CMD_BIAS_SETTING;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* 8 bit data length extension bit RE=0;IS=1 */
	cmd[1] = EALCD_CMD_FUNCTION_SET_2;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* BS0=1, Bias = 1/6 */
	cmd[1] = EALCD_CMD_INTERNAL_OSC;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* Divider on and Set Value */

	cmd[1] = EALCD_CMD_FOLLOWER_CONTROL;
#ifdef CONFIG_EADOGS164_DISPLAY_RAB2_0
	cmd[1] = cmd[1] & 0xF8;
	/* clears RAB2-0 */
	cmd[1] = cmd[1] | (CONFIG_EADOGS164_DISPLAY_RAB2_0 & 0x07);
	/* copy lower 3 bits into RAB2-0 */
#endif
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* Booster on and set contrast */
	cmd[1] = EALCD_CMD_POWER_CONTROL;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* Set Contrast */
	cmd[1] = EALCD_CMD_CONTRAST_SET;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* 8 bit data length extension bit RE=0;IS=0 */
	cmd[1] = EALCD_CMD_FUNCTION_SET_3;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* Display ON, Cursor on, Blink on */
	cmd[1] = EALCD_CMD_DISPLAY_ON;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* Clear the Display */
	cmd[1] = EALCD_CMD_SCREEN_CLEAR;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

#ifdef CONFIG_EADOGS164_DISPLAY_ROM
	/* Function Set 8-bit, RE=1 */
	cmd[1] = EALCD_CMD_FUNCTION_SET;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	/* ROM Selection Command */
	cmd[1] = EALCD_CMD_FUNCTION_SET_ROM_SEL;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

	cmd[1] = 0;
	/* clears the variable */
	cmd[1] = CONFIG_EADOGS164_DISPLAY_ROM << 2;
	/* shift ROM1,ROM2 bits to the command */
	/* it is Data(1) rather Command(0) */

	rc = eadogs164_write_i2c(dev, cmd, 2, false);
	if (rc != 0)
		return rc;

	/* Function Set 8-bit, RE=0 */
	cmd[1] = EALCD_CMD_FUNCTION_SET_3;
	rc = eadogs164_write_i2c(dev, cmd, 2, true);
	if (rc != 0)
		return rc;

#endif
	return 0;
}

DEVICE_AND_API_INIT(eadogs0, DT_INST_LABEL(0), eadogs164_initialize,
		    &eadogs0_display_driver, &eadogs0_display_cfg, POST_KERNEL,
		    CONFIG_APPLICATION_INIT_PRIORITY, NULL);
