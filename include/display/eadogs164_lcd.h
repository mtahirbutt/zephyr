/*
 * Copyright (c) 2020 Wilmers Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_EADOGS164_LCD_H_
#define ZEPHYR_INCLUDE_EADOGS164_LCD_H_

#include <device.h>
#include <kernel.h>
#include <drivers/gpio.h>

/* Defines for the EADOGS164 Data/Control Byte values */

#define DATA_BYTE 0x40
#define CONTROL_BYTE 0x00

/* LCD Display Commands */

#define EALCD_CMD_SCREEN_CLEAR 0x01
#define EALCD_CMD_FUNCTION_SET 0x3A
#define EALCD_CMD_EXTENDED_FUNCTION_SET 0x09
#define EALCD_CMD_ENTRY_MODE_SET_TOP_VIEW 0x05
#define EALCD_CMD_ENTRY_MODE_SET_BOTTOM_VIEW 0x06

#define EALCD_CMD_BIAS_SETTING 0x1E
#define EALCD_CMD_FUNCTION_SET_2 0x39
#define EALCD_CMD_INTERNAL_OSC 0x1B
#define EALCD_CMD_FOLLOWER_CONTROL 0x6B
#define EALCD_CMD_POWER_CONTROL 0x56
#define EALCD_CMD_CONTRAST_SET 0x7A
#define EALCD_CMD_FUNCTION_SET_3 0x38
#define EALCD_CMD_DISPLAY_ON 0x0F

/* Change Character Table */

#define EALCD_CMD_FUNCTION_SET_ROM_SEL 0x72

/* Length of GPIO pins for ERW backlight */

#define PIN_LEN 3

#ifdef __cplusplus
extern "C" {
#endif

struct eadogs164_display_config {
	char *bus_name;
	uint8_t i2c_address;
};

struct eadogs164_display_data {
	const struct device *dev;
	const struct device *bus;
};

enum eadogs164_orientation {
	TOP,
	BOTTOM,
};

enum eadogs164_color_selection {
	EADOGS164_E,
	EADOGS164_R,
	EADOGS164_W,
	EADOGS164_ER,
	EADOGS164_EW,
	EADOGS164_RW,
	EADOGS164_ERW,
	EADOGS164_ALL_OFF,
};

struct backlight_pin {
	const struct device *gpio_port_dev;
	char *dev_name;
	gpio_pin_t pin;
	gpio_flags_t init_flags;
};


#define BACKLIGHT_PIN(name_, pin_, flags_) { \
	.dev_name = name_, \
	.pin = pin_, \
	.init_flags = flags_ \
}

/*
 *  @brief sets the backlight GPIO of EADOGS164-ERW
 *
 *  @param color different color values with different combinations are given.
 *
 *  @return Returns 0 if successful operation performed
 */

int eadogs164_color_select(enum eadogs164_color_selection color);

/*  @brief print to the DDRAM memory of SSD1803 controller
 *  @param dev Pointer to device structure for driver instance.
 *  @param x,y are the coordinates of text starting from top left (0,0)
 *  @param buf_size is the length of text to be displayed to LCD
 *  @param *buf holding the data to be displayed
 *  @return Returns 0 if successful operation performed
 */

int eadogs164_print(const struct device *dev, const uint16_t x,
				  const uint16_t y,
				  const uint8_t buf_size,
				  const void *buf);

/*
 *  @brief Set contrast of EADOGS164
 *  @param dev Pointer to device structure for driver instance.
 *  @param value contrast value C0-C5.
 *  @return Returns 0 if successful operation performed
 */

int eadogs164_set_contrast(const struct device *dev, uint8_t value);

/*
 *  @brief sets the orientation of LCD (top, bottom)
 *  @param dev Pointer to device structure for driver instance.
 *  @param orientation is enum for TOP,BOTTOM views.
 *  @return Returns 0 if successful operation performed
 */

int eadogs164_set_orientation(const struct device *dev,
		const enum eadogs164_orientation orientation);

/*
 *  @brief sets the cursor on
 *  @param dev Pointer to device structure for driver instance.
 *  @return Returns 0 if successful operation performed
 */

int eadogs164_cursor_on(const struct device *dev);

/*
 *  @brief switch off the cursor
 *  @param dev Pointer to device structure for driver instance.
 *  @return Returns 0 if successful operation performed
 */

int eadogs164_cursor_off(const struct device *dev);

/*
 *  @brief clear the display
 *  @param dev Pointer to device structure for driver instance.
 *  @return Returns 0 if successful operation performed
 */

int eadogs164_clear(const struct device *dev);

/*
 *  @brief Set line count of EADOGS164
 *  @param dev Pointer to device structure for driver instance.
 *  @param line_4 set line count as 4 line display if true else 2 line.
 *  @return Returns 0 if successful operation performed
 */

int eadogs164_set_line_4(const struct device *dev, bool line_4);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_EADOGS164_LCD_H_ */
