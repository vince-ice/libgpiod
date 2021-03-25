/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* SPDX-FileCopyrightText: 2021 Bartosz Golaszewski <bgolaszewski@baylibre.com> */

#ifndef __LIBGPIOD_GPIOD_INTERNAL_H__
#define __LIBGPIOD_GPIOD_INTERNAL_H__

#include <gpiod.h>
#include <stddef.h>
#include <stdint.h>

#include "uapi/gpio.h"

/* For internal library use only. */

#define GPIOD_API	__attribute__((visibility("default")))
#define GPIOD_PACKED	__attribute__((packed))
#define GPIOD_UNUSED	__attribute__((unused))

#define GPIOD_BIT(nr)	(1UL << (nr))

struct gpiod_line_info *
gpiod_line_info_from_kernel(struct gpio_v2_line_info *infobuf);
int gpiod_request_config_to_kernel(struct gpiod_request_config *config,
				   struct gpio_v2_line_request *reqbuf);
int gpiod_line_config_to_kernel(struct gpiod_line_config *config,
				struct gpio_v2_line_config *cfgbuf,
				unsigned int num_lines,
				const unsigned int *offsets);
struct gpiod_line_request *
gpiod_line_request_from_kernel(struct gpio_v2_line_request *reqbuf);
int gpiod_edge_event_buffer_read_fd(int fd, struct gpiod_edge_event_buffer *buffer,
				    unsigned int max_events);
struct gpiod_info_event *
gpiod_info_event_from_kernel(struct gpio_v2_line_info_changed *evbuf);
struct gpiod_info_event *gpiod_info_event_read_fd(int fd);

int gpiod_poll_fd(int fd, uint64_t timeout);

void gpiod_line_mask_zero(uint64_t *mask);
void gpiod_line_mask_fill(uint64_t *mask);
bool gpiod_line_mask_test_bit(const uint64_t *mask, int nr);
void gpiod_line_mask_set_bit(uint64_t *mask, unsigned int nr);
void gpiod_line_mask_assign_bit(uint64_t *mask, unsigned int nr, bool value);

#endif /* __LIBGPIOD_GPIOD_INTERNAL_H__ */
