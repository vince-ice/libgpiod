// SPDX-License-Identifier: LGPL-2.1-or-later
// SPDX-FileCopyrightText: 2021 Bartosz Golaszewski <brgl@bgdev.pl>

/* Line attribute data structure and functions. */

#include <errno.h>
#include <gpiod.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "internal.h"

struct gpiod_line_request {
	unsigned int offsets[GPIO_V2_LINES_MAX];
	unsigned int num_lines;
	int fd;
};

struct gpiod_line_request *
gpiod_line_request_from_kernel(struct gpio_v2_line_request *reqbuf)
{
	struct gpiod_line_request *request;

	request = malloc(sizeof(*request));
	if (!request)
		return NULL;

	memset(request, 0, sizeof(*request));
	request->fd = reqbuf->fd;
	request->num_lines = reqbuf->num_lines;
	memcpy(request->offsets, reqbuf->offsets,
	       sizeof(*request->offsets) * request->num_lines);

	return request;
}

GPIOD_API void gpiod_line_request_release(struct gpiod_line_request *request)
{
	if (!request)
		return;

	close(request->fd);
	free(request);
}

GPIOD_API unsigned int
gpiod_line_request_get_num_lines(struct gpiod_line_request *request)
{
	return request->num_lines;
}

GPIOD_API void
gpiod_line_request_get_offsets(struct gpiod_line_request *request,
			       unsigned int *offsets)
{
	memcpy(offsets, request->offsets,
	       sizeof(*offsets) * request->num_lines);
}

GPIOD_API int gpiod_line_request_get_value(struct gpiod_line_request *request,
					   unsigned int offset)
{
	unsigned int ret;
	int val;

	ret = gpiod_line_request_get_values_subset(request, 1, &offset, &val);
	if (ret)
		return -1;

	return val;
}

static int offset_to_bit(struct gpiod_line_request *request,
			 unsigned int offset)
{
	unsigned int i;

	for (i = 0; i < request->num_lines; i++) {
		if (offset == request->offsets[i])
			return i;
	}

	return -1;
}

GPIOD_API int
gpiod_line_request_get_values_subset(struct gpiod_line_request *request,
				     unsigned num_lines,
				     const unsigned int *offsets, int *values)
{
	struct gpio_v2_line_values buf;
	uint64_t mask = 0, bits = 0;
	unsigned int i, j, off;
	int bit, ret;

	buf.bits = 0;

	for (i = 0; i < num_lines; i++) {
		bit = offset_to_bit(request, offsets[i]);
		if (bit < 0) {
			errno = EINVAL;
			return -1;
		}

		gpiod_line_mask_set_bit(&mask, bit);
	}

	buf.mask = mask;

	ret = ioctl(request->fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &buf);
	if (ret)
		return -1;

	bits = buf.bits;
	memset(values, 0, sizeof(*values) * num_lines);

	for (i = 0; i < num_lines; i++) {
		if (!gpiod_line_mask_test_bit(&mask, i))
			continue;

		off = request->offsets[i];

		for (j = 0; j < num_lines; j++) {
			if (off != offsets[j])
				continue;

			values[j] = gpiod_line_mask_test_bit(&bits, i) ? 1 : 0;
		}
	}

	return 0;
}

GPIOD_API int gpiod_line_request_get_values(struct gpiod_line_request *request,
					    int *values)
{
	return gpiod_line_request_get_values_subset(request, request->num_lines,
						    request->offsets, values);
}

GPIOD_API int gpiod_line_request_set_value(struct gpiod_line_request *request,
					   unsigned int offset, int value)
{
	return gpiod_line_request_set_values_subset(request, 1,
						    &offset, &value);
}

GPIOD_API int
gpiod_line_request_set_values_subset(struct gpiod_line_request *request,
				     unsigned int num_lines,
				     const unsigned int *offsets,
				     const int *values)
{
	struct gpio_v2_line_values buf;
	uint64_t mask = 0, bits = 0;
	unsigned int i;
	int bit;

	for (i = 0; i < num_lines; i++) {
		bit = offset_to_bit(request, offsets[i]);
		if (bit < 0) {
			errno = EINVAL;
			return -1;
		}

		gpiod_line_mask_set_bit(&mask, bit);
		gpiod_line_mask_assign_bit(&bits, bit, values[i]);
	}

	memset(&buf, 0, sizeof(buf));
	buf.mask = mask;
	buf.bits = bits;

	return ioctl(request->fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &buf);
}

GPIOD_API int gpiod_line_request_set_values(struct gpiod_line_request *request,
					    const int *values)
{
	return gpiod_line_request_set_values_subset(request, request->num_lines,
						    request->offsets, values);
}

GPIOD_API int
gpiod_line_request_reconfigure_lines(struct gpiod_line_request *request,
				     struct gpiod_line_config *config)
{
	struct gpio_v2_line_config cfgbuf;
	int ret;

	memset(&cfgbuf, 0, sizeof(cfgbuf));

	ret = gpiod_line_config_to_kernel(config, &cfgbuf,
					  request->num_lines, request->offsets);
	if (ret)
		return ret;

	ret = ioctl(request->fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &cfgbuf);
	if (ret)
		return ret;

	return 0;
}

GPIOD_API int gpiod_line_request_get_fd(struct gpiod_line_request *request)
{
	return request->fd;
}

GPIOD_API int
gpiod_line_request_edge_event_wait(struct gpiod_line_request *request,
				   uint64_t timeout)
{
	return gpiod_poll_fd(request->fd, timeout);
}

GPIOD_API int
gpiod_line_request_edge_event_read(struct gpiod_line_request *request,
				   struct gpiod_edge_event_buffer *buffer,
				   unsigned int max_events)
{
	return gpiod_edge_event_buffer_read_fd(request->fd, buffer, max_events);
}
