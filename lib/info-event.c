// SPDX-License-Identifier: LGPL-2.1-or-later
// SPDX-FileCopyrightText: 2021 Bartosz Golaszewski <brgl@bgdev.pl>

/* Line status watch. */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "internal.h"

struct gpiod_info_event {
	int event_type;
	uint64_t timestamp;
	struct gpiod_line_info *info;
};

struct gpiod_info_event *
gpiod_info_event_from_kernel(struct gpio_v2_line_info_changed *evbuf)
{
	struct gpiod_info_event *event;

	event = malloc(sizeof(*event));
	if (!event)
		return NULL;

	memset(event, 0, sizeof(*event));
	event->timestamp = evbuf->timestamp_ns;

	switch (evbuf->event_type) {
	case GPIOLINE_CHANGED_REQUESTED:
		event->event_type = GPIOD_INFO_EVENT_LINE_REQUESTED;
		break;
	case GPIOLINE_CHANGED_RELEASED:
		event->event_type = GPIOD_INFO_EVENT_LINE_RELEASED;
		break;
	case GPIOLINE_CHANGED_CONFIG:
		event->event_type = GPIOD_INFO_EVENT_LINE_CONFIG_CHANGED;
		break;
	default:
		/* Can't happen unless there's a bug in the kernel. */
		errno = ENOMSG;
		free(event);
		return NULL;
	}

	event->info = gpiod_line_info_from_kernel(&evbuf->info);
	if (!event->info) {
		free(event);
		return NULL;
	}

	return event;
}

GPIOD_API void gpiod_info_event_free(struct gpiod_info_event *event)
{
	if (!event)
		return;

	gpiod_line_info_free(event->info);
	free(event);
}

GPIOD_API int gpiod_info_event_get_event_type(struct gpiod_info_event *event)
{
	return event->event_type;
}

GPIOD_API uint64_t
gpiod_info_event_get_timestamp(struct gpiod_info_event *event)
{
	return event->timestamp;
}

GPIOD_API struct gpiod_line_info *
gpiod_info_event_get_line_info(struct gpiod_info_event *event)
{
	return event->info;
}

struct gpiod_info_event *gpiod_info_event_read_fd(int fd)
{
	struct gpio_v2_line_info_changed evbuf;
	ssize_t rd;

	memset(&evbuf, 0, sizeof(evbuf));

	rd = read(fd, &evbuf, sizeof(evbuf));
	if (rd < 0) {
		return NULL;
	} else if ((unsigned int)rd < sizeof(evbuf)) {
		errno = EIO;
		return NULL;
	}

	return gpiod_info_event_from_kernel(&evbuf);
}
