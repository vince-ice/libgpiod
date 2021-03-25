// SPDX-License-Identifier: LGPL-2.1-or-later
// SPDX-FileCopyrightText: 2021 Bartosz Golaszewski <brgl@bgdev.pl>

/* Line configuration data structure and functions. */

#include <errno.h>
#include <gpiod.h>
#include <stdlib.h>
#include <string.h>

#include "internal.h"

struct gpiod_request_config {
	char consumer[GPIO_MAX_NAME_SIZE];
	unsigned int offsets[GPIO_V2_LINES_MAX];
	unsigned int num_offsets;
	unsigned int event_buffer_size;
};

GPIOD_API struct gpiod_request_config *gpiod_request_config_new(void)
{
	struct gpiod_request_config *config;

	config = malloc(sizeof(*config));
	if (!config)
		return NULL;

	memset(config, 0, sizeof(*config));

	return config;
}

GPIOD_API void gpiod_request_config_free(struct gpiod_request_config *config)
{
	if (!config)
		return;

	free(config);
}

GPIOD_API void
gpiod_request_config_set_consumer(struct gpiod_request_config *config,
				  const char *consumer)
{
	strncpy(config->consumer, consumer, GPIO_MAX_NAME_SIZE - 1);
	config->consumer[GPIO_MAX_NAME_SIZE - 1] = '\0';
}

GPIOD_API const char *
gpiod_request_config_get_consumer(struct gpiod_request_config *config)
{
		return config->consumer;
}

GPIOD_API void
gpiod_request_config_set_offsets(struct gpiod_request_config *config,
				 unsigned int num_offsets,
				 const unsigned int *offsets)
{
	unsigned int i;

	config->num_offsets = num_offsets > GPIO_V2_LINES_MAX ?
					GPIO_V2_LINES_MAX : num_offsets;

	for (i = 0; i < config->num_offsets; i++)
		config->offsets[i] = offsets[i];
}

GPIOD_API unsigned int
gpiod_request_config_get_num_offsets(struct gpiod_request_config *config)
{
	return config->num_offsets;
}

GPIOD_API void
gpiod_request_config_get_offsets(struct gpiod_request_config *config,
				 unsigned int *offsets)
{
	memcpy(offsets, config->offsets,
	       sizeof(*offsets) * config->num_offsets);
}

GPIOD_API void
gpiod_request_config_set_event_buffer_size(struct gpiod_request_config *config,
					   unsigned int event_buffer_size)
{
	config->event_buffer_size = event_buffer_size;
}

GPIOD_API unsigned int
gpiod_request_config_get_event_buffer_size(struct gpiod_request_config *config)
{
	return config->event_buffer_size;
}

int gpiod_request_config_to_kernel(struct gpiod_request_config *config,
				   struct gpio_v2_line_request *reqbuf)
{
	unsigned int i;

	if (config->num_offsets == 0) {
		errno = EINVAL;
		return -1;
	}

	for (i = 0; i < config->num_offsets; i++)
		reqbuf->offsets[i] = config->offsets[i];

	reqbuf->num_lines = config->num_offsets;
	strcpy(reqbuf->consumer, config->consumer);
	reqbuf->event_buffer_size = config->event_buffer_size;

	return 0;
}
