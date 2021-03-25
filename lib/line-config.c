// SPDX-License-Identifier: LGPL-2.1-or-later
// SPDX-FileCopyrightText: 2021 Bartosz Golaszewski <brgl@bgdev.pl>

/* Line configuration data structure and functions. */

#include <errno.h>
#include <gpiod.h>
#include <stdlib.h>
#include <string.h>

#include "internal.h"

struct base_config {
	int direction;
	int edge;
	int drive;
	int bias;
	bool active_low;
	int clock;
	unsigned long debounce_period;
};

struct secondary_config {
	struct base_config config;
	/* Offsets are sorted and duplicates are removed. */
	unsigned int offsets[GPIO_V2_LINES_MAX];
	unsigned int num_offsets;
};

struct output_value {
	unsigned int offset;
	int value;
};

struct gpiod_line_config {
	bool too_complex;
	struct base_config primary;
	struct secondary_config secondary[GPIO_V2_LINE_NUM_ATTRS_MAX];
	unsigned int num_secondary;
	struct output_value output_values[GPIO_V2_LINES_MAX];
	unsigned int num_output_values;
	/*
	 * Used to temporarily store sorted offsets when looking for existing
	 * configuration
	 */
	unsigned int sorted_offsets[GPIO_V2_LINES_MAX];
	unsigned int num_sorted_offsets;
};

static void init_base_config(struct base_config *config)
{
	config->direction = GPIOD_LINE_DIRECTION_AS_IS;
	config->edge = GPIOD_LINE_EDGE_NONE;
	config->bias = GPIOD_LINE_BIAS_AS_IS;
	config->drive = GPIOD_LINE_DRIVE_PUSH_PULL;
	config->active_low = false;
	config->clock = GPIOD_LINE_EVENT_CLOCK_MONOTONIC;
	config->debounce_period = 0;
}

GPIOD_API struct gpiod_line_config *gpiod_line_config_new(void)
{
	struct gpiod_line_config *config;

	config = malloc(sizeof(*config));
	if (!config)
		return NULL;

	gpiod_line_config_reset(config);

	return config;
}

GPIOD_API void gpiod_line_config_free(struct gpiod_line_config *config)
{
	if (!config)
		return;

	free(config);
}

GPIOD_API void gpiod_line_config_reset(struct gpiod_line_config *config)
{
	int i;

	memset(config, 0, sizeof(*config));
	init_base_config(&config->primary);
	for (i = 0; i < GPIO_V2_LINE_NUM_ATTRS_MAX; i++)
		init_base_config(&config->secondary[i].config);
}

static int offset_compare(const void *a_ptr, const void *b_ptr)
{
	unsigned int a = *((unsigned int *)a_ptr);
	unsigned int b = *((unsigned int *)b_ptr);

	return a - b;
}

static void sanitize_offsets(struct gpiod_line_config *config,
			     unsigned int num_offsets,
			     const unsigned int *offsets)
{
	unsigned int i, count, *sorted = config->sorted_offsets;

	if (num_offsets == 0 || num_offsets == 1)
		return;

	count = num_offsets > GPIO_V2_LINES_MAX ? GPIO_V2_LINES_MAX
						: num_offsets;
	config->num_sorted_offsets = num_offsets;

	memcpy(config->sorted_offsets, offsets, count);
	qsort(sorted, count, sizeof(*sorted), offset_compare);

	for (i = 0; i < (count - 1); i++) {
		if (sorted[i] == sorted[i + 1]) {
			if (i < (count - 2))
				memmove(sorted + i + 1, sorted + i + 2,
					sizeof(*sorted) * num_offsets - i);
			config->num_sorted_offsets--;
		}
	}
}

static struct secondary_config *
find_matching_secondary_config(struct gpiod_line_config *config)
{
	unsigned int i, *offsets, num_offsets;
	struct secondary_config *secondary;

	offsets = config->sorted_offsets;
	num_offsets = config->num_sorted_offsets;

	for (i = 0; i < config->num_secondary; i++) {
		secondary = &config->secondary[i];

		if (num_offsets != secondary->num_offsets)
			continue;

		if (memcmp(secondary->offsets, offsets,
			   sizeof(*offsets) * num_offsets) == 0)
			return secondary;
	}

	return NULL;
}

static struct secondary_config *
get_secondary_config(struct gpiod_line_config *config,
		     unsigned int num_offsets, const unsigned int *offsets)
{
	struct secondary_config *secondary;

	if (config->too_complex)
		return NULL;

	sanitize_offsets(config, num_offsets, offsets);
	secondary = find_matching_secondary_config(config);
	if (!secondary) {
		if (config->num_secondary == GPIO_V2_LINE_NUM_ATTRS_MAX) {
			config->too_complex = true;
			return NULL;
		}

		secondary = &config->secondary[config->num_secondary++];
	}

	return secondary;
}

static struct base_config *
get_base_config_for_offset(struct gpiod_line_config *config,
			   unsigned int offset)
{
	struct secondary_config *secondary;
	unsigned int i, j;

	/*
	 * We're looking backwards as the settings get overwritten if set
	 * multiple times.
	 */
	for (i = config->num_secondary; i > 0; i--) {
		secondary = &config->secondary[i - 1];

		for (j = 0; j < secondary->num_offsets; j++) {
			if (secondary->offsets[j] == offset)
				return &secondary->config;
		}
	}

	return NULL;
}

static void set_direction(struct base_config *config, int direction)
{
	switch (direction) {
	case GPIOD_LINE_DIRECTION_INPUT:
	case GPIOD_LINE_DIRECTION_OUTPUT:
	case GPIOD_LINE_DIRECTION_AS_IS:
		config->direction = direction;
		break;
	default:
		config->direction = GPIOD_LINE_DIRECTION_AS_IS;
		break;
	}
}

GPIOD_API void
gpiod_line_config_set_direction(struct gpiod_line_config *config, int direction)
{
	set_direction(&config->primary, direction);
}

GPIOD_API void
gpiod_line_config_set_direction_offset(struct gpiod_line_config *config,
				       int direction, unsigned int offset)
{
	gpiod_line_config_set_direction_subset(config, direction, 1, &offset);
}

GPIOD_API void
gpiod_line_config_set_direction_subset(struct gpiod_line_config *config,
				       int direction, unsigned int num_offsets,
				       const unsigned int *offsets)
{
	struct secondary_config *secondary;

	secondary = get_secondary_config(config, num_offsets, offsets);
	if (!secondary)
		return;

	set_direction(&secondary->config, direction);
}

GPIOD_API int gpiod_line_config_get_direction(struct gpiod_line_config *config,
					      unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_offset(config, offset);
	if (!base)
		return config->primary.direction;

	return base->direction;
}

static void set_edge_detection(struct base_config *config, int edge)
{
	switch (edge) {
	case GPIOD_LINE_EDGE_NONE:
	case GPIOD_LINE_EDGE_RISING:
	case GPIOD_LINE_EDGE_FALLING:
	case GPIOD_LINE_EDGE_BOTH:
		config->edge = edge;
		break;
	default:
		config->edge = GPIOD_LINE_EDGE_NONE;
		break;
	}
}

GPIOD_API void
gpiod_line_config_set_edge_detection(struct gpiod_line_config *config, int edge)
{
	set_edge_detection(&config->primary, edge);
}

GPIOD_API void
gpiod_line_config_set_edge_detection_offset(struct gpiod_line_config *config,
					    int edge, unsigned int offset)
{
	gpiod_line_config_set_edge_detection_subset(config, edge, 1, &offset);
}

GPIOD_API void
gpiod_line_config_set_edge_detection_subset(struct gpiod_line_config *config,
					    int edge, unsigned int num_offsets,
					    const unsigned int *offsets)
{
	struct secondary_config *secondary;

	secondary = get_secondary_config(config, num_offsets, offsets);
	if (!secondary)
		return;

	set_edge_detection(&secondary->config, edge);
}

GPIOD_API int
gpiod_line_config_get_edge_detection(struct gpiod_line_config *config,
				     unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_offset(config, offset);
	if (!base)
		return config->primary.edge;

	return base->edge;
}

static void set_bias(struct base_config *config, int bias)
{
	switch (bias) {
	case GPIOD_LINE_BIAS_AS_IS:
	case GPIOD_LINE_BIAS_DISABLED:
	case GPIOD_LINE_BIAS_PULL_UP:
	case GPIOD_LINE_BIAS_PULL_DOWN:
		config->bias = bias;
		break;
	default:
		config->bias = GPIOD_LINE_BIAS_AS_IS;
		break;
	}
}

GPIOD_API void
gpiod_line_config_set_bias(struct gpiod_line_config *config, int bias)
{
	set_bias(&config->primary, bias);
}

GPIOD_API void
gpiod_line_config_set_bias_offset(struct gpiod_line_config *config,
				  int bias, unsigned int offset)
{
	gpiod_line_config_set_bias_subset(config, bias, 1, &offset);
}

GPIOD_API void
gpiod_line_config_set_bias_subset(struct gpiod_line_config *config,
				  int bias, unsigned int num_offsets,
				  const unsigned int *offsets)
{
	struct secondary_config *secondary;

	secondary = get_secondary_config(config, num_offsets, offsets);
	if (!secondary)
		return;

	set_bias(&secondary->config, bias);
}

GPIOD_API int gpiod_line_config_get_bias(struct gpiod_line_config *config,
					 unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_offset(config, offset);
	if (!base)
		return config->primary.bias;

	return base->bias;
}

static void set_drive(struct base_config *config, int drive)
{
	switch (drive) {
	case GPIOD_LINE_DRIVE_PUSH_PULL:
	case GPIOD_LINE_DRIVE_OPEN_DRAIN:
	case GPIOD_LINE_DRIVE_OPEN_SOURCE:
		config->drive = drive;
		break;
	default:
		config->drive = GPIOD_LINE_DRIVE_PUSH_PULL;
		break;
	}
}

GPIOD_API void
gpiod_line_config_set_drive(struct gpiod_line_config *config, int drive)
{
	set_drive(&config->primary, drive);
}

GPIOD_API void
gpiod_line_config_set_drive_offset(struct gpiod_line_config *config,
				   int drive, unsigned int offset)
{
	gpiod_line_config_set_drive_subset(config, drive, 1, &offset);
}

GPIOD_API void
gpiod_line_config_set_drive_subset(struct gpiod_line_config *config,
				   int drive, unsigned int num_offsets,
				   const unsigned int *offsets)
{
	struct secondary_config *secondary;

	secondary = get_secondary_config(config, num_offsets, offsets);
	if (!secondary)
		return;

	set_drive(&secondary->config, drive);
}

GPIOD_API int gpiod_line_config_get_drive(struct gpiod_line_config *config,
					  unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_offset(config, offset);
	if (!base)
		return config->primary.drive;

	return base->drive;
}

GPIOD_API void
gpiod_line_config_set_active_low(struct gpiod_line_config *config)
{
	config->primary.active_low = true;
}

GPIOD_API void
gpiod_line_config_set_active_low_offset(struct gpiod_line_config *config,
					unsigned int offset)
{
	gpiod_line_config_set_active_low_subset(config, 1, &offset);
}

GPIOD_API void
gpiod_line_config_set_active_low_subset(struct gpiod_line_config *config,
					unsigned int num_offsets,
					const unsigned int *offsets)
{
	struct secondary_config *secondary;

	secondary = get_secondary_config(config, num_offsets, offsets);
	if (!secondary)
		return;

	secondary->config.active_low = true;
}

GPIOD_API bool gpiod_line_config_is_active_low(struct gpiod_line_config *config,
					       unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_offset(config, offset);
	if (!base)
		return config->primary.active_low;

	return base->active_low;
}

GPIOD_API void
gpiod_line_config_set_active_high(struct gpiod_line_config *config)
{
	config->primary.active_low = false;
}

GPIOD_API void
gpiod_line_config_set_active_high_offset(struct gpiod_line_config *config,
					 unsigned int offset)
{
	gpiod_line_config_set_active_high_subset(config, 1, &offset);
}

GPIOD_API void
gpiod_line_config_set_active_high_subset(struct gpiod_line_config *config,
					 unsigned int num_offsets,
					 const unsigned int *offsets)
{
	struct secondary_config *secondary;

	secondary = get_secondary_config(config, num_offsets, offsets);
	if (!secondary)
		return;

	secondary->config.active_low = false;
}

GPIOD_API void
gpiod_line_config_set_debounce_period_us(struct gpiod_line_config *config,
				      unsigned long period)
{
	config->primary.debounce_period = period;
}

GPIOD_API void
gpiod_line_config_set_debounce_period_us_offset(
					struct gpiod_line_config *config,
					unsigned long period,
					unsigned int offset)
{
	gpiod_line_config_set_debounce_period_us_subset(config, period,
						     1, &offset);
}

GPIOD_API void
gpiod_line_config_set_debounce_period_us_subset(
					struct gpiod_line_config *config,
					unsigned long period,
					unsigned int num_offsets,
					const unsigned int *offsets)
{
	struct secondary_config *secondary;

	secondary = get_secondary_config(config, num_offsets, offsets);
	if (!secondary)
		return;

	secondary->config.debounce_period = period;
}

GPIOD_API unsigned long
gpiod_line_config_get_debounce_us_period(struct gpiod_line_config *config,
					 unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_offset(config, offset);
	if (!base)
		return config->primary.debounce_period;

	return base->debounce_period;
}

static void set_event_clock(struct base_config *config, int clock)
{
	switch (clock) {
	case GPIOD_LINE_EVENT_CLOCK_MONOTONIC:
	case GPIOD_LINE_EVENT_CLOCK_REALTIME:
		config->clock = clock;
		break;
	default:
		config->clock = GPIOD_LINE_EVENT_CLOCK_MONOTONIC;
		break;
	}
}

GPIOD_API void
gpiod_line_config_set_event_clock(struct gpiod_line_config *config, int clock)
{
	set_event_clock(&config->primary, clock);
}

GPIOD_API void
gpiod_line_config_set_event_clock_offset(struct gpiod_line_config *config,
					 int clock, unsigned int offset)
{
	gpiod_line_config_set_event_clock_subset(config, clock, 1, &offset);
}

GPIOD_API void
gpiod_line_config_set_event_clock_subset(struct gpiod_line_config *config,
					 int clock, unsigned int num_offsets,
					 const unsigned int *offsets)
{
	struct secondary_config *secondary;

	secondary = get_secondary_config(config, num_offsets, offsets);
	if (!secondary)
		return;

	set_event_clock(&secondary->config, clock);
}

GPIOD_API void
gpiod_line_config_set_output_value(struct gpiod_line_config *config,
				   unsigned int offset, int value)
{
	gpiod_line_config_set_output_values(config, 1, &offset, &value);
}

static int output_value_find_offset(struct gpiod_line_config *config,
				    unsigned int offset)
{
	unsigned int i;

	for (i = 0; i < config->num_output_values; i++) {
		if (config->output_values[i].offset == offset)
			return i;
	}

	return -1;
}

static void set_output_value(struct gpiod_line_config *config, unsigned int idx,
			     unsigned int offset, int value, bool inc)
{
	config->output_values[idx].offset = offset;
	config->output_values[idx].value = value;

	if (inc)
		config->num_output_values++;
}

GPIOD_API void
gpiod_line_config_set_output_values(struct gpiod_line_config *config,
				    unsigned int num_values,
				    const unsigned int *offsets,
				    const int *values)
{
	unsigned int i;
	int pos;

	if (config->too_complex)
		return;

	for (i = 0; i < num_values; i++) {
		pos = output_value_find_offset(config, offsets[i]);
		if (pos < 0) {
			if (config->num_output_values == GPIO_V2_LINES_MAX) {
				/* Too many output values specified. */
				config->too_complex = true;
				return;
			}

			/* Add new output value. */
			set_output_value(config, config->num_output_values,
					 offsets[i], values[i], true);
		} else {
			/* Overwrite old value for this offset. */
			set_output_value(config, pos,
					 offsets[i], values[i], false);
		}
	}
}

GPIOD_API unsigned int
gpiod_line_config_num_output_values(struct gpiod_line_config *config)
{
	return config->num_output_values;
}

GPIOD_API int
gpiod_line_config_get_output_value(struct gpiod_line_config *config,
				   unsigned int offset)
{
	unsigned int i;

	for (i = 0; i < config->num_output_values; i++) {
		if (config->output_values[i].offset == offset)
			return config->output_values[i].value;
	}

	errno = ENXIO;
	return -1;
}

GPIOD_API int
gpiod_line_config_get_output_value_index(struct gpiod_line_config *config,
					 unsigned int index,
					 unsigned int *offset, int *value)
{
	if (index >= config->num_output_values) {
		errno = EINVAL;
		return -1;
	}

	*offset = config->output_values[index].offset;
	*value = config->output_values[index].value;

	return 0;
}

GPIOD_API void
gpiod_line_config_get_output_values(struct gpiod_line_config *config,
				    unsigned int *offsets, int *values)
{
	unsigned int i;

	for (i = 0; i < config->num_output_values; i++) {
		offsets[i] = config->output_values[i].offset;
		values[i] = config->output_values[i].value;
	}
}

static uint64_t gpiod_make_kernel_flags(struct base_config *config)
{
	uint64_t flags = 0;

	switch (config->direction) {
	case GPIOD_LINE_DIRECTION_INPUT:
		flags |= GPIO_V2_LINE_FLAG_INPUT;
		break;
	case GPIOD_LINE_DIRECTION_OUTPUT:
		flags |= GPIO_V2_LINE_FLAG_OUTPUT;
		break;
	}

	switch (config->edge) {
	case GPIOD_LINE_EDGE_FALLING:
		flags |= (GPIO_V2_LINE_FLAG_EDGE_FALLING |
			   GPIO_V2_LINE_FLAG_INPUT);
		flags &= ~GPIOD_LINE_DIRECTION_OUTPUT;
		break;
	case GPIOD_LINE_EDGE_RISING:
		flags |= (GPIO_V2_LINE_FLAG_EDGE_RISING |
			   GPIO_V2_LINE_FLAG_INPUT);
		flags &= ~GPIOD_LINE_DIRECTION_OUTPUT;
		break;
	case GPIOD_LINE_EDGE_BOTH:
		flags |= (GPIO_V2_LINE_FLAG_EDGE_FALLING |
			   GPIO_V2_LINE_FLAG_EDGE_RISING |
			   GPIO_V2_LINE_FLAG_INPUT);
		flags &= ~GPIOD_LINE_DIRECTION_OUTPUT;
		break;
	}

	switch (config->drive) {
	case GPIOD_LINE_DRIVE_OPEN_DRAIN:
		flags |= GPIO_V2_LINE_FLAG_OPEN_DRAIN;
		break;
	case GPIOD_LINE_DRIVE_OPEN_SOURCE:
		flags |= GPIO_V2_LINE_FLAG_OPEN_SOURCE;
		break;
	}

	switch (config->bias) {
	case GPIOD_LINE_BIAS_DISABLED:
		flags |= GPIO_V2_LINE_FLAG_BIAS_DISABLED;
		break;
	case GPIOD_LINE_BIAS_PULL_UP:
		flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
		break;
	case GPIOD_LINE_BIAS_PULL_DOWN:
		flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
		break;
	}

	if (config->active_low)
		flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;

	switch (config->clock) {
	case GPIOD_LINE_EVENT_CLOCK_REALTIME:
		flags |= GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME;
		break;
	}

	return flags;
}

static int find_bitmap_index(unsigned int needle, unsigned int num_lines,
			     const unsigned int *haystack)
{
	unsigned int i;

	for (i = 0; i < num_lines; i++) {
		if (needle == haystack[i])
			return i;
	}

	return -1;
}

static int set_kernel_output_values(uint64_t *mask, uint64_t *vals,
				    struct gpiod_line_config *config,
				    unsigned int num_lines,
				    const unsigned int *offsets)
{
	struct output_value *outval;
	unsigned int i;
	int idx;

	gpiod_line_mask_zero(mask);
	gpiod_line_mask_zero(vals);

	for (i = 0; i < config->num_output_values; i++) {
		outval = &config->output_values[i];

		idx = find_bitmap_index(outval->offset, num_lines, offsets);
		if (idx < 0) {
			errno = EINVAL;
			return -1;
		}

		gpiod_line_mask_set_bit(mask, idx);
		gpiod_line_mask_assign_bit(vals, idx, !!outval->value);
	}

	return 0;
}

static int set_secondary_mask(uint64_t *mask,
			      struct secondary_config *sec_cfg,
			      unsigned int num_lines,
			      const unsigned int *offsets)
{
	unsigned int i;
	int idx;

	gpiod_line_mask_zero(mask);

	for (i = 0; i < sec_cfg->num_offsets; i++) {
		idx = find_bitmap_index(sec_cfg->offsets[i],
					num_lines, offsets);
		if (idx < 0) {
			errno = EINVAL;
			return -1;
		}

		gpiod_line_mask_set_bit(mask, idx);
	}

	return 0;
}

int gpiod_line_config_to_kernel(struct gpiod_line_config *config,
				struct gpio_v2_line_config *cfgbuf,
				unsigned int num_lines,
				const unsigned int *offsets)
{
	struct gpio_v2_line_config_attribute *attr;
	struct secondary_config *sec_cfg;
	unsigned int attr_idx = 0, i;
	uint64_t mask, values;
	int ret;

	if (!config) {
		cfgbuf->flags = GPIO_V2_LINE_FLAG_INPUT;
		return 0;
	}

	if (config->too_complex)
		goto err_2big;

	if (config->num_output_values) {
		if (config->num_output_values > num_lines)
			goto err_2big;

		attr = &cfgbuf->attrs[attr_idx++];
		attr->attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;

		ret = set_kernel_output_values(&mask, &values, config,
					       num_lines, offsets);
		if (ret)
			return ret;

		attr->attr.values = values;
		attr->mask = mask;
	}

	if (config->primary.debounce_period) {
		attr = &cfgbuf->attrs[attr_idx++];
		attr->attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
		attr->attr.debounce_period_us = config->primary.debounce_period;
		gpiod_line_mask_fill(&mask);
		attr->mask = mask;
	}

	for (i = 0; i < config->num_secondary; i++, attr_idx++) {
		if (attr_idx == GPIO_V2_LINE_NUM_ATTRS_MAX)
			goto err_2big;

		sec_cfg = &config->secondary[i];
		attr = &cfgbuf->attrs[attr_idx];

		if (sec_cfg->num_offsets > num_lines)
			goto err_2big;

		if (sec_cfg->config.debounce_period) {
			attr->attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
			attr->attr.debounce_period_us =
					sec_cfg->config.debounce_period;
		} else {
			attr->attr.id = GPIO_V2_LINE_ATTR_ID_FLAGS;

			attr->attr.flags = gpiod_make_kernel_flags(
							&sec_cfg->config);
		}

		ret = set_secondary_mask(&mask, sec_cfg, num_lines, offsets);
		if (ret)
			return -1;

		attr->mask = mask;
	}

	cfgbuf->flags = gpiod_make_kernel_flags(&config->primary);
	cfgbuf->num_attrs = attr_idx;

	return 0;

err_2big:
	config->too_complex = true;
	errno = E2BIG;
	return -1;
}
