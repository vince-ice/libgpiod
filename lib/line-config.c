// SPDX-License-Identifier: LGPL-2.1-or-later
// SPDX-FileCopyrightText: 2021 Bartosz Golaszewski <brgl@bgdev.pl>

/* Line configuration data structure and functions. */

#include <errno.h>
#include <gpiod.h>
#include <stdlib.h>
#include <string.h>

#include "internal.h"

struct base_config {
	unsigned int direction : 2;
	unsigned int edge : 3;
	unsigned int drive : 2;
	unsigned int bias : 3;
	bool active_low : 1;
	unsigned int clock : 2;
	unsigned long debounce_period_us;
} GPIOD_PACKED;

#define OVERRIDE_FLAG_DIRECTION		GPIOD_BIT(0)
#define OVERRIDE_FLAG_EDGE		GPIOD_BIT(1)
#define OVERRIDE_FLAG_DRIVE		GPIOD_BIT(2)
#define OVERRIDE_FLAG_BIAS		GPIOD_BIT(3)
#define OVERRIDE_FLAG_ACTIVE_LOW	GPIOD_BIT(4)
#define OVERRIDE_FLAG_CLOCK		GPIOD_BIT(5)
#define OVERRIDE_FLAG_DEBOUNCE_PERIOD	GPIOD_BIT(6)

/*
 * Config overriding the defaults for a single line offset. Only flagged
 * settings are actually overriden for a line.
 */
struct override_config {
	struct base_config base;
	unsigned int offset;
	bool value_set : 1;
	unsigned int value : 1;
	unsigned int override_flags : 7;
} GPIOD_PACKED;

struct gpiod_line_config {
	bool too_complex;
	struct base_config defaults;
	struct override_config overrides[GPIO_V2_LINES_MAX];
	unsigned int num_overrides;
};

static void init_base_config(struct base_config *config)
{
	config->direction = GPIOD_LINE_DIRECTION_AS_IS;
	config->edge = GPIOD_LINE_EDGE_NONE;
	config->bias = GPIOD_LINE_BIAS_AS_IS;
	config->drive = GPIOD_LINE_DRIVE_PUSH_PULL;
	config->active_low = false;
	config->clock = GPIOD_LINE_EVENT_CLOCK_MONOTONIC;
	config->debounce_period_us = 0;
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
	free(config);
}

GPIOD_API void gpiod_line_config_reset(struct gpiod_line_config *config)
{
	int i;

	memset(config, 0, sizeof(*config));
	init_base_config(&config->defaults);
	for (i = 0; i < GPIO_V2_LINE_NUM_ATTRS_MAX; i++)
		init_base_config(&config->overrides[i].base);
}

static struct override_config *
get_override_by_offset(struct gpiod_line_config *config, unsigned int offset)
{
	struct override_config *override;
	unsigned int i;

	for (i = 0; i < config->num_overrides; i++) {
		override = &config->overrides[i];

		if (override->offset == offset)
			return override;
	}

	return NULL;
}

static struct override_config *
get_new_override(struct gpiod_line_config *config, unsigned int offset)
{
	struct override_config *override;

	if (config->num_overrides == GPIO_V2_LINES_MAX) {
		config->too_complex = true;
		return NULL;
	}

	override = &config->overrides[config->num_overrides++];
	override->offset = offset;

	return override;
}

static struct override_config *
get_override_config_for_writing(struct gpiod_line_config *config,
				unsigned int offset)
{
	struct override_config *override;

	if (config->too_complex)
		return NULL;

	override = get_override_by_offset(config, offset);
	if (!override) {
		override = get_new_override(config, offset);
		if (!override)
			return NULL;
	}

	return override;
}

static struct base_config *
get_base_config_for_reading(struct gpiod_line_config *config,
			   unsigned int offset, unsigned int flag)
{
	struct override_config *override;

	override = get_override_by_offset(config, offset);
	if (override && (override->override_flags & flag))
		return &override->base;

	return &config->defaults;
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
	set_direction(&config->defaults, direction);
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
	struct override_config *override;
	unsigned int i;

	for (i = 0; i < num_offsets; i++) {
		override = get_override_config_for_writing(config, offsets[i]);
		if (!override)
			return;

		set_direction(&override->base, direction);
		override->override_flags |= OVERRIDE_FLAG_DIRECTION;
	}
}

GPIOD_API int gpiod_line_config_get_direction(struct gpiod_line_config *config,
					      unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_reading(config, offset,
					   OVERRIDE_FLAG_DIRECTION);

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
	set_edge_detection(&config->defaults, edge);
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
	struct override_config *override;
	unsigned int i;

	for (i = 0; i < num_offsets; i++) {
		override = get_override_config_for_writing(config, offsets[i]);
		if (!override)
			return;

		set_edge_detection(&override->base, edge);
		override->override_flags |= OVERRIDE_FLAG_EDGE;
	}
}

GPIOD_API int
gpiod_line_config_get_edge_detection(struct gpiod_line_config *config,
				     unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_reading(config, offset, OVERRIDE_FLAG_EDGE);

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
	set_bias(&config->defaults, bias);
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
	struct override_config *override;
	unsigned int i;

	for (i = 0; i < num_offsets; i++) {
		override = get_override_config_for_writing(config, offsets[i]);
		if (!override)
			return;

		set_bias(&override->base, bias);
		override->override_flags |= OVERRIDE_FLAG_BIAS;
	}
}

GPIOD_API int gpiod_line_config_get_bias(struct gpiod_line_config *config,
					 unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_reading(config, offset, OVERRIDE_FLAG_BIAS);

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
	set_drive(&config->defaults, drive);
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
	struct override_config *override;
	unsigned int i;

	for (i = 0; i < num_offsets; i++) {
		override = get_override_config_for_writing(config, offsets[i]);
		if (!override)
			return;

		set_drive(&override->base, drive);
		override->override_flags |= OVERRIDE_FLAG_DRIVE;
	}
}

GPIOD_API int gpiod_line_config_get_drive(struct gpiod_line_config *config,
					  unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_reading(config, offset, OVERRIDE_FLAG_DRIVE);

	return base->drive;
}

GPIOD_API void
gpiod_line_config_set_active_low(struct gpiod_line_config *config)
{
	config->defaults.active_low = true;
}

GPIOD_API void
gpiod_line_config_set_active_low_offset(struct gpiod_line_config *config,
					unsigned int offset)
{
	gpiod_line_config_set_active_low_subset(config, 1, &offset);
}

static void
gpiod_line_config_set_active_setting(struct gpiod_line_config *config,
				     unsigned int num_offsets,
				     const unsigned int *offsets,
				     bool active_low)
{
	struct override_config *override;
	unsigned int i;

	for (i = 0; i < num_offsets; i++) {
		override = get_override_config_for_writing(config, offsets[i]);
		if (!override)
			return;

		override->base.active_low = active_low;
		override->override_flags |= OVERRIDE_FLAG_ACTIVE_LOW;
	}
}

GPIOD_API void
gpiod_line_config_set_active_low_subset(struct gpiod_line_config *config,
					unsigned int num_offsets,
					const unsigned int *offsets)
{
	gpiod_line_config_set_active_setting(config,
					     num_offsets, offsets, true);
}

GPIOD_API bool
gpiod_line_config_get_active_low(struct gpiod_line_config *config,
				 unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_reading(config, offset,
					   OVERRIDE_FLAG_ACTIVE_LOW);

	return base->active_low;
}

GPIOD_API void
gpiod_line_config_set_active_high(struct gpiod_line_config *config)
{
	config->defaults.active_low = false;
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
	gpiod_line_config_set_active_setting(config,
					     num_offsets, offsets, false);
}

GPIOD_API void
gpiod_line_config_set_debounce_period_us(struct gpiod_line_config *config,
				      unsigned long period)
{
	config->defaults.debounce_period_us = period;
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
	struct override_config *override;
	unsigned int i, offset;

	for (i = 0; i < num_offsets; i++) {
		offset = offsets[i];

		override = get_override_config_for_writing(config, offset);
		if (!override)
			return;

		override->base.debounce_period_us = period;
		override->override_flags |= OVERRIDE_FLAG_DEBOUNCE_PERIOD;
	}
}

GPIOD_API unsigned long
gpiod_line_config_get_debounce_us_period(struct gpiod_line_config *config,
					 unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_reading(config, offset,
					   OVERRIDE_FLAG_DEBOUNCE_PERIOD);

	return base->debounce_period_us;
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
	set_event_clock(&config->defaults, clock);
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
	struct override_config *override;
	unsigned int i;

	for (i = 0; i < num_offsets; i++) {
		override = get_override_config_for_writing(config, offsets[i]);
		if (!override)
			return;

		set_event_clock(&override->base, clock);
	}
}

GPIOD_API int
gpiod_line_config_get_event_clock(struct gpiod_line_config *config,
				  unsigned int offset)
{
	struct base_config *base;

	base = get_base_config_for_reading(config, offset, OVERRIDE_FLAG_CLOCK);

	return base->clock;
}

static void set_output_value(struct override_config *override, int value)
{
	override->value = !!value;
	override->value_set = true;
}

GPIOD_API void
gpiod_line_config_set_output_value(struct gpiod_line_config *config,
				   unsigned int offset, int value)
{
	gpiod_line_config_set_output_values(config, 1, &offset, &value);
}

GPIOD_API void
gpiod_line_config_set_output_values(struct gpiod_line_config *config,
				    unsigned int num_values,
				    const unsigned int *offsets,
				    const int *values)
{
	struct override_config *override;
	unsigned int i, offset, val;

	for (i = 0; i < num_values; i++) {
		offset = offsets[i];
		val = values[i];

		override = get_override_by_offset(config, offset);
		if (!override) {
			override = get_new_override(config, offset);
			if (!override)
				return;
		}

		set_output_value(override, val);
	}
}

GPIOD_API int
gpiod_line_config_get_output_value(struct gpiod_line_config *config,
				   unsigned int offset)
{
	struct override_config *override;

	override = get_override_by_offset(config, offset);
	if (override && override->value_set)
		return override->value;

	errno = ENXIO;
	return -1;
}

static uint64_t make_kernel_flags(const struct base_config *config)
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
	struct override_config *override;
	unsigned int i;
	int idx;

	gpiod_line_mask_zero(mask);
	gpiod_line_mask_zero(vals);

	for (i = 0; i < config->num_overrides; i++) {
		override = &config->overrides[i];

		if (override->value_set) {
			idx = find_bitmap_index(override->offset,
						num_lines, offsets);
			if (idx < 0) {
				errno = EINVAL;
				return -1;
			}

			gpiod_line_mask_set_bit(mask, idx);
			gpiod_line_mask_assign_bit(vals, idx,
						   !!override->value);
		}
	}

	return 0;
}

static bool base_config_flags_are_equal(struct base_config *base,
					struct override_config *override)
{
	if (((override->override_flags & OVERRIDE_FLAG_DIRECTION) &&
	     base->direction != override->base.direction) ||
	    ((override->override_flags & OVERRIDE_FLAG_EDGE) &&
	     base->edge != override->base.edge) ||
	    ((override->override_flags & OVERRIDE_FLAG_DRIVE) &&
	     base->drive != override->base.drive) ||
	    ((override->override_flags & OVERRIDE_FLAG_BIAS) &&
	     base->bias != override->base.bias) ||
	    ((override->override_flags & OVERRIDE_FLAG_ACTIVE_LOW) &&
	     base->active_low != override->base.active_low) ||
	    ((override->override_flags & OVERRIDE_FLAG_CLOCK) &&
	     base->clock != override->base.clock))
		return false;

	return true;
}

static bool override_config_flags_are_equal(struct override_config *a,
					    struct override_config *b)
{
	if (((a->override_flags & ~OVERRIDE_FLAG_DEBOUNCE_PERIOD) ==
	     (b->override_flags & ~OVERRIDE_FLAG_DEBOUNCE_PERIOD)) &&
	    base_config_flags_are_equal(&a->base, b))
		return true;

	return false;
}

static void set_base_config_flags(struct gpio_v2_line_attribute *attr,
				  struct override_config *override,
				  struct gpiod_line_config *config)
{
	struct base_config base;

	memcpy(&base, &config->defaults, sizeof(base));

	if (override->override_flags & OVERRIDE_FLAG_DIRECTION)
		base.direction = override->base.direction;
	if (override->override_flags & OVERRIDE_FLAG_EDGE)
		base.edge = override->base.edge;
	if (override->override_flags & OVERRIDE_FLAG_BIAS)
		base.bias = override->base.bias;
	if (override->override_flags & OVERRIDE_FLAG_DRIVE)
		base.drive = override->base.drive;
	if (override->override_flags & OVERRIDE_FLAG_ACTIVE_LOW)
		base.active_low = override->base.active_low;
	if (override->override_flags & OVERRIDE_FLAG_CLOCK)
		base.clock = override->base.clock;

	attr->id = GPIO_V2_LINE_ATTR_ID_FLAGS;
	attr->flags = make_kernel_flags(&base);
}

static bool base_debounce_period_is_equal(struct base_config *base,
					  struct override_config *override)
{
	if ((override->override_flags & OVERRIDE_FLAG_DEBOUNCE_PERIOD) &&
	    base->debounce_period_us != override->base.debounce_period_us)
		return false;

	return true;
}

static bool override_config_debounce_period_is_equal(struct override_config *a,
						     struct override_config *b)
{
	if (base_debounce_period_is_equal(&a->base, b) &&
	    ((a->override_flags & OVERRIDE_FLAG_DEBOUNCE_PERIOD) ==
	     (b->override_flags & OVERRIDE_FLAG_DEBOUNCE_PERIOD)))
		return true;

	return false;
}

static void
set_base_config_debounce_period(struct gpio_v2_line_attribute *attr,
				struct override_config *override,
				struct gpiod_line_config *config GPIOD_UNUSED)
{
	attr->id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
	attr->debounce_period_us = override->base.debounce_period_us;
}

static int set_kernel_attr_mask(uint64_t *out, const uint64_t *in,
				unsigned int num_lines,
				const unsigned int *offsets,
				const struct gpiod_line_config *config)
{
	unsigned int i, j;
	int off;

	gpiod_line_mask_zero(out);

	for (i = 0; i < config->num_overrides; i++) {
		if (!gpiod_line_mask_test_bit(in, i))
			continue;

		for (j = 0, off = -1; j < num_lines; j++) {
			if (offsets[j] == config->overrides[i].offset) {
				off = j;
				break;
			}
		}

		if (off < 0) {
			errno = EINVAL;
			return -1;
		}

		gpiod_line_mask_set_bit(out, off);
	}

	return 0;
}

static int process_overrides(struct gpiod_line_config *config,
			     struct gpio_v2_line_config *cfgbuf,
			     unsigned int *attr_idx,
			     unsigned int num_lines,
			     const unsigned int *offsets,
			     bool (*defaults_equal_func)(struct base_config *,
						struct override_config *),
			     bool (*override_equal_func)(
						struct override_config *,
						struct override_config *),
			     void (*set_func)(struct gpio_v2_line_attribute *,
					      struct override_config *,
					      struct gpiod_line_config *))
{
	struct gpio_v2_line_config_attribute *attr;
	uint64_t processed = 0, marked = 0, mask;
	struct override_config *current, *next;
	unsigned int i, j;

	for (i = 0; i < config->num_overrides; i++) {
		if (gpiod_line_mask_test_bit(&processed, i))
			continue;

		current = &config->overrides[i];
		gpiod_line_mask_set_bit(&processed, i);

		if (defaults_equal_func(&config->defaults, current))
			continue;

		marked = 0;
		gpiod_line_mask_set_bit(&marked, i);

		for (j = i + 1; j < config->num_overrides; j++) {
			if (gpiod_line_mask_test_bit(&processed, j))
				continue;

			next = &config->overrides[j];

			if (override_equal_func(current, next)) {
				gpiod_line_mask_set_bit(&marked, j);
				gpiod_line_mask_set_bit(&processed, j);
			}
		}

		attr = &cfgbuf->attrs[(*attr_idx)++];
		if (*attr_idx == GPIO_V2_LINE_NUM_ATTRS_MAX) {
			errno = E2BIG;
			return -1;
		}

		set_kernel_attr_mask(&mask, &marked,
				     num_lines, offsets, config);
		attr->mask = mask;
		set_func(&attr->attr, current, config);
	}

	return 0;
}

int gpiod_line_config_to_kernel(struct gpiod_line_config *config,
				struct gpio_v2_line_config *cfgbuf,
				unsigned int num_lines,
				const unsigned int *offsets)
{
	struct gpio_v2_line_config_attribute *attr;
	struct override_config *override;
	unsigned int attr_idx = 0, i;
	uint64_t mask, values;
	int ret;

	if (!config) {
		cfgbuf->flags = GPIO_V2_LINE_FLAG_INPUT;
		return 0;
	}

	if (config->too_complex)
		goto err_2big;

	/*
	 * First check if we have at least one default output value configured.
	 * If so, let's take one attribute for the default values.
	 */
	for (i = 0; i < config->num_overrides; i++) {
		override = &config->overrides[i];

		if (override->value_set) {
			attr = &cfgbuf->attrs[attr_idx++];
			attr->attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;

			ret = set_kernel_output_values(&mask, &values, config,
						       num_lines, offsets);
			if (ret)
				return ret;

			attr->attr.values = values;
			attr->mask = mask;

			break;
		}
	}

	/* If we have a default debounce period - use another attribute. */
	if (config->defaults.debounce_period_us) {
		attr = &cfgbuf->attrs[attr_idx++];
		attr->attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
		attr->attr.debounce_period_us =
				config->defaults.debounce_period_us;
		gpiod_line_mask_fill(&mask);
		attr->mask = mask;
	}

	/*
	 * The overrides are processed independently for regular flags and the
	 * debounce period. We iterate over the configured line overrides. We
	 * first check if the given set of options is equal to the global
	 * defaults. If not, we mark it and iterate over the remaining
	 * overrides looking for ones that have the same config as the one
	 * currently processed. We mark them too and at the end we create a
	 * single kernel attribute with the translated config and the mask
	 * corresponding to all marked overrides. Those are now excluded from
	 * further processing.
	 */

	ret = process_overrides(config, cfgbuf, &attr_idx, num_lines, offsets,
				base_config_flags_are_equal,
				override_config_flags_are_equal,
				set_base_config_flags);
	if (ret)
		return -1;

	ret = process_overrides(config, cfgbuf, &attr_idx, num_lines, offsets,
				base_debounce_period_is_equal,
				override_config_debounce_period_is_equal,
				set_base_config_debounce_period);
	if (ret)
		return -1;

	cfgbuf->flags = make_kernel_flags(&config->defaults);
	cfgbuf->num_attrs = attr_idx;

	return 0;

err_2big:
	config->too_complex = true;
	errno = E2BIG;
	return -1;
}
