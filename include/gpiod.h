/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* SPDX-FileCopyrightText: 2017-2021 Bartosz Golaszewski <bartekgola@gmail.com> */
/* SPDX-FileCopyrightText: 2021 Bartosz Golaszewski <brgl@bgdev.pl> */

/**
 * @file gpiod.h
 */

#ifndef __LIBGPIOD_GPIOD_H__
#define __LIBGPIOD_GPIOD_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @mainpage libgpiod public API
 *
 * This is the complete documentation of the public API made available to
 * users of libgpiod.
 *
 * <p>The API is logically split into several parts such as: GPIO chip & line
 * operators, GPIO events handling etc.
 *
 * <p>General note on error handling: all routines exported by libgpiod that
 * can fail, set errno to one of the error values defined in errno.h upon
 * failure. The way of notifying the caller that an error occurred varies
 * between functions, but in general a function that returns an int, returns -1
 * on error, while a function returning a pointer indicates an error condition
 * by returning a NULL pointer. It's not practical to list all possible error
 * codes for every function as they propagate errors from the underlying libc
 * functions.
 *
 * <p>In general libgpiod functions are not NULL-aware and it's expected that
 * users pass valid pointers to objects as arguments. An exception to this rule
 * are the functions that free/close/release resources - which work when passed
 * a NULL-pointer as argument. Other exceptions are documented.
 */

struct gpiod_chip;
struct gpiod_line_info;
struct gpiod_line_config;
struct gpiod_request_config;
struct gpiod_line_request;
struct gpiod_info_event;
struct gpiod_edge_event;
struct gpiod_edge_event_buffer;

/**
 * @defgroup chips GPIO chips
 * @{
 *
 * Functions and data structures for GPIO chip operations.
 *
 * A GPIO chip object is associated with an open file descriptor to the GPIO
 * character device. It exposes basic information about the chip and allows
 * callers to retrieve information about each line, watch lines for state
 * changes and make line requests.
 */

/**
 * @brief Open a GPIO chip by path.
 * @param path Path to the gpiochip device file.
 * @return GPIO chip request or NULL if an error occurred.
 */
struct gpiod_chip *gpiod_chip_open(const char *path);

/**
 * @brief Close a GPIO chip and release all associated resources.
 * @param chip Chip to close.
 */
void gpiod_chip_close(struct gpiod_chip *chip);

/**
 * @brief Get the GPIO chip name as represented in the kernel.
 * @param chip GPIO chip object.
 * @return Pointer to a human-readable string containing the chip name.
 */
const char *gpiod_chip_get_name(struct gpiod_chip *chip);

/**
 * @brief Get the GPIO chip label as represented in the kernel.
 * @param chip GPIO chip object.
 * @return Pointer to a human-readable string containing the chip label.
 */
const char *gpiod_chip_get_label(struct gpiod_chip *chip);

/**
 * @brief Get the path used to open this GPIO chip.
 * @param chip GPIO chip object.
 * @return Path to the file passed as argument to ::gpiod_chip_open.
 */
const char *gpiod_chip_get_path(struct gpiod_chip *chip);

/**
 * @brief Get the number of GPIO lines exposed by this chip.
 * @param chip GPIO chip object.
 * @return Number of GPIO lines.
 */
unsigned int gpiod_chip_get_num_lines(struct gpiod_chip *chip);

/**
 * @brief Get the current snapshot of information about the line at given
 *        offset.
 * @param chip GPIO chip object.
 * @param offset The offset of the GPIO line.
 * @return New GPIO line info object or NULL if an error occurred. The returned
 *         object must be freed by the caller using ::gpiod_line_info_free.
 */
struct gpiod_line_info *gpiod_chip_get_line_info(struct gpiod_chip *chip,
						 unsigned int offset);

/**
 * @brief Get the current snapshot of information about the line at given
 *        offset and start watching it for future changes.
 * @param chip GPIO chip object.
 * @param offset The offset of the GPIO line.
 * @return New GPIO line info object or NULL if an error occurred. The returned
 *         object must be freed by the caller using ::gpiod_line_info_free.
 */
struct gpiod_line_info *gpiod_chip_watch_line_info(struct gpiod_chip *chip,
						   unsigned int offset);

/**
 * @brief Stop watching the line at given offset for status changes.
 * @param chip GPIO chip object.
 * @param offset The offset of the GPIO line.
 * @return 0 on success, -1 on failure.
 */
int gpiod_chip_unwatch_line_info(struct gpiod_chip *chip, unsigned int offset);

/**
 * @brief Get the file descriptor associated with this chip.
 * @param chip GPIO chip object.
 * @return File descriptor number. This function never fails. The returned file
 *         descriptor must not be closed by the caller.
 */
int gpiod_chip_get_fd(struct gpiod_chip *chip);

/**
 * @brief Wait for line status events on any of the watched lines exposed by
 *        this chip.
 * @param chip GPIO chip object.
 * @param timeout Wait time limit in nanoseconds.
 * @return 0 if wait timed out, -1 if an error occurred, 1 if an event is
 *         pending.
 */
int gpiod_chip_info_event_wait(struct gpiod_chip *chip, uint64_t timeout);

/**
 * @brief Read a single line status change event from this chip.
 * @param chip GPIO chip object.
 * @return Newly read watch event object or NULL on error. The event must be
 *         freed by the caller using ::gpiod_info_event_free.
 * @note If no events are pending, this function will block.
 */
struct gpiod_info_event *gpiod_chip_info_event_read(struct gpiod_chip *chip);

/**
 * @brief Map a GPIO line's name to its offset within the chip.
 * @param chip GPIO chip object.
 * @param name Name of the GPIO line to map.
 * @return Offset of the line within the chip or -1 on error.
 * @note If a line with given name is not exposed by the chip, the function
 *       sets errno to ENOENT.
 */
int gpiod_chip_find_line(struct gpiod_chip *chip, const char *name);

/**
 * @brief Request a set of lines for exclusive usage.
 * @param chip GPIO chip object.
 * @param req_cfg Request config object.
 * @param line_cfg Line config object.
 * @return New line request object or NULL if an error occurred. The request
 *         must be released by the caller using ::gpiod_line_request_release.
 */
struct gpiod_line_request *
gpiod_chip_request_lines(struct gpiod_chip *chip,
			 struct gpiod_request_config *req_cfg,
			 struct gpiod_line_config *line_cfg);

/**
 * @}
 *
 * @defgroup line_settings Line settings
 * @{
 *
 * These defines are used both by gpiod_line_info and gpiod_line_config.
 */

/**
 * @brief Direction settings.
 */
enum {
	GPIOD_LINE_DIRECTION_AS_IS = 1,
	/**< Request the line(s), but don't change current direction. */
	GPIOD_LINE_DIRECTION_INPUT,
	/**< Direction is input - we're reading the state of a GPIO line. */
	GPIOD_LINE_DIRECTION_OUTPUT
	/**< Direction is output - we're driving the GPIO line. */
};

/**
 * @brief Internal bias settings.
 */
enum {
	GPIOD_LINE_BIAS_AS_IS = 1,
	/**< Don't change the bias setting when applying line config. */
	GPIOD_LINE_BIAS_UNKNOWN,
	/**< The internal bias state is unknown. */
	GPIOD_LINE_BIAS_DISABLED,
	/**< The internal bias is disabled. */
	GPIOD_LINE_BIAS_PULL_UP,
	/**< The internal pull-up bias is enabled. */
	GPIOD_LINE_BIAS_PULL_DOWN
	/**< The internal pull-down bias is enabled. */
};

/**
 * @brief Drive settings.
 */
enum {
	GPIOD_LINE_DRIVE_PUSH_PULL = 1,
	/**< Drive setting is push-pull. */
	GPIOD_LINE_DRIVE_OPEN_DRAIN,
	/**< Line output is open-drain. */
	GPIOD_LINE_DRIVE_OPEN_SOURCE
	/**< Line output is open-source. */
};

/**
 * @brief Edge detection settings.
 */
enum {
	GPIOD_LINE_EDGE_NONE = 1,
	/**< Line edge detection is disabled. */
	GPIOD_LINE_EDGE_RISING,
	/**< Line detects rising edge events. */
	GPIOD_LINE_EDGE_FALLING,
	/**< Line detect falling edge events. */
	GPIOD_LINE_EDGE_BOTH
	/**< Line detects both rising and falling edge events. */
};

/**
 * @brief Event clock settings.
 */
enum {
	GPIOD_LINE_EVENT_CLOCK_MONOTONIC = 1,
	/**< Line uses the monotonic clock for edge event timestamps. */
	GPIOD_LINE_EVENT_CLOCK_REALTIME,
	/**< Line uses the realtime clock for edge event timestamps. */
};

/**
 * @}
 *
 * @defgroup line_info Line info
 * @{
 *
 * Functions for retrieving kernel information about both requested and free
 * lines.
 *
 * Line info object contains an immutable snapshot of the line's state at the
 * time when it was created.
 */

/**
 * @brief Free a line info object and release all associated resources.
 * @param info GPIO line info object to free.
 */
void gpiod_line_info_free(struct gpiod_line_info *info);

/**
 * @brief Copy the line info object.
 * @param info Line info to copy.
 * @return Copy of the line info or NULL on error. The returned object must
 *         be freed by the caller using :gpiod_line_info_free.
 */
struct gpiod_line_info *gpiod_line_info_copy(struct gpiod_line_info *info);

/**
 * @brief Get the hardware offset of the line.
 * @param info GPIO line info object.
 * @return Offset of the line within the parent chip.
 */
unsigned int gpiod_line_info_get_offset(struct gpiod_line_info *info);

/**
 * @brief Read the GPIO line name.
 * @param info GPIO line info object.
 * @return Name of the GPIO line as it is represented in the kernel. This
 *         routine returns a pointer to a null-terminated string or NULL if
 *         the line is unnamed.
 */
const char *gpiod_line_info_get_name(struct gpiod_line_info *info);

/**
 * @brief Check if the line is currently in use.
 * @param info GPIO line object.
 * @return True if the line is in use, false otherwise.
 *
 * The user space can't know exactly why a line is busy. It may have been
 * requested by another process or hogged by the kernel. It only matters that
 * the line is used and we can't request it.
 */
bool gpiod_line_info_is_used(struct gpiod_line_info *info);

/**
 * @brief Read the GPIO line consumer name.
 * @param info GPIO line info object.
 * @return Name of the GPIO consumer name as it is represented in the
 *         kernel. This routine returns a pointer to a null-terminated string
 *         or NULL if the line is not used.
 */
const char *gpiod_line_info_get_consumer(struct gpiod_line_info *info);

/**
 * @brief Read the GPIO line direction setting.
 * @param info GPIO line info object.
 * @return Returns GPIOD_LINE_DIRECTION_INPUT or GPIOD_LINE_DIRECTION_OUTPUT.
 */
int gpiod_line_info_get_direction(struct gpiod_line_info *info);

/**
 * @brief Check if the signal of this line is inverted.
 * @param info GPIO line object.
 * @return True if this line is "active-low", false otherwise.
 */
bool gpiod_line_info_is_active_low(struct gpiod_line_info *info);

/**
 * @brief Read the GPIO line bias setting.
 * @param info GPIO line object.
 * @return Returns GPIOD_LINE_BIAS_PULL_UP, GPIOD_LINE_BIAS_PULL_DOWN,
 *         GPIOD_LINE_BIAS_DISABLE or GPIOD_LINE_BIAS_UNKNOWN.
 */
int gpiod_line_info_get_bias(struct gpiod_line_info *info);

/**
 * @brief Read the GPIO line drive setting.
 * @param info GPIO line info object.
 * @return Returns GPIOD_LINE_DRIVE_PUSH_PULL, GPIOD_LINE_DRIVE_OPEN_DRAIN or
 *         GPIOD_LINE_DRIVE_OPEN_SOURCE.
 */
int gpiod_line_info_get_drive(struct gpiod_line_info *info);

/**
 * @brief Read the current edge detection setting of this line.
 * @param info GPIO line info object.
 * @return Returns GPIOD_LINE_EDGE_NONE, GPIOD_LINE_EDGE_RISING,
 *         GPIOD_LINE_EDGE_FALLING or GPIOD_LINE_EDGE_BOTH.
 */
int gpiod_line_info_get_edge_detection(struct gpiod_line_info *info);

/**
 * @brief Read the current event clock setting used for edge event timestamps.
 * @param info GPIO line info object.
 * @return Returns GPIOD_LINE_EVENT_CLOCK_MONOTONIC or
 *         GPIOD_LINE_EVENT_CLOCK_REALTIME.
 */
int gpiod_line_info_get_event_clock(struct gpiod_line_info *info);

/**
 * @brief Check if this line is debounced (either by hardware or by the kernel
 *        software debouncer).
 * @param info GPIO line info object.
 * @return True if the line is debounced, false otherwise.
 */
bool gpiod_line_info_is_debounced(struct gpiod_line_info *info);

/**
 * @brief Read the current debounce period in microseconds.
 * @param info GPIO line info object.
 * @return Current debounce period in microseconds, 0 if the line is not
 *         debounced.
 */
unsigned long
gpiod_line_info_get_debounce_period_us(struct gpiod_line_info *info);

/**
 * @}
 *
 * @defgroup line_watch Line status watch events
 * @{
 *
 * Accessors for the info event objects allowing to monitor changes in GPIO
 * line state.
 *
 * Callers can be notified about changes in line's state using the interfaces
 * exposed by GPIO chips. Each info event contains information about the event
 * itself (timestamp, type) as well as a snapshot of line's state in the form
 * of a line-info object.
 */

/**
 * @brief Line status change event types.
 */
enum {
	GPIOD_INFO_EVENT_LINE_REQUESTED = 1,
	/**< Line has been requested. */
	GPIOD_INFO_EVENT_LINE_RELEASED,
	/**< Previously requested line has been released. */
	GPIOD_INFO_EVENT_LINE_CONFIG_CHANGED
	/**< Line configuration has changed. */
};

/**
 * @brief Free the info event object and release all associated resources.
 * @param event Info event to free.
 */
void gpiod_info_event_free(struct gpiod_info_event *event);

/**
 * @brief Get the event type of this status change event.
 * @param event Line status watch event.
 * @return One of ::GPIOD_INFO_EVENT_LINE_REQUESTED,
 *         ::GPIOD_INFO_EVENT_LINE_RELEASED or
 *         ::GPIOD_INFO_EVENT_LINE_CONFIG_CHANGED.
 */
int gpiod_info_event_get_event_type(struct gpiod_info_event *event);

/**
 * @brief Get the timestamp of the event.
 * @param event Line status watch event.
 * @return Timestamp in nanoseconds.
 */
uint64_t gpiod_info_event_get_timestamp(struct gpiod_info_event *event);

/**
 * @brief Get the pointer to the line-info object associated with this event.
 * @param event Line info event object.
 * @return Returns a pointer to the line-info object associated with this event
 *         whose lifetime is tied to the event object. It must not be freed by
 *         the caller.
 */
struct gpiod_line_info *
gpiod_info_event_get_line_info(struct gpiod_info_event *event);

/**
 * @}
 *
 * @defgroup line_config Line configuration objects
 * @{
 *
 * Functions for manipulating line configuration objects.
 *
 * The line-config object stores the configuration for lines that can be used
 * in two cases: when making a line request and when reconfiguring a set of
 * already requested lines. The mutators for the line request don't return
 * errors. If the set of options is too complex to be translated into kernel
 * uAPI structures - the error will be returned at the time of the request or
 * reconfiguration. If an invalid value was passed to any of the getters - the
 * default value will be silently used instead. Each option can be set
 * globally, for a single line offset or for multiple line offsets.
 */

/**
 * @brief Create a new line config object.
 * @return New line config object or NULL on error.
 */
struct gpiod_line_config *gpiod_line_config_new(void);

/**
 * @brief Free the line config object and release all associated resources.
 * @param config Line config object to free.
 */
void gpiod_line_config_free(struct gpiod_line_config *config);

/**
 * @brief Reset the line config object.
 * @param config Line config object to free.
 *
 * Resets the entire configuration stored in this object. This is useful if
 * the user wants to reuse the object without reallocating it.
 */
void gpiod_line_config_reset(struct gpiod_line_config *config);

/**
 * @brief Set the direction of all lines.
 * @param config Line config object.
 * @param direction New direction.
 */
void gpiod_line_config_set_direction(struct gpiod_line_config *config,
				     int direction);

/**
 * @brief Set the direction for a single line at given offset.
 * @param config Line config object.
 * @param direction New direction.
 * @param offset Offset of the line for which to set the direction.
 */
void gpiod_line_config_set_direction_offset(struct gpiod_line_config *config,
					    int direction, unsigned int offset);

/**
 * @brief Set the direction for a subset of lines.
 * @param config Line config object.
 * @param direction New direction.
 * @param num_offsets Number of offsets in the array.
 * @param offsets Array of line offsets for which to set the direction.
 */
void gpiod_line_config_set_direction_subset(struct gpiod_line_config *config,
					    int direction,
					    unsigned int num_offsets,
					    const unsigned int *offsets);

/**
 * @brief Get the direction setting for a given line.
 * @param config Line config object.
 * @param offset Line offset for which to read the direction setting.
 * @return Direction setting that would have been used for given offset if the
 *         config object was used in a request at the time of the call.
 * @note If an offset is used for which no config was provided, the function
 *       will return the global default value.
 */
int gpiod_line_config_get_direction(struct gpiod_line_config *config,
				    unsigned int offset);

/**
 * @brief Set the edge event detection for all lines.
 * @param config Line config object.
 * @param edge Type of edge events to detect.
 */
void gpiod_line_config_set_edge_detection(struct gpiod_line_config *config,
					  int edge);

/**
 * @brief Set the edge event detection for a single line at given offset.
 * @param config Line config object.
 * @param edge Type of edge events to detect.
 * @param offset Offset of the line for which to set the edge detection.
 */
void
gpiod_line_config_set_edge_detection_offset(struct gpiod_line_config *config,
					    int edge, unsigned int offset);

/**
 * @brief Set the edge event detection for a subset of lines.
 * @param config Line config object.
 * @param edge Type of edge events to detect.
 * @param num_offsets Number of offsets in the array.
 * @param offsets Array of line offsets for which to set the edge detection.
 */
void
gpiod_line_config_set_edge_detection_subset(struct gpiod_line_config *config,
					    int edge, unsigned int num_offsets,
					    const unsigned int *offsets);

/**
 * @brief Get the edge event detection setting for a given line.
 * @param config Line config object.
 * @param offset Line offset for which to read the edge event detection setting.
 * @return Edge event detection setting that would have been used for given
 *         offset if the config object was used in a request at the time of
 *         the call.
 * @note If an offset is used for which no config was provided, the function
 *       will return the global default value.
 */
int gpiod_line_config_get_edge_detection(struct gpiod_line_config *config,
					 unsigned int offset);

/**
 * @brief Set the bias of all lines.
 * @param config Line config object.
 * @param bias New bias.
 */
void gpiod_line_config_set_bias(struct gpiod_line_config *config, int bias);

/**
 * @brief Set the bias for a single line at given offset.
 * @param config Line config object.
 * @param bias New bias.
 * @param offset Offset of the line for which to set the bias.
 */
void gpiod_line_config_set_bias_offset(struct gpiod_line_config *config,
				       int bias, unsigned int offset);

/**
 * @brief Set the bias for a subset of lines.
 * @param config Line config object.
 * @param bias New bias.
 * @param num_offsets Number of offsets in the array.
 * @param offsets Array of line offsets for which to set the bias.
 */
void gpiod_line_config_set_bias_subset(struct gpiod_line_config *config,
				       int bias, unsigned int num_offsets,
				       const unsigned int *offsets);

/**
 * @brief Get the bias setting for a given line.
 * @param config Line config object.
 * @param offset Line offset for which to read the bias setting.
 * @return Bias setting that would have been used for given offset if the
 *         config object was used in a request at the time of the call.
 * @note If an offset is used for which no config was provided, the function
 *       will return the global default value.
 */
int gpiod_line_config_get_bias(struct gpiod_line_config *config,
			       unsigned int offset);

/**
 * @brief Set the drive of all lines.
 * @param config Line config object.
 * @param drive New drive.
 */
void gpiod_line_config_set_drive(struct gpiod_line_config *config, int drive);

/**
 * @brief Set the drive for a single line at given offset.
 * @param config Line config object.
 * @param drive New drive.
 * @param offset Offset of the line for which to set the drive.
 */
void gpiod_line_config_set_drive_offset(struct gpiod_line_config *config,
					int drive, unsigned int offset);

/**
 * @brief Set the drive for a subset of lines.
 * @param config Line config object.
 * @param drive New drive.
 * @param num_offsets Number of offsets in the array.
 * @param offsets Array of line offsets for which to set the drive.
 */
void gpiod_line_config_set_drive_subset(struct gpiod_line_config *config,
					int drive, unsigned int num_offsets,
					const unsigned int *offsets);

/**
 * @brief Get the drive setting for a given line.
 * @param config Line config object.
 * @param offset Line offset for which to read the drive setting.
 * @return Drive setting that would have been used for given offset if the
 *         config object was used in a request at the time of the call.
 * @note If an offset is used for which no config was provided, the function
 *       will return the global default value.
 */
int gpiod_line_config_get_drive(struct gpiod_line_config *config,
				unsigned int offset);

/**
 * @brief Set all lines as active-low.
 * @param config Line config object.
 */
void gpiod_line_config_set_active_low(struct gpiod_line_config *config);

/**
 * @brief Set a single line as active-low.
 * @param config Line config object.
 * @param offset Offset of the line for which to set the active setting.
 */
void gpiod_line_config_set_active_low_offset(struct gpiod_line_config *config,
					     unsigned int offset);

/**
 * @brief Set a subset of lines as active-low.
 * @param config Line config object.
 * @param num_offsets Number of offsets in the array.
 * @param offsets Array of line offsets for which to set the active setting.
 */
void gpiod_line_config_set_active_low_subset(struct gpiod_line_config *config,
					     unsigned int num_offsets,
					     const unsigned int *offsets);

/**
 * @brief Check if the line at given offset was configured as active-low.
 * @param config Line config object.
 * @param offset Line offset for which to read the active-low setting.
 * @return Active-low setting that would have been used for given offset if the
 *         config object was used in a request at the time of the call.
 * @note If an offset is used for which no config was provided, the function
 *       will return the global default value.
 */
bool gpiod_line_config_is_active_low(struct gpiod_line_config *config,
				     unsigned int offset);

/**
 * @brief Set all lines as active-high.
 * @param config Line config object.
 */
void gpiod_line_config_set_active_high(struct gpiod_line_config *config);

/**
 * @brief Set a single line as active-high.
 * @param config Line config object.
 * @param offset Offset of the line for which to set the active setting.
 */
void gpiod_line_config_set_active_high_offset(struct gpiod_line_config *config,
					      unsigned int offset);

/**
 * @brief Set a subset of lines as active-high.
 * @param config Line config object.
 * @param num_offsets Number of offsets in the array.
 * @param offsets Array of line offsets for which to set the active setting.
 */
void gpiod_line_config_set_active_high_subset(struct gpiod_line_config *config,
					      unsigned int num_offsets,
					      const unsigned int *offsets);

/**
 * @brief Set the debounce period for all lines.
 * @param config Line config object.
 * @param period New debounce period. Disables debouncing if 0.
 */
void gpiod_line_config_set_debounce_period_us(struct gpiod_line_config *config,
					      unsigned long period);

/**
 * @brief Set the debounce period for a single line at given offset.
 * @param config Line config object.
 * @param period New debounce period. Disables debouncing if 0.
 * @param offset Offset of the line for which to set the debounce period.
 */
void
gpiod_line_config_set_debounce_period_us_offset(
					struct gpiod_line_config *config,
					unsigned long period,
					unsigned int offset);

/**
 * @brief Set the debounce period for a subset of lines.
 * @param config Line config object.
 * @param period New debounce period. Disables debouncing if 0.
 * @param num_offsets Number of offsets in the array.
 * @param offsets Array of line offsets for which to set the debounce period.
 */
void
gpiod_line_config_set_debounce_period_us_subset(
					struct gpiod_line_config *config,
					unsigned long period,
					unsigned int num_offsets,
					const unsigned int *offsets);

/**
 * @brief Get the debounce period for a given line.
 * @param config Line config object.
 * @param offset Line offset for which to read the debounce period.
 * @return Debounce period that would have been used for given offset if the
 *         config object was used in a request at the time of the call.
 * @note If an offset is used for which no config was provided, the function
 *       will return the global default value.
 */
unsigned long
gpiod_line_config_get_debounce_us_period(struct gpiod_line_config *config,
					 unsigned int offset);

/**
 * @brief Set the event timestamp clock for all lines.
 * @param config Line config object.
 * @param clock New clock to use.
 */
void gpiod_line_config_set_event_clock(struct gpiod_line_config *config,
				       int clock);

/**
 * @brief Set the event clock for a single line at given offset.
 * @param config Line config object.
 * @param clock New event clock to use.
 * @param offset Offset of the line for which to set the event clock type.
 */
void gpiod_line_config_set_event_clock_offset(struct gpiod_line_config *config,
					      int clock, unsigned int offset);

/**
 * @brief Set the event clock for a subset of lines.
 * @param config Line config object.
 * @param clock New event clock to use.
 * @param num_offsets Number of offsets in the array.
 * @param offsets Array of line offsets for which to set the event clock type.
 */
void gpiod_line_config_set_event_clock_subset(struct gpiod_line_config *config,
					      int clock,
					      unsigned int num_offsets,
					      const unsigned int *offsets);

/**
 * @brief Get the event clock setting for a given line.
 * @param config Line config object.
 * @param offset Line offset for which to read the event clock setting.
 * @return Event clock setting that would have been used for given offset if
 *         the config object was used in a request at the time of the call.
 * @note If an offset is used for which no config was provided, the function
 *       will return the global default value.
 */
int gpiod_line_config_get_event_clock(struct gpiod_line_config *config,
				      unsigned int offset);

/**
 * @brief Set the output value for a single offset.
 * @param config Line config object.
 * @param offset Offset of the line.
 * @param value Output value to set.
 */
void gpiod_line_config_set_output_value(struct gpiod_line_config *config,
					unsigned int offset, int value);

/**
 * @brief Set the output values for a set of offsets.
 * @param config Line config object.
 * @param num_values Number of offsets for which to set values.
 * @param offsets Array of line offsets to set values for.
 * @param values Array of output values associated with the offsets passed in
 *               the previous argument.
 */
void gpiod_line_config_set_output_values(struct gpiod_line_config *config,
					 unsigned int num_values,
					 const unsigned int *offsets,
					 const int *values);

/**
 * @brief Get the number of line offsets for which this config object stores
 *        output values.
 * @param config Line config object.
 * @return Number of output values currently configured for this object.
 */
unsigned int
gpiod_line_config_num_output_values(struct gpiod_line_config *config);

/**
 * @brief Get the output value configured for a given line.
 * @param config Line config object.
 * @param offset Line offset for which to read the value.
 * @return 1 or 0 if the value was configured for this line, -1 otherwise.
 */
int gpiod_line_config_get_output_value(struct gpiod_line_config *config,
				       unsigned int offset);

/**
 * @brief Get the output value mapping (offset -> value) at given index.
 * @param config Line config object.
 * @param index Position of the mapping in the internal array.
 * @param offset Buffer for storing the offset of the line.
 * @param value Buffer for storing the value corresponding to the offset.
 * @return Returns 0 on success, -1 if the index is out of range.
 *
 * This function together with ::gpiod_line_config_num_output_values allows to
 * iterate over all output value mappings currently held by this object.
 */
int gpiod_line_config_get_output_value_index(struct gpiod_line_config *config,
					     unsigned int index,
					     unsigned int *offset, int *value);

/**
 * @brief Get all output value mappings stored in this config object.
 * @param config Line config object.
 * @param offsets Buffer in which offsets will be stored.
 * @param values Buffer in which values will be stored.
 * @note Both the offsets and values buffers must be able to hold at least the
 *       number of elements returned by ::gpiod_line_config_num_output_values.
 *
 * Each offset in the offsets array corresponds to the value in the values
 * array at the same index.
 */
void gpiod_line_config_get_output_values(struct gpiod_line_config *config,
					 unsigned int *offsets, int *values);

/**
 * @}
 *
 * @defgroup request_config Request configuration objects
 * @{
 *
 * Functions for manipulating request configuration objects.
 *
 * Request config object is used to pass a set of options to the kernel at the
 * time of the line request. Similarly to the line-config - the mutators don't
 * return error values. If the values are invalid, in general they are silently
 * adjusted to acceptable ranges.
 */

/**
 * @brief Create a new request config object.
 * @return New request config object or NULL on error.
 */
struct gpiod_request_config *gpiod_request_config_new(void);

/**
 * @brief Free the request config object and release all associated resources.
 * @param config Line config object.
 */
void gpiod_request_config_free(struct gpiod_request_config *config);

/**
 * @brief Set the consumer string.
 * @param config Request config object.
 * @param consumer Consumer name.
 * @note If the consumer string is too long, it will be truncated to the max
 *       accepted length.
 */
void gpiod_request_config_set_consumer(struct gpiod_request_config *config,
				       const char *consumer);

/**
 * @brief Get the consumer string.
 * @param config Request config object.
 * @return Current consumer string stored in this request config.
 */
const char *
gpiod_request_config_get_consumer(struct gpiod_request_config *config);

/**
 * @brief Set line offsets for this request.
 * @param config Request config object.
 * @param num_offsets Number of offsets.
 * @param offsets Array of line offsets.
 * @note If too many offsets were specified, the offsets above the limit
 *       accepted by the kernel (64 lines) are silently dropped.
 */
void gpiod_request_config_set_offsets(struct gpiod_request_config *config,
				      unsigned int num_offsets,
				      const unsigned int *offsets);

/**
 * @brief Get the number of offsets configured in this request config.
 * @param config Request config object.
 * @return Number of line offsets in this request config.
 */
unsigned int
gpiod_request_config_get_num_offsets(struct gpiod_request_config *config);

/**
 * @brief Get the hardware offsets of lines in this request config.
 * @param config Request config object.
 * @param offsets Array to store offsets. Must hold at least the number of
 *                lines returned by ::gpiod_request_config_get_num_offsets.
 */
void gpiod_request_config_get_offsets(struct gpiod_request_config *config,
				      unsigned int *offsets);

/**
 * @brief Set the size of the kernel event buffer.
 * @param config Request config object.
 * @param event_buffer_size New event buffer size.
 * @note The kernel may adjust the value if it's too high. If set to 0, the
 *       default value will be used.
 */
void
gpiod_request_config_set_event_buffer_size(struct gpiod_request_config *config,
					   unsigned int event_buffer_size);

/**
 * @brief Get the edge event buffer size from this request config.
 * @param config Request config object.
 * @return Current edge event buffer size setting.
 */
unsigned int
gpiod_request_config_get_event_buffer_size(struct gpiod_request_config *config);

/**
 * @}
 *
 * @defgroup request_request Line request operations
 * @{
 *
 * Functions allowing interaction with a set of requested lines.
 */

/**
 * @brief Release the requested lines and free all associated resources.
 * @param request Line request object to release.
 */
void gpiod_line_request_release(struct gpiod_line_request *request);

/**
 * @brief Get the number of lines in this request.
 * @param request Line request object.
 * @return Number of requested lines.
 */
unsigned int gpiod_line_request_get_num_lines(struct gpiod_line_request *request);

/**
 * @brief Get the hardware offsets of lines in this request.
 * @param request Line request object.
 * @param offsets Array to store offsets. Must hold at least the number of
 *                lines returned by ::gpiod_line_request_get_num_lines.
 */
void gpiod_line_request_get_offsets(struct gpiod_line_request *request,
				    unsigned int *offsets);

/**
 * @brief Read the value of a single line associated with this request.
 * @param request Line request object.
 * @param offset Offset of the line of which the value should be read.
 * @return Returns 1 or 0 on success and -1 on error.
 */
int gpiod_line_request_get_value(struct gpiod_line_request *request,
				 unsigned int offset);

/**
 * @brief Read values of a subset of lines associated with this request.
 * @param request GPIO line request.
 * @param num_lines Number of lines for which to read values.
 * @param offsets Array of offsets corresponding with the lines associated with
 *                this request for which to read values.
 * @param values Array in which the values will be stored.
 * @return 0 on success, -1 on failure.
 */
int gpiod_line_request_get_values_subset(struct gpiod_line_request *request,
					 unsigned num_lines,
					 const unsigned int *offsets,
					 int *values);

/**
 * @brief Read values of all lines associated with this request.
 * @param request GPIO line request.
 * @param values Array in which the values will be stored. Must hold at least
 *               the number of lines returned by
 *               ::gpiod_line_request_get_num_lines. The index of each value
 *               will be associated with the offset at the same index in the
 *               offset array returned by ::gpiod_line_request_get_offsets.
 * @return 0 on success, -1 on failure.
 */
int gpiod_line_request_get_values(struct gpiod_line_request *request,
				  int *values);

/**
 * @brief Set the value of a single line associated with this request.
 * @param request Line request object.
 * @param offset Offset of the line of which the value should be set.
 * @param value Value to set.
 */
int gpiod_line_request_set_value(struct gpiod_line_request *request,
				 unsigned int offset, int value);

/**
 * @brief Set values of a subset of lines associated with this line request.
 * @param request GPIO line request.
 * @param num_lines Number of lines for which to set values.
 * @param offsets Array of offsets corresponding with the lines associated with
 *                this request for which to set values.
 * @param values Array of values to set. The members of this array must
 *               correspond with the offsets in the previous argument.
 * @return 0 on success, -1 on failure.
 */
int gpiod_line_request_set_values_subset(struct gpiod_line_request *request,
					 unsigned int num_lines,
					 const unsigned int *offsets,
					 const int *values);

/**
 * @brief Set values of all lines associated with this request.
 * @param request GPIO line request.
 * @param values Array containing the values to set. Must hold at least the
 *               number of lines returned by ::gpiod_line_request_get_num_lines.
 *               The index of each value be associated with the offset at the
 *               same index in the offset array returned by
 *               ::gpiod_line_request_get_offsets.
 */
int gpiod_line_request_set_values(struct gpiod_line_request *request,
				  const int *values);

/**
 * @brief Update the configuration of lines associated with this line request.
 * @param request GPIO line request.
 * @param config New line config to apply.
 * @return 0 on success, -1 on failure.
 */
int gpiod_line_request_reconfigure_lines(struct gpiod_line_request *request,
					 struct gpiod_line_config *config);

/**
 * @brief Get the file descriptor associated with this line request.
 * @param request GPIO line request.
 * @return Number of the file descriptor associated with this request. This
 *         function never fails.
 */
int gpiod_line_request_get_fd(struct gpiod_line_request *request);

/**
 * @brief Wait for edge events on any of the lines associated with this request.
 * @param request GPIO line request.
 * @param timeout Wait time limit in nanoseconds.
 * @return 0 if wait timed out, -1 if an error occurred, 1 if an event is
 *         pending.
 */
int gpiod_line_request_edge_event_wait(struct gpiod_line_request *request,
				       uint64_t timeout);

/**
 * @brief Read a number of edge events from a line request.
 * @param request GPIO line request.
 * @param buffer Edge event buffer.
 * @param max_events Maximum number of events to read.
 * @return On success returns the number of events read from the file
 *         descriptor, on failure return -1.
 * @note This function will block if no event was queued for this line.
 */
int gpiod_line_request_edge_event_read(struct gpiod_line_request *request,
				       struct gpiod_edge_event_buffer *buffer,
				       unsigned int max_events);

/**
 * @}
 *
 * @defgroup edge_event Line edge events handling
 * @{
 *
 * Functions and data types for handling edge events.
 *
 * An edge event object contains information about a single line event. It
 * contains the event type, timestamp and the offset of the line on which the
 * event occurred as well as two seqential numbers (global for all lines
 * associated with the parent chip and local for this line only).
 *
 * For performance and to limit the number of memory allocations when a lot of
 * events are being read, edge events are stored in an edge-event buffer object.
 */

/**
 * @brief Event types.
 */
enum {
	GPIOD_EDGE_EVENT_RISING_EDGE = 1,
	/**< Rising edge event. */
	GPIOD_EDGE_EVENT_FALLING_EDGE
	/**< Falling edge event. */
};

/**
 * @brief Free the edge event object.
 * @param event Edge event object to free.
 */
void gpiod_edge_event_free(struct gpiod_edge_event *event);

/**
 * @brief Copy the edge event object.
 * @param event Edge event to copy.
 * @return Copy of the edge event or NULL on error. The returned object must
 *         be freed by the caller using :gpiod_edge_event_free.
 */
struct gpiod_edge_event *gpiod_edge_event_copy(struct gpiod_edge_event *event);

/**
 * @brief Get the event type.
 * @param event GPIO edge event.
 * @return The event type (::GPIOD_EDGE_EVENT_RISING_EDGE or
 *         ::GPIOD_EDGE_EVENT_FALLING_EDGE).
 */
int gpiod_edge_event_get_event_type(struct gpiod_edge_event *event);

/**
 * @brief Get the timestamp of the event.
 * @param event GPIO edge event.
 * @return Timestamp in nanoseconds.
 */
uint64_t gpiod_edge_event_get_timestamp(struct gpiod_edge_event *event);

/**
 * @brief Get the hardware offset of the line on which the event was triggered.
 * @param event GPIO edge event.
 * @return Line offset.
 */
unsigned int gpiod_edge_event_get_line_offset(struct gpiod_edge_event *event);

/**
 * @brief Get the global sequence number of this event.
 * @param event GPIO edge event.
 * @return Sequence number of the event relative to all lines in the associated
 *         line request.
 */
unsigned long gpiod_edge_event_get_global_seqno(struct gpiod_edge_event *event);

/**
 * @brief Get the event sequence number specific to concerned line.
 * @param event GPIO edge event.
 * @return Sequence number of the event relative to this line within the
 *         lifetime of the associated line request.
 */
unsigned long gpiod_edge_event_get_line_seqno(struct gpiod_edge_event *event);

/**
 * @brief Create a new edge event buffer.
 * @param capacity Number of events this buffer can store (min = 1, max = 1024).
 * @return New edge event buffer or NULL on error.
 * @note If capacity equals 0, it will be set to a default value of 64. If
 *       capacity is larger than 1024, it will be limited to 1024.
 */
struct gpiod_edge_event_buffer *
gpiod_edge_event_buffer_new(unsigned int capacity);

/**
 * @brief Get the capacity (the max number of events that can be stored) of
 *        the event buffer.
 * @param buffer Edge event buffer.
 * @return The capacity of the buffer.
 */
unsigned int
gpiod_edge_event_buffer_get_capacity(struct gpiod_edge_event_buffer *buffer);

/**
 * @brief Free the edge event buffer and release all associated resources.
 * @param buffer Edge event buffer to free.
 */
void gpiod_edge_event_buffer_free(struct gpiod_edge_event_buffer *buffer);

/**
 * @brief Get a pointer to an event stored in the buffer.
 * @param buffer Edge event buffer.
 * @param index Index of the event in the buffer.
 * @return Pointer to an event stored in the buffer. The lifetime of this
 *         event is tied to the buffer object. Users must not free the event
 *         returned by this function.
 */
struct gpiod_edge_event *
gpiod_edge_event_buffer_get_event(struct gpiod_edge_event_buffer *buffer,
				  unsigned long index);

/**
 * @brief Get the number of events this buffers stores.
 * @param buffer Edge event buffer.
 * @return Number of events stored in this buffer.
 */
unsigned int
gpiod_edge_event_buffer_num_events(struct gpiod_edge_event_buffer *buffer);

/**
 * @}
 *
 * @defgroup misc Stuff that didn't fit anywhere else
 * @{
 *
 * Various libgpiod-related functions.
 */

/**
 * @brief Check if the file pointed to by path is a GPIO chip character device.
 * @param path Path to check.
 * @return True if the file exists and is a GPIO chip character device or a
 *         symbolic link to it.
 */
bool gpiod_is_gpiochip_device(const char *path);

/**
 * @brief Get the API version of the library as a human-readable string.
 * @return Human-readable string containing the library version.
 */
const char *gpiod_version_string(void);

/**
 * @}
 */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __LIBGPIOD_GPIOD_H__ */
