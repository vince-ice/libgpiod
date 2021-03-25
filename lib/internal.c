// SPDX-License-Identifier: LGPL-2.1-or-later
// SPDX-FileCopyrightText: 2021 Bartosz Golaszewski <brgl@bgdev.pl>

#include <errno.h>
#include <poll.h>
#include <string.h>

#include "internal.h"

int gpiod_poll_fd(int fd, uint64_t timeout)
{
	struct timespec ts;
	struct pollfd pfd;
	int ret;

	memset(&pfd, 0, sizeof(pfd));
	pfd.fd = fd;
	pfd.events = POLLIN | POLLPRI;

	ts.tv_sec = timeout / 1000000000ULL;
	ts.tv_nsec = timeout % 1000000000ULL;

	ret = ppoll(&pfd, 1, &ts, NULL);
	if (ret < 0)
		return -1;
	else if (ret == 0)
		return 0;

	return 1;
}

void gpiod_line_mask_zero(uint64_t *mask)
{
	*mask = 0ULL;
}

void gpiod_line_mask_fill(uint64_t *mask)
{
	*mask = UINT64_MAX;
}

bool gpiod_line_mask_test_bit(const uint64_t *mask, int nr)
{
	return *mask & (1ULL << nr);
}

void gpiod_line_mask_set_bit(uint64_t *mask, unsigned int nr)
{
	*mask |= (1ULL << nr);
}

void gpiod_line_mask_assign_bit(uint64_t *mask, unsigned int nr, bool value)
{
	if (value)
		gpiod_line_mask_set_bit(mask, nr);
	else
		*mask &= ~(1ULL << nr);
}
