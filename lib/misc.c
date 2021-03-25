// SPDX-License-Identifier: LGPL-2.1-or-later
// SPDX-FileCopyrightText: 2017-2021 Bartosz Golaszewski <bartekgola@gmail.com>

/* Misc code that didn't fit anywhere else. */

#include <errno.h>
#include <gpiod.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/types.h>
#include <unistd.h>

#include "internal.h"

GPIOD_API bool gpiod_is_gpiochip_device(const char *path)
{
	char *realname, *sysfsp, devpath[64];
	struct stat statbuf;
	bool ret = false;
	int rv;

	rv = lstat(path, &statbuf);
	if (rv)
		goto out;

	/*
	 * Is it a symbolic link? We have to resolve it before checking
	 * the rest.
	 */
	realname = S_ISLNK(statbuf.st_mode) ? realpath(path, NULL)
					    : strdup(path);
	if (realname == NULL)
		goto out;

	rv = stat(realname, &statbuf);
	if (rv)
		goto out_free_realname;

	/* Is it a character device? */
	if (!S_ISCHR(statbuf.st_mode)) {
		errno = ENOTTY;
		goto out_free_realname;
	}

	/* Is the device associated with the GPIO subsystem? */
	snprintf(devpath, sizeof(devpath), "/sys/dev/char/%u:%u/subsystem",
		 major(statbuf.st_rdev), minor(statbuf.st_rdev));

	sysfsp = realpath(devpath, NULL);
	if (!sysfsp)
		goto out_free_realname;

	if (strcmp(sysfsp, "/sys/bus/gpio") != 0) {
		/* This is a character device but not the one we're after. */
		errno = ENODEV;
		goto out_free_sysfsp;
	}

	ret = true;

out_free_sysfsp:
	free(sysfsp);
out_free_realname:
	free(realname);
out:
	return ret;
}

GPIOD_API const char *gpiod_version_string(void)
{
	return GPIOD_VERSION_STR;
}
