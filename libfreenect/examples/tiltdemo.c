/*
 * This file is based on tiltdemo.c:
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * Andrew Miller <amiller@dappervision.com>
 * Eli Cohen <cohene@eou.edu>
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include "libfreenect.h"
#include "libfreenect_sync.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void no_kinect_quit(void)
{
	printf("Error: Kinect not connected?\n");
	exit(1);
}

int
set_led(freenect_led_options led)
{
	// Set the LEDs to one of the possible states
	if (freenect_sync_set_led(led, 0))
		return -1;

	return 0;
}

int
set_tilt(int tilt)
{
	if (tilt <= -8)
	       tilt = -8;

	if (tilt >= 8)
		tilt = 8;

	// Set the tilt angle (in degrees)
	if (freenect_sync_set_tilt_degs(tilt, 0))
		return -1;

	return 0;
}

int
get_accel(double *dx, double *dy, double *dz)
{
	freenect_raw_tilt_state *state;

	// Get the raw accelerometer values and tilt data
	if (freenect_sync_get_tilt_state(&state, 0))
	       return -1;

	// Get the processed accelerometer values (calibrated to gravity)
	freenect_get_mks_accel(state, dx, dy, dz);

	return 0;
}

int
main(int argc, char *argv[])
{
	freenect_led_options led;
	double tilt = -12;
	double dx, dy, dz;

	if (set_tilt (tilt))
		return -1;

	while (1)
	{
		if (get_accel (&dx, &dy, &dz))
			return -1;

		printf ("%lf %lf %lf\n", dx, dy, dz);

		if (dy < 9.8) {
			tilt -= dz;
			set_tilt ((int)tilt);
		}
		usleep (100000);
	}
}
