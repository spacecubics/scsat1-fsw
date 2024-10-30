/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum mtq_axes {
	MTQ_AXES_MAIN_X = 0,
	MTQ_AXES_MAIN_Y,
	MTQ_AXES_MAIN_Z,
	MTQ_AXES_BKUP_X,
	MTQ_AXES_BKUP_Y,
	MTQ_AXES_BKUP_Z,
};

enum mtq_polarity {
	MTQ_POL_PLUS,
	MTQ_POL_MINUS,
	MTQ_POL_NON,
};

static const char mtq_axes_name[][15] = {
	"MTQ X (Main)",   "MTQ Y (Main)",   "MTQ Z (Main)",
	"MTQ X (Backup)", "MTQ Y (Backup)", "MTQ Z (Backup)",
};

static const char mtq_pol_name[][6] = {
	"Plus",
	"Minus",
	"Non",
};

int mtq_start(enum mtq_axes axes, enum mtq_polarity pol, float duty);
int mtq_stop(enum mtq_axes axes);
