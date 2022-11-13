/*
 * Copyright (c) 2022 Space Cubics, LLC.
 *
 */

#pragma once

/* IS_ENABLED(FOO) expands to 1 if defined to 1, or 0 otherwise even
 * if not defined.  This allows us to use defines in C functions
 * rather than #ifdef, which is much cleaner and compiler can verify
 * the code even if an option is not defined. This macro is originated
 * in Linux kernel and used in many open source projects. */
#ifndef IS_ENABLED
#define IS_ENABLED(config) TRCH_IS_ENABLED1(config)
#endif

/*
 * defined:   "__TRCH_CONFIG_PREFIX_1" -> '0,'
 * undefined: "__TRCH_CONFIG_PREFIX_XXXX"
 */
#define __TRCH_CONFIG_PREFIX_1 0,
#define TRCH_IS_ENABLED1(config) TRCH_IS_ENABLED2(__TRCH_CONFIG_PREFIX_##config)

/*
 * defined:   "0, 1, 0"
 * undefined: "__TRCH_CONFIG_PREFIX_XXXX 1, 0"
 */
#define TRCH_IS_ENABLED2(config) TRCH_IS_ENABLED3(config 1, 0)

/* Take the second arg; that is 1 if defined or 0 otherwise */
#define TRCH_IS_ENABLED3(__ignore, val, ...) val
