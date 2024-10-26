/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <csp/csp.h>

int csp_flash_handler_init(void);
int csp_flash_handler(csp_packet_t *packet);
