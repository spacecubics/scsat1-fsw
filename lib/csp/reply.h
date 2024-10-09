/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <csp/csp.h>

void csp_send_std_reply(csp_packet_t *packet, uint8_t command_id, int err_code);
