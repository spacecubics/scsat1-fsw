/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define CSP_ID_GND       (1U)
#define CSP_ID_SRS3      (2U)
#define CSP_ID_EPS       (4U)
#define CSP_ID_MAIN_CAN1 (8U)
#define CSP_ID_MAIN_CAN2 (9U)
#define CSP_ID_ADCS      (16U)
#define CSP_ID_ZERO      (24U)
#define CSP_ID_PICO      (26U)

#define CSP_TLM_ID_SYSHK (0U)
#define CSP_PORT_TLM     (10U)

#define CSP_CAN_BITRATE (1000000U)

#define CSP_TELEMETRY_ID_OFFSET (0U)
#define CSP_COMMAND_ID_OFFSET   (0U)
#define CSP_ERROR_CODE_OFFSET   (1U)

#define CSP_UNKNOWN_CMD_CODE (255U)
