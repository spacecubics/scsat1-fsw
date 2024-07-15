/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define CSP_ID_MAIN (1U)
#define CSP_ID_ADCS (2U)
#define CSP_ID_EPS  (4U)
#define CSP_ID_SRS3 (5U)
#define CSP_ID_GND  (10U)
#define CSP_ID_ZERO (12U)
#define CSP_ID_PICO (14U)

#define CSP_PORT_MAIN_SHELL_CMD (10U)

int csp_enable(void);
void csp_disable(void);
