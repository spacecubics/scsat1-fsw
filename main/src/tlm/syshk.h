/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

void send_syshk_to_ground(void);
void send_syshk_history_to_ground(uint32_t start_seq_num, uint32_t end_seq_num, uint16_t skip_count,
				  uint16_t send_intvl_ms);
int check_syshk_history_req_param(uint32_t start_seq_num, uint32_t end_seq_num,
				  uint16_t send_intvl_ms);
void cancel_syshk_history(void);
void start_send_syshk(void);
