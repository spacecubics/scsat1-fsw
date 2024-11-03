/*
 * Copyright (c) 2022 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SCOBCA1_FPGA_TEST_QSPI_COMMON_H_
#define SCOBCA1_FPGA_TEST_QSPI_COMMON_H_

#include <zephyr/kernel.h>

/* Base address */
#define SCOBCA1_FPGA_CFG_BASE_ADDR  (0x40000000)
#define SCOBCA1_FPGA_DATA_BASE_ADDR (0x40100000)
#define SCOBCA1_FPGA_FRAM_BASE_ADDR (0x40200000)

/* Offset */
#define QSPI_ACR_OFFSET	   (0x0000) /* QSPI Access Control Register */
#define QSPI_TDR_OFFSET	   (0x0004) /* QSPI TX Data Register */
#define QSPI_RDR_OFFSET	   (0x0008) /* QSPI RX Data Register */
#define QSPI_ASR_OFFSET	   (0x000C) /* QSPI Access Status Register */
#define QSPI_FIFOSR_OFFSET (0x0010) /* QSPI FIFO Status Register */
#define QSPI_FIFORR_OFFSET (0x0014) /* QSPI FIFO Reset Register */
#define QSPI_ISR_OFFSET	   (0x0020) /* QSPI Interrupt Status Register */
#define QSPI_IER_OFFSET	   (0x0024) /* QSPI Interrupt Enable Register */
#define QSPI_CCR_OFFSET	   (0x0030) /* QSPI Clock Control Register */
#define QSPI_DCMSR_OFFSET  (0x0034) /* QSPI Data Capture Mode Setting Register */
#define QSPI_FTLSR_OFFSET  (0x0038) /* QSPI FIFO Threshold Level Setting Register */
#define QSPI_VER_OFFSET	   (0xF000) /* QSPI Controller IP Version Register */

/* QSPI Control Register for Data/ Configutation Memory */
#define SCOBCA1_FPGA_NORFLASH_QSPI_ACR(base)	(base + QSPI_ACR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base)	(base + QSPI_TDR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_RDR(base)	(base + QSPI_RDR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_ASR(base)	(base + QSPI_ASR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FIFOSR(base) (base + QSPI_FIFOSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FIFORR(base) (base + QSPI_FIFORR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base)	(base + QSPI_ISR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_IER(base)	(base + QSPI_IER_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_CCR(base)	(base + QSPI_CCR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_DCMSR(base)	(base + QSPI_DCMSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FTLSR(base)	(base + QSPI_FTLSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_VER(base)	(base + QSPI_VER_OFFSET)

/* QSPI Control Register for FeRAM */
#define SCOBCA1_FPGA_FRAM_QSPI_ACR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_ACR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_TDR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_TDR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_RDR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_RDR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_ASR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_ASR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_FIFOSR (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_FIFOSR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_FIFORR (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_FIFORR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_ISR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_ISR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_IER    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_IER_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_CCR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_CCR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_DCMSR  (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_DCMSR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_FTLSR  (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_FTLSR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_VER    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_VER_OFFSET)

#define QSPI_DATA_MEM0		   (0u)
#define QSPI_DATA_MEM1		   (1u)
#define QSPI_CFG_MEM0		   (0u)
#define QSPI_CFG_MEM1		   (1u)
#define QSPI_NOR_FLASH_SECTOR_BYTE (4 * 1024)
#define QSPI_NOR_FLASH_BLOCK_BYTE  (64 * 1024)

enum QspiEraseType {
	QSPI_ERASE_SECTOR,     /* 4KB */
	QSPI_ERASE_HALF_BLOCK, /* 32KB */
	QSPI_ERASE_BLOCK,      /* 64KB */
};

uint32_t qspi_init(uint32_t test_no);
uint32_t qspi_create_fifo_data(uint8_t start_val, uint32_t *data, size_t size, bool fill);

#endif /* SCOBCA1_FPGA_TEST_QSPI_COMMON_H_ */