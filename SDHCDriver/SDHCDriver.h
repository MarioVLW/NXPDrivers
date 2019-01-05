/*
 * SDHCDriver.h
 *
 *  Created on: Dec 23, 2018
 *      Author: MarioVL
 */

#ifndef SDHCDRIVER_H_
#define SDHCDRIVER_H_

#define SDHC_NVIC_SHIFT 17
#define SDHC_NVICIPR_SHIFT 12

#define HIGH_PRIORITY_IRQ_MASK 7u

#define PORTE_NVIC_SHIFT 31
#define PORTE_NVICIPR_SHIFT 28
#define PORTE_PCR_IQR_CONF_SHIFT 16

#define PORTE_PCR_IQR_EDGE_MODE 11u
#define PORTE_PCR_IQR_STATUS 0x1000000u

#define PERIOD_PDB 0xFFFF

#define RSPTYP_NO_RESP 0u
#define RSPTYP_136 1u
#define RSPTYP_48 2u
#define RSPTYP_48_BUSY_CHECK 3u

#define CMDTYP_NORMAL 0u
#define CMDTYP_SUSPEND 1u
#define CMDTYP_RESUME 2u
#define CMDTYP_ABORT 3u

#define STEP_ZERO 0u
#define STEP_ONE 1u
#define STEP_TWO 2u
#define STEP_THREE 3u
#define STEP_FOUR 4u
#define FINAL_STEP 5u

#define GO_IDLE_STATE (uint32_t)0u
#define CMD0_INDEX 0u
#define CMD0_RESP 0u
#define CMD0_RESP_MASK 0xFFFFFFFFu

#define SEND_IF_COND (uint32_t)0x1AAu
#define CMD8_INDEX 8u
#define CMD8_RESP 0x1AAu
#define CMD8_RESP_MASK 0xFFFFFFFFu

#define APP_CMD (uint32_t)0x0u
#define CMD55_INDEX 55u
#define CMD55_RESP 0x120u
#define CMD55_RESP_MASK 0xFFFFFFFFu

#define SD_SEND_OP_COND (uint32_t)0x51100000u
#define ACMD41_INDEX 41u
#define ACMD41_RESP 0xC1000000u
#define ACMD41_RESP_MASK 0xC1000000u

#define SEND_STATUS (uint32_t)0x0u
#define CMD13_INDEX 13u
#define CMD13_RESP 0x1u

#define IS_CMD_SHIFT 22
#define CMD_INDEX_SHIFT 16
#define POLY_MULT_SHIFT 7

#define GENERATOR_POLY 137u
#define CRC_WIDTH 7u

#define PORTE_PDDR6 0x40
#define PORTE_PDDR4 0x10

typedef struct {
	uint8_t CINS : 1;
	uint8_t CRM  : 1;
}SDHC_flags;

typedef struct {
	uint8_t init_index;
	uint32_t expect_resp;
	uint32_t expect_resp_mask;
}SDHC_t;

void SDHC_init();
void SDHC_on();
void SDHC_stop();
void SDHC_clock_enable();
void SDHC_NVIC_enable();
void SDHC_NVIC_disable();
void SDHC_sys_init();
void send_cmd(uint8_t cmd_index, uint8_t cmd_type, uint8_t resp_type, uint32_t arg);
void SDHC_detection_enable();
void SDHC_detection_disable();
uint8_t checksum(uint16_t arg_HB, uint16_t arg_LB, uint8_t cmd_index, uint8_t is_cmd);
void divide_poly(uint32_t *cmd);
uint8_t is_SDclock_working();
uint8_t is_SDHC_inserted();
uint8_t is_SDclock_stable();

SDHC_flags SDHC_status;
SDHC_t SDHC_data;

#endif /* SDHCDRIVER_H_ */
