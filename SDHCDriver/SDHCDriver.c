/*
 * SDHCDriver.c
 *
 *  Created on: Dec 23, 2018
 *      Author: MarioVL
 */

#include "derivative.h"
#include "SDHCDriver.h"

void PORTE_IRQHandler()
{
	if(PORTE_PCR_IQR_STATUS == (PORTE_PCR6&PORTE_PCR_IQR_STATUS))
	{
		PORTE_PCR6 |= PORTE_PCR_IQR_STATUS;
		if(is_SDHC_inserted() && 0 == SDHC_status.CINS)
		{
			SDHC_status.CINS = 1u;
			SDHC_NVIC_enable();
			SDHC_init();
		} 
		else if(1u == SDHC_status.CINS && 0 == is_SDHC_inserted())
		{
			SDHC_status.CINS = 0;
			SDHC_NVIC_disable();
			SDHC_stop();
		}
	}
}

void SDHC_IRQHandler()
{
	
	if(SDHC_IRQSTAT_CC_MASK == (SDHC_IRQSTAT&SDHC_IRQSTAT_CC_MASK))
	{
		SDHC_IRQSTAT |= SDHC_IRQSTAT_CC_MASK;
		
		if(0 != SDHC_data.init_index)
		{
			if(SDHC_data.expect_resp == (SDHC_CMDRSP0&SDHC_data.expect_resp_mask))
			{
				SDHC_data.init_index++;
			} else {
				SDHC_data.init_index--;
			}
		} else {
			SDHC_data.init_index++;
		}
		
		switch(SDHC_data.init_index)
		{
			case STEP_ZERO:
				while(0 == is_SDclock_stable()){asm("nop");} //Wait until the SDclock is stable
					
				if(0 == (SDHC_PRSSTAT&SDHC_PRSSTAT_CIHB_MASK) && 0 == (SDHC_PRSSTAT&SDHC_PRSSTAT_CDIHB_MASK))
				{
					SDHC_SYSCTL |= SDHC_SYSCTL_INITA_MASK; //Send 80 SD-clocks and waits until it finishes
				}
				
				while(is_SDclock_working()){asm("nop");}
				SDHC_data.expect_resp = CMD0_RESP;
				SDHC_data.expect_resp_mask = CMD0_RESP_MASK;
				send_cmd(CMD0_INDEX,CMDTYP_NORMAL,RSPTYP_NO_RESP,GO_IDLE_STATE); //Send the CMD0
				break;
			case STEP_ONE:
				SDHC_data.expect_resp = CMD8_RESP;
				SDHC_data.expect_resp_mask = CMD8_RESP_MASK;
				send_cmd(CMD8_INDEX,CMDTYP_NORMAL,RSPTYP_48,SEND_IF_COND); //Send to check if the card supports voltage
				break;
			case STEP_TWO:
				SDHC_data.expect_resp = CMD55_RESP;
				SDHC_data.expect_resp_mask = CMD55_RESP_MASK;
				send_cmd(CMD55_INDEX,CMDTYP_NORMAL,RSPTYP_48,APP_CMD); //Send to indicate the next cmd as an app cmd
				break;
			case STEP_THREE:
				SDHC_data.expect_resp = ACMD41_RESP;
				SDHC_data.expect_resp_mask = ACMD41_RESP_MASK;
				send_cmd(ACMD41_INDEX,CMDTYP_NORMAL,RSPTYP_48_BUSY_CHECK,SD_SEND_OP_COND); //Sends host capacity support info
				break;
			default:
				break;
		}
	}
	
	if(SDHC_IRQSTAT_CCE_MASK == (SDHC_IRQSTAT&SDHC_IRQSTAT_CCE_MASK))
	{
		SDHC_IRQSTAT |= SDHC_IRQSTAT_CCE_MASK;
	}
	
	if(SDHC_IRQSTAT_CIE_MASK == (SDHC_IRQSTAT&SDHC_IRQSTAT_CIE_MASK))
	{
		SDHC_IRQSTAT |= SDHC_IRQSTAT_CIE_MASK;
	}
}

void SDHC_on()
{	
	SDHC_clock_enable();
	
	SDHC_SYSCTL &= ~SDHC_SYSCTL_SDCLKFS_MASK; //Cleans the SDCLKFS in order to assign the new value
	SDHC_SYSCTL &= ~SDHC_SYSCTL_DVS_MASK; //Cleans the DVS in order to assign the new value
	SDHC_SYSCTL |= (0x04 << SDHC_SYSCTL_SDCLKFS_SHIFT); //Assign the divider to 16
	SDHC_SYSCTL |= (0x6 << SDHC_SYSCTL_DVS_SHIFT); //Assign the divisor to 3
	
	SDHC_status.CINS = 0;
	SDHC_status.CRM = 0;
	SDHC_data.init_index = 0;
	
	PORTE_PCR6 |= PORT_PCR_MUX(1); //PTE6 mux
	PORTE_PCR5 |= PORT_PCR_MUX(4); //SDHC0_D2 mux
	PORTE_PCR4 |= PORT_PCR_MUX(4); //SDHC0_D3 mux
	PORTE_PCR3 |= PORT_PCR_MUX(4); //SDHC0_CMD mux
	PORTE_PCR2 |= PORT_PCR_MUX(4); //SDHC0_DCLK mux
	PORTE_PCR1 |= PORT_PCR_MUX(4); //SDHC0_D0 mux
	PORTE_PCR0 |= PORT_PCR_MUX(4); //SDHC0_D1 mux	
	
	GPIOE_PDDR &= ~PORTE_PDDR6; //Set PORTE6 as input
	PORTE_PCR6 |= PORT_PCR_PE_MASK; //Enable internal pull resistor
	PORTE_PCR5 |= PORT_PCR_PE_MASK; //Enable internal pull resistor
	PORTE_PCR4 |= PORT_PCR_PE_MASK; //Enable internal pull resistor
	PORTE_PCR3 |= PORT_PCR_PE_MASK; //Enable internal pull resistor
	PORTE_PCR1 |= PORT_PCR_PE_MASK; //Enable internal pull resistor
	PORTE_PCR0 |= PORT_PCR_PE_MASK; //Enable internal pull resistor
	
	PORTE_PCR6 &= ~PORT_PCR_PS_MASK; //Enable pull-down resistor
	PORTE_PCR5 |= PORT_PCR_PS_MASK; //Enable pull-up resistor
	PORTE_PCR4 |= PORT_PCR_PS_MASK; //Enable pull-up resistor
	PORTE_PCR3 |= PORT_PCR_PS_MASK; //Enable pull-up resistor
	PORTE_PCR1 |= PORT_PCR_PS_MASK; //Enable pull-up resistor
	PORTE_PCR0 |= PORT_PCR_PS_MASK; //Enable pull-up resistor
}

void SDHC_detection_enable()
{
	PORTE_PCR6 |= (PORTE_PCR_IQR_EDGE_MODE << PORTE_PCR_IQR_CONF_SHIFT); //Set interrupt on rising and falling edge
	NVICIP15 &= ~(HIGH_PRIORITY_IRQ_MASK << PORTE_NVICIPR_SHIFT); //Set higher priority
	NVICISER1 |= (1u << PORTE_NVIC_SHIFT); //Enables the interrupt (Set-enable register)
}

void SDHC_detection_disable()
{
	PORTE_PCR6 ^= PORTE_PCR_IQR_EDGE_MODE << PORTE_PCR_IQR_CONF_SHIFT; //Set interrupt on rising and falling edge
	NVICISER1 ^= 1u << PORTE_NVIC_SHIFT; //Enables the interrupt (Set-enable register)
}

void SDHC_NVIC_enable()
{
	//Enable Command Complete interrupt
	SDHC_IRQSTAT |= SDHC_IRQSTAT_CC_MASK;
	SDHC_IRQSTATEN |= SDHC_IRQSTAT_CC_MASK;
	SDHC_IRQSIGEN |= SDHC_IRQSTAT_CC_MASK;
	
	//Enable Command CRC Error interrupt
	SDHC_IRQSTAT |= SDHC_IRQSTAT_CCE_MASK;
	SDHC_IRQSTATEN |= SDHC_IRQSTAT_CCE_MASK;
	SDHC_IRQSIGEN |= SDHC_IRQSTAT_CCE_MASK;
	
	//Enable Command Index Error interrupt
	SDHC_IRQSTAT |= SDHC_IRQSTAT_CIE_MASK;
	SDHC_IRQSTATEN |= SDHC_IRQSTAT_CIE_MASK;
	SDHC_IRQSIGEN |= SDHC_IRQSTAT_CIE_MASK;
	
	NVICISER2 |= (1u << SDHC_NVIC_SHIFT); //Enable the SDHC interrupts
	NVICIP20 &= ~(HIGH_PRIORITY_IRQ_MASK << SDHC_NVICIPR_SHIFT); //Set them as high priority
}

void SDHC_NVIC_disable()
{
	//Disable Command Complete interrupt
	SDHC_IRQSTATEN &= ~(SDHC_IRQSTAT_CC_MASK);
	SDHC_IRQSIGEN &= ~(SDHC_IRQSTAT_CC_MASK);
		
	//Disable Command CRC Error interrupt
	SDHC_IRQSTATEN &= ~(SDHC_IRQSTAT_CCE_MASK);
	SDHC_IRQSIGEN &= ~(SDHC_IRQSTAT_CCE_MASK);
	
	NVICISER2 &= ~(1u << SDHC_NVIC_SHIFT); //Enable the SDHC interrupts
}

void SDHC_clock_enable()
{
	SIM_SCGC3 |= SIM_SCGC3_SDHC_MASK; //Enable clock for SDHC
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; //Enable clock for PORTE
}

uint8_t checksum(uint16_t arg_HB, uint16_t arg_LB, uint8_t cmd_index, uint8_t is_cmd)
{
	uint32_t cmd = (is_cmd << IS_CMD_SHIFT);
	cmd |= (cmd_index << CMD_INDEX_SHIFT);
	cmd |= arg_HB;
	
	divide_poly(&cmd);
	
	cmd <<= CMD_INDEX_SHIFT;
	cmd |= arg_LB;
	cmd <<= POLY_MULT_SHIFT;
	
	divide_poly(&cmd);
	
	return (uint8_t)cmd;
}

void divide_poly(uint32_t *cmd)
{
	uint8_t index = 0;
	
	while(*cmd > GENERATOR_POLY)
	{
		if(1 < (*cmd >> index))
		{
			index++;
		} else {
			*cmd ^= (GENERATOR_POLY << (index-CRC_WIDTH));
			index = 0;
		}
	}
}

void send_cmd(uint8_t cmd_index, uint8_t cmd_type, uint8_t resp_type, uint32_t arg)
{
	SDHC_CMDARG = arg; //Set the cmd to be sent
	
	//Set the index, cmd type and 
	SDHC_XFERTYP = ((SDHC_XFERTYP_CMDINX(cmd_index)) | (SDHC_XFERTYP_CMDTYP(cmd_type)) | (SDHC_XFERTYP_RSPTYP(resp_type)));
}

void SDHC_init()
{
	//SDHC_SYSCTL |= SDHC_SYSCTL_SDCLKEN_MASK; //SD clock enabled
	//SDHC_SYSCTL |= SDHC_SYSCTL_PEREN_MASK; //Peripheral clock enabled
	//SDHC_SYSCTL |= SDHC_SYSCTL_HCKEN_MASK; //System clock enabled
	//SDHC_SYSCTL |= SDHC_SYSCTL_IPGEN_MASK; //Bus clock enabled
	
	while(0 == is_SDclock_stable()){asm("nop");} //Wait until the SDclock is stable
	
	if(0 == (SDHC_PRSSTAT&SDHC_PRSSTAT_CIHB_MASK) && 0 == (SDHC_PRSSTAT&SDHC_PRSSTAT_CDIHB_MASK))
	{
		SDHC_SYSCTL |= SDHC_SYSCTL_INITA_MASK; //Send 80 SD-clocks and waits until it finishes
	}
	
	while(is_SDclock_working()){asm("nop");}
	
	SDHC_data.expect_resp = CMD0_RESP;
	send_cmd(CMD0_INDEX,CMDTYP_NORMAL,RSPTYP_NO_RESP,GO_IDLE_STATE); //Send the CMD0
	//send_cmd(CMD8_INDEX,CMDTYP_NORMAL,RSPTYP_48,SEND_IF_COND);
}

void SDHC_stop()
{
	SDHC_SYSCTL &= ~(SDHC_SYSCTL_SDCLKEN_MASK); //SD clock disabled
	SDHC_SYSCTL &= ~(SDHC_SYSCTL_PEREN_MASK); //Peripheral clock disabled
	SDHC_SYSCTL &= ~(SDHC_SYSCTL_HCKEN_MASK); //System clock disabled
	SDHC_SYSCTL &= ~(SDHC_SYSCTL_IPGEN_MASK); //Bus clock disabled
}

uint8_t is_SDHC_inserted()
{
	return (PORTE_PDDR6 == (GPIOE_PDIR&PORTE_PDDR6))? 1u : 0u;
}

uint8_t is_SDclock_working()
{
	return (SDHC_SYSCTL_INITA_MASK == (SDHC_SYSCTL&SDHC_SYSCTL_INITA_MASK)) ? 1u : 0u;
}

uint8_t is_SDclock_stable()
{
	return (SDHC_PRSSTAT_SDSTB_MASK == (SDHC_PRSSTAT&SDHC_PRSSTAT_SDSTB_MASK)) ? 1u : 0u;
}

