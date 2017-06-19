/*
 * Copyright (c) 2011 Roland Praml pram@gmx.de
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <util/parity.h>
#include "config.h"
#include "protocols/ecmd/ecmd-base.h"


#include "util.h"
#include "usart.h"
#include "kaco.h"


typedef struct {
    int status;
    int pv_u; // v/10
    int pv_i; // a/100
    int pv_p; // w
    int ac_u; // v/10
    int ac_i; // a/100
    int ac_p; // w
    int temp;
    int32_t total;
} kaco_t;

static int kaco_id ;
static kaco_t kaco_status[DATA_LOGGER_KACO_MAX];


void kaco_rx_485(void) {
    PIN_CLEAR(DATALOGGER_RS485_TX_ENABLE);
	PIN_CLEAR(DATALOGGER_RS485_RX_DISABLE);
}
void kaco_start(int id) {
    if (id < 1 || id > DATA_LOGGER_KACO_MAX) return;
    kaco_id = id;
	util_readline('\n', READLINE_CRLF); // to reset buffer
	sprintf_P(usart_tx_buffer(), PSTR("#%02d0\r"),id); // send request
	usart_setbaud(BAUDRATE(9600),8,'N',1);
	usart_tx_start(5, kaco_rx_485); // transmit 5 chars from buffer
}
/**
 * @return -1 wenn Fertig, 0 wenn nochmal, > 0 Dauer
 **/
int16_t kaco_process_rx(uint8_t ch) {
	char * line = util_readline(ch, READLINE_NUL);
    //if (!kaco_active) return -1;
    
    if (!line) {
        return 0; // no line yet there
    }
    int status, temp;
    int pv_u_hi, pv_u_lo, pv_i_hi, pv_i_lo, pv_p;
    int ac_u_hi, ac_u_lo, ac_i_hi, ac_i_lo, ac_p;
    int32_t total;
    
    //                          id  st pv_u  pv_i pv_p ac_u  ac_i ac_p t total
    uint8_t crc, crc_i, id;
    crc = 0;
    for(id = 1; id < 57; id++) crc += line[id];
    if (sscanf_P(line, PSTR("\n*0%c0 %d %d.%d %d.%d %d %d.%d %d.%d %d %u %ld %c"), &id, &status, &pv_u_hi, &pv_u_lo, &pv_i_hi, &pv_i_lo, &pv_p,
    &ac_u_hi, &ac_u_lo, &ac_i_hi, &ac_i_lo, &ac_p, &temp, &total, &crc_i)==15) {
        id-= '1';
        id = kaco_id - 1;
        if (id != (kaco_id-1)) {
            kaco_status[kaco_id-1].status = -11; // CRC
            return -1;
        }
        if (crc_i != crc) {
            kaco_status[kaco_id-1].status = -10; // CRC
            return -1;
        }
        kaco_status[id].status = status;
        kaco_status[id].pv_u = pv_u_hi*10 + pv_u_lo;
        kaco_status[id].pv_i = pv_i_hi*100 + pv_i_lo;
        kaco_status[id].pv_p = pv_p;
        kaco_status[id].ac_u = ac_u_hi*10 + ac_u_lo;
        kaco_status[id].ac_i = ac_i_hi*100 + ac_i_lo;
        kaco_status[id].ac_p = ac_p;
        kaco_status[id].temp = temp;
        kaco_status[id].total = total;
    } else {
        kaco_status[kaco_id-1].status = -12; // CRC
        return -1;
    }
    //kaco_active = 0;
	return -1; // finish
}

void kaco_process_rx_err(void) {
    kaco_status[kaco_id-1].status = -2; //offline
	//kaco_active->status = -2;
}


int32_t kaco_get_total_power() {
    int i;
    int32_t total;
    total = 0;
    for(i = 0; i < DATA_LOGGER_KACO_MAX; i++) {
        if (kaco_status[i].status > 0) {
            total += kaco_status[i].ac_p; 
        }
    }
    return total;
}

int16_t
kaco_ecmd_status(char *cmd, char *output, uint16_t len) {
	static uint8_t line;
    static int id;
	if (cmd[0] != 23) {
        sscanf_P(cmd, PSTR("%d"), &id);
        if (id < 1 || id > DATA_LOGGER_KACO_MAX) return ECMD_ERR_PARSE_ERROR;
        id--;
		cmd[0] = 23;
		line = 0;
	}
    
    
    
	switch (line++) {
	case 0:
        len = snprintf_P(output, len, PSTR("%4d,%6d,%6d,%6d,"),
		        kaco_status[id].status, 
                kaco_status[id].pv_u, 
                kaco_status[id].pv_i,
                kaco_status[id].pv_p);
		output[len] = ECMD_NO_NEWLINE;
		return ECMD_AGAIN(len);
	case 1:
        len = snprintf_P(output, len, PSTR("%6d,%6d,%6d,"),
		        kaco_status[id].ac_u, 
                kaco_status[id].ac_i,
                kaco_status[id].ac_p);
		output[len] = ECMD_NO_NEWLINE;
		return ECMD_AGAIN(len);
	
	case 2:
        len = snprintf_P(output, len, PSTR("%6d,%6ld"),
		        kaco_status[id].temp, 
                kaco_status[id].total);
		return ECMD_FINAL(len);
	}
	
    return ECMD_FINAL_OK;
}

int16_t
kaco_init(void) {
    int i;
    for (i = 0; i < DATA_LOGGER_KACO_MAX; i++) kaco_status[i].status = -1;
	return 0;
}
/*
  -- Ethersex META --
  header(services/user_datalogger/kaco.h)
  init(kaco_init)
*/
