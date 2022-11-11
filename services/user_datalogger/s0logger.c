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
#include "s0logger.h"
#ifdef DATA_LOGGER_KACO    
#include "kaco.h"
#endif
#define TICS_PER_HOUR 6328L
#define TICS_MAX 	  7000
#define TICS_MIN 	  300



#define CONVERT_FAIL 0
#define CONVERT_OK 1
/* Output des Zählers
/ACE0\3k260V01.18
"F.F(00)
C.1(1126110052708491)
C.5.0(00)
1.8.0(000052.5*kWh)
2.8.0(000172.3*kWh)
!
"
*/


static uint8_t datalogger_rx_state;

struct s0_status_t {
	uint16_t errorFlag;	// Bedeutung unbekannt
	uint16_t statusFlag;	// Bit 0 = Richtung, bit 1= Creep Flag
	uint8_t online; // 1 = Zähler online
	uint32_t energyFwd; // 100 Wh
	uint16_t powerFwd; // W
	uint32_t energyRev; // 100 Wh
	uint16_t powerRev; // w
	int32_t watt;
} s0_status;


typedef union {  // union erlaubt einen effektiven, seperaten Zugriff auf Teile der Variable
	uint32_t i32;
	struct {
		uint8_t i8l;        // low
		uint8_t i8m;        // mid
		uint16_t high;  // high, soft timer
	};
} convert32to8;

volatile uint32_t timestamp;    // volatile wegen Zugriff im Interrrupt
volatile uint16_t softtimer;
volatile uint32_t zeitdifferenz;
volatile uint16_t pulse;
volatile uint8_t block_zeitmessung;


static uint16_t s0_powerFwdTick;
static uint16_t s0_powerRevTick;

volatile uint16_t statusled;

void s0_ovfTick() {
	++softtimer; 		   // zählen der Überläufe
	if (block_zeitmessung) block_zeitmessung--;
	if (s0_powerFwdTick < TICS_MAX) s0_powerFwdTick++;
	if (s0_powerRevTick < TICS_MAX) s0_powerRevTick++;
    if (statusled-- == 2) {
        PIN_CLEAR(STATUSLED_S0_ACT);
    } else if(statusled-- == 1) {
        PIN_SET(STATUSLED_S0_ACT);
    }
}

void s0_capture() {
    
    statusled=2;
    convert32to8 cap;        // Variablendeklaration

	cap.i8l = ICR1L;         // low Byte zuerst, high Byte wird gepuffert
	cap.i8m = ICR1H;
	
	// overflow verpasst, wenn ICR1H klein und wartender Overflow Interrupt
	if ((cap.i8m < 128) && (TIFR0 & (1<<TOV1))) {
		// wartenden timer overflow Interrupt vorziehen
		++softtimer;
		TIFR0 = (1<<TOV1);    // timer overflow int. löschen, da schon hier ausgeführt
	}
	cap.high = softtimer;    // obere 16 Bit aus Software Zähler

	if (block_zeitmessung) {
		timestamp = cap.i32;     // Zeit merken
		pulse = 0;
	} else {
		zeitdifferenz = cap.i32 - timestamp;
		pulse++;
	}
	//(zeitdifferenz - (zeitdifferenz / 4)) + ((cap.i32 - timestamp) / 4);
}


void s0_start() {
	util_readline('\n', READLINE_CRLF); // to reset buffer
	datalogger_rx_state = 0;
	block_zeitmessung = 10;
	
	memcpy_P(usart_tx_buffer(), PSTR("/?!\r\n"),5); //
	usart_setbaud(BAUDRATE(300),7,'E',1);
	usart_tx_start(5, NULL); // transmit 5 chars from buffer
}
int s0_ready() {
    return pulse > 4;
}
/**
 * @return -1 wenn Fertig, 0 wenn nochmal, > 0 Dauer
 **/
int16_t s0_process_rx(uint8_t ch) {
	block_zeitmessung = 10;
	if (ch == 2) { // STX
		datalogger_rx_state = 10;
		return 0;
	}
	if (ch == 3) { // ETX
		s0_status.online = CONVERT_OK;
		return -1;
	}
	uint16_t tmp_lo;
	uint32_t tmp_hi, delta;
	static uint32_t last_consumed;
	static uint32_t last_produced;

	char * line = util_readline(ch, READLINE_CRLF);
	if (!line) return 0; // no line yet there

	switch (datalogger_rx_state) {
	case 0:
		//strncpy(datalogger_ident, datalogger_scratch, sizeof(datalogger_ident) );
		_delay_ms(50);
		memcpy_P(usart_tx_buffer(), PSTR("\x06""000\r\n"),6); // copy init sequence into buffer
		usart_tx_start(6, NULL);
		datalogger_rx_state++;
		break;

	case 10:
		// hier werden die Zeilen vom Zähler geparsed
		sscanf_P(line, PSTR("FF(%x)"), 	&s0_status.errorFlag);
		sscanf_P(line, PSTR("C.5.0(%x)"), 	&s0_status.statusFlag);
		s0_status.watt = (uint32_t)(F_CPU/64*3600) / (uint32_t)zeitdifferenz * pulse;
		if (s0_status.statusFlag & 0x01) s0_status.watt *=  -1;
		if (sscanf_P(line, PSTR("1.8.0(%lu.%1u*"), &tmp_hi, &tmp_lo) == 2) {
			tmp_hi = tmp_hi * 10 + tmp_lo;
			s0_status.energyFwd = tmp_hi;
			if (s0_powerFwdTick >= TICS_MAX) s0_status.powerFwd = 0;
			if (!last_consumed) last_consumed = tmp_hi;
			if (last_consumed != tmp_hi) { // Ändert sich die Kommastelles0_consumedTick++;
				if (s0_powerFwdTick < TICS_MIN) {
					//Messung ungültig / zu schnell
				} else {
					delta = tmp_hi - last_consumed;
					s0_status.powerFwd = (TICS_PER_HOUR*100L * delta)/s0_powerFwdTick;
					last_consumed = tmp_hi;
					s0_powerFwdTick = 0;
				}
				
			}
		}
		if (sscanf_P(line, PSTR("2.8.0(%lu.%1u*"), &tmp_hi, &tmp_lo) == 2) {
			tmp_hi = tmp_hi * 10 + tmp_lo;
			s0_status.energyRev  = tmp_hi;
			if (s0_powerRevTick >= TICS_MAX) s0_status.powerRev = 0;
			if (!last_produced) last_produced = tmp_hi;
			if (last_produced!= tmp_hi) { // Ändert sich die Kommastelles0_consumedTick++;
				if (s0_powerRevTick < TICS_MIN) {
					//Messung ungültig
				} else {
					delta = tmp_hi - last_produced;
					s0_status.powerRev = (TICS_PER_HOUR*100L * delta)/s0_powerRevTick;
					last_produced = tmp_hi;
					s0_powerRevTick = 0;
				}
			}
		}

		return 5; // timeout ~2 sec
	}
	return 0;
}

void s0_process_rx_err(void) {
	s0_status.online = CONVERT_FAIL;
}

int16_t
s0_ecmd_status(char *cmd, char *output, uint16_t len) {
	static uint8_t line;
    int32_t kaco_pwr;
	//uint16_t watt;
	if (cmd[0] != 23) {
		cmd[0] = 23;
		line = 0;
	}
	switch (line++) {
//	case 0:
//		return ECMD_AGAIN(snprintf_P(output, len, PSTR("#ERR,STAT,ONL,CURR_PWR,TOTAL_FWD ,PWR_FWD , TOTAL_REV ,PWR_REV"),
	case 0:
		/*
		1000 pulse = 1KWh
		1 Pulse = 1Wh
		Timerticks: 115200 / sec = F_CPU/64*3600 per hour	*/
		len = snprintf_P(output, len, PSTR("%4d,%4d,%3d,%8ld,"),
		        s0_status.errorFlag, 
			s0_status.statusFlag, 
			s0_status.online, s0_status.watt);
		output[len] = ECMD_NO_NEWLINE;
		return ECMD_AGAIN(len);
	
	case 1:
		len = snprintf_P(output, len, PSTR("%10lu00,%8u,"),
		        s0_status.energyFwd, s0_status.powerFwd);
		output[len] = ECMD_NO_NEWLINE;
		return ECMD_AGAIN(len);
	
	case 2:
		len = snprintf_P(output, len, PSTR("%10lu00,%8u"),
		        s0_status.energyRev, s0_status.powerRev);

#ifdef DATA_LOGGER_KACO    
		output[len] = ECMD_NO_NEWLINE;
		return ECMD_AGAIN(len);
    case 3:
        kaco_pwr = kaco_get_total_power();
        len = snprintf_P(output, len, PSTR(",%10ld,%10ld"),
		        kaco_pwr, kaco_pwr + s0_status.watt);

#endif
		return ECMD_FINAL(len);

	/*
		return ECMD_AGAIN(snprintf_P(output, len, PSTR("{e: %d, s: %d, o %d, p %ld,"),
		        s0_status.errorFlag, 
			s0_status.statusFlag, 
			s0_status.online, s0_status.watt));
		break;
	case 1:
		return ECMD_AGAIN(snprintf_P(output, len, PSTR("consumed: {energy: %lu00, power: %u},"),
		        s0_status.energyFwd, s0_status.powerFwd));
		break;
	case 2:
		return ECMD_FINAL(snprintf_P(output, len, PSTR("returned: {energy: %lu00, power: %u}}"),
		        s0_status.energyRev, s0_status.powerRev));
	*/
	}
	return ECMD_FINAL_OK;
}