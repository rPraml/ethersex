#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "util.h"
static char datalogger_scratch[70];

char * util_readline(uint8_t ch, int mode) {
	static uint8_t pos;
    
	if ((mode == READLINE_CRLF) && (ch == '\r')) return NULL; // ignore
	if (((mode != READLINE_NUL) && (ch == '\n')) || 
        ((mode == READLINE_CR)  && (ch == '\r')) ||
        ((mode == READLINE_NUL) && (ch == 0))   ) {
		if (!pos) return NULL;
		datalogger_scratch[pos] = 0;
		pos = 0;
		return datalogger_scratch;
	}
	if (pos < 68) datalogger_scratch[pos++] = ch;
	return 0;
}
