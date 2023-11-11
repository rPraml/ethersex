#ifndef UTIL_H
#define UTIL_H
#define READLINE_CR 1
#define READLINE_CRLF 2
#define READLINE_NUL 3
char * util_readline(uint8_t ch, int mode);
#define DATALOGGER_SCRATCH_SIZE 80;
static char datalogger_scratch[DATALOGGER_SCRATCH_SIZE];
#endif