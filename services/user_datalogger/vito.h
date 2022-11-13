#ifndef VITO_H
#define VITO_H

void vito_init(void);
void vito_ovfTick(void) ;
int16_t vito_getData(int mode, uint16_t addr, int len, uint32_t data, char *buf);

int16_t datalogger_vito_ecmd(char *cmd, char *output, uint16_t len);

#define VITO_OFF() PIN_CLEAR(DATALOGGER_VIESS_MODE)
#define VITO_ON()  PIN_SET(DATALOGGER_VIESS_MODE)

#endif