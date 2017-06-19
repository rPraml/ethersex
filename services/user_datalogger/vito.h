#ifndef VITO_H
#define VITO_H
void vito_ovfTick(void) ;
int16_t vito_getData(int mode, uint16_t addr, int len, uint32_t data, char *buf);

#endif