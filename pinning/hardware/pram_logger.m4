/* port the enc28j60 is attached to PB2 = INT2*/
pin(SPI_CS_NET, SPI_CS_HARDWARE)

/* port the sd-reader CS is attached to */
// pin(SPI_CS_SD_READER, PB1, OUTPUT)


pin(DATALOGGER_S0_MODE, PC1, OUTPUT)
pin(DATALOGGER_VIESS_MODE, PC0, OUTPUT)
pin(DATALOGGER_RS485_TX_ENABLE, PD5, OUTPUT)
pin(DATALOGGER_RS485_RX_DISABLE, PD4, OUTPUT)
pin(DATALOGGER_S0_IMPULS, PD6, INPUT)

/* Yellow */
pin(STATUSLED_S0_ACT, PC3, OUTPUT)
/* Green */
pin(STATUSLED_DATALOGGER_ACT, PC4, OUTPUT)
/* Red */
pin(STATUSLED_DEBUG, PC5, OUTPUT)



ONEWIRE_PORT_RANGE(PB0, PB1)

ifdef(`conf_BUTTONS_INPUT', `
  /* input buttons */
  pin(BTN_1, PA0, INPUT)
  pin(BTN_2, PA1, INPUT)
  pin(BTN_3, PA2, INPUT)
  pin(BTN_4, PA3, INPUT)

  #define BUTTONS_COUNT 4

  #define BUTTONS_CONFIG(_x) \
  _x(BTN_1)\
  _x(BTN_2)\
  _x(BTN_3)\
  _x(BTN_4)
')

ifdef(`conf_RFM12', `dnl
  /* port the rfm12 module CS is attached to */
  pin(SPI_CS_RFM12_0, PB3, OUTPUT)
  RFM12_USE_INT(1)
  /* nur INT 0 available = PD2 (Pin 10) */
  RFM12_ASK_SENSE_USE_INT(0)

  /* port the LEDS for rfm12 txrx attached to */
  ifdef(`conf_STATUSLED_RFM12_TX', `
    pin(STATUSLED_RFM12_TX, PC5, OUTPUT)
  ')
  ifdef(`conf_STATUSLED_RFM12_RX', `
    pin(STATUSLED_RFM12_RX, PC3, OUTPUT)
  ')
')dnl