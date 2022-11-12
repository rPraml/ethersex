
ifdef(`conf_STATUSLED_HB_ACT', `dnl
pin(STATUSLED_HB_ACT, PD4, OUTPUT)
')dnl


ifdef(`conf_RFM12', `dnl
  /* port the rfm12 module CS is attached to */
  pin(SPI_CS_RFM12, PD7,OUTPUT)
  RFM12_NO_INT
  /* nur INT 0 available = PD2 (Pin 10) */

  /* port the LEDS for rfm12 txrx attached to */
  ifdef(`conf_STATUSLED_RFM12_TX', `
    pin(STATUSLED_RFM12_TX, PD4, OUTPUT)
  ')
  ifdef(`conf_STATUSLED_RFM12_RX', `
    pin(STATUSLED_RFM12_RX, PD5, OUTPUT)
  ')
')dnl
ifdef(`conf_STELLA', `dnl
  STELLA_PORT1_RANGE(PC0,PC2)
  STELLA_USE_TIMER(2)
')

