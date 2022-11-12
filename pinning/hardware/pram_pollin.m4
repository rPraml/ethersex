/* port the enc28j60 is attached to PB2 = INT2*/
pin(SPI_CS_NET, SPI_CS_HARDWARE)

/* port the sd-reader CS is attached to */
pin(SPI_CS_SD_READER, PB1, OUTPUT)



ifdef(`conf_STATUSLED_HB_ACT', `dnl
pin(STATUSLED_HB_ACT, PD4, OUTPUT)
')dnl


ifdef(`conf_RFM12', `dnl
  /* port the rfm12 module CS is attached to */
  pin(SPI_CS_RFM12, PB0,OUTPUT)
  RFM12_NO_INT
  /* nur INT 0 available = PD2 (Pin 10) */

  /* port the LEDS for rfm12 txrx attached to */
  ifdef(`conf_STATUSLED_RFM12_TX', `
    pin(STATUSLED_RFM12_TX, PD6, OUTPUT)
  ')
  ifdef(`conf_STATUSLED_RFM12_RX', `
    pin(STATUSLED_RFM12_RX, PD4, OUTPUT)
  ')
')dnl
ifdef(`conf_STELLA', `dnl
  STELLA_PORT1_RANGE(PA4,PA7)
  STELLA_USE_TIMER(2)
')

