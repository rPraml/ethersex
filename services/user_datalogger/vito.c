#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "core/util/fixedpoint.h"
#include "config.h"
#include "protocols/ecmd/ecmd-base.h"
#include "protocols/mqtt/mqtt.h"
#include "vito.h"
#define USE_USART DATA_LOGGER_USE_USART
#include "usart.h"
#include "datalogger.h"
#include "core/periodic.h"

#define KW_START 0x01
#define KW_END 0x04
#define KW_SYNC 0x05


#define VITO_DATA_COUNT 20
#define VITO_NAME_LEN 8


#define VITO_STATE_SYNCING 0
#define VITO_STATE_SYNCED 1
#define VITO_STATE_TXC 2
#define VITO_STATE_NEXT_CMD 3

#define CONV_NONE 1
#define CONV_S_DIV2 2
#define CONV_U_DIV2 3
#define CONV_U_DIV10 4


typedef struct
{
  /* sensor has a name assigned */
  uint8_t type;
  uint16_t addr;
  uint8_t len;
  uint32_t value;
  uint8_t convType;
  char name[VITO_NAME_LEN + 1];
} vito_data_t;

vito_data_t vito_data[VITO_DATA_COUNT];

/*
volatile unsigned int vito_timeout_counter;
void vito_timeout(void) {
    if (vito_timeout_counter)  vito_timeout_counter--;
}

// -------------- Viessmann
int16_t viessmann_readChar(void) {
	int16_t ch;
	while (vito_timeout_counter) {
		wdt_kick();
		_delay_ms(1);
		ch = usart_rx_get();
		if (ch > 0) return ch;
	}
	return -1;
}


int16_t vito_sync(void) {
	//usart_tx_buffer()[0] = 0x04; // sync
	//usart_tx_start(5, NULL);
	vito_timeout_counter = PERIODIC_MS2MTICKS(2000);
  while (vito_timeout_counter) {
	 if (viessmann_readChar() == KW_SYNC) {
     return 1;
   }
  }
  return 0;
}
*/

uint8_t vito_convert(uint8_t convType, uint32_t input, char *target) {
  switch (convType) {
    case CONV_NONE: // *1
    default:
      return snprintf_P(target, 16, PSTR("%ld"), input);
    case CONV_S_DIV2: // Signed bytes div/2
      return itoa_fixedpoint(((int8_t)(input & 0xFF)) * 5, 1, target, 16);   
    case CONV_U_DIV2:
      return itoa_fixedpoint(input * 5, 1, target, 16);   
    case CONV_U_DIV10:
      return itoa_fixedpoint(input, 1, target, 16);   

  }
}

bool vito_mqtt_publish(uint8_t convType, uint32_t input, char *name) {
  char value[16];
  uint8_t value_length;
  char topic[64];
  uint8_t topic_len;
  
  topic_len = snprintf_P(topic, 64, PSTR("tele/vito_%s/temp"), name);
  topic[topic_len] = 0;
  value_length = vito_convert(convType, input, value);
  return mqtt_construct_publish_packet(topic, value, value_length, false);
}

//static volatile char vito_txc_flag;

// https://github.com/steand/optolink/blob/f45524259ba109d9a42e805e0c3667d77938a8af/src/main/java/de/myandres/optolink/ViessmannKW.java

static volatile int vito_state;
static volatile int vito_index;
static volatile int vito_index_publishing = -1;
static volatile int vito_bytes;
static volatile uint32_t vito_value;

void vito_txc(void) {
	//vito_txc_flag = 1;
  vito_state = VITO_STATE_TXC;
}


int16_t vito_send_cmd(bool sync) {
  vito_value = 0;
  char *ptr = usart_tx_buffer();
  vito_bytes = vito_data[vito_index].len;
  uint8_t mode = vito_data[vito_index].type;
  uint16_t addr = vito_data[vito_index].addr;
  uint8_t len = vito_data[vito_index].len;
  uint8_t datalen = 0;
  
  vito_state = VITO_STATE_SYNCED;
  if (sync) {
    *ptr++ = KW_START;
    datalen++;
  }
  
  *ptr++ = mode;
  datalen++;
  
  switch (mode) {
      // Lesebefehle
    case 0xF7: // Virtuell_Read (normales Lesen)
    case 0x6B: // 6B=GFA_Read
    case 0x7B: // 7B=PROZESS_READ
      *ptr++ = addr >> 8;
      *ptr++ = addr;
      *ptr++ = len;
      datalen += 3;
      break;
    
    case 0xC7: // VIRTUAL READ
    case 0xCB: // PHYSICAL READ
    case 0xAE: // EEPROM READ
    case 0xC5: // PHYSICAL XRAM READ
    case 0x6E: // PHYSICAL PORT READ
    case 0x9E: // PHYSICAL BE READ
    case 0x33: // PHYSICAL KMBUS RAM READ
    case 0x43: // PHYSICAL KMBUS EEPROM READ
      *ptr++ = addr;
      *ptr++ = len;
      datalen += 2;
      break;
    default:
      VITO_DEBUG("Mode: %02x not supported\n", mode);
      return FINISH_ERR;
  }
  
  *ptr++ = KW_END;
  datalen++;
  
  usart_tx_start(datalen, vito_txc);
  VITO_DEBUG("TX:");
  ptr = usart_tx_buffer();
  for (int i = 0; i < datalen; i++) {
    VITO_DEBUG_CONT(" %02x", ptr[i]);
  }
  return PERIODIC_MS2MTICKS(300); // Innerhalb von 300ms sollte die Antwort kommen, sonst->Timeout  
}
int16_t vito_process_rx(uint8_t ch) {
  //VITO_DEBUG("RX %d (%02x) State %d\n", ch, ch, vito_state);
  if (vito_state == VITO_STATE_SYNCING && ch == KW_SYNC) {
    return vito_send_cmd(true);
  }
  if (vito_state == VITO_STATE_TXC) {
    vito_value |= ch & 0xFF;
    if (--vito_bytes <= 0) {
      vito_data[vito_index].value = vito_value;
      VITO_DEBUG_CONT(" RX: %s %ld (%lx)\n",
        vito_data[vito_index].name, vito_value, vito_value);
        
      if (vito_data[vito_index].convType) {
        vito_index_publishing = vito_index;
      }
      if (vito_index + 1 < VITO_DATA_COUNT) {
        if (vito_data[vito_index + 1].type) {
          vito_index++;
          return vito_send_cmd(false);
        }
      }
      return FINISH_OK;
    }
    vito_value = vito_value << 8;
  }
  return CONTINUE;
}

void vito_process_err(void) {
  VITO_DEBUG_CONT(" Timeout\n");
  vito_data[vito_index].value = 0xFFFFFFFF;
}

void vito_mqtt_poll_cb(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (vito_index_publishing != -1) {
      vito_mqtt_publish(
        vito_data[vito_index_publishing].convType,
        vito_data[vito_index_publishing].value,
        vito_data[vito_index_publishing].name);
      vito_index_publishing = -1;
    }
  }
}

const mqtt_callback_config_t vito_mqtt_callback_config PROGMEM = {
  .topic = NULL,
  .connack_callback = NULL,
  .poll_callback = vito_mqtt_poll_cb,
  .close_callback = NULL,
  .publish_callback = NULL
};

void vito_start(void) {
  if (vito_index_publishing != -1) {
    datalogger_setmode(DATALOGGER_STATE_IDLE);
    return;
  }
  int i = VITO_DATA_COUNT;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    // find next index
    while (i-- > 0) {
      vito_index++;
      if (vito_index >= VITO_DATA_COUNT) {
        vito_index = 0;
      }
      if (vito_data[vito_index].type != 0) {
        break;
      }
    }
  }
  if (i == 0) {
    datalogger_setmode(DATALOGGER_STATE_IDLE);
    return;
  }
  if (vito_data[vito_index].convType == 5) {
    vito_data[vito_index].addr = (vito_data[vito_index].addr + 1) & 0xFF;
    int len = snprintf_P(vito_data[vito_index].name, VITO_NAME_LEN, PSTR("Tst%02x%02x"),
      vito_data[vito_index].type, vito_data[vito_index].addr);
    vito_data[vito_index].name[len] = 0;
  }
  
  VITO_DEBUG("Fetch %d %s\n", vito_index, vito_data[vito_index].name);
  vito_state = VITO_STATE_SYNCING;
  usart_setbaud(BAUDRATE(4800),8,'E',2);
	usart_rx_start();
  
}

/*
int16_t vito_getData(int mode, uint16_t addr, int len, uint32_t data, char *buf) {

	int16_t ch,i, datalen;
	if (!vito_sync()) {
    VITO_DEBUG("No sync!\n");
    return -1;
  }
	char *ptr = usart_tx_buffer();
	*ptr++ = KW_START;
	*ptr++ = mode;
	
	switch (mode) {
		// Lesebefehle
	case 0xF7: // Virtuell_Read (normales Lesen)
  case 0x6B: // 6B=GFA_Read
  case 0x7B: // 7B=PROZESS_READ
	  *ptr++ = addr >> 8;
    *ptr++ = addr;
    *ptr++ = len;
    datalen = 6;
    break;
  
  case 0xC7: // VIRTUAL READ
	case 0xCB: // PHYSICAL READ
	case 0xAE: // EEPROM READ
	case 0xC5: // PHYSICAL XRAM READ
	case 0x6E: // PHYSICAL PORT READ
	case 0x9E: // PHYSICAL BE READ
	case 0x33: // PHYSICAL KMBUS RAM READ
	case 0x43: // PHYSICAL KMBUS EEPROM READ
    *ptr++ = addr;
	  *ptr++ = len;
	  datalen = 5;
		break;
    
    // writes not yet implemented!
	**case 0xC8:

		for (datalen = 0; datalen < len; datalen++) {
			*ptr++ = data & 0xFF;
			data = data >> 8;
		}
		datalen =5 + len;
		break;**
	default:
		VITO_DEBUG("Mode: %02x not supported\n", mode);
    return -1;
	}

	*ptr++ = KW_END;
	vito_txc_flag = 0;
	usart_tx_start(datalen, vito_txc) ;
	while(!vito_txc_flag) { wdt_kick(); _delay_ms(1); }
	
	vito_timeout_counter = PERIODIC_MS2MTICKS(100);
	for (i = 0; i < len; i++) {
		ch = viessmann_readChar();
		if (ch < 0 || !vito_timeout_counter) {
      VITO_DEBUG("Mode: %02x, Addr: %04x, Len %02x - timeout\n", mode, addr, len);
      return -1;
    }
    if (ch != 0x05) {
      vito_timeout_counter = PERIODIC_MS2MTICKS(100); // do not retrigger on periodic sync!
		}
		snprintf_P(&buf[i*5], 6, PSTR("0x%02x,"),ch);
	}
  VITO_DEBUG("Mode: %02x, Addr: %04x, Len %02x, RX: %s\n", mode, addr, len, buf);
	return len * 5 - 1;
}

int16_t parse_cmd_datalogger_vito_cmd(char *cmd, char *output, uint16_t len) 
{
  int ret, fnct, addr, dataLen;
	uint32_t data;
  while (datalogger_state != DATALOGGER_STATE_IDLE) {
		wdt_kick();
  }
  datalogger_setmode(DATALOGGER_STATE_TX_VITO);
  
	sscanf_P(cmd, PSTR("%x %x %x %lx"), &fnct, &addr, &dataLen, &data);
	if (dataLen * 5 > len) return ECMD_FINAL(snprintf_P(output, len, PSTR("Too long")));

  
	timeout_counter = 10;
	usart_setbaud(BAUDRATE(4800),8,'E',2);
	usart_rx_start();
	if ((ret = vito_getData(fnct, addr,dataLen,data, output)) >0) {
		datalogger_setmode(DATALOGGER_STATE_IDLE);
		return ECMD_FINAL(ret);
	}
	datalogger_setmode(DATALOGGER_STATE_IDLE);
	return ECMD_FINAL(snprintf_P(output, len, PSTR("Error")));
}
*/

int16_t
parse_cmd_datalogger_vito_set(char *cmd, char *output, uint16_t len)
{
	uint16_t id, type, addr, dataLen, convType;
	char name[VITO_NAME_LEN + 1];
	sscanf_P(cmd, PSTR("%x %x %x %x %x %8s"), &id, &type, &addr, &dataLen, &convType, name);
  if (id > VITO_DATA_COUNT) {
    return ECMD_ERR_PARSE_ERROR;
  }
  vito_data[id].type = type;
  vito_data[id].addr = addr;
  vito_data[id].len = dataLen;
  vito_data[id].value = 0xFFFFFFFF;
  vito_data[id].convType = convType;
  strncpy(vito_data[id].name, name, VITO_NAME_LEN);
  return ECMD_FINAL_OK;
}

int16_t
parse_cmd_datalogger_vito_list(char *cmd, char *output, uint16_t len)
{
    /* trick: use bytes on cmd as "connection specific static variables" */
  if (cmd[0] != ECMD_STATE_MAGIC)       /* indicator flag: real invocation:  0 */
  {
    cmd[0] = ECMD_STATE_MAGIC;  /* continuing call: 23 */
    cmd[1] = 0;                 /* counter for sensors in list */
  }

  uint8_t i = cmd[1];

  /* This is a special case: the while loop below printed a sensor which was
   * last in the list, so we still need to send an 'OK' after the sensor id */
  if (i >= VITO_DATA_COUNT)
  {
    return ECMD_FINAL_OK;
  }

  
  while (vito_data[i].type == 0 || vito_data[i].value == 0xFFFFFFFF) {
    i++;
    if (i >= VITO_DATA_COUNT)
    {
      return ECMD_FINAL_OK;
    }
  }
  cmd[1] = i + 1;
  
  vito_data_t data = vito_data[i];

  char value[16];
  uint8_t value_length;
  value_length = vito_convert(data.convType, data.value, value);
  value[value_length] = 0;
  
  int ret = snprintf_P(output, len,
                   PSTR("%d\t%02x%02x%02x %02x %08lx\t%s\t%s"),
                   i,
                   data.type,
                   data.addr,
                   data.len,
                   data.convType,
                   data.value,
                   value,
                   data.name);

  /* set return value that the parser has to be called again */
  if (ret > 0) {
    ret = ECMD_AGAIN(ret);
  }
  return ECMD_FINAL(ret);
}

void vito_add(uint8_t type, uint8_t addr, uint8_t len, uint8_t convType, char * name) {
  static int i;
  vito_data[i].type = type;
  vito_data[i].addr = addr;
  vito_data[i].len = len;
  vito_data[i].value = 0xFFFFFFFF;
  vito_data[i].convType = convType;
  strncpy(vito_data[i].name, name, VITO_NAME_LEN);
  i++;
}
void vito_init(void) {
  vito_add(0xCB, 0x6F, 1, CONV_S_DIV2,  "Aussen");
  vito_add(0xCB, 0x69, 1, CONV_U_DIV2,  "VL_Soll");
  vito_add(0xCB, 0x70, 1, CONV_U_DIV2,  "Kessel_I");
  vito_add(0xCB, 0x71, 1, CONV_U_DIV2,  "Kessel_S");
  
  vito_add(0xCB, 0x42, 1, CONV_U_DIV2,  "WW_I");
  vito_add(0xCB, 0x5C, 1, CONV_U_DIV2,  "WW_S");
  
  //vito_add(0xCB, 0x64, 1, CONV_NONE,    "Niveau");  // OK
  //vito_add(0xCB, 0x65, 1, CONV_U_DIV10, "Neigung"); // OK 

  vito_add(0xCB, 0x16, 1, CONV_NONE,    "Brenner0");
  vito_add(0xCB, 0x17, 1, CONV_NONE,    "Brenner1");
  vito_add(0xCB, 0x18, 1, CONV_NONE,    "Brenner2");
  //vito_add(0xCB, 0x22, 1, CONV_NONE,    "Brenner3");
  vito_add(0xCB, 0x23, 1, CONV_NONE,    "Brenner4");

  //vito_add(0xAE, 0x17, 1, CONV_NONE,    "BStd1"); // FALSCH
  vito_add(0xAE, 0x18, 1, CONV_NONE,    "BStd2"); // FALSCH
  vito_add(0xC5, 0x17, 1, CONV_NONE,    "BStd3"); // FALSCH
  //vito_add(0xC5, 0x18, 1, CONV_NONE,    "BStd4"); // FALSCH

//  vito_add(0xCB, 0x23, 2, CONV_NONE,    "Brenner2");


  vito_add(0xCB, 0x3F, 1, CONV_NONE,    "Fehler");
  vito_add(0xCB, 0x51, 1, CONV_NONE,    "Programm");
  vito_add(0xCB, 0xB0, 1, CONV_NONE,    "Drehzahl");
 //vito_add(0xCB, 0x53, 1, CONV_NONE,    "Raum_S");
  

/*  
  #POLL, 01CB6D0104, 1ByteS, 2 , - , - 0 read
#POLL, 01CB6E0104, 1ByteS, 2 , - , - 0 read


POLL, 01CB3F0104, 1ByteS, 1 , Fehlermeldung , -
POLL, 01CB5C0104, 1ByteS, 2 , TempWW_Soll, -
POLL, 01CB420104, 1ByteS, 2 , TempWW_IST, -
POLL, 01CB530104, 1ByteS, 1 , RaumSollTag, -
POLL, 01CB540104, 1ByteS, 1 , RaumSollNacht, -
POLL, 01CB220104, mode, state , BrennerStatus, -
POLL, 01CB170104, 1ByteU, 1 , Brennerlaufzeit, day
#POLL, 01CB000104, 1ByteS, 1 , ExtBetriebsumschaltung, -
#POLL, 01CB410104, 1ByteS, 1 , Vorlauftemperatur_HKA , -
#POLL, 01CB440104, 1ByteS, 1 , RuecklaufTemperatur , -
POLL, 01CB510104, mode , state , Betriebsprogramm , -
#POLL, 01CBB00104, 1ByteS, 1 , GWG_Drehzahl_Soll , -
POLL, 01CBA90104, 1ByteU, state , Umwaelzpumpe , -
#POLL, 01CBC50104, 1ByteU, 1 , Heizkessel_Ext_Anfo , -
#POLL, 01CBC70104, 1ByteU, 1 , Sommer-Winter , -
POLL, 01CBF80104, 4Byte , 1 , SystemIdentifikation , -
#POLL, 01CB000104, 1ByteU, state , GWG_ExternSperren , -
  */
}
/*
  This function will be called on request by menuconfig, if wanted...
  You need to enable ECMD_SUPPORT for this.
  Otherwise you can use this function for anything you like
int16_t
datalogger_vito_ecmd(char *cmd, char *output, uint16_t len) {

	int ret, fnct, addr, dataLen;
	uint32_t data;
	sscanf_P(cmd, PSTR("%x %x %x %lx"), &fnct, &addr, &dataLen, &data);
	if (dataLen * 5 > len) return ECMD_FINAL(snprintf_P(output, len, PSTR("Too long")));


	datalogger_viessmann_start();
	if ((ret = viessmann_getData(fnct, addr,dataLen,data, output)) >0) {
		datalogger_setmode(DATALOGGER_STATE_IDLE);
		return ECMD_FINAL(ret);
	}
	datalogger_setmode(DATALOGGER_STATE_IDLE);
	//strncpy(output, datalogger_ident, len);
	return ECMD_FINAL(snprintf_P(output, len, PSTR("Timeout")));
}
*/
//periodic_milliticks_header(services/user_datalogger/datalogger.h)
  //periodic_milliticks_isr(vito_timeout())
  // ecmd_feature(datalogger_vito_cmd, "vito cmd",, Manually call application sample commands)

/*
  -- Ethersex META 
  header(services/user_datalogger/vito.h)
  init(vito_init)
  mqtt_conf(vito_mqtt_callback_config)
  ecmd_feature(datalogger_vito_set, "vito set",, Manually call application sample commands)
  ecmd_feature(datalogger_vito_list, "vito list",, Manually call application sample commands)

*/