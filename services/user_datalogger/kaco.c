/*
 * Copyright (c) 2011 Roland Praml pram@gmx.de
 *
 * Parser for KACO Powador 3600
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
#include <util/atomic.h>
#include <util/delay.h>
#include <util/parity.h>
#include <util/crc16.h>
#include "config.h"
#include "datalogger.h"
#include "protocols/ecmd/ecmd-base.h"

#ifdef MQTT_SUPPORT
#include "protocols/mqtt/mqtt.h"
#endif

#include "util.h"
#include "usart.h"
#include "kaco.h"

typedef struct
{
  int status;
  int pv_u; // v/10
  int pv_i; // a/100
  int pv_p; // w
  int ac_u; // v/10
  int ac_i; // a/100
  int ac_p; // w
  int temp;
  int32_t total;
  uint8_t mqtt_pending : 1;
} kaco_t;

typedef struct 
{
  uint16_t pv1_u; // Reg 768
  uint16_t pv2_u;
  uint16_t pv1_i;
  uint16_t pv2_i;
  uint16_t ac_l1_u;
  uint16_t ac_l2_u;
  uint16_t ac_l3_u;
  uint16_t ac_l1_i;
  uint16_t ac_l2_i;
  uint16_t ac_l3_i;
  uint16_t ac_l1_hz;
  uint16_t ac_l2_hz;
  uint16_t ac_l3_hz;
  uint16_t power;
  uint16_t status;
  int16_t temperature;
  uint16_t error_hi;
  uint16_t error_lo;
  uint16_t energy_hi;
  uint16_t energy_lo;
  uint16_t hours_hi;
  uint16_t hours_lo;

  uint16_t energy_today; // 800
} goodwe_t;

static int kaco_id;
static kaco_t kaco_status[DATA_LOGGER_KACO_MAX];
static goodwe_t goodwe_status;
void kaco_rx_485(void)
{
  RS485_0_RX();
}

void kaco_start(int id)
{
  if (id < 1 || id > DATA_LOGGER_KACO_MAX)
    return;
  kaco_id = id;
  KACO_DEBUG("Start %02d\n", id);
  util_readline('\n', READLINE_CRLF);                 // to reset buffer
  sprintf_P(usart_tx_buffer(), PSTR("#%02d0\r"), id); // send request
  usart_setbaud(BAUDRATE(9600), 8, 'N', 1);
  usart_tx_start(5, kaco_rx_485); // transmit 5 chars from buffer and switch to RX
}

/**
 * @return -1 wenn Fertig, 0 wenn nochmal, > 0 Dauer
 **/
int16_t kaco_process_rx(uint8_t ch)
{
  char *line = util_readline(ch, READLINE_NUL);

  if (!line)
  {
    return CONTINUE; // no line yet there
  }

  int status, temp;
  int pv_u_hi, pv_u_lo, pv_i_hi, pv_i_lo, pv_p;
  int ac_u_hi, ac_u_lo, ac_i_hi, ac_i_lo, ac_p;
  int32_t total;
  uint8_t crc, crc_i, id;

  crc = 0;
  for (id = 1; id < 57; id++)
    crc += line[id];
  if (sscanf_P(line, PSTR("\n*0%c0 %d %d.%d %d.%d %d %d.%d %d.%d %d %u %ld %c"),
               &id, &status,
               &pv_u_hi, &pv_u_lo, &pv_i_hi, &pv_i_lo, &pv_p,
               &ac_u_hi, &ac_u_lo, &ac_i_hi, &ac_i_lo, &ac_p,
               &temp, &total, &crc_i) == 15)
  {
    id -= '1';
    id = kaco_id - 1;
    if (id != (kaco_id - 1))
    {
      KACO_DEBUG("ID err %02d\n", kaco_id);
      kaco_status[kaco_id - 1].status = -11; // CRC
      kaco_status[kaco_id - 1].mqtt_pending = 1;
      return FINISH_ERR;
    }
    if (crc_i != crc)
    {
      KACO_DEBUG("CRC err %02d\n", kaco_id);
      kaco_status[kaco_id - 1].status = -10; // CRC
      kaco_status[kaco_id - 1].mqtt_pending = 1;
      return FINISH_ERR;
    }
    kaco_status[id].status = status;
    kaco_status[id].pv_u = pv_u_hi * 10 + pv_u_lo;
    kaco_status[id].pv_i = pv_i_hi * 100 + pv_i_lo;
    kaco_status[id].pv_p = pv_p;
    kaco_status[id].ac_u = ac_u_hi * 10 + ac_u_lo;
    kaco_status[id].ac_i = ac_i_hi * 100 + ac_i_lo;
    kaco_status[id].ac_p = ac_p;
    kaco_status[id].temp = temp;
    kaco_status[id].total = total;
    KACO_DEBUG("%02d %s\n", kaco_id, line);
  }
  else
  {
    KACO_DEBUG("Comm err %02d\n", kaco_id);
    kaco_status[kaco_id - 1].status = -12; // CRC
    kaco_status[kaco_id - 1].mqtt_pending = 1;
    return FINISH_ERR; // Error
  }
  return FINISH_OK; // finish
}

void kaco_process_rx_err(void)
{
  KACO_DEBUG("Offline %02d\n", kaco_id);
  kaco_status[kaco_id - 1].status = -2; // offline
  kaco_status[kaco_id - 1].mqtt_pending = 1;
}

uint16_t
modbus_crc_calc(uint8_t *data, uint8_t len)
{
  uint16_t crc = 0xffff;
  uint8_t i = 0;
  while (i < len)
    crc = _crc16_update(crc, data[i++]);
  return crc;
}


static volatile uint8_t rx_addr; // address we expect to receive
static volatile uint8_t rx_len; // negth in byte
static volatile uint8_t rx_pos;  // buffer pos
static volatile uint16_t rx_reg; // start register

int16_t goodwe_process_rx(uint8_t ch)
{
  // KACO_DEBUG("CH %02x\n", ch);
  if (rx_pos < DATALOGGER_SCRATCH_SIZE)
  {
    datalogger_scratch[rx_pos++] = ch;
  }
  if (rx_pos == 1 && ch == rx_addr)
    return CONTINUE;

  if (rx_pos == 2 && ch == 0x03)
    return CONTINUE;

  if (rx_pos == 3 && ch < DATALOGGER_SCRATCH_SIZE - 5)
  {
    rx_len = ch;
    return CONTINUE;
  }

  if (rx_pos <= rx_len + 4)
  {
    return PERIODIC_MS2MTICKS(100);
  }

  if (rx_pos == rx_len + 5)
  {
    uint16_t crc = modbus_crc_calc(datalogger_scratch, rx_len + 3);
    uint16_t rx_crc =  datalogger_scratch[rx_pos - 1] << 8 | datalogger_scratch[rx_pos - 2];
    if (crc == rx_crc) {
      char * src = datalogger_scratch + 3;
      char * dst = &goodwe_status;
      for (crc = 0; crc < sizeof(goodwe_t); crc += 2) {
        dst[crc] = src[crc+1];
        dst[crc+1] = src[crc];
      }
      KACO_DEBUG("PV %d %d %d %d\n",
        goodwe_status.pv1_u, goodwe_status.pv2_u,
        goodwe_status.pv1_i, goodwe_status.pv2_i);
      KACO_DEBUG("AC-U %d %d %d\n",
        goodwe_status.ac_l1_u, goodwe_status.ac_l2_u, goodwe_status.ac_l3_u);
      KACO_DEBUG("AC-I %d %d %d\n",
        goodwe_status.ac_l1_i, goodwe_status.ac_l2_i, goodwe_status.ac_l3_i);
      KACO_DEBUG("AC-HZ %d %d %d\n",
        goodwe_status.ac_l1_hz, goodwe_status.ac_l2_hz, goodwe_status.ac_l3_hz);
      KACO_DEBUG("STATE %d %d %d\n",
        goodwe_status.power, goodwe_status.status, goodwe_status.temperature);

    }
    

    return FINISH_OK;
  }

  KACO_DEBUG("Invalid packet %02x %d %d\n", ch, rx_len, rx_pos);
  return FINISH_ERR;
}

void goodwe_process_rx_err(void)
{
}

typedef struct
{
  uint8_t addr;
  uint8_t func;
  uint16_t start;
  uint16_t count;
  uint16_t crc;
} modbus_tx_t;

void goodwe_rx_485(void)
{
  KACO_DEBUG("Modbus RX");
  RS485_1_RX();
}

void goodwe_read_register(uint8_t modbusAddr, uint16_t start, uint16_t length)
{
  KACO_DEBUG("Read register %04x %04x %04x ", modbusAddr, start, length);
  usart_setbaud(BAUDRATE(9600), 8, 'N', 1);
  uint8_t *buf = (uint8_t *)usart_tx_buffer();
  buf[0] = modbusAddr;
  buf[1] = 0x03; // Command
  buf[2] = start >> 8;
  buf[3] = start & 0xFF;
  buf[4] = length >> 8;
  buf[5] = length & 0xFF;
  uint16_t crc = modbus_crc_calc(buf, 6);
  buf[6] = crc & 0xFF;
  buf[7] = crc >> 8;
  rx_pos = 0;
  rx_addr = modbusAddr;
  rx_reg = start;
  // KACO_DEBUG("BUF %02x %02x %02x %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  usart_tx_start(8, goodwe_rx_485); // transmit 5 chars from buffer and switch to RX
}

void goodwe_start()
{
  goodwe_read_register(0xF7, 0x300, 33);
}

int32_t kaco_get_total_power()
{
  int i;
  int32_t total;
  total = 0;
  for (i = 0; i < DATA_LOGGER_KACO_MAX; i++)
  {
    if (kaco_status[i].status > 0)
    {
      total += kaco_status[i].ac_p;
    }
  }
  return total;
}

int16_t
kaco_ecmd_status(char *cmd, char *output, uint16_t len)
{
  static uint8_t line;
  static int id;

  if (cmd[0] != 23)
  {
    sscanf_P(cmd, PSTR("%d"), &id);
    if (id < 1 || id > DATA_LOGGER_KACO_MAX)
      return ECMD_ERR_PARSE_ERROR;
    id--;
    cmd[0] = 23;
    line = 0;
  }

  switch (line++)
  {
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

void kaco_init(void)
{
  for (int i = 0; i < DATA_LOGGER_KACO_MAX; i++)
    kaco_status[i].status = -1;
}

#ifdef MQTT_SUPPORT
bool kaco_mqtt_publish(int i)
{
  char topic[24];
  uint8_t topic_length;
  char buf[128];
  uint8_t buf_length;

  topic_length = snprintf_P(topic, 32, PSTR("tele/kaco_%d/value"), i + 1);
  topic[topic_length] = 0;

  buf_length = snprintf_P(buf, 128,
                          PSTR("{\"status\":%d,\"pv_u\":%d,\"pv_i\":%ld,\"pv_p\":%d,\"ac_u\":%d,\"ac_i\":%ld,\"ac_p\":%d,\"temp\":%d,\"total\":%ld}"),
                          kaco_status[i].status,
                          kaco_status[i].pv_u,
                          kaco_status[i].pv_i,
                          kaco_status[i].pv_p,
                          kaco_status[i].ac_u,
                          kaco_status[i].ac_i,
                          kaco_status[i].ac_p,
                          kaco_status[i].temp,
                          kaco_status[i].total);

  return mqtt_construct_publish_packet(topic, buf, buf_length, false);
}
#endif
void kaco_mqtt_poll_cb(void)
{
#ifdef MQTT_SUPPORT
  for (int8_t i = 0; i < DATA_LOGGER_KACO_MAX; i++)
  {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      if (kaco_status[i].mqtt_pending)
      {
        if (kaco_mqtt_publish(i))
        {
          kaco_status[i].mqtt_pending = 0;
        }
        else
        {
          return; // queue is full
        }
      }
    }
  }
#endif
}

const mqtt_callback_config_t kaco_mqtt_callback_config PROGMEM = {
    .topic = NULL,
    .connack_callback = NULL,
    .poll_callback = kaco_mqtt_poll_cb,
    .close_callback = NULL,
    .publish_callback = NULL};

/*
  -- Ethersex META --
  header(services/user_datalogger/kaco.h)
  init(kaco_init)
#ifdef MQTT_SUPPORT
  mqtt_conf(kaco_mqtt_callback_config)
#endif
*/
