/*
 * Copyright (c) 2009 by Stefan Riepenhausen <rhn@gmx.net>
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

#include <avr/io.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "config.h"
#include "vito.h"
#include "s0logger.h"
#include "kaco.h"
#include "protocols/ecmd/ecmd-base.h"


#ifdef DATA_LOGGER_KACO
int16_t parse_cmd_datalogger_kaco(char *cmd, char *output, uint16_t len) 
{
  return kaco_ecmd_status(cmd, output, len);
}
#endif

#ifdef DATA_LOGGER_VITO
int16_t parse_cmd_datalogger_vito(char *cmd, char *output, uint16_t len) 
{
  return datalogger_vito_ecmd(cmd,output,len);
}
#endif

#ifdef DATA_LOGGER_S0
int16_t parse_cmd_datalogger_s0(char *cmd, char *output, uint16_t len) 
{
  return s0_ecmd_status(cmd, output, len);
}
#endif


/*
-- Ethersex META --
block([[datalogger]])
ecmd_ifdef(DATA_LOGGER_KACO)
ecmd_feature(datalogger_kaco, "kaco",, Manually call application sample commands)
ecmd_endif()

ecmd_ifdef(DATA_LOGGER_S0)
ecmd_feature(datalogger_s0, "s0",, Manually call application sample commands)
ecmd_endif()
*/
