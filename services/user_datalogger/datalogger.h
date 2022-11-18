/*
 * Copyright (c) 2011 Roland Praml pram@gmx.de
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

#ifndef HAVE_DATALOGGER_H
#define HAVE_DATALOGGER_H


#ifdef DEBUG_KACO
#include "core/debug.h"
#define KACO_DEBUG(str...) debug_printf ("KACO: " str)
#else
#define KACO_DEBUG(...)    ((void) 0)
#endif

#ifdef DEBUG_LOGGER
#include "core/debug.h"
#define LOGGER_DEBUG(str...) debug_printf ("LOGGER: " str)
#else
#define LOGGER_DEBUG(...)    ((void) 0)
#endif

int16_t
datalogger_status(char *cmd, char *output, uint16_t len);


void
datalogger_periodic(void);

void
datalogger_timeout(void);

int16_t
datalogger_mainloop(void);


#define CONTINUE 0
#define FINISH_OK -1
#define FINISH_ERR -2

#include "config.h"
#ifdef DEBUG_DATA_LOGGER
# include "core/debug.h"
# define DATALOGGERDEBUG(a...)  debug_printf("data logger: " a)
#else
# define DATALOGGERDEBUG(a...)
#endif

#endif  /* HAVE_DATALOGGER_H */
