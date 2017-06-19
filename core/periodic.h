/*
 *
 * (c) by Alexander Neumann <alexander@bumpern.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License (either version 2 or
 * version 3) as published by the Free Software Foundation.
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

#ifndef _PERIODIC_H
#define _PERIODIC_H

#include <avr/io.h>

/* initialize hardware timer */
void periodic_init(void);

#define MAX_OVERFLOW          65535UL /* timer1 max value */
#define HZ                    50 /* 20ms */
/*    #if (F_CPU/256) < MAX_OVERFLOW
#define CLOCK_PRESCALER       256UL
#define CLOCK_PRESCALER_MASK  _BV(CS12)
#else */
#define CLOCK_PRESCALER       64UL
#define CLOCK_PRESCALER_MASK  _BV(CS11)|_BV(CS10)
/*#endif */

#define CLOCK_SECONDS F_CPU/CLOCK_PRESCALER
#define CLOCK_TICKS F_CPU/CLOCK_PRESCALER/HZ

#endif /* _PERIODIC_H */
