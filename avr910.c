/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003  Theodore A. Roth  <troth@openavr.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id$ */

/*
 * avrdude interface for Atmel Low Cost Serial programmers which adher to the
 * protocol described in application note avr910.
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "avr.h"
#include "pgm.h"
#include "avr910.h"
#include "serial.h"

extern char * progname;
extern int do_cycles;

#define show_func_info() \
  fprintf(stderr, "%s: %d: called %s()\n", __FILE__, __LINE__, __FUNCTION__)



static int avr910_send(PROGRAMMER * pgm, char * buf, size_t len)
{
  show_func_info();

  return serial_send(pgm->fd, buf, len);
}


static int avr910_recv(PROGRAMMER * pgm, char * buf, size_t len)
{
  show_func_info();

  return serial_recv(pgm->fd, buf, len);
}


static int avr910_drain(PROGRAMMER * pgm, int display)
{
  show_func_info();

  /* Do nothing. */

  return serial_drain(pgm->fd, display);
}


static int avr910_rdy_led(PROGRAMMER * pgm, int value)
{
  show_func_info();

  /* Do nothing. */

  return 0;
}


static int avr910_err_led(PROGRAMMER * pgm, int value)
{
  show_func_info();

  /* Do nothing. */

  return 0;
}


static int avr910_pgm_led(PROGRAMMER * pgm, int value)
{
  show_func_info();

  /* Do nothing. */

  return 0;
}


static int avr910_vfy_led(PROGRAMMER * pgm, int value)
{
  show_func_info();

  /* Do nothing. */

  return 0;
}


/*
 * issue the 'chip erase' command to the AVR device
 */
static int avr910_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  show_func_info();

  return 0;
}

/*
 * issue the 'program enable' command to the AVR device
 */
static int avr910_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  show_func_info();

  return -1;
}


/*
 * apply power to the AVR processor
 */
static void avr910_powerup(PROGRAMMER * pgm)
{
  show_func_info();

  /* Do nothing. */

  return;
}


/*
 * remove power from the AVR processor
 */
static void avr910_powerdown(PROGRAMMER * pgm)
{
  show_func_info();

  /* Do nothing. */

  return;
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
static int avr910_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  char pn[8];
  char sw[2];
  char hw[2];
  unsigned char c;

  show_func_info();

  /* Programmer returns exactly 7 chars _without_ the null.*/

  avr910_send(pgm, "S", 1);
  memset (pn, 0, sizeof(pn));
  avr910_recv(pgm, pn, sizeof(pn)-1);

  /* Get the HW and SW versions to see if the programmer is present. */

  avr910_send(pgm, "V", 1);
  avr910_recv(pgm, sw, sizeof(sw));

  avr910_send(pgm, "v", 1);
  avr910_recv(pgm, hw, sizeof(hw));

  fprintf(stderr, "Found programmer: %s\n", pn);
  fprintf(stderr, "    Software Version = %c.%c; "
          "Hardware Version = %c.%c\n", sw[0], sw[1], hw[0], hw[1]);

  /* Get list of devices that the programmer supports. */

  avr910_send(pgm, "t", 1);
  while (1) {
    avr910_recv(pgm, &c, 1);
    if (c == 0)
      break;
    fprintf(stderr, "Device code: 0x%02x\n", c);
  };

  return 0;
}


static int avr910_save(PROGRAMMER * pgm)
{
  show_func_info();

  /* Do nothing. */

  return 0;
}


static void avr910_restore(PROGRAMMER * pgm)
{
  show_func_info();

  /* Do nothing. */

  return;
}


static void avr910_disable(PROGRAMMER * pgm)
{
  show_func_info();

  return;
}


static void avr910_enable(PROGRAMMER * pgm)
{
  show_func_info();

  /* Do nothing. */

  return;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
static int avr910_cmd(PROGRAMMER * pgm, unsigned char cmd[4], 
                      unsigned char res[4])
{
  show_func_info();

  return 0;
}


static void avr910_open(PROGRAMMER * pgm, char * port)
{
  show_func_info();

  strcpy(pgm->port, port);
  pgm->fd = serial_open(port, 19200);

  /*
   * drain any extraneous input
   */
  avr910_drain (pgm, 0);
}

static void avr910_close(PROGRAMMER * pgm)
{
  show_func_info();

  serial_close(pgm->fd);
  pgm->fd = -1;
}


static void avr910_display(PROGRAMMER * pgm, char * p)
{
  show_func_info();

  return;
}


void avr910_initpgm(PROGRAMMER * pgm)
{
  show_func_info();

  strcpy(pgm->type, "avr910");

  /*
   * mandatory functions
   */
  pgm->rdy_led        = avr910_rdy_led;
  pgm->err_led        = avr910_err_led;
  pgm->pgm_led        = avr910_pgm_led;
  pgm->vfy_led        = avr910_vfy_led;
  pgm->initialize     = avr910_initialize;
  pgm->display        = avr910_display;
  pgm->save           = avr910_save;
  pgm->restore        = avr910_restore;
  pgm->enable         = avr910_enable;
  pgm->disable        = avr910_disable;
  pgm->powerup        = avr910_powerup;
  pgm->powerdown      = avr910_powerdown;
  pgm->program_enable = avr910_program_enable;
  pgm->chip_erase     = avr910_chip_erase;
  pgm->cmd            = avr910_cmd;
  pgm->open           = avr910_open;
  pgm->close          = avr910_close;

  /*
   * optional functions
   */
}