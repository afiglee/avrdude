/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2026 Dmitriy Fitisov
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * avrdude interface for the OddProg 8051-based SPI bridge programmer,
 * https://github.com/afiglee/OddProg
 *
 * OddProg is a transparent SPI bridge intended for Atmel/Microchip
 * AT89LP-family targets (e.g. AT89LP51ED2). The host composes the raw ISP
 * byte streams of the target (preamble AA 55, opcode, address, data; see
 * AT89LP51RD2/ED2/ID2 datasheet Table 23-21) and ships them to OddProg in
 * SLIP-framed packets over a serial line; the firmware clocks them out over
 * SPI and sends back a SLIP-framed response with the status and, for
 * read-direction packets, the bytes captured on MISO.
 *
 * Request packet:  [packet_size, packet_checksum, cmd, data...]
 *   cmd flags: NEW_PACKET, LAST_PACKET, WRITE_DIRECTION, DATASIZE_256,
 *   DATASIZE, OPTIONS, plus 2 bits of (instruction length - 1).
 *   In a NEW packet data[] is: total transfer size (0 = 256), 1-4
 *   instruction bytes, payload. Continuation packets carry raw payload.
 *   packet_size counts everything after the packet_size byte; the checksum
 *   is the two's complement of cmd + sum(data).
 * Response packet: [packet_size, packet_checksum, status, data...]
 *   data echoes the request's data area (with MISO bytes filled in) only
 *   for a successful non-options packet without WRITE_DIRECTION.
 *
 * The firmware work buffer is 64 bytes, so transfers are chunked; slave
 * select is held low from the NEW packet until the LAST packet, framing one
 * target command across several serial packets.
 */

#include <ac_cfg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "oddprog.h"

// SLIP framing (RFC 1055)
#define ODD_SLIP_END            0xC0
#define ODD_SLIP_ESC            0xDB
#define ODD_SLIP_ESC_END        0xDC
#define ODD_SLIP_ESC_ESC        0xDD

// Packet cmd flags, must match OddProg defs.h
#define ODD_CMD_NEW_PACKET      0x80
#define ODD_CMD_LAST_PACKET     0x40
#define ODD_CMD_WRITE_DIRECTION 0x20
#define ODD_CMD_DATASIZE_256    0x10
#define ODD_CMD_DATASIZE        0x08
#define ODD_CMD_OPTIONS         0x04
#define ODD_CMD_INSTR_SIZE_MASK 0x03

#define ODD_OPTION_USE_SS       0x01

#define ODD_ERROR_OK            0x00

#define ODD_BUFFER_SIZE         64 // OddProg work buffer
#define ODD_MAX_DATA            (ODD_BUFFER_SIZE - 3) // Max data bytes in one packet

#define ODD_DEFAULT_BAUD        57600

// AT89LP ISP command preamble and opcodes not taken from the part description
#define LP_PREAMBLE_1           0xAA
#define LP_PREAMBLE_2           0x55
#define LP_PROGRAM_ENABLE       0xAC
#define LP_PROGRAM_ENABLE_ADDR  0x53
#define LP_CHIP_ERASE           0x8A
#define LP_READ_STATUS          0x60
#define LP_STATUS_BUSY          0x01 // Low while the memory is busy programming
#define LP_STATUS_SUCCESS       0x04

#define LP_PAGE_SIZE            128 // Flash/user-signature page (erase unit)
#define LP_HALF_PAGE            64  // Write/read unit and address rollover boundary
#define LP_EEPROM_PAGE          32  // EEPROM page, byte-granular auto-erase

// Sends one packet SLIP-framed
static int oddprog_send_packet(const PROGRAMMER *pgm, const unsigned char *pkt, int len) {
  unsigned char frame[2*ODD_BUFFER_SIZE + 2];
  int n = 0;

  frame[n++] = ODD_SLIP_END;
  for(int k = 0; k < len; k++) {
    if(pkt[k] == ODD_SLIP_END) {
      frame[n++] = ODD_SLIP_ESC;
      frame[n++] = ODD_SLIP_ESC_END;
    } else if(pkt[k] == ODD_SLIP_ESC) {
      frame[n++] = ODD_SLIP_ESC;
      frame[n++] = ODD_SLIP_ESC_ESC;
    } else {
      frame[n++] = pkt[k];
    }
  }
  frame[n++] = ODD_SLIP_END;

  return serial_send(&pgm->fd, frame, n);
}

// Receives one SLIP frame and returns its unescaped length, < 0 on error
static int oddprog_recv_frame(const PROGRAMMER *pgm, unsigned char *buf, int maxlen) {
  unsigned char c;
  int n = 0, in_frame = 0, esc = 0;

  for(;;) {
    if(serial_recv(&pgm->fd, &c, 1) < 0) {
      pmsg_error("programmer did not respond\n");
      return -1;
    }
    if(!in_frame) {
      if(c == ODD_SLIP_END)
        in_frame = 1;
      continue;
    }
    if(c == ODD_SLIP_END) {
      if(n == 0)                // Back-to-back END, still at frame start
        continue;
      return n;
    }
    if(c == ODD_SLIP_ESC) {
      esc = 1;
      continue;
    }
    if(esc) {
      esc = 0;
      if(c == ODD_SLIP_ESC_END)
        c = ODD_SLIP_END;
      else if(c == ODD_SLIP_ESC_ESC)
        c = ODD_SLIP_ESC;
      else {
        pmsg_error("invalid SLIP escape sequence 0x%02x in response\n", c);
        return -1;
      }
    }
    if(n >= maxlen) {
      pmsg_error("response frame too long\n");
      return -1;
    }
    buf[n++] = c;
  }
}

// Receives a response packet, validates it and returns the number of
// echoed data bytes copied to data (0 if data is NULL), < 0 on error
static int oddprog_recv_response(const PROGRAMMER *pgm, unsigned char *data, int maxlen) {
  unsigned char resp[ODD_BUFFER_SIZE + 3];
  int len = oddprog_recv_frame(pgm, resp, sizeof resp);

  if(len < 0)
    return -1;
  if(len < 3 || resp[0] != len - 1) {
    pmsg_error("malformed response packet (%d bytes)\n", len);
    return -1;
  }

  unsigned char sum = 0;

  for(int k = 1; k < len; k++)
    sum += resp[k];
  if(sum) {
    pmsg_error("response packet checksum error\n");
    return -1;
  }
  if(resp[2] != ODD_ERROR_OK) {
    pmsg_error("programmer returned error status 0x%02x\n", resp[2]);
    return -1;
  }

  int datalen = len - 3;

  if(data == NULL)
    return 0;
  if(datalen > maxlen) {
    pmsg_error("unexpected %d data bytes in response (%d expected)\n", datalen, maxlen);
    return -1;
  }
  memcpy(data, resp + 3, datalen);
  return datalen;
}

/*
 * Executes one target ISP command: sends the instr_len (1..4) instruction
 * bytes followed by len data bytes taken from out (0x00 dummies if out is
 * NULL). If in is not NULL the command is a read: the bytes captured on MISO
 * during the data phase are stored to in. The transfer is chunked into as
 * many packets as needed; slave select frames the whole command.
 */
static int oddprog_transfer(const PROGRAMMER *pgm, const unsigned char *instr, int instr_len,
  const unsigned char *out, unsigned char *in, int len) {

  unsigned char pkt[ODD_BUFFER_SIZE];
  unsigned char echo[ODD_MAX_DATA];
  int sent = 0, first = 1;

  if(instr_len < 1 || instr_len > 4 || len < 0 || len > 256) {
    pmsg_error("invalid transfer size (instr %d, data %d)\n", instr_len, len);
    return -1;
  }

  while(first || sent < len) {
    unsigned char cmd = 0;
    int pos = 3;

    if(first) {
      cmd |= ODD_CMD_NEW_PACKET | (instr_len - 1);
      if(len == 256)
        cmd |= ODD_CMD_DATASIZE_256;
      else if(len > 0)
        cmd |= ODD_CMD_DATASIZE;
      pkt[pos++] = len & 0xFF;  // Total transfer size, 0 means 256
      memcpy(pkt + pos, instr, instr_len);
      pos += instr_len;
    }
    if(!in)
      cmd |= ODD_CMD_WRITE_DIRECTION;

    int chunk = ODD_MAX_DATA - (pos - 3);

    if(chunk > len - sent)
      chunk = len - sent;
    for(int k = 0; k < chunk; k++)
      pkt[pos++] = out? out[sent + k]: 0x00;
    if(sent + chunk >= len)
      cmd |= ODD_CMD_LAST_PACKET;

    pkt[0] = pos - 1;
    pkt[2] = cmd;

    unsigned char sum = 0;

    for(int k = 2; k < pos; k++)
      sum += pkt[k];
    pkt[1] = (unsigned char) (0 - sum);

    if(oddprog_send_packet(pgm, pkt, pos) < 0) {
      pmsg_error("unable to send packet\n");
      return -1;
    }

    if(in) {
      // Response data echoes the packet's data area: skip the size and
      // instruction bytes of a NEW packet, the rest are the MISO bytes
      int skip = first? 1 + instr_len: 0;
      int datalen = oddprog_recv_response(pgm, echo, sizeof echo);

      if(datalen < 0)
        return -1;
      if(datalen != skip + chunk) {
        pmsg_error("short read: %d data bytes in response, %d expected\n", datalen, skip + chunk);
        return -1;
      }
      memcpy(in + sent, echo + skip, chunk);
    } else {
      if(oddprog_recv_response(pgm, NULL, 0) < 0)
        return -1;
    }

    sent += chunk;
    first = 0;
  }

  return 0;
}

// Sends an options packet configuring the bridge (slave select handling)
static int oddprog_set_options(const PROGRAMMER *pgm, unsigned char opts) {
  unsigned char pkt[5] = {4, 0, ODD_CMD_OPTIONS, 0, opts};

  pkt[1] = (unsigned char) (0 - (pkt[2] + pkt[3] + pkt[4]));
  if(oddprog_send_packet(pgm, pkt, sizeof pkt) < 0)
    return -1;
  return oddprog_recv_response(pgm, NULL, 0);
}

// Reads the target status register (Table 23-22)
static int oddprog_read_status(const PROGRAMMER *pgm, unsigned char *status) {
  unsigned char instr[4] = {LP_PREAMBLE_1, LP_PREAMBLE_2, LP_READ_STATUS, 0x00};
  unsigned char out[2] = {0x00, 0x00}, in[2];

  if(oddprog_transfer(pgm, instr, 4, out, in, 2) < 0)
    return -1;
  *status = in[1];              // First byte returns during addr low, ignore
  return 0;
}

// Polls the status register until the memory finished programming
static int oddprog_wait_ready(const PROGRAMMER *pgm, unsigned int max_us) {
  unsigned char status;

  for(unsigned int elapsed = 0; ; elapsed += 500) {
    if(oddprog_read_status(pgm, &status) < 0)
      return -1;
    if(status & LP_STATUS_BUSY) { // BUSY is low while busy
      if(!(status & LP_STATUS_SUCCESS))
        pmsg_warning("programming cycle completed without success flag (status 0x%02x)\n", status);
      return 0;
    }
    if(elapsed >= 4*max_us + 100000) {
      pmsg_error("timeout waiting for programming cycle to complete (status 0x%02x)\n", status);
      return -1;
    }
    usleep(500);
  }
}

// Builds opcode + address bytes for a memory operation from the part description
static int oddprog_op_cmd(const AVRMEM *m, int opnum, unsigned long addr, unsigned char *cmd4) {
  OPCODE *op = m->op[opnum];

  if(op == NULL) {
    pmsg_error("%s operation %d undefined in part description\n", m->desc, opnum);
    return -1;
  }
  memset(cmd4, 0, 4);
  avr_set_bits(op, cmd4);
  avr_set_addr(op, cmd4, addr);
  return 0;
}

// The universal 4-byte SPI command, wrapped into the AA 55 preamble.
// The data-out byte of read commands is returned in res[3].
static int oddprog_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  unsigned char instr[4] = {LP_PREAMBLE_1, LP_PREAMBLE_2, cmd[0], cmd[1]};
  unsigned char out[2] = {cmd[2], cmd[3]}, in[2];

  if(oddprog_transfer(pgm, instr, 4, out, in, 2) < 0)
    return -1;

  res[0] = 0x00;
  res[1] = cmd[0];
  res[2] = in[0];
  res[3] = in[1];
  return 0;
}

static int oddprog_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char instr[4] = {LP_PREAMBLE_1, LP_PREAMBLE_2, LP_PROGRAM_ENABLE, LP_PROGRAM_ENABLE_ADDR};

  if(p->op[AVR_OP_PGM_ENABLE]) {
    unsigned char cmd4[4];

    memset(cmd4, 0, 4);
    avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd4);
    instr[2] = cmd4[0];
    instr[3] = cmd4[1];
  }

  return oddprog_transfer(pgm, instr, 4, NULL, NULL, 0);
}

static int oddprog_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char instr[3] = {LP_PREAMBLE_1, LP_PREAMBLE_2, LP_CHIP_ERASE};

  if(p->op[AVR_OP_CHIP_ERASE]) {
    unsigned char cmd4[4];

    memset(cmd4, 0, 4);
    avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd4);
    instr[2] = cmd4[0];
  }

  if(oddprog_transfer(pgm, instr, 3, NULL, NULL, 0) < 0)
    return -1;
  return oddprog_wait_ready(pgm, p->chip_erase_delay? (unsigned int) p->chip_erase_delay: 10000);
}

static int oddprog_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  if(!(p->prog_modes & PM_ISP)) {
    pmsg_error("part %s has no ISP interface\n", p->desc);
    return -1;
  }

  if(oddprog_set_options(pgm, ODD_OPTION_USE_SS) < 0) {
    pmsg_error("unable to configure programmer options; is OddProg connected?\n");
    return -1;
  }

  // Program Enable must be the first target command of a programming session
  if(pgm->program_enable(pgm, p) < 0) {
    pmsg_error("program enable failed; check target connections and reset\n");
    return -1;
  }

  return 0;
}

/*
 * Reads n bytes from a memory into m->buf, chunked so that no target command
 * crosses a half-page (or EEPROM page) boundary: the target's byte address
 * rolls over within the addressed half page (datasheet section 23.6.2).
 */
static int oddprog_read_region(const PROGRAMMER *pgm, const AVRMEM *m,
  unsigned int addr, unsigned int n_bytes) {

  unsigned int unit = mem_is_eeprom(m)? LP_EEPROM_PAGE: LP_HALF_PAGE;
  unsigned int end = addr + n_bytes;

  while(addr < end) {
    unsigned int chunk = unit - (addr % unit);

    if(chunk > end - addr)
      chunk = end - addr;

    unsigned char cmd4[4];

    if(oddprog_op_cmd(m, AVR_OP_READ, addr, cmd4) < 0)
      return -1;

    unsigned char instr[4] = {LP_PREAMBLE_1, LP_PREAMBLE_2, cmd4[0], cmd4[1]};
    unsigned char out[1 + LP_HALF_PAGE], in[1 + LP_HALF_PAGE];

    out[0] = cmd4[2];           // Address low; the byte returned meanwhile is garbage
    memset(out + 1, 0, chunk);
    if(oddprog_transfer(pgm, instr, 4, out, in, chunk + 1) < 0)
      return -1;
    memcpy(m->buf + addr, in + 1, chunk);
    addr += chunk;
  }

  return 0;
}

static int oddprog_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  if(m->op[AVR_OP_READ] == NULL)
    return -1;
  if(oddprog_read_region(pgm, m, addr, n_bytes) < 0)
    return -1;
  return n_bytes;
}

/*
 * Paged write. Flash and user signature pages are 128 bytes but a single
 * write command programs at most one 64-byte half page; the auto-erase
 * variant of the write erases the whole 128-byte page. A page is therefore
 * programmed as write-low-half-with-auto-erase followed by plain
 * write-high-half (datasheet section 23.1). The EEPROM erases and writes
 * with byte granularity within its 32-byte pages, so every EEPROM chunk
 * uses the auto-erase write.
 */
static int oddprog_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  int is_eeprom = mem_is_eeprom(m);
  unsigned int unit = is_eeprom? LP_EEPROM_PAGE: LP_HALF_PAGE;
  unsigned int end = addr + n_bytes;

  if(m->op[AVR_OP_WRITEPAGE] == NULL)
    return -1;

  while(addr < end) {
    unsigned int chunk = unit - (addr % unit);

    if(chunk > end - addr)
      chunk = end - addr;

    unsigned char cmd4[4];

    if(oddprog_op_cmd(m, AVR_OP_WRITEPAGE, addr, cmd4) < 0)
      return -1;

    unsigned char opcode = cmd4[0]; // Write with auto-erase from part description

    // Auto-erase wipes the full 128-byte page: only erase on the first half,
    // program the second half plain (0x70 -> 0x50, 0x72 -> 0x52, Table 23-21)
    if(!is_eeprom && addr % LP_PAGE_SIZE >= LP_HALF_PAGE)
      opcode &= ~0x20;

    unsigned char instr[4] = {LP_PREAMBLE_1, LP_PREAMBLE_2, opcode, cmd4[1]};
    unsigned char out[1 + LP_HALF_PAGE];

    out[0] = cmd4[2];           // Address low
    memcpy(out + 1, m->buf + addr, chunk);
    if(oddprog_transfer(pgm, instr, 4, out, NULL, chunk + 1) < 0)
      return -1;
    if(oddprog_wait_ready(pgm, m->max_write_delay > 0? (unsigned int) m->max_write_delay: 10000) < 0)
      return -1;
    addr += chunk;
  }

  return n_bytes;
}

/*
 * Writing a single fuse requires an auto-erase of the whole fuse row
 * (datasheet section 23.2): read all fuses, modify the requested one and
 * rewrite the row with the Write User Fuses with Auto-Erase command.
 */
static int oddprog_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned long addr, unsigned char value) {

  if(mem_is_a_fuse(m) || mem_is_fuses(m) || str_eq(m->desc, "fuse")) {
    unsigned char fuses[LP_HALF_PAGE];
    int size = m->size > LP_HALF_PAGE? LP_HALF_PAGE: m->size;

    if(m->op[AVR_OP_READ] == NULL || m->op[AVR_OP_WRITE] == NULL)
      return -1;
    if(addr >= (unsigned long) size)
      return -1;

    unsigned char cmd4[4];

    if(oddprog_op_cmd(m, AVR_OP_READ, 0, cmd4) < 0)
      return -1;

    unsigned char instr[4] = {LP_PREAMBLE_1, LP_PREAMBLE_2, cmd4[0], cmd4[1]};
    unsigned char out[1 + LP_HALF_PAGE], in[1 + LP_HALF_PAGE];

    out[0] = cmd4[2];
    memset(out + 1, 0, size);
    if(oddprog_transfer(pgm, instr, 4, out, in, size + 1) < 0)
      return -1;
    if(in[1 + addr] == value)   // Nothing to do
      return 0;
    memcpy(fuses, in + 1, size);
    fuses[addr] = value;

    if(oddprog_op_cmd(m, AVR_OP_WRITE, 0, cmd4) < 0)
      return -1;
    instr[2] = cmd4[0];         // Write User Fuses with Auto-Erase
    instr[3] = cmd4[1];
    out[0] = cmd4[2];
    memcpy(out + 1, fuses, size);
    if(oddprog_transfer(pgm, instr, 4, out, NULL, size + 1) < 0)
      return -1;
    return oddprog_wait_ready(pgm, m->max_write_delay > 0? (unsigned int) m->max_write_delay: 10000);
  }

  return avr_write_byte_default(pgm, p, m, addr, value);
}

static int oddprog_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m) {
  if(m->op[AVR_OP_READ] == NULL)
    return -1;
  if(oddprog_read_region(pgm, m, 0, m->size) < 0)
    return -1;
  return m->size;
}

static int oddprog_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  pgm->port = port;
  pinfo.serialinfo.baud = pgm->baudrate? pgm->baudrate: ODD_DEFAULT_BAUD;
  pinfo.serialinfo.cflags = SERIAL_8N1;
  if(serial_open(port, pinfo, &pgm->fd) < 0)
    return -1;

  (void) serial_drain(&pgm->fd, 0);

  return 0;
}

static void oddprog_close(PROGRAMMER *pgm) {
  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}

static void oddprog_display(const PROGRAMMER *pgm, const char *p) {
  return;
}

static void oddprog_enable(PROGRAMMER *pgm, const AVRPART *p) {
  return;
}

static void oddprog_disable(const PROGRAMMER *pgm) {
  return;
}

const char oddprog_desc[] = "OddProg serial SPI bridge for AT89LP microcontrollers";

void oddprog_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "oddprog");

  // Mandatory functions
  pgm->initialize = oddprog_initialize;
  pgm->display = oddprog_display;
  pgm->enable = oddprog_enable;
  pgm->disable = oddprog_disable;
  pgm->program_enable = oddprog_program_enable;
  pgm->chip_erase = oddprog_chip_erase;
  pgm->cmd = oddprog_cmd;
  pgm->open = oddprog_open;
  pgm->close = oddprog_close;

  // Optional functions
  pgm->paged_write = oddprog_paged_write;
  pgm->paged_load = oddprog_paged_load;
  pgm->read_byte = avr_read_byte_default;
  pgm->write_byte = oddprog_write_byte;
  pgm->read_sig_bytes = oddprog_read_sig_bytes;
}
