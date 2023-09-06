/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */

/*
 * serialsim - Emulate a serial device in a loopback and/or pipe
 */

/*
 * TTY IOCTLs for controlling the modem control and for error injection.
 * See drivers/tty/serial/serialsim.c and Documentation/serial/serialsim.rst
 * for details.
 */

#ifndef LINUX_SERIALSIM_H
#define LINUX_SERIALSIM_H

#include <linux/ioctl.h>
#include <asm/termbits.h>

#define TIOCSERSREMNULLMODEM	0x54e4
#define TIOCSERSREMMCTRL	0x54e5
#define TIOCSERSREMERR		0x54e6
#ifdef TCGETS2
#define TIOCSERGREMTERMIOS	_IOR('T', 0xe7, struct termios2)
#else
#define TIOCSERGREMTERMIOS	_IOR('T', 0xe7, struct termios)
#endif
#define TIOCSERGREMNULLMODEM	_IOR('T', 0xe8, int)
#define TIOCSERGREMMCTRL	_IOR('T', 0xe9, unsigned int)
#define TIOCSERGREMERR		_IOR('T', 0xea, unsigned int)
#define TIOCSERGREMRS485	_IOR('T', 0xeb, struct serial_rs485)

/* For the dynamic add/remove interface. */
#define SERIALSIM_ALLOC_ID	0x5391
#define SERIALSIM_FREE_ID	0x5392

#endif
