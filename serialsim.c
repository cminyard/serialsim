// SPDX-License-Identifier: GPL-2.0+

/*
 * serialsim - Emulate a serial device in a loopback and/or pipe
 *
 * See the Documentation/serial/serialsim.rts for more details.
 *
 * Copyright 2019, Corey Minyard <minyard@acm.org>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/device.h>
#include <linux/serial_core.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <linux/serialsim.h>

#ifndef TCGETS2
#error "This module is only supported with TCGETS2"
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,1,0)
/* This went away in 6.1. */
static inline int kernel_termios_to_user_termios(struct termios2 __user *u,
                                                 struct ktermios *k)
{
        return copy_to_user(u, k, sizeof(struct termios2));
}

/* New in 6.1 */
#define CONST_KTERMIOS const
#else
#define CONST_KTERMIOS
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,0,0)
/* New in 6.0. */
#define RS485_HAS_TERMIOS struct ktermios *termios,

/* Also new in 6.0. */
#define HAS_RS485_SUPPORTED
#else
#define RS485_HAS_TERMIOS
#endif

/*
 * Port types for the various interfaces.
 */
#define PORT_SERIALECHO		72549
#define PORT_SERIALPIPEA	72550
#define PORT_SERIALPIPEB	72551

#ifdef CONFIG_HIGH_RES_TIMERS
#define SERIALSIM_WAKES_PER_SEC	1000
#else
#define SERIALSIM_WAKES_PER_SEC	HZ
#endif

#define SERIALSIM_XBUFSIZE	32

/* For things to send on the line, in flags field. */
#define DO_FRAME_ERR		(1 << TTY_FRAME)
#define DO_PARITY_ERR		(1 << TTY_PARITY)
#define DO_OVERRUN_ERR		(1 << TTY_OVERRUN)
#define DO_BREAK		(1 << TTY_BREAK)
#define FLAGS_MASK (DO_FRAME_ERR | DO_PARITY_ERR | DO_OVERRUN_ERR | DO_BREAK)

struct serialsim_intf {
	struct uart_port port;

	/* We need a device for the interface, just use a platform device. */
	struct platform_device pdev;

	/* idr we are stored in, or NULL if not in an idr (pipeb). */
	struct idr *idr;

	/* The driver we are registered against, NULL if none. */
	struct uart_driver *driver;

	/*
	 * This is my transmit buffer, my thread picks this up and
	 * injects them into the other port's uart.
	 */
	unsigned char xmitbuf[SERIALSIM_XBUFSIZE];
	struct circ_buf buf;

	/* Error flags to send. */
	bool break_reported;
	unsigned int flags;

	/* Modem state. */
	unsigned int mctrl;
	bool do_null_modem;
	spinlock_t mctrl_lock;
	struct tasklet_struct mctrl_tasklet;

	/*
	 * The modem control state handling is not perfect.
	 * Currently, if data is being transferred in the thread, it
	 * will delay the modemcontrol until all the data is
	 * transferred.  Otherwise it will deliver the data
	 * immediately in the tasklet.
	 *
	 * The problem is that there is a race between setting the
	 * mctrl and transferring the data.  When the last byte is
	 * read from the origin serial port on a close, the modem
	 * control lines are set immediately by close code.  But you
	 * want the data being transferred into the receive buffer
	 * before you issue the modem control to the destination port.
	 *
	 * The current scheme doesn't handle multiple modem control
	 * changes, and the modem control changes should really ride
	 * with the data to be after specific bytes.  But the current
	 * scheme works ok.
	 */

	/* Modem state to send to the other side. */
	bool mctrl_pending;
	bool hold_mctrl;
	unsigned int new_mctrl;

	/* My transmitter is enabled. */
	bool tx_enabled;

	/* I can receive characters. */
	bool rx_enabled;

	/*
	 * The serial echo port on the other side of this pipe (or points
	 * to myself in loopback mode.
	 */
	struct serialsim_intf *ointf;

	unsigned int div;
	unsigned int bytes_per_interval;
	unsigned int per_interval_residual;

	struct ktermios termios;

	const char *threadname;
	struct task_struct *thread;

	struct serial_rs485 rs485;
};

#define circ_sbuf_space(buf) CIRC_SPACE((buf)->head, (buf)->tail, \
					SERIALSIM_XBUFSIZE)
#define circ_sbuf_empty(buf) ((buf)->head == (buf)->tail)
#define circ_sbuf_next(idx) (((idx) + 1) % SERIALSIM_XBUFSIZE)

static struct serialsim_intf *serialsim_port_to_intf(struct uart_port *port)
{
	return container_of(port, struct serialsim_intf, port);
}

static unsigned int serialsim_tx_empty(struct uart_port *port)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);

	if (circ_sbuf_empty(&intf->buf))
		return TIOCSER_TEMT;
	return 0;
}

/*
 * We have to lock multiple locks, make sure to do it in the same order all
 * the time.
 */
static void serialsim_null_modem_lock_irq(struct serialsim_intf *intf)
	__acquires(intf->port.lock) __acquires(intf->mctrl_lock)
	__acquires(intf->ointf->mctrl_lock)
{
	spin_lock_irq(&intf->port.lock);
	if (intf == intf->ointf) {
		spin_lock(&intf->mctrl_lock);
	} else if (intf < intf->ointf) {
		spin_lock(&intf->mctrl_lock);
		spin_lock(&intf->ointf->mctrl_lock);
	} else {
		spin_lock(&intf->ointf->mctrl_lock);
		spin_lock(&intf->mctrl_lock);
	}
}

static void serialsim_null_modem_unlock_irq(struct serialsim_intf *intf)
	__releases(intf->port.lock) __releases(intf->mctrl_lock)
	__releases(intf->ointf->mctrl_lock)
{
	if (intf == intf->ointf) {
		spin_unlock(&intf->mctrl_lock);
	} else {
		/* Order doesn't matter here. */
		spin_unlock(&intf->mctrl_lock);
		spin_unlock(&intf->ointf->mctrl_lock);
	}
	spin_unlock_irq(&intf->port.lock);
}

/*
 * This must be called holding intf->port.lock and intf->mctrl_lock.
 */
static void _serialsim_set_modem_lines(struct serialsim_intf *intf,
					unsigned int mask,
					unsigned int new_mctrl)
{
	unsigned int changes;
	unsigned int mctrl = (intf->mctrl & ~mask) | (new_mctrl & mask);

	if (mctrl == intf->mctrl)
		return;

	if (!intf->rx_enabled) {
		intf->mctrl = mctrl;
		return;
	}

	changes = mctrl ^ intf->mctrl;
	intf->mctrl = mctrl;
	if (changes & TIOCM_CAR)
		uart_handle_dcd_change(&intf->port, mctrl & TIOCM_CAR);
	if (changes & TIOCM_CTS)
		uart_handle_cts_change(&intf->port, mctrl & TIOCM_CTS);
	if (changes & TIOCM_RNG)
		intf->port.icount.rng++;
	if (changes & TIOCM_DSR)
		intf->port.icount.dsr++;
}

#define NULL_MODEM_MCTRL (TIOCM_CAR | TIOCM_CTS | TIOCM_DSR)
#define LOCAL_MCTRL (NULL_MODEM_MCTRL | TIOCM_RNG)

/*
 * Must be called holding intf->port.lock, intf->mctrl_lock, and
 * intf->ointf.mctrl_lock.
 */
static void serialsim_handle_null_modem_update(struct serialsim_intf *intf)
{
	unsigned int mctrl = 0;

	/* Pull the values from the remote side for myself. */
	if (intf->ointf->mctrl & TIOCM_DTR)
		mctrl |= TIOCM_CAR | TIOCM_DSR;
	if (intf->ointf->mctrl & TIOCM_RTS)
		mctrl |= TIOCM_CTS;

	_serialsim_set_modem_lines(intf, NULL_MODEM_MCTRL, mctrl);
}

static void serialsim_set_null_modem(struct serialsim_intf *intf, bool val)
{
	serialsim_null_modem_lock_irq(intf);

	if (!!val == !!intf->do_null_modem)
		goto out_unlock;

	if (!val) {
		intf->do_null_modem = false;
		goto out_unlock;
	}

	/* Enabling NULL modem. */
	intf->do_null_modem = true;

	serialsim_handle_null_modem_update(intf);

out_unlock:
	serialsim_null_modem_unlock_irq(intf);
}

static void serialsim_set_modem_lines(struct serialsim_intf *intf,
				      unsigned int mask,
				      unsigned int new_mctrl)
{
	mask &= LOCAL_MCTRL;

	spin_lock_irq(&intf->port.lock);
	spin_lock(&intf->mctrl_lock);

	if (intf->do_null_modem)
		mask &= ~NULL_MODEM_MCTRL;

	_serialsim_set_modem_lines(intf, mask, new_mctrl);

	spin_unlock(&intf->mctrl_lock);
	spin_unlock_irq(&intf->port.lock);
}

static void mctrl_tasklet(unsigned long data)
{
	struct serialsim_intf *intf = (void *) data;

	serialsim_null_modem_lock_irq(intf);
	if (intf->ointf->do_null_modem)
		serialsim_handle_null_modem_update(intf);
	serialsim_null_modem_unlock_irq(intf);
}

#define SETTABLE_MCTRL (TIOCM_RTS | TIOCM_DTR)

static void serialsim_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	bool do_tasklet = false;

	spin_lock(&intf->mctrl_lock);
	if (circ_sbuf_empty(&intf->buf) && !intf->mctrl_pending &&
	    !intf->hold_mctrl) {
		intf->mctrl &= ~SETTABLE_MCTRL;
		intf->mctrl |= mctrl & SETTABLE_MCTRL;
		do_tasklet = true;
	} else {
		intf->mctrl_pending = true;
		intf->new_mctrl = mctrl;
	}
	spin_unlock(&intf->mctrl_lock);

	/*
	 * We are called holding port->lock, but we must be able to claim
	 * intf->ointf->port.lock, and that can result in deadlock.  So
	 * we have to run this elsewhere.  Note that we run the other
	 * end's tasklet.
	 */
	if (do_tasklet)
		tasklet_schedule(&intf->ointf->mctrl_tasklet);
}

static unsigned int serialsim_get_mctrl(struct uart_port *port)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	unsigned int rv;

	spin_lock(&intf->mctrl_lock);
	rv = intf->mctrl;
	spin_unlock(&intf->mctrl_lock);

	return rv;
}

static void serialsim_stop_tx(struct uart_port *port)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);

	intf->tx_enabled = false;
}

static void serialsim_set_baud_rate(struct serialsim_intf *intf,
				     unsigned int baud, unsigned int cflag)
{
	unsigned int bits_per_char;

	switch (cflag & CSIZE) {
	case CS5:
		bits_per_char = 7;
		break;
	case CS6:
		bits_per_char = 8;
		break;
	case CS7:
		bits_per_char = 9;
		break;
	default:
		bits_per_char = 10;
		break; /* CS8 and others. */
	}
	if (cflag & CSTOPB)
		bits_per_char++;

	intf->div = SERIALSIM_WAKES_PER_SEC * bits_per_char;
	intf->bytes_per_interval = baud / intf->div;
	intf->per_interval_residual = baud % intf->div;
}

static void serialsim_transfer_data(struct uart_port *port,
				    struct circ_buf *tbuf)
{
	struct circ_buf *cbuf = &port->state->xmit;

	while (!uart_circ_empty(cbuf) && circ_sbuf_space(tbuf)) {
		unsigned char c = cbuf->buf[cbuf->tail];

		cbuf->tail = (cbuf->tail + 1) % UART_XMIT_SIZE;
		tbuf->buf[tbuf->head] = c;
		tbuf->head = circ_sbuf_next(tbuf->head);
		port->icount.tx++;
	}
	if (uart_circ_chars_pending(cbuf) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static unsigned int serialsim_get_flag(struct serialsim_intf *intf,
				       unsigned int *status)
{
	unsigned int flags = intf->flags;

	*status = flags;

	/* Overrun is always reported through a different way. */
	if (flags & DO_OVERRUN_ERR) {
		intf->port.icount.overrun++;
		intf->flags &= ~DO_OVERRUN_ERR;
	}

	/* Break is handled separately. */
	if (flags & DO_FRAME_ERR) {
		intf->port.icount.frame++;
		intf->flags &= ~DO_FRAME_ERR;
		return TTY_FRAME;
	}
	if (flags & DO_PARITY_ERR) {
		intf->port.icount.parity++;
		intf->flags &= ~DO_PARITY_ERR;
		return TTY_PARITY;
	}

	return TTY_NORMAL;
}

static void serialsim_set_flags(struct serialsim_intf *intf,
				 unsigned int status)
{
	spin_lock_irq(&intf->port.lock);
	intf->flags |= status;
	spin_unlock_irq(&intf->port.lock);
}

static void serialsim_thread_delay(void)
{
#ifdef CONFIG_HIGH_RES_TIMERS
	ktime_t timeout;

	timeout = ns_to_ktime(1000000000 / SERIALSIM_WAKES_PER_SEC);
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_hrtimeout(&timeout, HRTIMER_MODE_REL);
#else
	schedule_timeout_interruptible(1);
#endif
}

static int serialsim_thread(void *data)
{
	struct serialsim_intf *intf = data;
	struct serialsim_intf *ointf = intf->ointf;
	struct uart_port *port = &intf->port;
	struct uart_port *oport = &ointf->port;
	struct circ_buf *tbuf = &intf->buf;
	unsigned int residual = 0;

	while (!kthread_should_stop()) {
		unsigned int to_send, i;
		unsigned int div;
		unsigned int flag;
		unsigned int status = 0;
		bool mctrl_pending;
		unsigned int new_mctrl;

		spin_lock_irq(&intf->mctrl_lock);
		intf->hold_mctrl = true;
		spin_unlock_irq(&intf->mctrl_lock);

		spin_lock_irq(&oport->lock);
		if (ointf->tx_enabled && oport->state->xmit.buf)
			/*
			 * Move bytes from the other port's transmit buffer to
			 * the interface buffer.
			 */
			serialsim_transfer_data(oport, tbuf);
		spin_unlock_irq(&oport->lock);

		/*
		 *  Calculate how many bytes to send based on the
		 *  simulated serial speed.
		 */
		to_send = intf->bytes_per_interval;
		residual += intf->per_interval_residual;
		div = intf->div;
		if (residual >= div) {
			residual -= div;
			to_send++;
		}

		/*
		 * Move from the interface buffer into my receive
		 * buffer.
		 */
		spin_lock_irq(&port->lock);
		if (intf->rx_enabled) {
			if (intf->flags & DO_BREAK && !intf->break_reported) {
				intf->port.icount.brk++;
				intf->break_reported = true;
				uart_insert_char(port, intf->flags,
						 DO_OVERRUN_ERR, 0, TTY_BREAK);
				if (to_send == 0) {
					to_send = 1; /* Force a push. */
					goto skip_send;
				}
			}
			for (i = 0; i < to_send && !circ_sbuf_empty(tbuf);
			     i++) {
				unsigned char c = tbuf->buf[tbuf->tail];

				tbuf->tail = circ_sbuf_next(tbuf->tail);
				flag = serialsim_get_flag(intf, &status);
				port->icount.rx++;
				uart_insert_char(port, status,
						 DO_OVERRUN_ERR,
						 c, flag);
			}
		}
	skip_send:
		spin_unlock_irq(&port->lock);

		if (to_send)
			tty_flip_buffer_push(&port->state->port);

		spin_lock_irq(&intf->mctrl_lock);
		intf->hold_mctrl = false;
		mctrl_pending = intf->mctrl_pending;
		new_mctrl = intf->new_mctrl;
		intf->mctrl_pending = false;
		if (mctrl_pending) {
			intf->mctrl &= ~SETTABLE_MCTRL;
			intf->mctrl |= new_mctrl & SETTABLE_MCTRL;
		}
		spin_unlock_irq(&intf->mctrl_lock);
		if (mctrl_pending)
			mctrl_tasklet((unsigned long) ointf);

		serialsim_thread_delay();
	}

	return 0;
}

static void serialsim_start_tx(struct uart_port *port)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);

	intf->tx_enabled = true;
}

static void serialsim_stop_rx(struct uart_port *port)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);

	intf->rx_enabled = false;
}

static void serialsim_break_ctl(struct uart_port *port, int break_state)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	struct serialsim_intf *ointf = intf->ointf;

	spin_lock_irq(&ointf->port.lock);
	if (!break_state && ointf->flags & DO_BREAK) {
		/* Turning break off. */
		ointf->break_reported = false;
		ointf->flags &= ~DO_BREAK;
	} else if (break_state) {
		ointf->flags |= DO_BREAK;
	}
	spin_unlock_irq(&ointf->port.lock);
}

static int serialsim_startup(struct uart_port *port)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	int rv = 0;
	struct circ_buf *cbuf = &port->state->xmit;
	unsigned long flags;

	/*
	 * This is technically wrong, but otherwise tests using it
	 * can get stale data.
	 */
	spin_lock_irqsave(&port->lock, flags);
	cbuf->head = cbuf->tail = 0;
	spin_unlock_irqrestore(&port->lock, flags);

	intf->buf.head = intf->buf.tail = 0;
	intf->thread = kthread_run(serialsim_thread, intf,
				   "%s%d", intf->threadname, port->line);
	if (IS_ERR(intf->thread)) {
		rv = PTR_ERR(intf->thread);
		intf->thread = NULL;
		pr_err("serialsim: Could not start thread: %d", rv);
	} else {
		spin_lock_irqsave(&port->lock, flags);
		intf->tx_enabled = true;
		intf->rx_enabled = true;

		serialsim_set_baud_rate(intf, 9600, CS8);
		spin_unlock_irqrestore(&port->lock, flags);
	}

	return rv;
}

static void serialsim_shutdown(struct uart_port *port)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags);
	kthread_stop(intf->thread);
}

static void serialsim_release_port(struct uart_port *port)
{
}

static void
serialsim_set_termios(struct uart_port *port, struct ktermios *termios,
		      CONST_KTERMIOS struct ktermios *old)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	unsigned int baud = uart_get_baud_rate(port, termios, old,
					       10, 100000000);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	serialsim_set_baud_rate(intf, baud, termios->c_cflag);
	intf->termios = *termios;
	spin_unlock_irqrestore(&port->lock, flags);
}

static int serialsim_rs485(struct uart_port *port, RS485_HAS_TERMIOS
			   struct serial_rs485 *newrs485)
{
	struct serialsim_intf *intf = serialsim_port_to_intf(port);

	intf->rs485 = *newrs485;
	return 0;
}

static const char *serialecho_type(struct uart_port *port)
{
	return "SerialEcho";
}

static const char *serialpipea_type(struct uart_port *port)
{
	return "SerialPipeA";
}

static const char *serialpipeb_type(struct uart_port *port)
{
	return "SerialPipeB";
}

static void serialecho_config_port(struct uart_port *port, int type)
{
	port->type = PORT_SERIALECHO;
}

static void serialpipea_config_port(struct uart_port *port, int type)
{
	port->type = PORT_SERIALPIPEA;
}

static void serialpipeb_config_port(struct uart_port *port, int type)
{
	port->type = PORT_SERIALPIPEB;
}

static int serialpipe_ioctl(struct uart_port *port, unsigned int cmd,
			    unsigned long arg)
{
	int rv = 0;
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	unsigned int mask;
	int val;

	switch (cmd) {
	case TIOCSERSREMNULLMODEM:
		serialsim_set_null_modem(intf->ointf, !!arg);
		break;

	case TIOCSERGREMNULLMODEM:
		val = intf->ointf->do_null_modem;
		if (copy_to_user((int __user *) arg, &val, sizeof(int)))
			rv = -EFAULT;
		break;

	case TIOCSERSREMMCTRL:
		mask = (arg >> 16) & 0xffff;
		arg &= 0xffff;
		if (mask & ~LOCAL_MCTRL || arg & ~LOCAL_MCTRL)
			rv = -EINVAL;
		else
			serialsim_set_modem_lines(intf->ointf, mask, arg);
		break;

	case TIOCSERGREMMCTRL:
		if (copy_to_user((unsigned int __user *) arg,
				 &intf->ointf->mctrl,
				 sizeof(unsigned int)))
			rv = -EFAULT;
		break;

	case TIOCSERSREMERR:
		if (arg & ~FLAGS_MASK)
			rv = -EINVAL;
		else
			serialsim_set_flags(intf, arg);
		break;

	case TIOCSERGREMERR:
		if (copy_to_user((unsigned int __user *) arg, &intf->flags,
				 sizeof(unsigned int)))
			rv = -EFAULT;
		break;

	case TIOCSERGREMTERMIOS:
	{
		struct ktermios otermios;

		spin_lock_irq(&intf->ointf->port.lock);
		otermios = intf->ointf->termios;
		spin_unlock_irq(&intf->ointf->port.lock);
#ifdef TCGETS2
		rv = kernel_termios_to_user_termios((struct termios2 __user *)
						    arg,
						    &otermios);
#else
		rv = kernel_termios_to_user_termios((struct termios __user *)
						    arg,
						    &otermios);
#endif
		if (rv)
			rv = -EFAULT;
		break;
	}

	case TIOCSERGREMRS485:
	{
		struct serial_rs485 ors485;

		spin_lock_irq(&intf->ointf->port.lock);
		ors485 = intf->ointf->rs485;
		spin_unlock_irq(&intf->ointf->port.lock);

		if (copy_to_user((struct serial_rs485 __user *) arg,
				 &ors485, sizeof(ors485)))
			rv = -EFAULT;
		break;
	}

	default:
		rv = -ENOIOCTLCMD;
	}

	return rv;
}

static unsigned int nr_echo_ports = 4;
module_param(nr_echo_ports, uint, 0444);
MODULE_PARM_DESC(nr_echo_ports,
		 "The number of echo ports to create.  Defaults to 4");

static unsigned int nr_dyn_echo_ports = 16;
module_param(nr_dyn_echo_ports, uint, 0444);
MODULE_PARM_DESC(nr_dyn_echo_ports,
		 "Max dynamic echo ports available.  Defaults to 16");

static unsigned int nr_pipe_ports = 4;
module_param(nr_pipe_ports, uint, 0444);
MODULE_PARM_DESC(nr_pipe_ports,
		 "The number of pipe ports to create.  Defaults to 4");

static unsigned int nr_dyn_pipe_ports = 16;
module_param(nr_dyn_pipe_ports, uint, 0444);
MODULE_PARM_DESC(nr_dyn_pipe_ports,
		 "Max dynamic pipe ports available.  Defaults to 16");

static char *gettok(char **s)
{
	char *t = skip_spaces(*s);
	char *p = t;

	while (*p && !isspace(*p))
		p++;
	if (*p)
		*p++ = '\0';
	*s = p;

	return t;
}

static bool tokeq(const char *t, const char *m)
{
	return strcmp(t, m) == 0;
}

static unsigned int parse_modem_line(char op, unsigned int flag,
				     unsigned int *mctrl)
{
	if (op == '+')
		*mctrl |= flag;
	else
		*mctrl &= ~flag;
	return flag;
}

static void serialsim_ctrl_append(char **val, int *left, char *n, bool enabled)
{
	int count;

	count = snprintf(*val, *left, " %c%s", enabled ? '+' : '-', n);
	*left -= count;
	*val += count;
}

static ssize_t serialsim_ctrl_read(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct tty_port *tport = dev_get_drvdata(dev);
	struct uart_state *state = container_of(tport, struct uart_state, port);
	struct uart_port *port = state->uart_port;
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	unsigned int mctrl = intf->mctrl;
	char *val = buf;
	int left = PAGE_SIZE;

	serialsim_ctrl_append(&val, &left, "nullmodem", intf->do_null_modem);
	serialsim_ctrl_append(&val, &left, "cd", mctrl & TIOCM_CAR);
	serialsim_ctrl_append(&val, &left, "dsr", mctrl & TIOCM_DSR);
	serialsim_ctrl_append(&val, &left, "cts", mctrl & TIOCM_CTS);
	serialsim_ctrl_append(&val, &left, "ring", mctrl & TIOCM_RNG);
	serialsim_ctrl_append(&val, &left, "dtr", mctrl & TIOCM_DTR);
	serialsim_ctrl_append(&val, &left, "rts", mctrl & TIOCM_RTS);
	*val++ = '\n';

	return val - buf;
}

static ssize_t serialsim_ctrl_write(struct device *dev,
				    struct device_attribute *attr,
				    const char *val, size_t count)
{
	struct tty_port *tport = dev_get_drvdata(dev);
	struct uart_state *state = container_of(tport, struct uart_state, port);
	struct uart_port *port = state->uart_port;
	struct serialsim_intf *intf = serialsim_port_to_intf(port);
	char *str = kstrndup(val, count, GFP_KERNEL);
	char *p, *s = str;
	int rv = count;
	unsigned int flags = 0;
	unsigned int nullmodem = 0;
	unsigned int mctrl_mask = 0, mctrl = 0;

	if (!str)
		return -ENOMEM;

	p = gettok(&s);
	while (*p) {
		char op = '\0';
		int err = 0;

		switch (*p) {
		case '+':
		case '-':
			op = *p++;
			break;
		default:
			break;
		}

		if (tokeq(p, "frame"))
			flags |= DO_FRAME_ERR;
		else if (tokeq(p, "parity"))
			flags |= DO_PARITY_ERR;
		else if (tokeq(p, "overrun"))
			flags |= DO_OVERRUN_ERR;
		else if (tokeq(p, "nullmodem"))
			nullmodem = op;
		else if (tokeq(p, "dsr"))
			mctrl_mask |= parse_modem_line(op, TIOCM_DSR, &mctrl);
		else if (tokeq(p, "cts"))
			mctrl_mask |= parse_modem_line(op, TIOCM_CTS, &mctrl);
		else if (tokeq(p, "cd"))
			mctrl_mask |= parse_modem_line(op, TIOCM_CAR, &mctrl);
		else if (tokeq(p, "ring"))
			mctrl_mask |= parse_modem_line(op, TIOCM_RNG, &mctrl);
		else
			err = -EINVAL;

		if (err) {
			rv = err;
			goto out;
		}
		p = gettok(&s);
	}

	if (flags)
		serialsim_set_flags(intf, flags);
	if (nullmodem)
		serialsim_set_null_modem(intf, nullmodem == '+');
	if (mctrl_mask)
		serialsim_set_modem_lines(intf, mctrl_mask, mctrl);

out:
	kfree(str);

	return rv;
}

static DEVICE_ATTR(ctrl, 0660, serialsim_ctrl_read, serialsim_ctrl_write);

static struct attribute *serialsim_dev_attrs[] = {
	&dev_attr_ctrl.attr,
	NULL,
};

/*
 * We pass this into the uart_port and it does the register.  It's
 * not const in uart_port, so we can't make it const here.
 */
static struct attribute_group serialsim_dev_attr_group = {
	.attrs = serialsim_dev_attrs,
};

static const struct uart_ops serialecho_ops = {
	.tx_empty =		serialsim_tx_empty,
	.set_mctrl =		serialsim_set_mctrl,
	.get_mctrl =		serialsim_get_mctrl,
	.stop_tx =		serialsim_stop_tx,
	.start_tx =		serialsim_start_tx,
	.stop_rx =		serialsim_stop_rx,
	.break_ctl =		serialsim_break_ctl,
	.startup =		serialsim_startup,
	.shutdown =		serialsim_shutdown,
	.release_port =		serialsim_release_port,
	.set_termios =		serialsim_set_termios,
	.type =			serialecho_type,
	.config_port =		serialecho_config_port
};

static const struct uart_ops serialpipea_ops = {
	.tx_empty =		serialsim_tx_empty,
	.set_mctrl =		serialsim_set_mctrl,
	.get_mctrl =		serialsim_get_mctrl,
	.stop_tx =		serialsim_stop_tx,
	.start_tx =		serialsim_start_tx,
	.stop_rx =		serialsim_stop_rx,
	.break_ctl =		serialsim_break_ctl,
	.startup =		serialsim_startup,
	.shutdown =		serialsim_shutdown,
	.release_port =		serialsim_release_port,
	.set_termios =		serialsim_set_termios,
	.type =			serialpipea_type,
	.config_port =		serialpipea_config_port,
	.ioctl =		serialpipe_ioctl
};

static const struct uart_ops serialpipeb_ops = {
	.tx_empty =		serialsim_tx_empty,
	.set_mctrl =		serialsim_set_mctrl,
	.get_mctrl =		serialsim_get_mctrl,
	.stop_tx =		serialsim_stop_tx,
	.start_tx =		serialsim_start_tx,
	.stop_rx =		serialsim_stop_rx,
	.break_ctl =		serialsim_break_ctl,
	.startup =		serialsim_startup,
	.shutdown =		serialsim_shutdown,
	.release_port =		serialsim_release_port,
	.set_termios =		serialsim_set_termios,
	.type =			serialpipeb_type,
	.config_port =		serialpipeb_config_port,
	.ioctl =		serialpipe_ioctl
};

static struct uart_driver serialecho_driver = {
	.owner = THIS_MODULE,
	.driver_name = "ttyEcho",
	.dev_name = "ttyEcho"
};

static struct uart_driver serialpipea_driver = {
	.owner = THIS_MODULE,
	.driver_name = "ttyPipeA",
	.dev_name = "ttyPipeA"
};

static struct uart_driver serialpipeb_driver = {
	.owner = THIS_MODULE,
	.driver_name = "ttyPipeB",
	.dev_name = "ttyPipeB"
};

#ifdef HAS_RS485_SUPPORTED
static const struct serial_rs485 serialsim_rs485_supported = {
        .flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND |
                 SER_RS485_RTS_AFTER_SEND | SER_RS485_RX_DURING_TX,
        .delay_rts_before_send = 1,
        .delay_rts_after_send = 1,
};
#endif

static DEFINE_MUTEX(serialsim_num_mutex);
static DEFINE_IDR(serialsim_echo_nums);
static DEFINE_IDR(serialsim_pipe_nums);

static int serialsim_alloc_intf(struct idr *idr,
				unsigned int i,
				const char *name,
				const struct uart_ops *ops,
				int min_id, int max_id,
				struct serialsim_intf **rintf)
{
	struct serialsim_intf *intf;
	struct uart_port *port;
	int rv;

	intf = kzalloc(sizeof(*intf), GFP_KERNEL);
	if (!intf) {
		pr_err("serialsim: Unable to alloc %s port %u\n", name, i);
		return -ENOMEM;
	}

	intf->buf.buf = intf->xmitbuf;
	intf->threadname = "serialecho";
	spin_lock_init(&intf->mctrl_lock);
	tasklet_init(&intf->mctrl_tasklet, mctrl_tasklet, (long) intf);

	intf->pdev.name = name;
	intf->pdev.id = i;
	rv = platform_device_register(&intf->pdev);
	if (rv) {
		platform_device_put(&intf->pdev);
		pr_err("serialsim: Unable to register device for %s:%u: %d\n",
		       name, i, rv);
		return rv;
	}

	port = &intf->port;
	port->dev = &intf->pdev.dev;
	/* Won't configure without some I/O or mem address set. */
	port->type = UPIO_PORT;
	port->iobase = 1;
	port->flags = UPF_BOOT_AUTOCONF | UPF_SOFT_FLOW;
	spin_lock_init(&port->lock);
	port->attr_group = &serialsim_dev_attr_group;
	port->ops = ops;

	if (idr) {
		int id = idr_alloc(idr, intf, min_id, max_id, GFP_KERNEL);

		if (id < 0) {
			pr_err("serialsim: Unable to alloc id for %s %u: %d\n",
			       name, i, id);
			kfree(intf);
			return id;
		}
		intf->idr = idr;
		port->line = id;
	} else {
		port->line = min_id;
	}

	*rintf = intf;
	return 0;
}

static void serialsim_free_intf(struct serialsim_intf *intf)
{
	if (intf->driver)
		uart_remove_one_port(intf->driver, &intf->port);
	tasklet_kill(&intf->mctrl_tasklet);
	if (intf->idr)
		idr_remove(intf->idr, intf->port.line);
	kfree(intf);
}

static int serialsim_add_port(int i, const char *name,
			      struct uart_driver *driver,
			      struct serialsim_intf *intf)
{
	int rv;

	rv = uart_add_one_port(driver, &intf->port);
	if (rv) {
		pr_err("serialsim: Unable to add uart %s port %u: %d\n",
		       name, i, rv);
		return rv;
	}
	intf->driver = driver;
	return 0;
}

struct serialsim_dyninfo {
	struct idr nums;
};

static long serialsim_echo_ioctl(struct file *file,
				 unsigned int cmd, unsigned long data)
{
	int rv = -ENOTTY;
	struct serialsim_dyninfo *di = file->private_data;
	struct serialsim_intf *intf;

	mutex_lock(&serialsim_num_mutex);
	switch (cmd) {
	case SERIALSIM_ALLOC_ID:
		rv = serialsim_alloc_intf(&serialsim_echo_nums, -1, "ttyEcho",
					  &serialecho_ops, nr_echo_ports,
					  nr_echo_ports + nr_dyn_echo_ports,
					  &intf);
		if (rv)
			break;
		rv = idr_alloc(&di->nums, intf,
			       intf->port.line, intf->port.line + 1,
			       GFP_KERNEL);
		if (rv < 0) {
			serialsim_free_intf(intf);
			break;
		}
		intf->ointf = intf;
		intf->do_null_modem = true;

		rv = serialsim_add_port(-1, "echo", &serialecho_driver, intf);
		if (rv) {
			idr_remove(&di->nums, intf->port.line);
			serialsim_free_intf(intf);
		} else {
			rv = intf->port.line;
		}
		break;

	case SERIALSIM_FREE_ID:
		struct serialsim_intf *intf = idr_remove(&di->nums, data);

		rv = 0;
		if (intf)
			serialsim_free_intf(intf);
		else
			rv = -ENOENT;
		break;

	default:
		break;
	}
	mutex_unlock(&serialsim_num_mutex);

	return rv;
}

static int serialsim_echo_dyn_release(int id, void *p, void *data)
{
	struct serialsim_intf *intf = idr_find(&serialsim_echo_nums, id);

	if (intf)
		serialsim_free_intf(intf);

	return 0;
}

static int serialsim_echo_release(struct inode *ino, struct file *file)
{
	struct serialsim_dyninfo *di = file->private_data;

	mutex_lock(&serialsim_num_mutex);
	idr_for_each(&di->nums, serialsim_echo_dyn_release, NULL);
	mutex_unlock(&serialsim_num_mutex);
	idr_destroy(&di->nums);
	kfree(di);
	return 0;
}

static int serialsim_echo_open(struct inode *ino, struct file *file)
{
	struct serialsim_dyninfo *di = kzalloc(sizeof(*di), GFP_KERNEL);

	if (!di)
		return -ENOMEM;

	idr_init(&di->nums);

	file->private_data = di;
	return 0;
}

const struct file_operations serialsim_echo_misc_fops = {
        .owner = THIS_MODULE,
        .unlocked_ioctl = serialsim_echo_ioctl,
        .open = serialsim_echo_open,
        .release = serialsim_echo_release
};

static struct miscdevice serialsim_echo_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ttyEcho",
	.fops = &serialsim_echo_misc_fops,
	.mode = 0660
};

static long serialsim_pipe_ioctl(struct file *file,
				 unsigned int cmd, unsigned long data)
{
	int rv = -ENOTTY;
	struct serialsim_dyninfo *di = file->private_data;
	struct serialsim_intf *intfa, *intfb;

	mutex_lock(&serialsim_num_mutex);
	switch (cmd) {
	case SERIALSIM_ALLOC_ID:
		rv = serialsim_alloc_intf(&serialsim_pipe_nums, -1, "ttyPipeA",
					  &serialpipea_ops, nr_pipe_ports,
					  nr_pipe_ports + nr_dyn_pipe_ports,
					  &intfa);
		if (rv)
			break;
		rv = serialsim_alloc_intf(NULL, -1, "ttyPipeB",
					  &serialpipeb_ops,
					  intfa->port.line, 0,
					  &intfb);
		if (rv) {
			serialsim_free_intf(intfa);
			break;
		}
		rv = idr_alloc(&di->nums, intfa,
			       intfa->port.line, intfa->port.line + 1,
			       GFP_KERNEL);
		if (rv < 0) {
			serialsim_free_intf(intfb);
			serialsim_free_intf(intfa);
			break;
		}
		intfa->ointf = intfb;
		intfb->ointf = intfa;

		intfa->port.rs485_config = serialsim_rs485;
		intfb->port.rs485_config = serialsim_rs485;
#ifdef HAS_RS485_SUPPORTED
		intfa->port.rs485_supported = serialsim_rs485_supported;
		intfb->port.rs485_supported = serialsim_rs485_supported;
#endif

		rv = serialsim_add_port(-1, "pipea", &serialpipea_driver,
					intfa);
		if (rv) {
			idr_remove(&di->nums, intfa->port.line);
			serialsim_free_intf(intfb);
			serialsim_free_intf(intfa);
			break;
		}

		rv = serialsim_add_port(-1, "pipeb", &serialpipeb_driver,
					intfb);
		if (rv) {
			idr_remove(&di->nums, intfa->port.line);
			serialsim_free_intf(intfb);
			serialsim_free_intf(intfa);
			break;
		} else {
			rv = intfa->port.line;
		}

		serialsim_set_null_modem(intfa, true);
		serialsim_set_null_modem(intfb, true);
		break;

	case SERIALSIM_FREE_ID:
		struct serialsim_intf *intf = idr_remove(&di->nums, data);

		rv = 0;
		if (intf) {
			serialsim_free_intf(intf->ointf);
			serialsim_free_intf(intf);
		} else {
			rv = -ENOENT;
		}
		break;

	default:
		break;
	}
	mutex_unlock(&serialsim_num_mutex);

	return rv;
}

static int serialsim_pipe_dyn_release(int id, void *p, void *data)
{
	struct serialsim_intf *intf = idr_find(&serialsim_pipe_nums, id);

	if (intf) {
		serialsim_free_intf(intf->ointf);
		serialsim_free_intf(intf);
	}

	return 0;
}

static int serialsim_pipe_release(struct inode *ino, struct file *file)
{
	struct serialsim_dyninfo *di = file->private_data;

	mutex_lock(&serialsim_num_mutex);
	idr_for_each(&di->nums, serialsim_pipe_dyn_release, NULL);
	mutex_unlock(&serialsim_num_mutex);
	idr_destroy(&di->nums);
	kfree(di);
	return 0;
}

static int serialsim_pipe_open(struct inode *ino, struct file *file)
{
	struct serialsim_dyninfo *di = kzalloc(sizeof(*di), GFP_KERNEL);

	if (!di)
		return -ENOMEM;

	idr_init(&di->nums);

	file->private_data = di;
	return 0;
}

const struct file_operations serialsim_pipe_misc_fops = {
        .owner = THIS_MODULE,
        .unlocked_ioctl = serialsim_pipe_ioctl,
        .open = serialsim_pipe_open,
        .release = serialsim_pipe_release
};

static struct miscdevice serialsim_pipe_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ttyPipe",
	.fops = &serialsim_pipe_misc_fops,
	.mode = 0660
};

static int __init serialsim_init(void)
{
	unsigned int i;
	int rv;

	serialecho_driver.nr = nr_echo_ports + nr_dyn_echo_ports;
	rv = uart_register_driver(&serialecho_driver);
	if (rv) {
		pr_err("serialsim: Unable to register echo driver.\n");
		goto out;
	}

	serialpipea_driver.nr = nr_pipe_ports + nr_dyn_pipe_ports;
	rv = uart_register_driver(&serialpipea_driver);
	if (rv) {
		pr_err("serialsim: Unable to register pipe a driver.\n");
		goto out_unreg_echo;
	}

	serialpipeb_driver.nr = nr_pipe_ports + nr_dyn_pipe_ports;
	rv = uart_register_driver(&serialpipeb_driver);
	if (rv) {
		pr_err("serialsim: Unable to register pipe b driver.\n");
		goto out_unreg_pipea;
	}

	for (i = 0; i < nr_echo_ports; i++) {
		struct serialsim_intf *intf;

		rv = serialsim_alloc_intf(&serialsim_echo_nums, i, "ttyEcho",
					  &serialecho_ops, 0, nr_echo_ports,
					  &intf);
		if (rv)
			break;
		intf->ointf = intf;
		intf->do_null_modem = true;

		rv = serialsim_add_port(i, "echo", &serialecho_driver, intf);
		if (rv) {
			serialsim_free_intf(intf);
			break;
		}
	}

	for (i = 0; !rv && i < nr_pipe_ports; i++) {
		struct serialsim_intf *intfa;
		struct serialsim_intf *intfb;

		rv = serialsim_alloc_intf(&serialsim_pipe_nums, i, "ttyPipeA",
					  &serialpipea_ops,
					  0, nr_pipe_ports, &intfa);
		if (rv)
			break;
		rv = serialsim_alloc_intf(NULL, i, "ttyPipeB",
					  &serialpipeb_ops,
					  intfa->port.line, 0, &intfb);
		if (rv) {
			serialsim_free_intf(intfa);
			break;
		}
		intfa->ointf = intfb;
		intfb->ointf = intfa;

		intfa->port.rs485_config = serialsim_rs485;
		intfb->port.rs485_config = serialsim_rs485;
#ifdef HAS_RS485_SUPPORTED
		intfa->port.rs485_supported = serialsim_rs485_supported;
		intfb->port.rs485_supported = serialsim_rs485_supported;
#endif

		/*
		 * uart_add_one_port() does an mctrl operation, which will
		 * claim the other port's lock.  So everything needs to be
		 * fully initialized, and we need null modem off until we
		 * get things added.
		 */

		rv = serialsim_add_port(i, "pipea", &serialpipea_driver, intfa);
		if (rv) {
			serialsim_free_intf(intfb);
			serialsim_free_intf(intfa);
			break;
		}

		rv = serialsim_add_port(i, "pipeb", &serialpipeb_driver, intfb);
		if (rv) {
			serialsim_free_intf(intfb);
			serialsim_free_intf(intfa);
			break;
		}

		serialsim_set_null_modem(intfa, true);
		serialsim_set_null_modem(intfb, true);
	}

	rv = misc_register(&serialsim_echo_misc);
	if (rv) {
		pr_err("serialsim: Error registering dynamic echo device: %d\n",
		       rv);
	}

	rv = misc_register(&serialsim_pipe_misc);
	if (rv) {
		pr_err("serialsim: Error registering dynamic pipe device: %d\n",
		       rv);
	}

	return 0;

out_unreg_pipea:
	uart_unregister_driver(&serialpipea_driver);
out_unreg_echo:
	uart_unregister_driver(&serialecho_driver);
out:
	return rv;
}

static int serialsim_clear_intf(int id, void *p, void *dummy)
{
	struct serialsim_intf *intf = p;

	if (intf->ointf != intf)
		serialsim_free_intf(intf->ointf);
	serialsim_free_intf(intf);
	return 0;
}

static void __exit serialsim_exit(void)
{
	misc_deregister(&serialsim_pipe_misc);
	misc_deregister(&serialsim_echo_misc);
	idr_for_each(&serialsim_echo_nums, serialsim_clear_intf, NULL);
	idr_for_each(&serialsim_pipe_nums, serialsim_clear_intf, NULL);
	uart_unregister_driver(&serialecho_driver);
	uart_unregister_driver(&serialpipea_driver);
	uart_unregister_driver(&serialpipeb_driver);

	pr_info("serialsim unloaded\n");
}

module_init(serialsim_init);
module_exit(serialsim_exit);

MODULE_AUTHOR("Corey Minyard");
MODULE_DESCRIPTION("Serial simulation device");
MODULE_LICENSE("GPL");
