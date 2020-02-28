=====================================
serialsim - A kernel serial simulator
=====================================

:Author: Corey Minyard <minyard@mvista.com> / <minyard@acm.org>

The serialsim device is a serial simulator with echo and pipe devices.
It is quite useful for testing programs that use serial ports.

This attempts to emulate a basic serial device.  It uses the baud rate
and sends the bytes through the loopback or pipe at approximately the
speed it would on a normal serial device.

There is a python interface to the special ioctls for controlling the
remote end of the termios.  It is in the swig directory and is a
standard autoconf build.

========
Building
========
"make" should build the module serialsim.ko, which can be loaded
onto your system with insmod.  You can do "sudo make install" which
will install serialsim.ko in /lib/modules/<kernver>/local and
serialsim.h in /usr/local/include/linux.

To build the python modules, cd to "swig" and run:

./configure
make

=====
Using
=====

The serialsim.ko module creates two types of devices.  Echo devices
simply echo back the data to the same device.  These devices will
appear as /dev/ttyEcho<n>.

Pipe devices will transfer the data between two devices.  The
devices will appear as /dev/ttyPipeA<n> and /dev/ttyPipeB<n>.  And
data written to PipeA reads from PipeB, and vice-versa.

You may create an arbitrary number of devices by setting the
nr_echo ports and nr_pipe_ports module parameters.  The default is
four for both.

This driver supports modifying the modem control lines and
injecting various serial errors.  It also supports a simulated null
modem between the two pipes, or in a loopback on the echo device.

By default a pipe or echo comes up in null modem configuration,
meaning that the DTR line is hooked to the DSR and CD lines on the
other side and the RTS line on one side is hooked to the CTS line
on the other side.

The RTS and CTS lines don't currently do anything for flow control.

You can modify null modem and control the lines individually
through an interface in /sys/class/tty/ttyECHO<n>/ctrl,
/sys/class/tty/ttyPipeA<n>/ctrl, and
/sys/class/tty/ttyPipeB<n>/ctrl.  The following may be written to
those files:

[+-]nullmodem
    enable/disable null modem

[+-]cd
    enable/disable Carrier Detect (no effect if +nullmodem)

[+-]dsr
    enable/disable Data Set Ready (no effect if +nullmodem)

[+-]cts
    enable/disable Clear To Send (no effect if +nullmodem)

[+-]ring
    enable/disable Ring

frame
    inject a frame error on the next byte

parity
    inject a parity error on the next byte

overrun
    inject an overrun error on the next byte

The contents of the above files has the following format:

tty[Echo|PipeA|PipeB]<n>
    <mctrl values>

where <mctrl values> is the modem control values above (not frame,
parity, or overrun) with the following added:

[+-]dtr
    value of the Data Terminal Ready

[+-]rts
    value of the Request To Send

The above values are not settable through this interface, they are
set through the serial port interface itself.

So, for instance, ttyEcho0 comes up in the following state::

   # cat /sys/class/tty/ttyEcho0/ctrl
   ttyEcho0: +nullmodem -cd -dsr -cts -ring -dtr -rts

If something connects, it will become::

   ttyEcho0: +nullmodem +cd +dsr +cts -ring +dtr +rts

To enable ring::

   # echo "+ring" >/sys/class/tty/ttyEcho0/ctrl
   # cat /sys/class/tty/ttyEcho0/ctrl
   ttyEcho0: +nullmodem +cd +dsr +cts +ring +dtr +rts

Now disable NULL modem and the CD line::

   # echo "-nullmodem -cd" >/sys/class/tty/ttyEcho0/ctrl
   # cat /sys/class/tty/ttyEcho0/ctrl
   ttyEcho0: -nullmodem -cd -dsr -cts +ring -dtr -rts

Note that these settings are for the side you are modifying.  So if
you set nullmodem on ttyPipeA0, that controls whether the DTR/RTS
lines from ttyPipeB0 affect ttyPipeA0.  It doesn't affect ttyPipeB's
modem control lines.

The PIPEA and PIPEB devices also have the ability to set these
values for the other end via an ioctl.  The following ioctls are
available:

TIOCSERSNULLMODEM
    Set the null modem value, the arg is a boolean.

TIOCSERSREMMCTRL
    Set the modem control lines, bits 16-31 of the arg is
    a 16-bit mask telling which values to set, bits 0-15 are the
    actual values.  Settable values are TIOCM_CAR, TIOCM_CTS,
    TIOCM_DSR, and TIOC_RNG.  If NULLMODEM is set to true, then only
    TIOC_RNG is settable.  The DTR and RTS lines are not here, you can
    set them through the normal interface.

TIOCSERSREMERR
    Send an error or errors on the next sent byte.  arg is
    a bitwise OR of (1 << TTY_xxx).  Allowed errors are TTY_BREAK,
    TTY_FRAME, TTY_PARITY, and TTY_OVERRUN.

TIOCSERGREMTERMIOS
    Return the termios structure for the other side of the pipe.
    arg is a pointer to a standard termios struct.

TIOCSERGREMRS485
    Return the remote RS485 settings, arg is a pointer to a struct
    serial_rs485.

Note that unlike the sysfs interface, these ioctls affect the other
end.  So setting nullmodem on the ttyPipeB0 interface sets whether
the DTR/RTS lines on ttyPipeB0 affect ttyPipeA0.

================
Python Interface
================
The python interface is a straight conversion of the C interface into
python.  It is in the serialsim python module and has the following
interfaces::

   termios = get_remote_termios(fd)

The termios are the standard python termios::

   rs485 = get_remote_rs485(fd)

rs485 is a string representation of the rs485 paramters, in the form::

   "<delay_rts_before_send> <delay_rts_after_send> [<option> []]"

The two given values are integers, options are::

   enabled
   rts_on_send
   rts_after_send
   rx_during_tx
   terminate_bus

You will need to review RS485 documentation for details.  To get and
set the modem control lines::

   set_remote_modem_ctl(fd, val)
   val = get_remote_modem_ctl(fd);

The value is a bitmask of::

   SERIALSIM_TIOCM_CAR
   SERIALSIM_TIOCM_CTS
   SERIALSIM_TIOCM_DSR
   SERIALSIM_TIOCM_RNG
   SERIALSIM_TIOCM_DTR
   SERIALSIM_TIOCM_RTS

You cannot set DTR or RTS, they are outputs from the other side::

   set_remote_serial_err(fd, val)
   val = get_remote_serial_err(fd);

You can inject serial errors on the other end.  The value is a bitmask
of::

   SERIALSIM_TTY_BREAK
   SERIALSIM_TTY_FRAME
   SERIALSIM_TTY_PARITY
   SERIALSIM_TTY_OVERRUN

Hopefully the meanings of these are obvious.  The null modem setting
for the remote serial port::

   set_remote_null_modem(fd, bool_val)
   bool_val = get_remote_null_modem(fd);


