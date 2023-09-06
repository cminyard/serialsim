import ctypes
import fcntl
import platform
import termios

if platform.machine() == "sparc":
    # https://github.com/torvalds/linux/blob/v6.0/arch/sparc/include/uapi/asm/termbits.h#L10
    tcflag_t = ctypes.c_ulong
else:
    tcflag_t = ctypes.c_uint

cc_t = ctypes.c_ubyte

speed_t = ctypes.c_uint

if platform.machine() in ["mips", "mips64"]:
    # https://github.com/torvalds/linux/blob/v6.0/arch/mips/include/uapi/asm/termbits.h#L22
    NCCS = 23
else:
    NCCS = 19

UNCCS = 32


class termios2(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("c_iflag", tcflag_t),
        ("c_oflag", tcflag_t),
        ("c_cflag", tcflag_t),
        ("c_lflag", tcflag_t),
        ("c_line", cc_t),
        ("c_cc", cc_t * NCCS),
        ("c_ispeed", speed_t),
        ("c_ospeed", speed_t),
    ]


class serial_rs485(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("flags", ctypes.c_uint32),
        ("delay_rts_before_send", ctypes.c_uint32),
        ("delay_rts_after_send", ctypes.c_uint32),
        ("addr_recv", ctypes.c_uint8),
        ("addr_dest", ctypes.c_uint8),
        ("padding0", ctypes.c_uint8 * 2),
        ("padding1", ctypes.c_uint32 * 4),
    ]


def _IOR(ty, nr, size):
    return (2 << 30) | (ord(ty) << 8) | (nr << 0) | (size << 16)


SERIALSIM_TIOCM_CAR = 0x40
SERIALSIM_TIOCM_CTS = 0x20
SERIALSIM_TIOCM_DSR = 0x100
SERIALSIM_TIOCM_RNG = 0x80
SERIALSIM_TIOCM_DTR = 0x2
SERIALSIM_TIOCM_RTS = 0x4
SERIALSIM_TTY_BREAK = 0x2
SERIALSIM_TTY_FRAME = 0x4
SERIALSIM_TTY_PARITY = 0x8
SERIALSIM_TTY_OVERRUN = 0x10

SER_RS485_ENABLED = 1 << 0
SER_RS485_RTS_ON_SEND = 1 << 1
SER_RS485_RTS_AFTER_SEND = 1 << 2
SER_RS485_RX_DURING_TX = 1 << 4
SER_RS485_TERMINATE_BUS = 1 << 5

TIOCSERSREMNULLMODEM = 0x54E4
TIOCSERSREMMCTRL = 0x54E5
TIOCSERSREMERR = 0x54E6
TIOCSERGREMTERMIOS = _IOR("T", 0xE7, ctypes.sizeof(termios2))
TIOCSERGREMNULLMODEM = _IOR("T", 0xE8, ctypes.sizeof(ctypes.c_int))
TIOCSERGREMMCTRL = _IOR("T", 0xE9, ctypes.sizeof(ctypes.c_uint))
TIOCSERGREMERR = _IOR("T", 0xEA, ctypes.sizeof(ctypes.c_uint))
TIOCSERGREMRS485 = _IOR("T", 0xEB, ctypes.sizeof(serial_rs485))

SERIALSIM_ALLOC_ID = 0x5391
SERIALSIM_FREE_ID = 0x5392


def getspeed(baudrate):
    return getattr(termios, "B{}".format(baudrate))


def get_remote_termios(fd):
    ktermios = termios2()
    assert fcntl.ioctl(fd, TIOCSERGREMTERMIOS, ktermios) == 0
    user_c_cc = []
    for i in range(0, UNCCS):
        if i == termios.VTIME or i == termios.VMIN:
            user_c_cc.append(ktermios.c_cc[i])
        elif i < NCCS:
            user_c_cc.append(chr(ktermios.c_cc[i]))
        else:
            user_c_cc.append(chr(0))
    return (
        ktermios.c_iflag,
        ktermios.c_oflag,
        ktermios.c_cflag,
        ktermios.c_lflag,
        getspeed(ktermios.c_ispeed),
        getspeed(ktermios.c_ospeed),
        tuple(user_c_cc),
    )


def get_remote_rs485(fd):
    rs485 = serial_rs485()
    assert fcntl.ioctl(fd, TIOCSERGREMRS485, rs485) == 0
    tmplist = []
    tmplist.append(str(rs485.delay_rts_before_send))
    tmplist.append(str(rs485.delay_rts_after_send))
    if rs485.flags & SER_RS485_ENABLED:
        tmplist.append("enabled")
    if rs485.flags & SER_RS485_RTS_ON_SEND:
        tmplist.append("rts_on_send")
    if rs485.flags & SER_RS485_RTS_AFTER_SEND:
        tmplist.append("rts_after_send")
    if rs485.flags & SER_RS485_RX_DURING_TX:
        tmplist.append("rx_during_tx")
    if rs485.flags & SER_RS485_TERMINATE_BUS:
        tmplist.append("terminate_bus")
    return " ".join(tmplist)


def set_remote_modem_ctl(fd, val):
    assert fcntl.ioctl(fd, TIOCSERSREMMCTRL, val) == 0


def get_remote_modem_ctl(fd):
    mctl = ctypes.c_uint()
    assert fcntl.ioctl(fd, TIOCSERGREMMCTRL, mctl) == 0
    return mctl.value


def set_remote_serial_err(fd, err):
    assert fcntl.ioctl(fd, TIOCSERSREMERR, err) == 0


def get_remote_serial_err(fd):
    err = ctypes.c_uint()
    assert fcntl.ioctl(fd, TIOCSERGREMERR, err) == 0
    return err.value


def set_remote_null_modem(fd, val):
    assert fcntl.ioctl(fd, TIOCSERSREMNULLMODEM, val) == 0


def get_remote_null_modem(fd):
    val = ctypes.c_int()
    assert fcntl.ioctl(fd, TIOCSERGREMNULLMODEM, val) == 0
    return val.value

def alloc_id(fd):
    rv = fcntl.ioctl(fd, SERIALSIM_ALLOC_ID, 0)
    assert rv >= 0
    return rv

def free_id(fd, val):
    assert fcntl.ioctl(fd, SERIALSIM_FREE_ID, fd) == 0
    
