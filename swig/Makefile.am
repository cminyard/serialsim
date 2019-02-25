
ACLOCAL_AMFLAGS = -I m4

AM_CFLAGS = $(PYTHON_CPPFLAGS) \
	    -I $(top_srcdir)

pythonlibdir=$(PYTHON_INSTALL_LIB_DIR)
PYPATH=$(top_builddir)/swig/python:$(top_builddir)/swig/python/.libs

pythonlib_LTLIBRARIES = _serialsim.la

serialsim_wrap.c serialsim.py: serialsim.i
	$(SWIG) $(DEFS) -python $(PYTHON_SWIG_FLAGS) -o serialsim_wrap.c \
	$<

EXTRA_DIST = serialsim.i

nodist__serialsim_la_SOURCES = serialsim_wrap.c
_serialsim_la_SOURCES = serialsim.c
_serialsim_la_LDFLAGS = -module -avoid-version

clean-local:
	rm -rf  serialsim_wrap.c serialsim.py*

install-exec-local: _serialsim.la serialsim.py
	$(INSTALL_DATA) serialsim.py "$(DESTDIR)$(PYTHON_INSTALL_DIR)"

uninstall-local:
	rm -f "$(DESTDIR)$(PYTHON_INSTALL_DIR)/serialsim.py"
