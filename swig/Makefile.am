pythonlibdir=$(PYTHON_INSTALL_LIB_DIR)
PYPATH=$(top_builddir)/swig/python

install-exec-local: serialsim.py
	$(INSTALL_DATA) serialsim.py "$(DESTDIR)$(PYTHON_INSTALL_DIR)"

uninstall-local:
	rm -f "$(DESTDIR)$(PYTHON_INSTALL_DIR)/serialsim.py"
