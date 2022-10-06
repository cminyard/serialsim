AC_INIT([serialsim], [1.0-rc2], [minyard@acm.org])
AM_INIT_AUTOMAKE([-Wall])
AC_PROG_CC
AM_PROG_AR
AM_PROG_LIBTOOL

pythonprog=
trypython=yes
AC_ARG_WITH(python,
[  --with-python[[=yes|no|PATH]]     Look for python, with the optional path.],
    if test "x$withval" = "x"; then
      trypython=yes
    elif test "x$withval" = "xyes"; then
      trypython=yes
    elif test "x$withval" = "xno"; then
      trypython=no
    else
      pythonprog="$withval"
      trypython=yes
    fi,
)

pythoninstalldir=
AC_ARG_WITH(pythoninstall,
[  --with-pythoninstall=PATH       Install python modules in the given location.],
    pythoninstalldir="$withval",
)

pythoninstalllibdir=
AC_ARG_WITH(pythoninstalllib,
[  --with-pythoninstalllib=PATH       Install python libraries in the given location.],
    pythoninstalllibdir="$withval",
)

pythoncflags=
AC_ARG_WITH(pythoncflags,
[  --with-pythoncflags=PATH        Use the given flags when compiling python parts.],
    pythoncflags="$withval",
)

pythonusepthreads=
AC_ARG_WITH(pythonusepthreads,
[  --with-pythonusepthreads[[=yes|no]] Use threads with python.],
   if test "x$withval" = "xyes"; then
      pythonusepthreads="yes"
   elif test "x$withval" = "xno"; then
      pythonusepthreads="no"
   elif test "x$withval" = "x"; then
      pythonusepthreads="yes"
   fi,
)

# enable silent build
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

AC_CONFIG_MACRO_DIR([m4])
AC_STDC_HEADERS

# Handle PYTHON support
PYTHON_DIR=
PYTHON_INSTALL_DIR=
PYTHON_INSTALL_LIB_DIR=
if test "x$trypython" = "xyes"; then
   if test "x$pythonprog" != "x"; then
      PYTHON="$pythonprog"
   fi
   if test "x$pythoncflags" != "x"; then
      PYTHON_CPPFLAGS="$pythoncflags"
   fi
   AX_PYTHON_DEVEL
fi
if test "x$PYTHON" != "x"; then
   PYTHON_DIR=python
   if test "x$pythoninstalldir" = "x"; then
      PYTHON_INSTALL_DIR="$PYTHON_SITE_PKG"
   else
      PYTHON_INSTALL_DIR="$pythoninstalldir"
   fi
   if test "x$pythoninstalllibdir" = "x"; then
      PYTHON_INSTALL_LIB_DIR="$PYTHON_INSTALL_DIR"
   else
      PYTHON_INSTALL_LIB_DIR="$pythoninstalllibdir"
   fi
fi

AC_SUBST(PYTHON_DIR)
AC_SUBST(PYTHON_INSTALL_DIR)
AC_SUBST(PYTHON_INSTALL_LIB_DIR)

AC_OUTPUT([Makefile])
