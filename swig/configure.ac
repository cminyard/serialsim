AC_INIT([serialsim], [1.0-rc2], [minyard@acm.org])
AM_INIT_AUTOMAKE([-Wall])
AC_PROG_CC
AM_PROG_AR
AM_PROG_LIBTOOL

tryswig=yes
swigprog=
AC_ARG_WITH(swig,
[  --with-swig[[=yes|no|PATH]]       Look for swig, with the optional path.],
    if test "x$withval" = "x"; then
      tryswig=yes
    elif test "x$withval" = "xyes"; then
      tryswig=yes
    elif test "x$withval" = "xno"; then
      tryswig=no
    else
      swigprog=$withval
    fi,
)

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

# Now check for swig
SWIG_DIR=
SWIG=
if test "x$tryswig" = "xyes"; then
   if test "x$swigprog" != "x"; then
      SWIG="$swigprog"
   fi
   AX_PKG_SWIG([1.3.21])
   if test "x$SWIG" != "x"; then
      AC_DEFINE([HAVE_SWIG], [], [Have swig installed])
      SWIG_DIR=swig
   fi
fi
AC_SUBST(SWIG_DIR)
AC_SUBST(SWIG)

# Handle PYTHON support
PYTHON_DIR=
PYTHON_INSTALL_DIR=
PYTHON_INSTALL_LIB_DIR=
PYTHON_SWIG_FLAGS=
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
   AX_PROG_PYTHON_VERSION([3.0.0], [PYTHON_SWIG_FLAGS=-py3], [])
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

   if test "x$pythonusepthreads" = "x"; then
      cat - <<_ACEOF >conftest.py
try:
  import threading
  print('yes')
except:
  print('no')
_ACEOF
      pythonusepthreads=`$PYTHON conftest.py`
      rm -f conftest.py
   fi
   echo "checking for python threads... $pythonusepthreads"

   if test "x$pythonusepthreads" = "xyes"; then
      PYTHON_HAS_POSIX_THREADS=1
   else
      PYTHON_HAS_POSIX_THREADS=0
   fi
fi

AC_SUBST(PYTHON_DIR)
AC_SUBST(PYTHON_INSTALL_DIR)
AC_SUBST(PYTHON_INSTALL_LIB_DIR)
AC_SUBST(PYTHON_HAS_POSIX_THREADS)
AC_SUBST(PYTHON_SWIG_FLAGS)

AC_OUTPUT([Makefile])
