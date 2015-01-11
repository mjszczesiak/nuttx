apps/interpreters README file
=============================

This apps/ directory is set aside to hold interpreters that may be
incorporated into NuttX.

ficl
----

  This is DIY port of Ficl (the "Forth Inspired Command Language").  See
  http://ficl.sourceforge.net/.  It is a "DIY" port because the Ficl source
  is not in that directory, only an environment and instructions that will
  let you build Ficl under NuttX.  The rest is up to you.

micropython
-----------

  This is a port of a build environment for Micro Python:

    https://micropython.org/

  Configuration Options:

    CONFIG_INTERPRETERS_MICROPYTHON - Enables support for the Micro Python
      interpreter
    CONFIG_INTERPRETERS_MICROPYTHON_URL - URL where Micro Python can be
      downloaded. default "https://github.com/micropython/micropython/archive"
    CONFIG_INTERPRETERS_MICROPYTHON_VERSION - Version number. Default "1.3.8"
    CONFIG_INTERPRETERS_MICROPYTHON_APPNAME - Executable name.  Only needed
      if CONFIG_NSH_BUILTIN_APPS=y.  Default: "micropython"
    CONFIG_INTERPRETERS_MICROPYTHON_STACKSIZE - Interpreter stack size.  Only
      needed if CONFIG_NSH_BUILTIN_APPS=y.  Default: 2048
    CONFIG_INTERPRETERS_MICROPYTHON_PRIORITY - Interpreter priority.  Only
      needed if CONFIG_NSH_BUILTIN_APPS=y.  Default: 100
    CONFIG_INTERPRETERS_MICROPYTHON_PROGNAME - Program name.  Only needed
      if CONFIG_BUILD_KERNEL=y.  Default: "micropython"

  NOTE that Micro Python is not included in this directory.  Be default,
  it will be downloaded at build time from the github .  You can avoid
  this download by pre-installing Micro Python.  Before building, just
  download Micro Python from:

    https://micropython.org/download/
    https://github.com/micropython/micropython/releases

  Or clone from the GIT repository at:

    https://github.com/micropython/
    https://github.com/micropython/micropython

  The Micro Python should be provided as a tarbll name:

    apps/interpreters/micropython/v$(CONFIG_INTERPRETERS_MICROPYTHON_VERSION).tar.gz

  and the unpacked code should reside in directory at:

    apps/interpreters/micropython/micropython-$(CONFIG_INTERPRETERS_MICROPYTHON_VERSION)

  This port was contributed by Dave Marples using Micro Python circa
  1.3.8.  It may not be compatible with other versions.

  NOTE: Right now, Micro Python will not build on Windows with a Windows
  native toolchain due to usage of POSIX paths in the Micro Python build
  system.  It should build correctly on Linux or under Cygwin with the
  NuttX buildroot tools.

pcode
-----

  At present, only the NuttX Pascal add-on is supported.  This NuttX add-on
  must be downloaded separately (or is available in an GIT snapshot in the
  misc/pascal directory).

  This Pascal add-on must be installed into the NuttX apps/ directory.  After
  unpacking the Pascal add-on package, an installation script and README.txt
  instructions can be found at pascal/nuttx.

  INSTALL.sh -- The script that performs the operation.  Usage:

     ./INSTALL.sh [-16|-32] <install-dir>

      If you are using this standard NuttX apps/ package, the correct
      location for the <install-dir> is apps/interpreters.  That is
      where the examples and build logic will expect to find the pcode
      sub-directory.

    Example:

      ./INSTALL.sh -16 $PWD/../../../apps/interpreters

    After installation, the NuttX apps/interpresters directory will contain
    the following files

      pcode
      |-- Makefile
      |-- include
      |   `-- Common header files
      |-- libboff
      |   `-- Pascal object format (POFF) library
      `--insn
          |-- include
          |   `-- model-specific header files
          `-- prun
              `-- model-specific source files

  pashello

    There is a simple Pascal example at apps/examples/pashello.  This is the
    standard "Hello, World!" example written in Pascal and interpreted from
    Pascal P-Code at runtime.  To use this example, place the following in
    your defonfig file:

      CONFIG_EXAMPLES_PASHELLO=y
      CONFIG_INTERPRETERS_PCODE=y

prun

  This directory holds some simple, convenience functions to simplify and
  standardize the interaction with the P-Code library.
