# Mostly written by Jonathan Larmour, Red Hat, Inc.
# Reference to ecos.mak added by John Dallaway, eCosCentric Limited, 2003-01-20
# This file is in the public domain and may be used for any purpose

# Usage:   make INSTALL_DIR=/path/to/ecos/install

# INSTALL_DIR=$$(INSTALL_DIR) # override on make command line
# INSTALL_DIR = ../ROM_slow/install
# INSTALL_DIR = ../LPC2294_ram/LPC2294_ram_install
INSTALL_DIR = ../LPC2294_ram/install

OBJECT_FILES = main.o oc_gpio.o
HEADER_FILES = lpc22xx.h lib_dbg_sh.h oc_gpio.h


include $(INSTALL_DIR)/include/pkgconf/ecos.mak

XCC           = $(ECOS_COMMAND_PREFIX)gcc
XCXX          = $(XCC)
XLD           = $(XCC)

CFLAGS        = -I$(INSTALL_DIR)/include
CXXFLAGS      = $(CFLAGS)
LDFLAGS       = -nostartfiles -L$(INSTALL_DIR)/lib -Ttarget.ld

# RULES

.PHONY: all clean

all:  debug 

clean:
	-rm -f debug $(OBJECT_FILES)

%.o: %.c
	$(XCC) -c -o $*.o $(CFLAGS) $(ECOS_GLOBAL_CFLAGS) $<

%.o: %.cxx
	$(XCXX) -c -o $*.o $(CXXFLAGS) $(ECOS_GLOBAL_CFLAGS) $<

%.o: %.C
	$(XCXX) -c -o $*.o $(CXXFLAGS) $(ECOS_GLOBAL_CFLAGS) $<

%.o: %.cc
	$(XCXX) -c -o $*.o $(CXXFLAGS) $(ECOS_GLOBAL_CFLAGS) $<

debug: $(OBJECT_FILES) $(HEADER_FILES) 
	$(XLD) $(LDFLAGS) $(ECOS_GLOBAL_LDFLAGS) -o $@ $(OBJECT_FILES) -static -L../shell -ldbg_sh

