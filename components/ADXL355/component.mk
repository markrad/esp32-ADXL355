#
# Component Makefile
#

COMPONENT_SRCDIRS := .

COMPONENT_ADD_INCLUDEDIRS = include

COMPONENT_ADD_LDFLAGS = -lADXL355

# do not produce gcov info for this module, it is used as transport for gcov
CFLAGS := $(subst --coverage,,$(CFLAGS))


