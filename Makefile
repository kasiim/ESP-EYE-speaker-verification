#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := who_is_it

#SOLUTION_PATH ?= $(ESPWHO_PATH)

#EXTRA_COMPONENT_DIRS += $(ADF_PATH)/components/
EXTRA_COMPONENT_DIRS += ./components/

#include $(SOLUTION_PATH)/components/component_conf.mk
include $(IDF_PATH)/make/project.mk

