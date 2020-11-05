
This is a project Makefile. It is assumed the directory this Makefile resides in is a
project subdirectory.

PROJECT_NAME := mqtt_tcp

COMPONENT_EXTRA_INCLUDES = $(IDF_PATH)/examples/common_components/protocol_examples_common 
# COMPONENT_EXTRA_INCLUDES = $(IDF_PATH)/components/bt

include $(IDF_PATH)/make/project.mk
