# Simple makefile for eps-lisp
PROGRAM=*.c
FW_ADDR_1	= 0x00000 
FW_ADDR_2	= 0x20000 
FW_FILE_1    = $(addprefix $(FIRMWARE_DIR),$(FW_ADDR_1).bin) 
FW_FILE_2    = $(addprefix $(FIRMWARE_DIR),$(FW_ADDR_2).bin) 
include ../esp-open-rtos/common.mk

