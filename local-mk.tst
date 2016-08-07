# makefile additions due to rtos changes
FW_ADDR_1	= 0x00000 
FW_ADDR_2	= 0x20000 
FW_FILE_1    = $(addprefix $(FIRMWARE_DIR),$(addsuffix .bin,$(FW_ADDR_1))) 
FW_FILE_2    = $(addprefix $(FIRMWARE_DIR),$(addsuffix .bin,$(FW_ADDR_2))) 

