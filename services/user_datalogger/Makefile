TOPDIR ?= ../..
include $(TOPDIR)/.config

$(DATA_LOGGER_SUPPORT)_SRC += services/user_datalogger/util.c
$(DATA_LOGGER_SUPPORT)_SRC += services/user_datalogger/usart.c
$(DATA_LOGGER_SUPPORT)_SRC += services/user_datalogger/logger.c
$(DATA_LOGGER_SUPPORT)_ECMD_SRC += services/user_datalogger/ecmd.c
$(DATA_LOGGER_S0)_SRC += services/user_datalogger/s0logger.c
$(DATA_LOGGER_KACO)_SRC += services/user_datalogger/kaco.c
$(DATA_LOGGER_VITO)_SRC += services/user_datalogger/vito.c

##############################################################################
# generic fluff
include $(TOPDIR)/scripts/rules.mk
