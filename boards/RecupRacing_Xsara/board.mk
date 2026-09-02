# Activation de la pile logicielle pour le bus CAN
EFI_CAN_SUPPORT = yes

ifneq ($(PROJECT_CPU),simulator)
BOARDCPPSRC += \
    $(BOARD_DIR)/board_configuration.cpp \

endif