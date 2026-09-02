# Activation de la pile logicielle pour le bus CAN
EFI_CAN_SUPPORT = yes

# Activation du bootloader OpenBLT (décale l'adresse de liaison à 0x08010000)
USE_OPENBLT = yes

ifneq ($(PROJECT_CPU),simulator)
BOARDCPPSRC += \
    $(BOARD_DIR)/board_configuration.cpp \

endif