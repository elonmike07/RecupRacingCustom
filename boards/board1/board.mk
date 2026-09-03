# Activation de la pile logicielle pour le bus CAN
EFI_CAN_SUPPORT = yes

# ==========================================
# ACTIVATION DES MODULES LOGICIELS RUSEFI
# ==========================================

# 1. Activation du module de traitement DSP pour le Cliquetis (Knock)
BOARD_EXT_CFLAGS += -DEFI_KNOCK=1
BOARD_EXT_CPPFLAGS += -DEFI_KNOCK=1

# 2. Activation de la gestion de la sonde lambda interne discrete (WBO)
BOARD_EXT_CFLAGS += -DEFI_INTERNAL_WIDEBAND=1
BOARD_EXT_CPPFLAGS += -DEFI_INTERNAL_WIDEBAND=1

ifneq ($(PROJECT_CPU),simulator)
BOARDCPPSRC += $(BOARD_DIR)/board_configuration.cpp
endif