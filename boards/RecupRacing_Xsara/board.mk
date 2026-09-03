# Activation de la pile logicielle pour le bus CAN
EFI_CAN_SUPPORT = yes

# ==========================================================
# FORÇAGE MATÉRIEL POUR LA SONDE LARGE BANDE (ISOLATION MCU)
# Ces lignes ordonnent au compilateur de forcer l'activation
# de l'ADC3 et des TIM12 / TIM8 spécifiquement pour la carte,
# contournant ainsi les limitations du mcuconf.h par défaut.
# ==========================================================
DDEFS += -DSTM32_ADC_USE_ADC3=TRUE
DDEFS += -DSTM32_PWM_USE_TIM12=TRUE
DDEFS += -DSTM32_PWM_USE_TIM8=TRUE

ifneq ($(PROJECT_CPU),simulator)
BOARDCPPSRC += \
    $(BOARD_DIR)/board_configuration.cpp \
    $(BOARD_DIR)/wideband_driver.cpp \

endif