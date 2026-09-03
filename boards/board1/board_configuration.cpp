#include "pch.h"
#include "board_overrides.h"

Gpio getCommsLedPin() {
    return Gpio::Unassigned;
}

Gpio getRunningLedPin() {
    return Gpio::Unassigned;
}

Gpio getWarningLedPin() {
    return Gpio::Unassigned;
}

// =========================================================================
// 1. LA NOUVELLE FONCTION QUI CONTIENT TOUS TES RÉGLAGES
// =========================================================================
static void customBoardDefaultConfiguration() {

    // ==========================================
    // 1. COMMUNICATION SÉRIE & USB
    // ==========================================
    engineConfiguration->scriptSerialRxPin = Gpio::C10;   // STM_RX3
    engineConfiguration->scriptSerialTxPin = Gpio::C11;   // STM_TX3
    engineConfiguration->tunerStudioBaudRate = 115200;
    engineConfiguration->usbVbusPin = Gpio::A9;           // USB_VBUS_SENSE

    // ==========================================
    // 2. CAPTEURS ANALOGIQUES (ADC)
    // ==========================================
    engineConfiguration->map.sensor.hwChannel = EFI_ADC_0; // MAP-EXT (Canal ADC 0 / PC0)
    engineConfiguration->tps1_1AdcChannel = EFI_ADC_2;    // TPS Sensor (PC2 -> ADC2)
    engineConfiguration->clt.adcChannel = EFI_ADC_9;      // CLT_SENSOR (PB1 -> ADC9)
    engineConfiguration->iat.adcChannel = EFI_ADC_15;     // IAT-SENSOR (PC5 -> ADC15)
    engineConfiguration->vbattAdcChannel = EFI_ADC_14;    // Reference-Battery-V (PC4 -> ADC14)
    
    // ==========================================
    // 3. CAPTEUR DE CLIQUETIS (KNOCK)
    // ==========================================
    // PC1 correspond au canal ADC 11 sur STM32F4
    engineConfiguration->knockPin = Gpio::C1;
    engineConfiguration->knockAdcChannel = EFI_ADC_11;

    // ==========================================
    // 4. BUS CAN, VITESSE & FLEX FUEL
    // ==========================================
    engineConfiguration->canRxPin = Gpio::D0;             // CAN_RX
    engineConfiguration->canTxPin = Gpio::D1;             // CAN_TX
    engineConfiguration->vehicleSpeedSensorInputPin = Gpio::B4; // VSS
    engineConfiguration->flexSensorPin = Gpio::E9;        // Flex_Sensor_STM

    // ==========================================
    // 5. BUS SPI & EXPANDEUR (MCP23S17)
    // ==========================================
    engineConfiguration->is_enabled_spi_1 = true;
    engineConfiguration->spi1sckPin = Gpio::A5;           // MCP-SCK
    engineConfiguration->spi1misoPin = Gpio::A6;          // MCP-MISO
    engineConfiguration->spi1mosiPin = Gpio::A7;          // MCP-MOSI
    // Le Chip Select (PE3) sera géré par l'infrastructure d'extension de rusEFI

    // ==========================================
    // 6. GESTION DE L'ALIMENTATION & DÉMARRAGE
    // ==========================================
    engineConfiguration->ignitionKeyDigitalPin = Gpio::B12; // IGN_KEY_SENSE
    engineConfiguration->mainRelayPin = Gpio::E0;           // MAIN_RLY_IN-ESP
    engineConfiguration->fuelPumpPin = Gpio::D2;            // FUELPUMP_IN-ESP
    engineConfiguration->tachOutputPin = Gpio::A8;          // TACHO-ESP

    // ==========================================
    // 7. INJECTION & ALLUMAGE
    // ==========================================
    engineConfiguration->injectionPins[0] = Gpio::B8;     // INJ-1_4_IN-ESP
    engineConfiguration->injectionPins[1] = Gpio::B9;     // INJ-2_3_IN-ESP
    engineConfiguration->ignitionPins[0] = Gpio::C7;      // IGN_1_IN-ESP
    engineConfiguration->ignitionPins[1] = Gpio::C6;      // IGN_2_IN-ESP

    // ==========================================
    // 8. TRIGGER / CAPTEUR DE VILEBREQUIN
    // ==========================================
    engineConfiguration->triggerInputPins[0] = Gpio::E11; // CRANK_OUT-ESP

    // ==========================================
    // 9. CONTRÔLE RALENTI (MOTEUR PAS-À-PAS)
    // ==========================================
    engineConfiguration->stepperEnablePin = Gpio::D13;         // NENBL-ESP
    engineConfiguration->idle.stepperDirectionPin = Gpio::D14; // DIR-ESP 
    engineConfiguration->idle.stepperStepPin = Gpio::D12;      // STEP-ESP 

    // ==========================================
    // 10. GESTION ANNEXE (CLIMATISATION)
    // ==========================================
    engineConfiguration->acSwitch = Gpio::A4;             // AC_SW
    engineConfiguration->acRelayPin = Gpio::E1;           // AC_RELAY_IN-ESP

    // ==========================================
    // 11. SONDE LARGE BANDE (WBO DISCRÈTE)
    // ==========================================
    engineConfiguration->o2heaterPin = Gpio::B14;         // WBO_HEAT_ESP
    engineConfiguration->wboPumpPin = Gpio::C8;           // WBO_PUMP_PWM
    engineConfiguration->wboNernstPin = Gpio::C9;         // WBO_NERNST_AC
    
    // Tensions analogiques WBO (PA2 = ADC_2, PA3 = ADC_3)
    engineConfiguration->wboUrAdcChannel = EFI_ADC_2;     // PA2 WBO UR
    engineConfiguration->wboUaAdcChannel = EFI_ADC_3;     // PA3 WBO UA

    // ==========================================
    // 12. SORTIES PROGRAMMABLES (FSIO)
    // ==========================================
    engineConfiguration->fsiopins[0] = Gpio::A10; // Programmable Output 1 : ESP32_RST (PA10)
    engineConfiguration->fsiopins[1] = Gpio::E2;  // Programmable Output 2 : CANISTER_IN (PE2)
    engineConfiguration->fsiopins[2] = Gpio::E8;  // Programmable Output 3 : GLOBAL_ENABLE (PE8)
}

// =========================================================================
// 2. LE "CROCHET" DE DÉMARRAGE POUR RUSEFI
// =========================================================================
void setup_custom_board_overrides() {
    // Lors d'un "Factory Reset", rusEFI appellera notre fonction customBoardDefaultConfiguration
    // au lieu de charger un tableau vide.
    custom_board_DefaultConfiguration = customBoardDefaultConfiguration;
}