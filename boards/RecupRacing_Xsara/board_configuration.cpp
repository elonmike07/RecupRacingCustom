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

void setup_custom_board_overrides() {
    // ==========================================
    // 1. COMMUNICATION SÉRIE & USB
    // ==========================================
    // Note : Les broches USB VBUS et les lignes de port série de script sont gérées 
    // nativement ou via les profils matériels STM32F4 de base.

    // ==========================================
    // 2. CAPTEURS ANALOGIQUES (ADC)
    // ==========================================
    engineConfiguration->map.sensor.hwChannel = EFI_ADC_0; // MAP-EXT (Canal ADC 0 / PC0)
    engineConfiguration->tps1_1AdcChannel = EFI_ADC_2;    // TPS Sensor (PC2 -> ADC2)
    engineConfiguration->clt.adcChannel = EFI_ADC_9;      // CLT_SENSOR (PB1 -> ADC9)
    engineConfiguration->iat.adcChannel = EFI_ADC_15;     // IAT-SENSOR (PC5 -> ADC15)
    engineConfiguration->vbattAdcChannel = EFI_ADC_14;    // Reference-Battery-V (PC4 -> ADC14)
    
    // Pour le capteur de cliquetis (Knock sur PC1)
    // (Le canal ADC ou l'entrée knock s'active généralement par la configuration logicielle du module knock)

    // ==========================================
    // 3. BUS CAN, VITESSE & FLEX FUEL
    // ==========================================
    engineConfiguration->canRxPin = Gpio::D0;             // CAN_RX
    engineConfiguration->canTxPin = Gpio::D1;             // CAN_TX
    engineConfiguration->vehicleSpeedSensorInputPin = Gpio::B4; // VSS (Nom officiel validé dans rusefi_config.txt)
    engineConfiguration->flexSensorPin = Gpio::E9;        // Flex_Sensor_STM

    // ==========================================
    // 4. BUS SPI & EXPANDEUR (MCP23S17)
    // ==========================================
    engineConfiguration->is_enabled_spi_1 = true;
    engineConfiguration->spi1sckPin = Gpio::A5;           // MCP-SCK
    engineConfiguration->spi1misoPin = Gpio::A6;          // MCP-MISO
    engineConfiguration->spi1mosiPin = Gpio::A7;          // MCP-MOSI
    // Le Chip Select de l'expandeur (PE3 / IO_CS) se configure via les broches d'extension ou les structures dédiées

    // ==========================================
    // 5. GESTION DE L'ALIMENTATION & DÉMARRAGE
    // ==========================================
    engineConfiguration->ignitionKeyDigitalPin = Gpio::B12; // IGN_KEY_SENSE
    engineConfiguration->mainRelayPin = Gpio::E0;           // MAIN_RLY_IN-ESP
    engineConfiguration->fuelPumpPin = Gpio::D2;            // FUELPUMP_IN-ESP
    engineConfiguration->tachOutputPin = Gpio::A8;          // TACHO-ESP

    // ==========================================
    // 6. INJECTION & ALLUMAGE
    // ==========================================
    engineConfiguration->injectionPins[0] = Gpio::B8;     // INJ-1_4_IN-ESP
    engineConfiguration->injectionPins[1] = Gpio::B9;     // INJ-2_3_IN-ESP
    engineConfiguration->ignitionPins[0] = Gpio::C7;      // IGN_1_IN-ESP
    engineConfiguration->ignitionPins[1] = Gpio::C6;      // IGN_2_IN-ESP

    // ==========================================
    // 7. TRIGGER / CAPTEUR DE VILEBREQUIN
    // ==========================================
    engineConfiguration->triggerInputPins[0] = Gpio::E11; // CRANK_OUT-ESP

    // ==========================================
    // 8. CONTRÔLE RALENTI (MOTEUR PAS-À-PAS)
    // ==========================================
    engineConfiguration->stepperEnablePin = Gpio::D13;    // NENBL-ESP
    engineConfiguration->idle.stepperDirectionPin = Gpio::D14; // DIR-ESP (Structure idle)
    engineConfiguration->idle.stepperStepPin = Gpio::D12;      // STEP-ESP (Structure idle)

    // ==========================================
    // 9. GESTION ANNEXE (CLIMATISATION, WBO)
    // ==========================================
    engineConfiguration->acSwitch = Gpio::A4;             // AC_SW
    engineConfiguration->acRelayPin = Gpio::E1;           // AC_RELAY_IN-ESP
    // engineConfiguration->canisterPin = Gpio::E2;       // Canister (géré en sortie programmable auxiliaire)

    // Sonde Large Bande (Chauffage)
    engineConfiguration->o2heaterPin = Gpio::B14;         // WBO_HEAT_ESP
}