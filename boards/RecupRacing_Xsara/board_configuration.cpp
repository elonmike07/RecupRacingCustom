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
    engineConfiguration->scriptSerialRxPin = Gpio::PC10;   // STM_RX3
    engineConfiguration->scriptSerialTxPin = Gpio::PC11;   // STM_TX3
    engineConfiguration->tunerStudioBaudRate = 115200;
    engineConfiguration->usbVbusPin = Gpio::PA9;           // USB_VBUS_SENSE

    // ==========================================
    // 2. CAPTEURS ANALOGIQUES (ADC)
    // ==========================================
    engineConfiguration->map.sensorPin = Gpio::PC0;        // MAP-EXT
    engineConfiguration->knockPin = Gpio::PC1;             // Knock Sensor
    engineConfiguration->tpsPin = Gpio::PC2;               // TPS Sensor
    engineConfiguration->cltPin = Gpio::PB1;               // CLT_SENSOR
    engineConfiguration->iatPin = Gpio::PC5;               // IAT-SENSOR
    engineConfiguration->vbattPin = Gpio::PC4;             // Reference-Battery-V

    // ==========================================
    // 3. BUS CAN, VITESSE & FLEX FUEL
    // ==========================================
    engineConfiguration->canRxPin = Gpio::PD0;             // CAN_RX
    engineConfiguration->canTxPin = Gpio::PD1;             // CAN_TX
    engineConfiguration->vssPin = Gpio::PB4;               // ADS_VSS_Sensor_STM
    engineConfiguration->flexSensorPin = Gpio::PE9;        // Flex_Sensor_STM

    // ==========================================
    // 4. BUS SPI & EXPANDEUR (MCP23S17)
    // ==========================================
    engineConfiguration->spi1SckPin = Gpio::PA5;           // MCP-SCK
    engineConfiguration->spi1MisoPin = Gpio::PA6;          // MCP-MISO
    engineConfiguration->spi1MosiPin = Gpio::PA7;          // MCP-MOSI
    engineConfiguration->mcpCsPin = Gpio::PE3;             // IO_CS

    // ==========================================
    // 5. GESTION DE L'ALIMENTATION & DÉMARRAGE
    // ==========================================
    engineConfiguration->ignitionKeyPin = Gpio::PB12;      // IGN_KEY_SENSE (Détection +12V Contact)
    engineConfiguration->mainRelayPin = Gpio::PE0;         // MAIN_RLY_IN-ESP
    engineConfiguration->fuelPumpPin = Gpio::PD2;          // FUELPUMP_IN-ESP
    engineConfiguration->tachoPin = Gpio::PA8;             // TACHO-ESP

    // ==========================================
    // 6. INJECTION, ALLUMAGE & ACTIVATION LOGIQUE
    // ==========================================
    // Activation globale du buffer 74AHCT125 (INDISPENSABLE)
    // Si la syntaxe globalEnable n'existe pas dans ta version, il faudra la configurer en sortie programmable.
    // engineConfiguration->idler1Pin = Gpio::PE8;            // GLOBAL_ENABLE (Assigné temporairement ici si pas de pin dédiée)
    
    engineConfiguration->injectionPins[0] = Gpio::PB8;     // INJ-1_4_IN-ESP
    engineConfiguration->injectionPins[1] = Gpio::PB9;     // INJ-2_3_IN-ESP
    engineConfiguration->ignitionPins[0] = Gpio::PC7;      // IGN_1_IN-ESP
    engineConfiguration->ignitionPins[1] = Gpio::PC6;      // IGN_2_IN-ESP

    // ==========================================
    // 7. TRIGGER / CAPTEUR DE VILEBREQUIN
    // ==========================================
    engineConfiguration->triggerInputPins[0] = Gpio::PE11; // CRANK_OUT-ESP

    // ==========================================
    // 8. CONTRÔLE RALENTI (MOTEUR PAS-À-PAS / DRV8825)
    // ==========================================
    engineConfiguration->stepperEnablePin = Gpio::PD13;    // NENBL-ESP
    engineConfiguration->stepperDirPin = Gpio::PD14;       // DIR-ESP
    engineConfiguration->stepperStepPin = Gpio::PD12;      // STEP-ESP

    // ==========================================
    // 9. GESTION ANNEXE (CLIMATISATION, CANISTER, WBO)
    // ==========================================
    // Climatisation et Purge
    engineConfiguration->acSwitchPin = Gpio::PA4;          // AC_SW
    engineConfiguration->acRelayPin = Gpio::PE1;           // AC_RELAY_IN-ESP
    engineConfiguration->canisterPin = Gpio::PE2;          // CANISTER_IN-ESP

    // Sonde Large Bande (Internal CJ125 ou circuit discret)
    engineConfiguration->wboHeaterPin = Gpio::PB14;        // WBO_HEAT_ESP
    // Les broches de pompe WBO nécessitent une implémentation logicielle spécifique
    // engineConfiguration->wboPumpPin = Gpio::PC8;        // WBO_PUMP_PWM-STM
    // engineConfiguration->wboNernstPin = Gpio::PC9;      // WBO_NERNST_AC-STM

    // ==========================================
    // 10. AUTRES CONFIGURATIONS
    // ==========================================
    engineConfiguration->communicationLedPin = Gpio::Unassigned;
}