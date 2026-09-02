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
    engineConfiguration->scriptSerialRxPin = Gpio::C10;   // STM_RX3
    engineConfiguration->scriptSerialTxPin = Gpio::C11;   // STM_TX3
    engineConfiguration->tunerStudioBaudRate = 115200;
    engineConfiguration->usbVbusPin = Gpio::A9;           // USB_VBUS_SENSE

    // ==========================================
    // 2. CAPTEURS ANALOGIQUES (ADC)
    // ==========================================
    engineConfiguration->map.sensorPin = Gpio::C0;        // MAP-EXT
    engineConfiguration->knockPin = Gpio::C1;             // Knock Sensor
    engineConfiguration->tpsPin = Gpio::C2;               // TPS Sensor
    engineConfiguration->cltPin = Gpio::B1;               // CLT_SENSOR
    engineConfiguration->iatPin = Gpio::C5;               // IAT-SENSOR
    engineConfiguration->vbattPin = Gpio::C4;             // Reference-Battery-V

    // ==========================================
    // 3. BUS CAN, VITESSE & FLEX FUEL
    // ==========================================
    engineConfiguration->canRxPin = Gpio::D0;             // CAN_RX
    engineConfiguration->canTxPin = Gpio::D1;             // CAN_TX
    engineConfiguration->vssPin = Gpio::B4;               // ADS_VSS_Sensor_STM
    engineConfiguration->flexSensorPin = Gpio::E9;        // Flex_Sensor_STM

    // ==========================================
    // 4. BUS SPI & EXPANDEUR (MCP23S17)
    // ==========================================
    engineConfiguration->spi1sckPin = Gpio::A5;           // MCP-SCK
    engineConfiguration->spi1misoPin = Gpio::A6;          // MCP-MISO
    engineConfiguration->spi1mosiPin = Gpio::A7;          // MCP-MOSI
    engineConfiguration->mcpCsPin = Gpio::PE3;            // IO_CS

    // ==========================================
    // 5. GESTION DE L'ALIMENTATION & DÉMARRAGE
    // ==========================================
    engineConfiguration->ignitionKeyPin = Gpio::B12;      // IGN_KEY_SENSE
    engineConfiguration->mainRelayPin = Gpio::E0;         // MAIN_RLY_IN-ESP
    engineConfiguration->fuelPumpPin = Gpio::D2;          // FUELPUMP_IN-ESP
    engineConfiguration->tachoPin = Gpio::A8;             // TACHO-ESP

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
    // 8. CONTRÔLE RALENTI (MOTEUR PAS-À-PAS / DRV8825)
    // ==========================================
    engineConfiguration->stepperEnablePin = Gpio::D13;    // NENBL-ESP
    engineConfiguration->idle.stepperDirectionPin = Gpio::D14; // DIR-ESP (Chemin correct struct idle)
    engineConfiguration->idle.stepperStepPin = Gpio::D12;      // STEP-ESP (Chemin correct struct idle)

    // ==========================================
    // 9. GESTION ANNEXE (CLIMATISATION, WBO)
    // ==========================================
    engineConfiguration->acSwitch = Gpio::A4;             // AC_SW (Nom corrigé sans 'Pin')
    engineConfiguration->acRelayPin = Gpio::E1;           // AC_RELAY_IN-ESP
    // engineConfiguration->canisterPin = Gpio::E2;       // Canister (géré via sortie programmable si besoin)

    // Sonde Large Bande (Chauffage)
    engineConfiguration->o2heaterPin = Gpio::B14;         // WBO_HEAT_ESP (Nom corrigé en o2heaterPin)
    // engineConfiguration->wboPumpPin = Gpio::PC8;        // WBO_PUMP_PWM-STM (Optionnel / Avancé)
    // engineConfiguration->wboNernstPin = Gpio::PC9;      // WBO_NERNST_AC-STM (Optionnel / Avancé)
}