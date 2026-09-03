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
// 1. CONFIGURATION MATÉRIELLE PAR DÉFAUT
// =========================================================================
static void customBoardDefaultConfiguration() {

    // ==========================================
    // 1. CAPTEURS ANALOGIQUES (ADC)
    // ==========================================
    engineConfiguration->map.sensor.hwChannel = EFI_ADC_0; // MAP-EXT (PC0 -> ADC0)
    engineConfiguration->tps1_1AdcChannel = EFI_ADC_2;    // TPS Sensor (PC2 -> ADC2)
    engineConfiguration->clt.adcChannel = EFI_ADC_9;      // CLT_SENSOR (PB1 -> ADC9)
    engineConfiguration->iat.adcChannel = EFI_ADC_15;     // IAT-SENSOR (PC5 -> ADC15)
    engineConfiguration->vbattAdcChannel = EFI_ADC_14;    // Battery Reference (PC4 -> ADC14)

    // ==========================================
    // 2. BUS CAN, VITESSE & FLEX FUEL
    // ==========================================
    engineConfiguration->canRxPin = Gpio::D0;                   // CAN_RX
    engineConfiguration->canTxPin = Gpio::D1;                   // CAN_TX
    engineConfiguration->vehicleSpeedSensorInputPin = Gpio::B4; // ADS_VSS_Sensor
    engineConfiguration->flexSensorPin = Gpio::E9;              // Flex_Sensor

    // ==========================================
    // 3. BUS SPI & EXPANDEUR (MCP23S17)
    // ==========================================
    engineConfiguration->is_enabled_spi_1 = true;
    engineConfiguration->spi1sckPin = Gpio::A5;                 // MCP-SCK
    engineConfiguration->spi1misoPin = Gpio::A6;                // MCP-MISO
    engineConfiguration->spi1mosiPin = Gpio::A7;                // MCP-MOSI

    // ==========================================
    // 4. GESTION DE L'ALIMENTATION & DÉMARRAGE
    // ==========================================
    engineConfiguration->ignitionKeyDigitalPin = Gpio::B12;     // IGN_KEY_SENSE
    engineConfiguration->mainRelayPin = Gpio::E0;               // MAIN_RLY_IN
    engineConfiguration->fuelPumpPin = Gpio::D2;                // FUELPUMP_IN
    engineConfiguration->tachOutputPin = Gpio::A8;              // TACHO

    // ==========================================
    // 5. INJECTION & ALLUMAGE
    // ==========================================
    engineConfiguration->injectionPins[0] = Gpio::B8;           // INJ-1_4
    engineConfiguration->injectionPins[1] = Gpio::B9;           // INJ-2_3
    engineConfiguration->ignitionPins[0] = Gpio::C7;            // IGN_1
    engineConfiguration->ignitionPins[1] = Gpio::C6;            // IGN_2

    // ==========================================
    // 6. TRIGGER (CAPTEUR PMH / VILEBREQUIN)
    // ==========================================
    engineConfiguration->triggerInputPins[0] = Gpio::E11;       // CRANK_OUT

    // ==========================================
    // 7. CONTRÔLE RALENTI (MOTEUR PAS-À-PAS)
    // ==========================================
    engineConfiguration->stepperEnablePin = Gpio::D13;         // NENBL
    engineConfiguration->idle.stepperDirectionPin = Gpio::D14; // DIR
    engineConfiguration->idle.stepperStepPin = Gpio::D12;      // STEP

    // ==========================================
    // 8. CLIMATISATION & SONDE LAMBDA (CHAUFFE)
    // ==========================================
    engineConfiguration->acSwitch = Gpio::A4;                   // AC_SW
    engineConfiguration->acRelayPin = Gpio::E1;                 // AC_RELAY_IN
    engineConfiguration->o2heaterPin = Gpio::B14;               // WBO_HEAT
}

// =========================================================================
// 2. INITIALISATION ET OVERRIDE DU RESET USINE
// =========================================================================
void setup_custom_board_overrides() {
    custom_board_DefaultConfiguration = customBoardDefaultConfiguration;
}