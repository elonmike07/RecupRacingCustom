#include "pch.h"
#include "board_overrides.h"
#include "wideband_driver.h" 

Gpio getCommsLedPin() { return Gpio::Unassigned; }
Gpio getRunningLedPin() { return Gpio::Unassigned; }
Gpio getWarningLedPin() { return Gpio::Unassigned; }

void setup_custom_board_overrides() {
    // ==========================================
    // CAPTEURS ANALOGIQUES (CORRIGÉS POUR STM32F4)
    // ==========================================
    engineConfiguration->map.sensor.hwChannel = EFI_ADC_10; // PC0 = ADC10
    engineConfiguration->tps1_1AdcChannel = EFI_ADC_12;     // PC2 = ADC12
    engineConfiguration->clt.adcChannel = EFI_ADC_9;        // PB1 = ADC9
    engineConfiguration->iat.adcChannel = EFI_ADC_15;       // PC5 = ADC15
    engineConfiguration->vbattAdcChannel = EFI_ADC_14;      // PC4 = ADC14

    // ==========================================
    // BUS CAN, VITESSE & FLEX FUEL
    // ==========================================
    engineConfiguration->canRxPin = Gpio::D0;
    engineConfiguration->canTxPin = Gpio::D1;
    engineConfiguration->vehicleSpeedSensorInputPin = Gpio::B4;
    engineConfiguration->flexSensorPin = Gpio::E9;

    // ==========================================
    // BUS SPI & EXPANDEUR
    // ==========================================
    engineConfiguration->is_enabled_spi_1 = true;
    engineConfiguration->spi1sckPin = Gpio::A5;
    engineConfiguration->spi1misoPin = Gpio::A6;
    engineConfiguration->spi1mosiPin = Gpio::A7;

    // ==========================================
    // ALIMENTATION & DÉMARRAGE
    // ==========================================
    engineConfiguration->ignitionKeyDigitalPin = Gpio::B12;
    engineConfiguration->mainRelayPin = Gpio::E0;
    engineConfiguration->fuelPumpPin = Gpio::D2;
    engineConfiguration->tachOutputPin = Gpio::A8;

    // ==========================================
    // INJECTION & ALLUMAGE
    // ==========================================
    engineConfiguration->injectionPins[0] = Gpio::B8;
    engineConfiguration->injectionPins[1] = Gpio::B9;
    engineConfiguration->ignitionPins[0] = Gpio::C7;
    engineConfiguration->ignitionPins[1] = Gpio::C6;

    // ==========================================
    // TRIGGER
    // ==========================================
    engineConfiguration->triggerInputPins[0] = Gpio::E11;

    // ==========================================
    // RALENTI
    // ==========================================
    engineConfiguration->stepperEnablePin = Gpio::D13;
    engineConfiguration->idle.stepperDirectionPin = Gpio::D14;
    engineConfiguration->idle.stepperStepPin = Gpio::D12;

    // ==========================================
    // CLIMATISATION & WIDEBAND
    // ==========================================
    engineConfiguration->acSwitch = Gpio::A4;
    engineConfiguration->acRelayPin = Gpio::E1;
    
    // Lancement du driver autonome de la sonde Lambda (DÉSACTIVÉ POUR TEST)
    // initWidebandDriver();
}