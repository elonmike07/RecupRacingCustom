#include "pch.h"
#include "hal.h"

// Le bootloader désactive les pilotes ADC et PWM dans son halconf.h.
// On compile tout le driver Wideband uniquement si ces HAL sont activés (Firmware Principal).
#if (defined(HAL_USE_ADC) && HAL_USE_ADC == TRUE) && (defined(HAL_USE_PWM) && HAL_USE_PWM == TRUE)

#include "wideband_driver.h"
#include <rusefi/interpolation.h>
#include "ch.h"
#include "osal.h"
#include "board.h"
#include "hal_adc.h"
#include "hal_pwm.h"
#include "sensor.h"

#define ADC_GRP_NUM_CHANNELS   2
#define ADC_GRP_BUF_DEPTH      4

static adcsample_t samples[ADC_GRP_NUM_CHANNELS * ADC_GRP_BUF_DEPTH];

static constexpr float ESR_SENSE_ALPHA = 0.002f;
static constexpr float PUMP_FILTER_ALPHA = 0.02f;
static constexpr float PUMP_CURRENT_SENSE_GAIN = 10.0f;
static constexpr float LSU_SENSE_R = 61.9f;
static constexpr float NERNST_TARGET = 0.45f;
static constexpr float VCC_VOLTS = 3.3f;
static constexpr float ESR_SUPPLY_R = 22000.0f; 
static constexpr float VM_RESISTOR_VALUE = 0.0f;
static constexpr float TARGET_ESR = 300.0f;     

static volatile float nernstDc = 0.45f;
static volatile float nernstAc = 0.0f;
static volatile float pumpCurrentSenseVoltage = 0.0f;

static float r_1 __attribute__((unused)) = 0.0f;
static float r_2 __attribute__((unused)) = 0.0f;
static float r_3 __attribute__((unused)) = 0.0f;

static inline float f_abs(float x) { return x > 0.0f ? x : -x; }

static float GetPhiLsu49(float pumpCurrent) {
    if (pumpCurrent > 1.11f) return 0.5f;
    if (pumpCurrent < -3.5f) return 1.0f / 0.5f;
    float gain = pumpCurrent < 0.0f ? -0.28299f : -0.44817f;
    return gain * pumpCurrent + 0.99559f;
}

static float CalculateLambda(float pumpCurrentmA) {
    float phi = GetPhiLsu49(pumpCurrentmA);
    if (phi <= 0.0001f) return 1.0f;
    return 1.0f / phi;
}

static void adccallback(ADCDriver *adcp) {
    (void)adcp;
    palTogglePad(GPIOC, 9); 

    uint32_t sumNernst = 0, sumPump = 0;
    for (size_t i = 0; i < ADC_GRP_BUF_DEPTH; i++) {
        sumNernst += samples[i * ADC_GRP_NUM_CHANNELS + 0];
        sumPump   += samples[i * ADC_GRP_NUM_CHANNELS + 1];
    }

    r_1 = ((float)sumNernst / ADC_GRP_BUF_DEPTH) * (3.3f / 4095.0f);
    float pumpV = ((float)sumPump / ADC_GRP_BUF_DEPTH) * (3.3f / 4095.0f);

    float r2_opposite_phase = (r_1 + r_3) * 0.5f;
    float nernstAcLocal = f_abs(r2_opposite_phase - r_2);
    nernstDc = (r2_opposite_phase + r_2) * 0.5f;

    nernstAc = (1.0f - ESR_SENSE_ALPHA) * nernstAc + (ESR_SENSE_ALPHA * nernstAcLocal);
    pumpCurrentSenseVoltage = (1.0f - PUMP_FILTER_ALPHA) * pumpCurrentSenseVoltage + (PUMP_FILTER_ALPHA * pumpV);

    r_3 = r_2; r_2 = r_1;
}

static const ADCConversionGroup adcgrpcfg = {
    true,                                                                  
    (uint16_t)ADC_GRP_NUM_CHANNELS,                                        
    adccallback,                                                           
    nullptr,                                                               
    0,                                                                     
    ADC_CR2_SWSTART,                                                       
    0,                                                                     
    ADC_SMPR2_SMP_AN2(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_56),   
    (uint16_t)ADC_SQR1_NUM_CH(ADC_GRP_NUM_CHANNELS),                       
    0,                                                                     
    ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3),     
    0,                                                                     
    0                                                                      
};

static PWMConfig pwmcfg_heater = {
    .frequency = 100000,
    .period    = 1000,
    .callback  = nullptr,
    .channels  = {
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr}, 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}, 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}, 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}  
    },
    .cr2       = 0,
    .bdtr      = 0,
    .dier      = 0
};

// Configuration pour le Timer 3 (PWMD3) corrigée : 1 MHz / période 100 = 10 kHz (stable sur APB1 à 84MHz)
static PWMConfig pwmcfg_pump = {
    .frequency = 1000000,
    .period    = 100,
    .callback  = nullptr,
    .channels  = {
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}, 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}, 
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr}, // Channel 3 actif sur PC8
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}  
    },
    .cr2       = 0,
    .bdtr      = 0,
    .dier      = 0
};

enum class HeaterState { Preheat, WarmupRamp, ClosedLoop, Stopped };

static THD_WORKING_AREA(waWidebandThread, 2048);
static THD_FUNCTION(WidebandThread, arg) {
    (void)arg;
    chRegSetThreadName("WBO Controller");
    
    chThdSleepMilliseconds(3000);

    // Initialisation globale
    adcStart(&ADCD3, NULL);
    pwmStart(&PWMD12, &pwmcfg_heater);
    pwmStart(&PWMD3, &pwmcfg_pump);
    adcStartConversion(&ADCD3, &adcgrpcfg, samples, ADC_GRP_BUF_DEPTH);

    HeaterState heaterState = HeaterState::Preheat;
    systime_t stateStartTime = chVTGetSystemTime();
    
    float rampVoltage = 7.0f;
    float integrator = 0.0f;
    float prevError = 0.0f;
    float pumpDuty = 50.0f; // Échelle 0-100 pour la période de 100
    float currentLambda = 1.0f; 

    while (true) {
        systime_t now = chVTGetSystemTime();
        float stateElapsedSec = (float)TIME_I2MS(chVTTimeElapsedSinceX(stateStartTime)) / 1000.0f;
        
        float currentNernstAc = nernstAc;
        float sensorEsr = 5000.0f;

        if (currentNernstAc > 0.001f && (VCC_VOLTS / currentNernstAc) > 1.0f) {
            sensorEsr = (ESR_SUPPLY_R / ((VCC_VOLTS / currentNernstAc) - 1.0f)) - VM_RESISTOR_VALUE;
        }

        float vBatt = Sensor::get(SensorType::BatteryVoltage).value_or(13.5f);
        if (vBatt < 3.0f) vBatt = 13.5f;

        float targetHeaterVoltage = 0.0f;

        switch (heaterState) {
            case HeaterState::Preheat:
                targetHeaterVoltage = 2.0f; 
                if (stateElapsedSec >= 5.0f || sensorEsr < 400.0f) {
                    heaterState = HeaterState::WarmupRamp;
                    stateStartTime = now;
                    rampVoltage = 7.0f;
                }
                break;
            case HeaterState::WarmupRamp:
                if (rampVoltage < 12.0f) rampVoltage += (0.4f * 0.05f);
                targetHeaterVoltage = rampVoltage;
                if (sensorEsr <= 350.0f) {
                    heaterState = HeaterState::ClosedLoop;
                    stateStartTime = now;
                } else if (stateElapsedSec >= 60.0f) {
                    heaterState = HeaterState::Stopped;
                }
                break;
            case HeaterState::ClosedLoop: {
                float error = TARGET_ESR - sensorEsr; 
                integrator += error * 0.05f;
                if (integrator > 3.0f) integrator = 3.0f;
                if (integrator < -3.0f) integrator = -3.0f;
                float derivative = (error - prevError) / 0.05f;
                prevError = error;
                float pidOutput = (0.3f * error) + (0.3f * integrator) + (0.01f * derivative);
                targetHeaterVoltage = 7.5f - pidOutput; 
                break;
            }
            case HeaterState::Stopped:
            default:
                targetHeaterVoltage = 0.0f;
                break;
        }

        if (targetHeaterVoltage > 12.0f) targetHeaterVoltage = 12.0f;
        if (targetHeaterVoltage < 0.0f)  targetHeaterVoltage = 0.0f;

        float voltageRatio = (vBatt < 1.0f) ? 0.0f : (targetHeaterVoltage / vBatt);
        float dutyFraction = voltageRatio * voltageRatio;
        if (dutyFraction > 1.0f) dutyFraction = 1.0f;
        if (vBatt >= 23.0f) dutyFraction = 0.0f; 

        // PWM Chauffage (TIM12)
        pwmEnableChannel(&PWMD12, 0, (pwmcnt_t)(dutyFraction * 1000.0f));

        if (heaterState == HeaterState::ClosedLoop) {
            float nernstErr = nernstDc - NERNST_TARGET;
            pumpDuty += nernstErr * 2.5f; // Ajusté pour l'échelle 0-100
            if (pumpDuty > 95.0f) pumpDuty = 95.0f;
            if (pumpDuty < 5.0f)  pumpDuty = 5.0f;
            
            // PWM Pompe (TIM3) - Échelle 0-100 pour la période de 100
            pwmEnableChannel(&PWMD3, 2, (pwmcnt_t)pumpDuty);

            float ratio = -1000.0f / (PUMP_CURRENT_SENSE_GAIN * LSU_SENSE_R);
            currentLambda = CalculateLambda(pumpCurrentSenseVoltage * ratio);
            
            Sensor::setMockValue(SensorType::Lambda1, currentLambda);
        } else {
            // PWM Pompe en mode attente (Duty à 50% -> valeur 50 pour une période de 100)
            pwmEnableChannel(&PWMD3, 2, 50); 
            currentLambda = 1.0f; 
        }

        chThdSleepMilliseconds(50); 
    }
}

void initWidebandDriver(void) {
    palSetPadMode(GPIOC, 9, PAL_MODE_OUTPUT_PUSHPULL);   
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);      
    palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);      
    
    // TIM12_CH1 pour le Heater sur PB14
    palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(9));    
    
    // TIM3_CH3 pour la pompe sur PC8 (AF2)
    palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(2));     

    chThdCreateStatic(waWidebandThread, sizeof(waWidebandThread), NORMALPRIO + 1, WidebandThread, NULL);
}

#else
void initWidebandDriver(void) {}
#endif