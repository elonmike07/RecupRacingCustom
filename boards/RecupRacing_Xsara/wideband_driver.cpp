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

// Signature standard et valide pour adccallback_t dans ChibiOS
static void adccallback(ADCDriver *adcp) {
    (void)adcp;
    palTogglePad(GPIOC, 9); // PC9 NERNST_AC (synchronisation AC)

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

// Initialisation complète avec tous les champs requis (sqr2 et sqr3 inclus)
static const ADCConversionGroup adcgrpcfg = {
    true,
    (uint16_t)ADC_GRP_NUM_CHANNELS,
    adccallback,
    NULL,
    0,
    ADC_CR2_SWSTART,
    0,
    ADC_SMPR2_SMP_AN2(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_56),
    (uint16_t)ADC_SQR1_NUM_CH(ADC_GRP_NUM_CHANNELS),
    0,                                    // sqr2
    ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3) // sqr3 (PA2 & PA3)
};

// Configuration PWMD12 (TIM12) pour le chauffage (PB14 - TIM12_CH1)
static PWMConfig pwmcfg_heater = {
    100000,                               // frequency (100 Hz)
    1000,                                 // period
    NULL,                                 // callback
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},   // CH1 (PB14 WBO HEAT)
        {PWM_OUTPUT_DISABLED, NULL},      // CH2
        {PWM_OUTPUT_DISABLED, NULL},      // CH3
        {PWM_OUTPUT_DISABLED, NULL}       // CH4
    },
    0,                                    // dier
    0                                     // cr2
};

// Configuration PWMD8 (TIM8) pour la pompe (PC8 - TIM8_CH3)
static PWMConfig pwmcfg_pump = {
    10000000,                             // frequency (10 MHz)
    1000,                                 // period
    NULL,                                 // callback
    {
        {PWM_OUTPUT_DISABLED, NULL},      // CH1
        {PWM_OUTPUT_DISABLED, NULL},      // CH2
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},   // CH3 (PC8 WBO PUMP PWM)
        {PWM_OUTPUT_DISABLED, NULL}       // CH4
    },
    0,                                    // dier
    0                                     // cr2
};

enum class HeaterState { Preheat, WarmupRamp, ClosedLoop, Stopped };

static THD_WORKING_AREA(waWidebandThread, 1024);
static THD_FUNCTION(WidebandThread, arg) {
    (void)arg;
    chRegSetThreadName("WBO Controller");
    adcStartConversion(&ADCD3, &adcgrpcfg, samples, ADC_GRP_BUF_DEPTH);

    HeaterState heaterState = HeaterState::Preheat;
    systime_t stateStartTime = chVTGetSystemTime();
    float rampVoltage = 7.0f, integrator = 0.0f, prevError = 0.0f, pumpDuty = 500.0f;

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

        // Pilotage du chauffage sur PWMD12 (Canal 0 / CH1 pour PB14)
        pwmEnableChannel(&PWMD12, 0, (pwmcnt_t)(dutyFraction * 1000.0f));

        if (heaterState == HeaterState::ClosedLoop) {
            float nernstErr = nernstDc - NERNST_TARGET;
            pumpDuty += nernstErr * 25.0f;
            if (pumpDuty > 950.0f) pumpDuty = 950.0f;
            if (pumpDuty < 50.0f)  pumpDuty = 50.0f;
            
            // Pilotage de la pompe sur PWMD8 (Canal 2 / CH3 pour PC8)
            pwmEnableChannel(&PWMD8, 2, (pwmcnt_t)pumpDuty);

            float ratio = -1000.0f / (PUMP_CURRENT_SENSE_GAIN * LSU_SENSE_R);
            float currentLambda = CalculateLambda(pumpCurrentSenseVoltage * ratio);
            
            // Assignation de la valeur Lambda via l'API rusEFI standard
            Sensor::set(SensorType::Lambda1, currentLambda);
        } else {
            pwmEnableChannel(&PWMD8, 2, 500); 
        }

        chThdSleepMilliseconds(50); 
    }
}

void initWidebandDriver(void) {
    palSetPadMode(GPIOC, 9, PAL_MODE_OUTPUT_PUSHPULL);   // PC9 NERNST_AC
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);     // PA2 WBO UR (ADC3_IN2)
    palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);     // PA3 WBO UA (ADC3_IN3)

    palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(9));    // PB14 WBO HEAT (TIM12_CH1 - AF9)
    palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(3));     // PC8 WBO PUMP PWM (TIM8_CH3 - AF3)

    adcStart(&ADCD3, NULL);
    pwmStart(&PWMD12, &pwmcfg_heater);
    pwmStart(&PWMD8, &pwmcfg_pump);

    chThdCreateStatic(waWidebandThread, sizeof(waWidebandThread), NORMALPRIO + 1, WidebandThread, NULL);
}

#else

// Stub vide pour le bootloader
void initWidebandDriver(void) {}

#endif