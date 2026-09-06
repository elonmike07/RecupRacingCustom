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

// ==========================================
// CONSTANTES ET TABLES DE CALIBRATION OEM
// ==========================================
static constexpr float ESR_SENSE_ALPHA = 0.002f;
static constexpr float PUMP_FILTER_ALPHA = 0.02f;
static constexpr float PUMP_CURRENT_SENSE_GAIN = 10.0f;
static constexpr float LSU_SENSE_R = 61.9f;
static constexpr float NERNST_TARGET = 0.45f;
static constexpr float VCC_VOLTS = 3.3f;
static constexpr float ESR_SUPPLY_R = 22000.0f; 
static constexpr float VM_RESISTOR_VALUE = 0.0f;
static constexpr float VIRTUAL_GROUND = VCC_VOLTS / 2.0f;

static constexpr float TARGET_ESR = 300.0f;          // Cible de régulation PID (Ohms)
static constexpr float TARGET_TEMP = 780.0f;         // Cible thermique pour sécurités (°C)

// Tables d'interpolation Bosch LSU 4.9 (ESR vers Température en °C)
static const float lsu49TempBins[] =   {80,   100, 150, 200, 250, 300, 350, 400, 450, 550, 650, 800, 1000, 1200, 2500, 4500};
static const float lsu49TempValues[] = {1030, 972, 888, 840, 806, 780, 761, 744, 729, 703, 686, 665, 642,  628,  567,  500};

// ==========================================
// VARIABLES GLOBALES PARTAGÉES
// ==========================================
static volatile float nernstDc = 0.45f;
static volatile float nernstAc = 0.0f;
static volatile float pumpCurrentSenseVoltage = 0.0f;
static volatile float currentSensorTemp = 0.0f; 

static float r_1 __attribute__((unused)) = 0.0f;
static float r_2 __attribute__((unused)) = 0.0f;
static float r_3 __attribute__((unused)) = 0.0f;

enum class HeaterState { Preheat, WarmupRamp, ClosedLoop, Stopped };
static volatile HeaterState heaterState = HeaterState::Preheat;

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

// ==========================================
// INTERRUPTION ADC MATÉRIELLE (10 kHz)
// ==========================================
static void adccallback(ADCDriver *adcp) {
    (void)adcp;

    uint32_t sumNernst = 0, sumPump = 0;
    for (size_t i = 0; i < ADC_GRP_BUF_DEPTH; i++) {
        sumNernst += samples[i * ADC_GRP_NUM_CHANNELS + 0];
        sumPump   += samples[i * ADC_GRP_NUM_CHANNELS + 1];
    }

    float absoluteNernst = ((float)sumNernst / ADC_GRP_BUF_DEPTH) * (3.3f / 4095.0f);
    float absolutePump   = ((float)sumPump / ADC_GRP_BUF_DEPTH) * (3.3f / 4095.0f);

    r_1 = absoluteNernst - VIRTUAL_GROUND; 
    float pumpV = absolutePump - VIRTUAL_GROUND; 

    float r2_opposite_phase = (r_1 + r_3) * 0.5f;
    float nernstAcLocal = f_abs(r2_opposite_phase - r_2);
    nernstDc = (r2_opposite_phase + r_2) * 0.5f;

    nernstAc = (1.0f - ESR_SENSE_ALPHA) * nernstAc + (ESR_SENSE_ALPHA * nernstAcLocal);
    pumpCurrentSenseVoltage = (1.0f - PUMP_FILTER_ALPHA) * pumpCurrentSenseVoltage + (PUMP_FILTER_ALPHA * pumpV);

    r_3 = r_2; r_2 = r_1;
}

static const ADCConversionGroup adcgrpcfg = {
    true, (uint16_t)ADC_GRP_NUM_CHANNELS, adccallback, nullptr, 0, 
    ADC_CR2_EXTEN_RISING | (8U << ADC_CR2_EXTSEL_Pos), 0, 
    ADC_SMPR2_SMP_AN2(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_56),   
    (uint16_t)ADC_SQR1_NUM_CH(ADC_GRP_NUM_CHANNELS), 0, 
    ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3), 0, 0 
};

// CORRECTION DE COMPILATION (-Werror=missing-field-initializers)
static PWMConfig pwmcfg_heater = {
    100000, 1000, nullptr,
    { 
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr}, 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}, 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}, 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr} 
    }, 
    0, 0, 0
};

// CORRECTION DE COMPILATION (-Werror=missing-field-initializers)
static PWMConfig pwmcfg_pump = {
    1000000, 100, nullptr,
    { 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}, 
        {.mode = PWM_OUTPUT_DISABLED,    .callback = nullptr}, 
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr}, 
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr}  
    },
    STM32_TIM_CR2_MMS(2), 0, 0
};

// ==========================================
// THREAD 1 : CONTRÔLE DE LA POMPE (500 Hz)
// Stack sécurisé à 1024 octets
// ==========================================
static THD_WORKING_AREA(waPumpThread, 1024);
static THD_FUNCTION(PumpThread, arg) {
    (void)arg;
    chRegSetThreadName("WBO Pump");

    float pumpDuty = 50.0f; 
    float currentLambda = 1.0f; 
    float pumpIntegrator = 0.0f;
    
    const float kP_pump = 40.0f; 
    const float kI_pump = 80.0f; 
    const float dt = 0.002f; 

    while (true) {
        if (heaterState == HeaterState::ClosedLoop || currentSensorTemp >= (TARGET_TEMP - 50.0f)) {
            float nernstErr = nernstDc - NERNST_TARGET;
            
            pumpIntegrator += nernstErr * kI_pump * dt;
            if (pumpIntegrator > 40.0f) pumpIntegrator = 40.0f;
            if (pumpIntegrator < -40.0f) pumpIntegrator = -40.0f;
            
            pumpDuty = 50.0f + (nernstErr * kP_pump) + pumpIntegrator;
            
            if (pumpDuty > 95.0f) pumpDuty = 95.0f;
            if (pumpDuty < 5.0f)  pumpDuty = 5.0f;
            
            pwmEnableChannel(&PWMD3, 2, (pwmcnt_t)pumpDuty);

            float ratio = -1000.0f / (PUMP_CURRENT_SENSE_GAIN * LSU_SENSE_R);
            currentLambda = CalculateLambda(pumpCurrentSenseVoltage * ratio);
            Sensor::setMockValue(SensorType::Lambda1, currentLambda);
            
        } else {
            pumpIntegrator = 0.0f;
            pwmEnableChannel(&PWMD3, 2, 50); 
            currentLambda = 1.0f; 
        }
        chThdSleepMilliseconds(2); 
    }
}

// ==========================================
// THREAD 2 : CONTRÔLE DU CHAUFFAGE (20 Hz)
// ==========================================
static THD_WORKING_AREA(waWidebandThread, 1024);
static THD_FUNCTION(WidebandThread, arg) {
    (void)arg;
    chRegSetThreadName("WBO Heater");
    
    systime_t stateStartTime = chVTGetSystemTime();
    
    float rampVoltage = 7.0f;
    float integrator = 0.0f;
    float prevError = 0.0f;
    
    uint8_t overheatCounter = 0;
    uint8_t underheatCounter = 0;

    while (true) {
        systime_t now = chVTGetSystemTime();
        float stateElapsedSec = (float)TIME_I2MS(chVTTimeElapsedSinceX(stateStartTime)) / 1000.0f;
        
        float currentNernstAc = nernstAc;
        float sensorEsr = 5000.0f;

        if (currentNernstAc > 0.001f && (VCC_VOLTS / currentNernstAc) > 1.0f) {
            sensorEsr = (ESR_SUPPLY_R / ((VCC_VOLTS / currentNernstAc) - 1.0f)) - VM_RESISTOR_VALUE;
        }

        if (sensorEsr <= 5000.0f) {
            currentSensorTemp = interpolate2d(sensorEsr, lsu49TempBins, lsu49TempValues);
        } else {
            currentSensorTemp = 0.0f;
        }

        float vBatt = Sensor::get(SensorType::BatteryVoltage).value_or(13.5f);
        
        // PROTECTION CRANKING (Seuil à 6.0V pour tolérer la chute du démarreur sans couper intempestivement)
        if (vBatt < 6.0f) {
            heaterState = HeaterState::Stopped;
        }

        float targetHeaterVoltage = 0.0f;

        switch (heaterState) {
            case HeaterState::Preheat:
                targetHeaterVoltage = 2.0f; 
                if (stateElapsedSec >= 5.0f || currentSensorTemp > (TARGET_TEMP - 100.0f)) {
                    heaterState = HeaterState::WarmupRamp;
                    stateStartTime = now;
                    rampVoltage = 7.0f;
                }
                break;
                
            case HeaterState::WarmupRamp:
                if (rampVoltage < 12.0f) rampVoltage += (0.4f * 0.05f);
                targetHeaterVoltage = rampVoltage;
                
                if (currentSensorTemp >= (TARGET_TEMP - 30.0f)) {
                    heaterState = HeaterState::ClosedLoop;
                    stateStartTime = now;
                    integrator = 0.0f; 
                } else if (stateElapsedSec >= 60.0f) {
                    heaterState = HeaterState::Stopped; 
                }
                break;
                
            case HeaterState::ClosedLoop: {
                if (currentSensorTemp > 880.0f) {
                    if (++overheatCounter > 10) heaterState = HeaterState::Stopped;
                } else {
                    overheatCounter = 0;
                }
                
                if (currentSensorTemp < 680.0f) {
                    if (++underheatCounter > 10) heaterState = HeaterState::Stopped;
                } else {
                    underheatCounter = 0;
                }

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
                if (vBatt >= 12.5f) {
                    heaterState = HeaterState::Preheat;
                    stateStartTime = now;
                }
                break;
        }

        if (targetHeaterVoltage > 12.0f) targetHeaterVoltage = 12.0f;
        if (targetHeaterVoltage < 0.0f)  targetHeaterVoltage = 0.0f;

        float voltageRatio = (vBatt < 1.0f) ? 0.0f : (targetHeaterVoltage / vBatt);
        float dutyFraction = voltageRatio * voltageRatio;
        if (dutyFraction > 1.0f) dutyFraction = 1.0f;
        if (vBatt >= 23.0f) dutyFraction = 0.0f; 

        pwmEnableChannel(&PWMD12, 0, (pwmcnt_t)(dutyFraction * 1000.0f));
        chThdSleepMilliseconds(50); 
    }
}

void initWidebandDriver(void) {
    palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(2));      // PC9 = TIM3_CH4 (Injection AC Hardware)
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);      
    palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);      
    palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(9));    
    palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(2));     

    adcStart(&ADCD3, NULL);
    pwmStart(&PWMD12, &pwmcfg_heater);
    pwmStart(&PWMD3, &pwmcfg_pump);
    
    pwmEnableChannel(&PWMD3, 4, 50);

    adcStartConversion(&ADCD3, &adcgrpcfg, samples, ADC_GRP_BUF_DEPTH);

    chThdCreateStatic(waPumpThread, sizeof(waPumpThread), NORMALPRIO + 4, PumpThread, NULL);
    chThdCreateStatic(waWidebandThread, sizeof(waWidebandThread), NORMALPRIO + 3, WidebandThread, NULL);
}

#else
void initWidebandDriver(void) {}
#endif