#include "pch.h"
#include "hal.h"

#if (defined(HAL_USE_ADC) && HAL_USE_ADC == TRUE) && (defined(HAL_USE_PWM) && HAL_USE_PWM == TRUE)

#include "wideband_driver.h"
#include <rusefi/interpolation.h>
#include <cmath>
#include "ch.h"
#include "osal.h"
#include "board.h"
#include "hal_adc.h"
#include "hal_pwm.h"
#include "sensor.h"

#define ADC_GRP_NUM_CHANNELS    2
#define ADC_GRP_BUF_DEPTH       4

static adcsample_t samples[ADC_GRP_NUM_CHANNELS * ADC_GRP_BUF_DEPTH];

// ==========================================
// CONSTANTES ET TABLES DE CALIBRATION
// ==========================================
static constexpr float ESR_SENSE_ALPHA = 0.002f;
static constexpr float PUMP_FILTER_ALPHA = 0.02f;
static constexpr float DERIVATIVE_ALPHA = 0.1f; 
static constexpr float PUMP_CURRENT_SENSE_GAIN = 10.0f;
static constexpr float LSU_SENSE_R = 61.9f;
static constexpr float NERNST_TARGET = 0.45f;
static constexpr float VCC_VOLTS = 3.3f;
static constexpr float ESR_SUPPLY_R = 22000.0f; 
static constexpr float VM_RESISTOR_VALUE = 10.0f; 

// Masse Virtuelle (Fixe car générée matériellement par le REF3033 de haute précision)
static constexpr float VIRTUAL_GROUND = 1.65f;

static constexpr float TARGET_ESR = 300.0f;          
static constexpr float TARGET_TEMP = 780.0f;         

static const float lsu49TempBins[] =   {80,   100, 150, 200, 250, 300, 350, 400, 450, 550, 650, 800, 1000, 1200, 2500, 4500};
static const float lsu49TempValues[] = {1030, 972, 888, 840, 806, 780, 761, 744, 729, 703, 686, 665, 642,  628,  567,  500};

// ==========================================
// VARIABLES GLOBALES PARTAGÉES
// ==========================================
static volatile float nernstDc = 0.45f;
static volatile float nernstAc = 0.0f;
static volatile float pumpCurrentSenseVoltage = 0.0f;
static volatile float currentSensorTemp = 0.0f; 

static volatile uint32_t heaterThreadAliveCounter = 0;

// Verrou global d'arrêt d'urgence pour bloquer les appels HAL
static volatile bool eStopTriggered = false;

static float r_1 = 0.0f;
static float r_2 = 0.0f;
static float r_3 = 0.0f;

enum class HeaterState { Preheat, WarmupRamp, ClosedLoop, Stopped, Fault };
static volatile HeaterState heaterState = HeaterState::Stopped; 

static inline float f_abs(float x) { return x > 0.0f ? x : -x; }

// ==========================================
// ARRÊT MATÉRIEL D'URGENCE (SÉCURITÉ PARANO BARE-METAL)
// ==========================================
extern "C" void wboHardwareEmergencyStop(void) {
    // 0. Verrouillage logiciel : Empêche les threads de réactiver les PWM via la HAL
    eStopTriggered = true;

    // 1. Désactivation pure et dure des Timers via les registres CMSIS
    // TIM12 (Chauffage) : Désactivation des sorties et arrêt du compteur
    if (TIM12) {
        TIM12->CCER = 0; 
        TIM12->CR1 &= ~TIM_CR1_CEN; 
    }

    // TIM3 (Pompe et Nernst AC) : Désactivation des sorties et arrêt du compteur
    if (TIM3) {
        TIM3->CCER = 0;
        TIM3->CR1 &= ~TIM_CR1_CEN;
    }

    // 2. Verrouillage matériel des buffers via GLOBAL_ENABLE (PE8)
    // Utilisation explicite de .u32 pour l'écriture atomique sur le registre BSRR
    if (GPIOE) {
        GPIOE->BSRR.u32 = (1U << 8); 
    }
}

static float GetPhiLsu49(float pumpCurrent) {
    if (pumpCurrent > 1.11f) return 0.5f;
    if (pumpCurrent < -3.5f) return 1.0f / 0.5f;
    float gain = pumpCurrent < 0.0f ? -0.28299f : -0.44817f;
    return gain * pumpCurrent + 0.99559f;
}

static float CalculateLambda(float pumpCurrentmA) {
    float phi = GetPhiLsu49(pumpCurrentmA);
    if (phi <= 0.0001f || std::isnan(phi)) return 1.0f;
    return 1.0f / phi;
}

// ==========================================
// LECTURE ADC SYNCHRONISÉE
// ==========================================
static void adccallback(ADCDriver *adcp) {
    (void)adcp;

    uint32_t sumNernst = 0, sumPump = 0;
    for (size_t i = 0; i < ADC_GRP_BUF_DEPTH; i++) {
        sumNernst += samples[i * ADC_GRP_NUM_CHANNELS + 0];
        sumPump   += samples[i * ADC_GRP_NUM_CHANNELS + 1];
    }

    // Calcul statique (Assumant VDDA = 3.3V)
    float absoluteNernst = ((float)sumNernst / ADC_GRP_BUF_DEPTH) * (3.3f / 4095.0f);
    float absolutePump   = ((float)sumPump / ADC_GRP_BUF_DEPTH) * (3.3f / 4095.0f);

    r_1 = absoluteNernst - VIRTUAL_GROUND; 
    float pumpV = absolutePump - VIRTUAL_GROUND; 

    // Calcul de l'ESR par soustraction de phase (Annulation offset DC)
    float r2_opposite_phase = (r_1 + r_3) * 0.5f;
    float nernstAcLocal = f_abs(r2_opposite_phase - r_2);
    float nernstDcLocal = (r2_opposite_phase + r_2) * 0.5f;

    float nernstAcFiltered = (1.0f - ESR_SENSE_ALPHA) * nernstAc + (ESR_SENSE_ALPHA * nernstAcLocal);
    float pumpVoltFiltered = (1.0f - PUMP_FILTER_ALPHA) * pumpCurrentSenseVoltage + (PUMP_FILTER_ALPHA * pumpV);

    chSysLockFromISR();
    nernstDc = nernstDcLocal;
    nernstAc = nernstAcFiltered;
    pumpCurrentSenseVoltage = pumpVoltFiltered;
    chSysUnlockFromISR();

    r_3 = r_2; r_2 = r_1;
}

// Configuration ADC3 déclenchée par TIM3_CH1 (EXTSEL = 7U) - Échantillonnage maximisé à 480 cycles
static const ADCConversionGroup adcgrpcfg = {
    true, (uint16_t)ADC_GRP_NUM_CHANNELS, adccallback, nullptr, 0, 
    ADC_CR2_EXTEN_RISING | (7U << ADC_CR2_EXTSEL_Pos), 0, 
    ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_480),   
    (uint16_t)ADC_SQR1_NUM_CH(ADC_GRP_NUM_CHANNELS), 0, 
    ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3), 0, 0 
};

// Initialisation complète des configurations PWM pour satisfaire -Werror=missing-field-initializers
static PWMConfig pwmcfg_heater = { 
    100000, 1000, nullptr, 
    {
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr},
        {.mode = PWM_OUTPUT_DISABLED, .callback = nullptr},
        {.mode = PWM_OUTPUT_DISABLED, .callback = nullptr},
        {.mode = PWM_OUTPUT_DISABLED, .callback = nullptr}
    }, 
    0, 0, 0 
};

static PWMConfig pwmcfg_pump = { 
    1000000, 100, nullptr, 
    {
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr},
        {.mode = PWM_OUTPUT_DISABLED, .callback = nullptr},
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr},
        {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = nullptr}
    }, 
    0, 0, 0 
};

// ==========================================
// THREAD 1 : CONTRÔLE DE LA POMPE (500 Hz)
// ==========================================
static THD_WORKING_AREA(waPumpThread, 1024);
static THD_FUNCTION(PumpThread, arg) {
    (void)arg;
    chRegSetThreadName("WBO Pump");

    float pumpDuty = 50.0f; 
    float currentLambda = 1.0f; 
    float pumpIntegrator = 0.0f;
    
    const float kP_pump = 50.0f; 
    const float kI_pump = 10000.0f; 
    const float dt = 0.002f; 

    while (true) {
        chSysLock();
         HeaterState localState = heaterState;
         float localNernstDc = nernstDc;
         float localPumpSense = pumpCurrentSenseVoltage;
         float localTemp = currentSensorTemp;
        chSysUnlock();

        if ((localState == HeaterState::ClosedLoop || localTemp >= (TARGET_TEMP - 50.0f)) && localState != HeaterState::Fault) {
            float nernstErr = localNernstDc - NERNST_TARGET;
            
            pumpIntegrator += nernstErr * kI_pump * dt;
            if (pumpIntegrator > 10.0f) pumpIntegrator = 10.0f;
            if (pumpIntegrator < -10.0f) pumpIntegrator = -10.0f;
            
            pumpDuty = 50.0f + (nernstErr * kP_pump) + pumpIntegrator;
            
            if (pumpDuty > 95.0f) pumpDuty = 95.0f;
            if (pumpDuty < 5.0f)  pumpDuty = 5.0f;
            
            if (!eStopTriggered) {
                pwmEnableChannel(&PWMD3, 2, (pwmcnt_t)pumpDuty); // TIM3_CH3 (Pompe)
            }

            float ratio = -1000.0f / (PUMP_CURRENT_SENSE_GAIN * LSU_SENSE_R);
            currentLambda = CalculateLambda(localPumpSense * ratio);
            Sensor::setMockValue(SensorType::Lambda1, currentLambda);
            
        } else {
            pumpIntegrator = 0.0f;
            
            if (!eStopTriggered && TIM3) {
                TIM3->CCER &= ~TIM_CCER_CC3E; // Désactive TIM3_CH3
            }
            
            currentLambda = (localState == HeaterState::Fault) ? 0.0f : 1.0f; 
            Sensor::setMockValue(SensorType::Lambda1, currentLambda); 
        }
        chThdSleepMilliseconds(2); 
    }
}

// ==========================================
// THREAD 2 : CONTRÔLE DU CHAUFFAGE (100 Hz)
// ==========================================
static THD_WORKING_AREA(waWidebandThread, 1024);
static THD_FUNCTION(WidebandThread, arg) {
    (void)arg;
    chRegSetThreadName("WBO Heater");
    
    systime_t stateStartTime = chVTGetSystemTime();
    
    float rampVoltage = 7.0f;
    float integrator = 0.0f;
    float prevError = 0.0f;
    float filteredDerivative = 0.0f;
    
    uint8_t overheatCounter = 0;
    uint8_t underheatCounter = 0;
    uint8_t openLoadCounter = 0;
    float batteryStableTimerSec = 0.0f; 

    while (true) {
        heaterThreadAliveCounter++; 

        systime_t now = chVTGetSystemTime();
        float stateElapsedSec = (float)TIME_I2MS(chVTTimeElapsedSinceX(stateStartTime)) / 1000.0f;
        
        chSysLock();
        float currentNernstAc = nernstAc;
        chSysUnlock();

        float sensorEsr = 5000.0f;

        if (currentNernstAc > 0.001f && !std::isnan(currentNernstAc) && !std::isinf(currentNernstAc)) {
            float ratioAc = VCC_VOLTS / currentNernstAc;
            if (ratioAc > 1.0001f) {
                sensorEsr = (ESR_SUPPLY_R / (ratioAc - 1.0f)) - VM_RESISTOR_VALUE;
            }
        }

        if (sensorEsr < 10.0f || sensorEsr > 5000.0f || std::isnan(sensorEsr)) {
            sensorEsr = 5000.0f;
        }

        float computedTemp = interpolate2d(sensorEsr, lsu49TempBins, lsu49TempValues);

        chSysLock();
        currentSensorTemp = computedTemp;
        chSysUnlock();

        auto vBattOpt = Sensor::get(SensorType::BatteryVoltage);
        auto rpmOpt = Sensor::get(SensorType::Rpm);
        auto cltOpt = Sensor::get(SensorType::Clt);

        float vBatt = vBattOpt.value_or(0.0f);
        float rpm = rpmOpt.value_or(0.0f);
        float clt = cltOpt.value_or(20.0f); 
        
        if ((!vBattOpt || vBatt < 8.5f || rpm < 350.0f) && heaterState != HeaterState::Fault) {
            heaterState = HeaterState::Stopped;
            batteryStableTimerSec = 0.0f; 
        }

        float targetHeaterVoltage = 0.0f;

        if (sensorEsr >= 4500.0f) {
            if (++openLoadCounter > 50) heaterState = HeaterState::Fault;
        } else if (sensorEsr <= 20.0f) {
            if (++openLoadCounter > 10) heaterState = HeaterState::Fault;
        } else {
            if (openLoadCounter > 0) openLoadCounter--;
        }

        switch (heaterState) {
            case HeaterState::Preheat: {
                targetHeaterVoltage = 2.0f; 
                float requiredPreheatTime = (clt < 60.0f) ? 20.0f : 5.0f;

                if (stateElapsedSec >= requiredPreheatTime || computedTemp > (TARGET_TEMP - 100.0f)) {
                    heaterState = HeaterState::WarmupRamp;
                    stateStartTime = now;
                    rampVoltage = 7.0f;
                }
                break;
            }
                
            case HeaterState::WarmupRamp:
                if (rampVoltage < 12.0f) rampVoltage += (0.4f * 0.01f);
                targetHeaterVoltage = rampVoltage;
                
                if (computedTemp >= (TARGET_TEMP - 30.0f)) {
                    heaterState = HeaterState::ClosedLoop;
                    stateStartTime = now;
                    integrator = 0.0f; 
                    filteredDerivative = 0.0f;
                } else if (stateElapsedSec >= 60.0f) {
                    heaterState = HeaterState::Fault; 
                }
                break;
                
            case HeaterState::ClosedLoop: {
                if (computedTemp > (TARGET_TEMP + 100.0f)) {
                    if (++overheatCounter > 20) heaterState = HeaterState::Fault;
                } else {
                    if (overheatCounter > 0) overheatCounter--;
                }
                
                if (computedTemp < (TARGET_TEMP - 100.0f)) {
                    if (++underheatCounter > 20) heaterState = HeaterState::Fault;
                } else {
                    if (underheatCounter > 0) underheatCounter--;
                }

                float error = TARGET_ESR - sensorEsr; 
                integrator += error * 0.01f; 
                
                if (integrator > 3.0f) integrator = 3.0f;
                if (integrator < -3.0f) integrator = -3.0f;
                
                float rawDerivative = (error - prevError) / 0.01f;
                filteredDerivative = (1.0f - DERIVATIVE_ALPHA) * filteredDerivative + (DERIVATIVE_ALPHA * rawDerivative);
                prevError = error;
                
                float pidOutput = (0.3f * error) + (0.3f * integrator) + (0.01f * filteredDerivative);
                targetHeaterVoltage = 7.5f - pidOutput; 
                break;
            }
            
            case HeaterState::Fault:
                targetHeaterVoltage = 0.0f;
                integrator = 0.0f;
                break; 

            case HeaterState::Stopped:
            default:
                targetHeaterVoltage = 0.0f;
                if (vBatt >= 12.2f && rpm >= 450.0f) {
                    batteryStableTimerSec += 0.01f; 
                    if (batteryStableTimerSec >= 4.0f) {
                        heaterState = HeaterState::Preheat;
                        stateStartTime = now;
                        batteryStableTimerSec = 0.0f;
                    }
                } else {
                    batteryStableTimerSec = 0.0f; 
                }
                break;
        }

        if (targetHeaterVoltage > 12.0f) targetHeaterVoltage = 12.0f;
        if (targetHeaterVoltage < 0.0f)  targetHeaterVoltage = 0.0f;

        float voltageRatio = (vBatt < 1.0f) ? 0.0f : (targetHeaterVoltage / vBatt);
        float dutyFraction = voltageRatio * voltageRatio;
        if (dutyFraction > 1.0f) dutyFraction = 1.0f;
        if (vBatt >= 23.0f || heaterState == HeaterState::Fault) dutyFraction = 0.0f; 

        if (!eStopTriggered) {
            pwmEnableChannel(&PWMD12, 0, (pwmcnt_t)(dutyFraction * 1000.0f));
        }
        chThdSleepMilliseconds(10); 
    }
}

// ==========================================
// THREAD 3 : WATCHDOG LOGICIEL
// ==========================================
static THD_WORKING_AREA(waWboWatchdogThread, 256);
static THD_FUNCTION(WboWatchdogThread, arg) {
    (void)arg;
    chRegSetThreadName("WBO Watchdog");
    uint32_t lastCounter = 0;

    while (true) {
        chThdSleepMilliseconds(500); 

        if (heaterThreadAliveCounter == lastCounter) {
            wboHardwareEmergencyStop(); 
            heaterState = HeaterState::Fault;
        }
        lastCounter = heaterThreadAliveCounter;
    }
}

void initWidebandDriver(void) {
    palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(2)); // NERNST AC (TIM3_CH4)
    palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(2)); // PUMP PWM (TIM3_CH3)
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);      
    palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);      
    palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(9));    

    adcStart(&ADCD3, NULL);
    
    pwmStart(&PWMD12, &pwmcfg_heater);
    pwmStart(&PWMD3, &pwmcfg_pump);
    
    // Utilisation de la macro CMSIS standard pour le mode centré
    PWMD3.tim->CR1 |= TIM_CR1_CMS;
    
    pwmEnableChannel(&PWMD3, 0, 80); 
    pwmEnableChannel(&PWMD3, 3, 50); 
    pwmEnableChannel(&PWMD3, 2, 50); 

    adcStartConversion(&ADCD3, &adcgrpcfg, samples, ADC_GRP_BUF_DEPTH);

    chThdCreateStatic(waPumpThread, sizeof(waPumpThread), NORMALPRIO + 4, PumpThread, NULL);
    chThdCreateStatic(waWidebandThread, sizeof(waWidebandThread), NORMALPRIO + 3, WidebandThread, NULL);
    chThdCreateStatic(waWboWatchdogThread, sizeof(waWboWatchdogThread), NORMALPRIO + 5, WboWatchdogThread, NULL);
}

#else
void initWidebandDriver(void) {}
#endif