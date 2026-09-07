#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Flag de synchronisation pour éviter la Race Condition au démarrage
extern volatile bool wboPwmInitialized;

void initWidebandDriver(void);

#ifdef __cplusplus
}
#endif