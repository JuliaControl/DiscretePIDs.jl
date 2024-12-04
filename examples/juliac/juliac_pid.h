#ifndef JULIAC_PID_H
#define JULIAC_PID_H

#ifdef __cplusplus
extern "C" {
#endif

// Type definitions for function pointers
typedef void (*jl_init_with_image_t)(const char *bindir, const char *sysimage);
typedef double (*calculate_control_t)(double r, double y, double uff);
typedef void (*set_K_t)(double K, double r, double y);
typedef void (*set_Ti_t)(double Ti);
typedef void (*set_Td_t)(double Td);
typedef void (*reset_state_t)();

#ifdef __cplusplus
}
#endif

#endif // JULIAC_PID_H