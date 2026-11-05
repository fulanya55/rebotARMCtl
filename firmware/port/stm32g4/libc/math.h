#ifndef RAC_MATH_H
#define RAC_MATH_H

/* These three become single FPU instructions on the Cortex-M4F. */
#define sqrtf(x) __builtin_sqrtf(x)
#define fabsf(x) __builtin_fabsf(x)
#define copysignf(x, y) __builtin_copysignf((x), (y))

long lroundf(float x);
float logf(float x);

#endif
