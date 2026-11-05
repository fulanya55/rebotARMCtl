/*
 * The little of a C library the firmware needs. Target builds are freestanding: there
 * is no newlib on this toolchain, and nothing here wants one.
 */
#include <stddef.h>
#include <stdint.h>

void *memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    while (n--) *d++ = *s++;
    return dst;
}

void *memmove(void *dst, const void *src, size_t n)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        d += n;
        s += n;
        while (n--) *--d = *--s;
    }
    return dst;
}

void *memset(void *dst, int value, size_t n)
{
    uint8_t *d = dst;
    while (n--) *d++ = (uint8_t)value;
    return dst;
}

int memcmp(const void *a, const void *b, size_t n)
{
    const uint8_t *x = a, *y = b;
    for (; n--; x++, y++) {
        if (*x != *y) return (int)*x - (int)*y;
    }
    return 0;
}

size_t strlen(const char *s)
{
    size_t n = 0;
    while (s[n]) n++;
    return n;
}

long lroundf(float x)
{
    return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

float logf(float x)
{
    /* Natural log, for the servo thermistor reading: a coarse value is plenty. Split
     * off the binary exponent, then use a short series around 1. */
    union {
        float f;
        uint32_t u;
    } v = {x};
    if (x <= 0.0f) return -1e30f;
    int exponent = (int)((v.u >> 23) & 0xFFu) - 127;
    v.u = (v.u & 0x007FFFFFu) | 0x3F800000u;
    float m = v.f;
    if (m > 1.41421356f) {
        m *= 0.5f;
        exponent += 1;
    }
    float t = (m - 1.0f) / (m + 1.0f);
    float t2 = t * t;
    float series = t * (2.0f + t2 * (0.666666667f + t2 * (0.4f + t2 * 0.285714286f)));
    return series + (float)exponent * 0.693147181f;
}
