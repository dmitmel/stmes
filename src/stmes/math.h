#include <math.h>
#include <stm32f4xx_hal.h>

__STATIC_INLINE float lerpf(float a, float b, float t) {
  // <https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/>
  // Complies down to just two instructions: vfms + vfma.
  return fmaf(t, b, fmaf(-t, a, a));
}

__STATIC_INLINE float logerpf(float a, float b, float t) {
  return a * powf(b / a, t);
}

__STATIC_INLINE float mapf(float x, float a_start, float a_end, float b_start, float b_end) {
  return (x - a_start) / (a_end - a_start) * (b_end - b_start) + b_start;
}

__STATIC_INLINE float clampf(float x, float min, float max) {
  return fmaxf(min, fminf(max, x));
}
