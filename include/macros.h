#ifndef MACROS_H
#define MACROS_H

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(x, low, high) (MIN(MAX(x, low), high))
#define MAP(x, in_min, in_max, out_min, out_max)                               \
  (((float)(x - in_min) * (out_max - out_min)) /                               \
   ((float)((in_max - in_min) + out_min)))

#define COUNT_OF(x)                                                            \
  ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

#endif
