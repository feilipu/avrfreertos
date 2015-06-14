#ifndef _FT_HAL_UTILS_H_
#define _FT_HAL_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define sq(v)				({ __typeof__ (v) _v = (v); _v * _v; })
#define SQ(v)				({ __typeof__ (v) _v = (v); _v * _v; })

#define ABS(x)				({ __typeof__ (x) _x = (x); _x > (0) ? _x : -_x; })

#define max(a,b)  			({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define MAX(a,b)  			({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })

#define min(a,b)  			({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define MIN(a,b)  			({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

#ifdef __cplusplus
}
#endif

#endif /* _FT_HAL_UTILS_H_ */

