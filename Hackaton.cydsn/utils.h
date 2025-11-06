/* ========================================
 * utils.h
 * ========================================
*/

#ifdef __cplusplus
extern "C" {
#endif


#define HIGH (1u)
#define LOW  (0u)

// function to limit a value within a range
// constrain() ensures value stays within minVal and maxVal
int constrain_int(int x, int a, int b);
float constrain_float(float x, float a, float b);


// Re-maps a number from one range to another. That is, a value of fromLow 
// would get mapped to toLow, a value of fromHigh to toHigh, values 
// in-between to values in-between, etc.
// Does not constrain values to within the range, because out-of-range 
// values are sometimes intended and useful. The constrain() function may 
// be used either before or after this function, if limits to the ranges are
// desired.
// Note that the "lower bounds" of either range may be larger or smaller than
// the "upper bounds", so the map() function may be used to reverse a range of
// numbers, for example.
int map_int(int value, int in_min, int in_max, int out_min, int out_max);
float map_float(float value, float in_min, float in_max, float out_min, float out_max);

#ifdef __cplusplus
}
#endif

/* [] END OF FILE */