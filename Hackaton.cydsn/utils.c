/* ========================================
 * utils.c
 * ========================================
 */

// function to limit a value within a range
// constrain() ensures value stays within minVal and maxVal
int constrain_int(int x, int a, int b) 
{
  if (x < a) 
    return a;       // Return lower limit if x is below a
  else if (x > b) 
    return b;  // Return upper limit if x exceeds b
  else 
    return x;             // Return x if it's within the range
}

float constrain_float(float x, float a, float b) 
{
  if (x < a) 
    return a;       // Return lower limit if x is below a
  else if (x > b) 
    return b;  // Return upper limit if x exceeds b
  else 
    return x;             // Return x if it's within the range
}


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
int map_int(int value, int in_min, int in_max, int out_min, int out_max) 
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float map_float(float value, float in_min, float in_max, float out_min, float out_max) 
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/* [] END OF FILE */
