
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "maths.h"

#define sinPolyCoef3 -1.666665710e-1f // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5 8.333017292e-3f  // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9 2.600054768e-6f  // Double:  2.600054767890361277123254766503271638682e-6

float constrainf(float amt, float low, float high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

float sin_approx(float x)
{
	int32_t xint = x;
	if (xint < -32 || xint > 32)
		return 0.0f; // Stop here on error input (5 * 360 Deg)
	while (x > M_PIf)
		x -= (2.0f * M_PIf); // always wrap input angle to -PI..PI
	while (x < -M_PIf)
		x += (2.0f * M_PIf);
	if (x > (0.5f * M_PIf))
		x = (0.5f * M_PIf) - (x - (0.5f * M_PIf)); // We just pick -90..+90 Degree
	else if (x < -(0.5f * M_PIf))
		x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
	float x2 = x * x;
	return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
	return sin_approx(x + (0.5f * M_PIf));
}
