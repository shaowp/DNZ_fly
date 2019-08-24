#ifndef _KALMAN_H
#define _KALMAN_H

struct _1_ekf_filter
{
	float LastP;
	float Now_P;
	float out;
	float Kg;
	float Q;
	float R;
};

extern void kalman_1(struct _1_ekf_filter *ekf, float input);
#endif
