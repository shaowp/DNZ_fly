#ifndef __RC_PROCESSING_H
#define __RC_PROCESSING_H

#include "sys.h"

typedef struct RC_DATA_RECEIER
{
	u32 thr;
	u32 pit;
	u32 yaw;
	u32 rol;
} Struct_RC_DATA;

extern Struct_RC_DATA RC_data;

#endif
