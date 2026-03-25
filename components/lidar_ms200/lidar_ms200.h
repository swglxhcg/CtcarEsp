#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "ms200.h"


void Lidar_Ms200_Init(void);
uint16_t Lidar_Ms200_Get_Distance(uint16_t angle);
void Lidar_Ms200_Get_Data(ms200_data_t* out_data);

#ifdef __cplusplus
}
#endif
