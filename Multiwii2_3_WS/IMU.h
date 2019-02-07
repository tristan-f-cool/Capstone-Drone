#ifndef IMU_H_
#define IMU_H_

#define BARO_TAB_SIZE   21

#if BARO
uint8_t getEstimatedAltitude();
#endif

void computeIMU();
int16_t _atan2(int32_t y, int32_t x);

#endif /* IMU_H_ */
