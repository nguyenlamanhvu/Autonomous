#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "MPU9250.h"
#include "mpu9250_app.h"

extern imu_9250_t* imu_9250_0;
extern Struct_Angle Angle;

void setup(void);
void loop(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAINPP_H_ */
