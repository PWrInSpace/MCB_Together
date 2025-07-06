#ifndef BMI08X_WRAPPER_H
#define BMI08X_WRAPPER_H

#include "bmi08_defs.h"
#include <stdbool.h>

typedef struct bmi08_dev bmi08_dev;

bool bmi08_wrapper_init(void);

bool bmi08_acc_data_ready(void);
bool bmi08_gyro_data_ready(void);
bool bmi08_get_acc_data(struct bmi08_sensor_data_f *acc);
bool bmi08_get_gyro_data(struct bmi08_sensor_data_f *gyro);

#endif