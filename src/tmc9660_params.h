#ifndef TMC9660_PARAMS_H_
#define TMC9660_PARAMS_H_

#include "tmc9660.h"

/* Read all RWE parameters from TMC9660 and save them to microcontroller flash */
int tmc_params_load(struct tmc9660_dev *dev);

/* Write all RWE parameters to TMC9660 using values from microcontroller flash */
int tmc_params_save(struct tmc9660_dev *dev);

#endif /* TMC9660_PARAMS_H_ */
