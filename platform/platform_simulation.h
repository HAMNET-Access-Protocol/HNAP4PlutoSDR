/*
 * platform_simulation.h
 *
 *  Created on: Jan 17, 2020
 *      Author: lukas
 */

#ifndef PLATFORM_PLATFORM_SIMULATION_H_
#define PLATFORM_PLATFORM_SIMULATION_H_

#include "platform.h"
#include <stdio.h>

platform platform_init_simulation(unsigned int buflen);
void simulation_connect(platform p, platform remote);

#endif /* PLATFORM_PLATFORM_SIMULATION_H_ */
