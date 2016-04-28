/******************************************************************************
 * @file    sphere_calibration.h
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2016-04-27
 * @brief   Module contains a Gauss-Newton Algorithm for Sphere Fitting
 ******************************************************************************/

#ifndef __SPHERE_CALIBRATION_H
#define __SPHERE_CALIBRATION_H

#include <arm_math.h>


void addNewSample(const float32_t samples[3]);
void calibrate(float32_t calibParams[6]);

#endif /* __SPHERE_CALIBRATION_H */
