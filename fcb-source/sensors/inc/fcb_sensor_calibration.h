/*
 * fcb_sensor_calibration.h
 *
 */

#ifndef FCB_SENSOR_CALIBRATION_H
#define FCB_SENSOR_CALIBRATION_H

typedef enum FcbSensorCalibrationParmIndex {
  X_OFFSET_CALIB_IDX = 0,
  Y_OFFSET_CALIB_IDX = 1,
  Z_OFFSET_CALIB_IDX = 2,
  X_SCALING_CALIB_IDX = 3,
  Y_SCALING_CALIB_IDX = 4,
  Z_SCALING_CALIB_IDX = 5,
  CALIB_IDX_MAX = 6} FcbSensorCalibrationParmIndex;

#endif /* FCB_SENSOR_CALIBRATION_H */
