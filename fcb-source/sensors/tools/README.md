# Calibrating LSM303DLHC

Install Scilab, get it at www.scilab.org.

## Magnetometer

Don't keep the device near any laptops or other electric sources when calibrating.


## Accelerometer

Make sure the device is on a flat/orthogonal surface.


## Gyroscope

Variance script was run with FCB board stationary on the desk.

## SensorVariance.sce

* Store comma separated list of values in a file named `SensorSamples.txt`
 * The list is in N x 3 format (one each for X Y and Z => 3)
 * Example with 4 samples (in practice many times more are desirable):
 ```
 -0.015, 0.019, 0.949
 -0.013, 0.017, 0.953
 -0.013, 0.017, 0.958
 -0.011, 0.020, 0.954
 -0.011, 0.018, 0.958
```
