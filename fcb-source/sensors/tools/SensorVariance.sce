
fileName = "SensorSamples.txt";
printf("Analysing data file %s.\n", fileName)

sensorSamples = fscanfMat(fileName, "%lg");

[nSamp, dummy] = size(sensorSamples);
printf("found %i samples in file\n", nSamp);

[sensorSampleVarianceX,sensorSampleMeanX] = variance(sensorSamples(:,1));
[sensorSampleVarianceY, sensorSampleMeanY]  = variance(sensorSamples(:,2));
[sensorSampleVarianceZ, sensorSampleMeanZ] = variance(sensorSamples(:,3));
printf("sensorSampleMeanXYZ: %f %f %f\n", sensorSampleMeanX, sensorSampleMeanY, sensorSampleMeanZ);
printf("sensorSampleVarianceXYZ: %f %f %f\n", sensorSampleVarianceX, sensorSampleVarianceY, sensorSampleVarianceZ);
printf("Mission complete.");


// Gyroscope angle rate mean/variance [radians/sec])
// From 240 samples at 500ms interval:
// sensorSampleMeanXYZ:0.004042 -0.010511 0.003183
// sensorSampleVarianceXYZ:0.000007 0.000016 0.000256


// Accelerometer derived angles [radians]:
// from 480 samples taken at 500ms interval with a stationary FCB:
// sensorSampleMeanXYZ: -0.013921 0.018307 1.084920
// sensorSampleVarianceXYZ: 0.000006 0.000006 0.000185

