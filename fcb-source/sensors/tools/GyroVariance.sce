
gyroSamples = fscanfMat( "MagnetometerSamples.txt", "%lg");



[gyroSampleVarianceX,gyroSampleMeanX] = variance(gyroSamples(:,1));
[gyroSampleVarianceY, gyroSampleMeanY]  = variance(gyroSamples(:,2));
[gyroSampleVarianceZ, gyroSampleMeanZ] = variance(gyroSamples(:,3));
printf("gyroSampleMeanXYZ:%f %f %f\n", gyroSampleMeanX, gyroSampleMeanY, gyroSampleMeanZ);
printf("gyroSampleVarianceXYZ:%f %f %f\n", gyroSampleVarianceX, gyroSampleVarianceY, gyroSampleVarianceZ);

// from 240 samples taken at 500ms interval with a stationary FCB:
// gyroSampleMeanXYZ:0.084834 0.046541 0.022250
// gyroSampleVarianceXYZ:0.098603 0.104274 0.103256
