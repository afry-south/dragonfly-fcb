// this file contains functions that present data to console or the graphic plot
// window

// displays norms of raw and calibrated samples respectively.
function DisplayNorms(samples, calBeta)
    [nSamp, dummy] = size(samples);
    for i=1:nSamp
        calibrated(1:3) = [ (samples(i, 1) - calBeta(1, 1)) * calBeta(4,1),
        (samples(i, 2) - calBeta(2, 1)) * calBeta(5,1),
        (samples(i, 3) - calBeta(3, 1)) * calBeta(6,1) ];

        printf("norm(samples[%i]): %f norm(calibrated[%i]):%f\n", i, norm(samples(i, 1:3)), i, norm(calibrated));
    end
endfunction

// plots the residuals, the intial ones in blue, the final ones in green
function PlotResidualsInitialFinal(initialResiduals, finalResiduals)
    [nSamp, sampleColumns] = size(initialResiduals);
    title("Residuals - before(blue) & after(green)");
    xLabel = msprintf("Samples n = 1 to %i", nSamp);
    xlabel(xLabel);
    ylabel("1 - sum (((sample - offset_d) * scaling_d))^2) where d=[x,y,z]");
    plot(1:nSamp, initialResiduals,'.b-', 1:nSamp, finalResiduals, '.g-');
endfunction

// Calculates and returns standard deviations for X Y Z and prints them to console
//
// @param samples Nx3 matrix
// @param calBeta the calibration offset and scaling parameters
// @return X Y and Z standard deviations as 1x3 matrix
function retVal = DisplayXYZStdDev(samples, calBeta)
    // apply calibration parameters to samples
    [nSamp, sampleColumns] = size(samples);
    
    for i=1:nSamp
        calibratedSamples(i,1) = (samples(i, 1) - calBeta(1, 1)) * calBeta(4,1);
        calibratedSamples(i,2) = (samples(i, 2) - calBeta(2, 1)) * calBeta(5,1);
        calibratedSamples(i,3) = (samples(i, 3) - calBeta(3, 1)) * calBeta(6,1);
    end
    
    // then calculate standard deviations for X Y and Z axes respectively
    xStDev = stdev(calibratedSamples(:,1));
    yStDev = stdev(calibratedSamples(:,2));
    zStDev = stdev(calibratedSamples(:,3));
    
    // and display the result
    printf("\nStandard Deviations of calibrated measurements:\n");
    printf("std dev X, Y, Z: %f, %f ,%f\n", xStDev, yStDev, zStDev);
    // and return the results
    retVal = [xStDev, yStDev, zStDev];
    
    return retVal
endfunction
