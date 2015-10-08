// this file contains functions that present data to console or the graphic plot
// window

// displays norms of raw and calibrated samples respectively.
function DisplayNorms(samples, calBeta)
    [nSamp, dummy] = size(samples);
    for i=1:nSamp
        calibrated(1:3) = [ (samples(i, 1) - calBeta(1, 1)) * calBeta(4,1),
        (samples(i, 2) - calBeta(2, 1)) * calBeta(5,1),
        (samples(i, 3) - calBeta(3, 1)) * calBeta(6,1) ];

        printf("norm(MagSamples[%i]): %f norm(calibrated[%i]):%f\n", i, norm(samples(i, 1:3)), i, norm(calibrated));
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
