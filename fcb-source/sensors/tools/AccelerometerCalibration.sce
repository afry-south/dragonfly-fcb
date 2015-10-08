
// Additional Web links & references - see GaussNewton.sce
// ref[1]: http://physics.nist.gov/cgi-bin/cuu/Value?gn (g: acceleration due to gravity)

exec('GaussNewton.sce'); // working directory is assumed to be dir of this file.
exec('PresentationUtilities.sce');

// ============ entry point of script ==========================

// see documentaition of fscanfMat on how to format sample file.
accSamples = fscanfMat( "AccelerometerSamples.txt", "%lg");
// printf("accSamples:\n");
// disp(accSamples);

// ref[1]
g = 9.80665; // m / s^2

[nSamp, nCols] = size(accSamples);

if nCols <> 3 then
    printf("ERROR (nCols:%i) (number of sample columns) must be 3", nCols);
    return;
end

for i = 1:nSamp
    normSamps(i, 1:3) = 1 / g * accSamples(i, 1:3);
end

// having a look at the values and see where they tend to skew
calBetaInitialGuess = [
// x scaling, y scaling, z scaling, x offset, y offset, z offset          
0.01; 0.01; 0.01; 1; 1; 1
];

disp(size(calBetaInitialGuess));
maxIterations = 10;
calBeta = GaussNewtonLeastSquares(normSamps, calBetaInitialGuess, maxIterations);

// denormalise the offsets - leave the scalings
calBeta(1:3) = g * calBeta(1:3);

DisplayNorms(accSamples , calBeta);

showVar("calBeta", calBeta');

// calBeta
 //  - 0.1155430  - 0.0068883    0.4760651    0.9649397    0.9764581    0.9652411  
// need flatter surface and more samples to improve it.
// The resulting norms fluctuate quite a bit from 9.82 m/s2
