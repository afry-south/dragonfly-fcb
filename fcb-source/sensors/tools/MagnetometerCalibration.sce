exec('GaussNewton.sce'); // working directory is assumed to be dir of this file.

// ============ entry point of script ==========================
// entry point means things run sequentially from here

// don't hold STM32F3 board near laptops, smartphones or PCs - this will distort values.
magSamples = fscanfMat( "MagnetometerSamples.txt", "%lg");

// Initial guess of calibration vector
// beta[1-3] are offsets
// beta[4-6] are scaling
//
// x_j : calibrated value j = 1 .. 3 ( x y z axis)
// value : sensor sample
// beta_j, beta_j+3: offset, scaling3
//
// It is useful to choose the initial value with some care, since
// we are linearising a nonlinear function O(n^2), the GN method
// may not converge if the inital guess is too far from the solution.
//
// NOTE:
// offsets may not be zero - that will mean divide by zero in the betaChange
// calculation in function GaussNewtonLeastSquares!
calBeta = [-0.01; 0.01; 0.1; 1; 1; 1]; // N x 1 matrix, initial guess of beta

[sampleRows, sampleColumns] = size(magSamples);
disp(magSamples);
printf("sampleRows:%i sampleColumns:%i\n", sampleRows, sampleColumns);

if sampleColumns <> 3 then
    printf("ERROR: samples must be on Nx3 form\n");
    return;
end

// Earth's total magnetic field amplitude in Malmo see ref[2]
// then conversion of nano Tesla from above to Gauss
localMagField = 0.502559
nSamp = sampleRows;
normSamp = zeros(nSamp,3);
for i = 1:nSamp
   normSamp(i, 1:3) =  1 / localMagField * magSamples(i, 1:3);
end

debugShowVar("normSamp", normSamp);

maxIterations = 10
calBeta = GaussNewtonLeastSquares(normSamp, calBeta, maxIterations);

// then convert beta offsets (calBeta[1..3])back to local magnetic vector ...
// the beta scaling (calBeta[4..6]) is already correct.
calBeta(1:3,1) = calBeta(1:3,1) * localMagField;

printf("Displaying norems of uncailbrated and calibrated values:\n");

DisplayNorms(magSamples, calBeta);

showVar("calBeta - denormalised", calBeta');


// calBeta result 

// (123 samples):  - 0.0105836    0.0906626    0.0484313    0.8757337    0.8965046    0.8977956
