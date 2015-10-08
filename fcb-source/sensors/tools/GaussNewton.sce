// Web links for explanation & credits to Rolfe Schmidt, the author
//
// [1] https://chionophilous.wordpress.com/2012/09/01/implementing-the-gauss-newton-algorithm-for-sphere-fitting-1-of-3/
// [2] http://www.ngdc.noaa.gov/geomag-web/
// [3] https://chionophilous.wordpress.com/2011/08/26/accelerometer-calibration-iii-improving-accuracy-with-least-squares-and-the-gauss-newton-method/

// GNU Octave file (also by Rolfe Schmidt):
// http://rolfeschmidt.com/mathtools/skimetrics/gaussnewton.m

// calBeta will send the normalised normSamp values to a sphere with
// radius 1 at good calibration. It is 
// 
// @param normSamp N by 3 matrix
// @param calBetaInitialGuess - 6 x 1 matrix
//                              (xij - Bij) * Bij+3 for j = 1..3. A good helps lets GN converge faster
//                                                                a bad one lets GN diverge
// @param maxIterations set the maximum number of iterations, unless the iterations have already converged before.
function retVal = GaussNewtonLeastSquares(normSamp, calBetaInitialGuess, maxIterations)
    calBeta = calBetaInitialGuess;
    iteration = 0;
    while iteration < maxIterations do
        oldBeta = calBeta;
        
        printf("iteration:%i", iteration);
        [calBeta] = gnStep(normSamp, calBeta);
        
        disp("calBeta");
        disp(calBeta'); // transpose because 1 x 6 displays better than 6 x 1

        // sum up the changes of the beta constants
        sumBetaChange = 0;
        for i = 1:6
            sumBetaChange = sumBetaChange + abs(((calBeta(i, 1) - oldBeta(i, 1))) / oldBeta(i, 1));
        end
        printf("sumBetaChange:%f\n", sumBetaChange);

        // ... and break the loop if the change is sufficiently small.
        // What counts as 'small' was arbitrarily chosen.
        if sumBetaChange < 0.001 then
            printf("break at sumBetaChange: %f\n", sumBetaChange);
            break
        end
        iteration = iteration + 1;
    end
    retVal = calBeta;
endfunction




// ------ debug utilities ----------
showDebug = 1;

// show debug variable
// it will print the name of the variable on a line, the value of the variable
// on the following lines.
function debugShowVar(variableName, variable)
    if showDebug <> 0 then
        printf("%s\n", variableName);
        disp(variable);
    end
endfunction

// show a variable unconditionally
function showVar(variableName, variable)
        printf("%s\n", variableName);
        disp(variable);
endfunction

// ---------- Gauss-Newton subroutines ---------------

// compute the residual error when each sample has been calibrated
// So we we will have N residuals resulting from N 3-tuples
//
// param samples: the N x 6 matrix of sensor samples.
// param calBeta: the 1 x 6 matrix of offsets & scalings (1 scaling plus 1 offset per x y z axis)
// x_ij = (sample_ij - beta_j) * beta_j+3
function retval = GetResiduals(samples, calBeta)
    [rSampSize, cSampSize] = size(samples);
    myResiduals = ones(rSampSize, 1);
    for i = 1:rSampSize
        for j = 1:3
            // NOTE we are using multiplication here for the scaling, not division
            // as in Definition 2 (Residual) in web page ref[1]

            // where x y z axis values of sample xi is: xi1, xi2, xi3
           
            // r_i(x, beta) = 1 - sum (( x_ij - beta_offset) **2 * (beta_scaling) **2) for j = 1 .. 3
            myResiduals(i,1) = myResiduals(i,1) - ( (samples(i, j) - calBeta(j, 1)) * calBeta(j+3, 1) )**2 ;
        end
    end
    retval = myResiduals;
endfunction

function retVal = GetJacobian(samples, calBeta)
   [rSampSize, cSampSize] = size(samples); // nbr of rows & columns
    myJacobian = zeros(rSampSize, 6);
    for i = 1:rSampSize
        for j = 1:3
            // Ref [1] Equation 4 - but modified to multiply by scaling, not division
            
            // derivative of residual function with respect to offset
            myJacobian(i,j) = 2 * (samples(i,j) - calBeta(j,1))  * (calBeta(j+3,1)^2);
            
            // derivative of residual function with respect to scaling
            myJacobian(i,j+3) = - 2 * calBeta(j+3,1) * ((samples(i, j) - calBeta(j, 1))^2);
            end
    end
    retVal = myJacobian
endfunction

function retval = gnStep(samples, calBeta)
    myResiduals = GetResiduals(samples, calBeta);
    disp("myResiduals");
    disp(myResiduals');
    
    // Jacobian elements are residual function derived with
    // respect to offset & scaling respectively - see GetJacobian comments
    myJacobian = GetJacobian(samples, calBeta);
    debugShowVar("myJacobian", myJacobian);

    // let's choose S, a number representing total error, to be a sum
    // of the residuals squared:
    
    // i = 1 .. N
    // S =  sum( residual_i ^ 2)
    
    // the first-order derivative of S with respect to beta are then 
    // d_S/dbeta_j = 2 * Jacobian_transposed * residuals - j
    //
    // the second derivative "Hessian matrix" of S is approximated by
    // d2S / (d_betaj dbeta_k) ~ 2 * Jacobian_transposed * Jacobian
    //
    // due to S being a sum of square residuals.
    JS = myJacobian' * myJacobian;
    myDelta = JS \ (myJacobian' * myResiduals); // a bit shaky on the theory here.
    disp("myDelta");
    disp(myDelta');
    
    new_calBeta = calBeta - myDelta;
    
    retval = new_calBeta
endfunction

// displays norms of raw and calibrated samples respectively.
function DisplayNorms(samples, calBeta)
[nSamp, dummy] = size(samples);
for i=1:nSamp
    calibrated(1:3) = [ (samples(i, 1) - calBeta(1, 1)) * calBeta(4,1),
        (samples(i, 2) - calBeta(2, 1)) * calBeta(5,1),
        (samples(i, 3) - calBeta(3, 1)) * calBeta(6,1) ];
                            
    printf("norm(MagSamples[%i]): %f norm(calibrated[%i]):%f\n", i, norm(samples(i, 1:3)),i, norm(calibrated));
end
endfunction
