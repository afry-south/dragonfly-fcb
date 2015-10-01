// Web link for explanation & credits to Rolfe Schmidt, the author
// https://chionophilous.wordpress.com/2011/08/26/accelerometer-calibration-iii-improving-accuracy-with-least-squares-and-the-gauss-newton-method/
//
// [1] https://chionophilous.wordpress.com/2012/09/01/implementing-the-gauss-newton-algorithm-for-sphere-fitting-1-of-3/
//
// GNU Octave file:
// http://rolfeschmidt.com/mathtools/skimetrics/gaussnewton.m

// clc
disp('Danger, Will Robinson!')

// quit()


// 6 samples with start-calibration-mag CLI function on FCB
MagSamples = [-0.221818 0.090909  0.516327
0.207273 0.048182  0.545918
0.018182 0.282727  0.532653
-0.036364 -0.140000  0.534694
0.539091 -0.078182  0.067347
-0.562727 -0.077273  -0.074490 ]

// begin loop
// compute matrix JSubBetaSuperT_JSubBeta "Jacobian, subscript Beta superscript T (Transpose)"

// compute vector JSubBetaSuperTResiduals
// solve "equation 2"
// update Beta
// decide whether to continue



// beta[1-3] are offsets
// beta[4-6] are scaling
//
// x_j : calibrated value j = 1 .. 3 ( x y z axis)
// value : sensor sample
// beta_j, beta_j+3 
// x_j  = (value - beta_j) * beta_j+3
calBeta = [0.1; 0.1; 0.1; 1; 1; 1]; // N x 1 matrix


// compute the residual error when each sample has been calibrated
// So we we will have N residuals resulting from N 3-tuples
//
// x_ij = (sample_ij - beta_j) * beta_j+3
function retval = getResiduals(samples, calBeta)
    [rSampSize, cSampSize] = size(samples);
    myResiduals = ones(rSampSize, 1);
    for i = 1:rSampSize
        for j = 1:3
            // NOTE we are using multiplication here for the scaling, not division
            // as in Definition 2 (Residual) in the web page
            
            // r_i = 1 - sum (( x_ij - beta_offset) **2 * (beta_scaling) **2) for j = 1 .. 3
            myResiduals(i,1) = myResiduals(i,1) - ( calBeta(j+3, 1) * (samples(i, j) - calBeta(j, 1)))**2 ;
        end
    end
    retval = myResiduals;
endfunction

function retVal = getJacobian(samples, calBeta)
   [rSampSize, cSampSize] = size(samples); // nbr of rows & columns
    myJacobian = zeros(rSampSize, 1);
    for i = 1:rSampSize
        for j = 1:3
            // Ref [1] Equation 4 - but modified to multiply by scaling, not division
            
            // derivative of residual function with respect to offset
            myJacobian(i,j) = 2 * (samples(i,j) - calBeta(j,1))  * (calBeta(j+3,1)^2)
            
            // derivative of residual function with respect to scaling
            myJacobian(i,j+3) = - 2 * calBeta(j+3,1) * ((samples(i, j) - calBeta(j+3, 1))^2)
            end
    end
    retVal = myJacobian
endfunction

function retval = gnStep(samples, calBeta)
    myResiduals = getResiduals(samples, calBeta);
    disp("myResiduals");
    disp(myResiduals');
    // Jacobian elements are residual function derived with
    // respect to offset & scaling respectively - see getJacobian comments
    myJacobian = getJacobian(samples, calBeta);

    // let's choose S, a number representing total error, to be a sum
    // of the residuals squared:
    //
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
    
       // total error is 
    retval = new_calBeta
endfunction

step = 0;

[rSampSize, cSampSize] = size(MagSamples);
disp(MagSamples);
disp("rSize:" + string(rSampSize) + " cSize:" + string(cSampSize));

while step < 10 do
        oldBeta = calBeta;
        disp("step:" + string(step));
        [calBeta] = gnStep(MagSamples, calBeta);
        beta_change = 0;
        disp("calBeta");
        disp(calBeta');
        for i = 1:6
            beta_change = beta_change + abs(((calBeta(i) - oldBeta(i))) / oldBeta(i));
        end
        
        if beta_change < 0.001 then
            disp("break at beta_change:" + string(beta_change));
            break
        end
        step = step + 1;
end
