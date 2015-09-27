// Web link for explanation
// https://chionophilous.wordpress.com/2011/08/26/accelerometer-calibration-iii-improving-accuracy-with-least-squares-and-the-gauss-newton-method/
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