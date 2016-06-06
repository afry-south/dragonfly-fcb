/******************************************************************************
 * @file    sphere_calibration.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2016-04-27
 * @brief   Module contains a Gauss-Newton Algorithm for Sphere Fitting
 * The implementation is derived from Rolfe Schmidt's blog at
 * https://chionophilous.wordpress.com/2012/09/15/implementing-the-gauss-newton-algorithm-for-sphere-fitting-3-of-3/
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "sphere_calibration.h"
#include <arm_math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

//observation summary structures
float32_t mu[3]; //sum of all observations in each dimension
float32_t mu2[3];  //sum of squares of all observations in each dimension
float32_t ipXX[6]; //Symmetric matrix of inner products of observations with themselves
float32_t ipX2X[3][3]; //matrix of inner products of squares of observations with the observations
float32_t ipX2X2[6]; //Symmetric matrix of inner products of squares of observations with themselves
int32_t N;  //The number of observations

float32_t obsMin[3]; // Keep track of min observation in each dimension to guess parameters
float32_t obsMax[3]; // Keep track of max observation in each dimension to guess parameters

/* Private function prototypes -----------------------------------------------*/
uint32_t upperTriangularIndex(uint32_t i, uint32_t j);

void computeGNMatrices(float32_t JtJ[][6], float32_t JtR[], float32_t beta[6]);

/* Exported functions --------------------------------------------------------*/

void clearObservationMatrices() {
	memset(mu, 0, sizeof(mu));
	memset(mu2, 0, sizeof(mu));
	memset(ipXX, 0, sizeof(ipXX));
	memset(ipX2X, 0, sizeof(ipX2X));
	memset(ipX2X2, 0, sizeof(ipX2X2));
	memset(obsMin, 0, sizeof(obsMin));
	memset(obsMax, 0, sizeof(obsMax));
	N = 0;
}

void addNewSample(const float32_t samples[3]) {
    uint32_t i, j;

    //increment sample count
    ++N;

    float32_t squareObs[3];
    for (i=0; i < 3; ++i) {
        squareObs[i] = samples[i]*samples[i];  //square of 16-bit int will fit in 32-bit int
    }

    for (i=0; i < 3; ++i) {
        //Keep track of min and max in each dimension
        obsMin[i] = (samples[i] < obsMin[i]) ? samples[i] : obsMin[i];
        obsMax[i] = (samples[i] > obsMax[i]) ? samples[i] : obsMax[i];

        //accumulate sum and sum of squares in each dimension
        mu[i] += samples[i];
        mu2[i] += squareObs[i];

        //accumulate inner products of the vector of observations and the vector of squared observations.
        for(j=0;j<3;++j) {
            ipX2X[i][j] += squareObs[i]*samples[j];
            if(i <= j) {
                uint32_t idx = upperTriangularIndex(i,j);
                ipXX[idx] += samples[i]*samples[j];
                ipX2X2[idx] += squareObs[i]*squareObs[j];
            }
        }
    }
}


uint32_t upperTriangularIndex(uint32_t i, uint32_t j) {
  if (i > j) {
    uint32_t temp = i;
    i = j;
    j = temp;
  }

  return (j*(j+1))/2 + i;
}

void guessParameters(float32_t beta[6]) {
    for(int i=0;i<3;++i) {
      beta[i] = (obsMax[i] + obsMin[i]) / 2.0;
      beta[3+i] = (obsMax[i] - obsMin[i]) / 2.0;
  }
}

void clearGNMatrices(float32_t JtJ[][6], float32_t JtR[]){
    uint32_t j, k;
    for(j=0; j<6; ++j) {
        JtR[j] = 0.0;
        for(k=0; k<6; ++k) {
            JtJ[j][k] = 0.0;
        }
    }
}

void computeGNMatrices(float JtJ[][6], float JtR[], float32_t beta[6]) {
  uint32_t i,j;

  float32_t beta2[6]; //precompute the squares of the model parameters
  for (i=0; i<6; ++i) {
    beta2[i] = pow(beta[i],2);
  }

  //compute the inner product of the vector of residuals with the constant 1 vector, the vector of
  // observations, and the vector of squared observations.
  float32_t r = N; //sum of residuals
  float32_t rx[3];  //Inner product of vector of residuals with each observation vector
  float32_t rx2[3];  //Inner product of vector of residuals with each square observation vector

  //now correct the r statistics
  for(i=0 ;i<3; ++i) {
    r -= (beta2[i]*N + mu2[i] - 2*beta[i]*mu[i])/beta2[3+i];
    rx[i] = mu[i];
    rx2[i] = mu2[i];
    for(j=0;j<3;++j) {
      rx[i] -= (beta2[j]*mu[i] + ipX2X[j][i] - 2*ipXX[upperTriangularIndex(i,j)]*beta[j])/beta2[3+j];
      rx2[i] -= (beta2[j]*mu2[i] + ipX2X2[upperTriangularIndex(i,j)] - 2*ipX2X[i][j]*beta[j])/beta2[3+j];
    }
  }

  for(i=0;i<3;++i) {
    //Compute product of Jacobian matrix with the residual vector
    JtR[i] = 2*(rx[i] - beta[i]*r)/beta2[3+i];
    JtR[3+i] = 2*(rx2[i] - 2*beta[i]*rx[i] + beta2[i]*r)/(beta2[3+i]*beta[3+i]);

    //Now compute the product of the transpose of the jacobian with itself
    //Start with the diagonal blocks
    for(j=i;j<3;++j) {
      JtJ[i][j] = JtJ[j][i] = 4*(ipXX[upperTriangularIndex(i,j)] - beta[i]*mu[j] - beta[j]*mu[i] + beta[i]*beta[j]*N)/(beta2[3+i]*beta2[3+j]);
      JtJ[3+i][3+j] = JtJ[3+j][3+i]
                =  4*(ipX2X2[upperTriangularIndex(i,j)] - 2*beta[j]*ipX2X[i][j] + beta2[j]*mu2[i]
                       -2*beta[i]*ipX2X[j][i] + 4*beta[i]*beta[j]*ipXX[upperTriangularIndex(i,j)] - 2*beta[i]*beta2[j]*mu[i]
                       +beta2[i]*mu2[j] - 2*beta2[i]*beta[j]*mu[j] + beta2[i]*beta2[j]*N)/pow(beta[3+i]*beta[3+j], 3);
    }
    //then get the off diagonal blocks
    for(j=0;j<3;++j) {
      JtJ[i][3+j] = JtJ[3+j][i]
          = 4*(ipX2X[j][i] - 2*beta[j]*ipXX[upperTriangularIndex(i,j)] + beta2[j]*mu[i]
                -beta[i]*mu2[j] + 2*beta[i]*beta[j]*mu[j] - beta[i]*beta2[j]*N)/(beta2[3+i]*beta2[3+j]*beta[3+j]);
    }
  }
}

void findDelta(float32_t JtJ[][6], float32_t JtR[]){
	//Solve 6-d matrix equation JtJS*x = JtR
	//first put in upper triangular form
	//Serial.println("find delta");
	int i,j,k;
	float32_t lambda;

	//make upper triangular
	for(i=0;i<6;++i) {
		//eliminate all nonzero entries below JS[i][i]

		if( JtJ[i][i] == 0.0) {
//      	Serial.print("Diagonal entry ");
//      	Serial.print(i);
//      	Serial.print(" is zero!\n");
		}

		for(j=i+1;j<6;++j) {
			lambda = JtJ[j][i]/JtJ[i][i];
			if(lambda != 0.0) {
				JtR[j] -= lambda*JtR[i];
				for(k=i;k<6;++k) {
					JtJ[j][k] -= lambda*JtJ[i][k];
				}
			}
		}
	}

	//back-substitute
	for(i=5; i>=0; --i) {
	    JtR[i] /= JtJ[i][i];
	    JtJ[i][i] = 1.0;
	    for(j=0; j<i; ++j) {
	        lambda = JtJ[j][i];
	        JtR[j] -= lambda*JtR[i];
	        JtJ[j][i] = 0.0;
	    }
	}
}

void calibrate(float32_t calibParams[6]) {
	//Final calibration parameters
	float32_t beta[6];

	guessParameters(beta);
	float32_t JtJ[6][6];
	float32_t JtR[6];
	clearGNMatrices(JtJ,JtR);

	float32_t eps = 0.000000001;
	int num_iterations = 20;
	float32_t change = 100.0;

	while (--num_iterations >=0 && change > eps) {
		computeGNMatrices(JtJ, JtR, beta);
		findDelta(JtJ, JtR);

		change = JtR[0]*JtR[0] +
				JtR[1]*JtR[1] +
				JtR[2]*JtR[2] +
				JtR[3]*JtR[3]*(beta[3]*beta[3]) +
				JtR[4]*JtR[4]*(beta[4]*beta[4]) +
				JtR[5]*JtR[5]*(beta[5]*beta[5]);

		if (!isnan(change)) {
			uint32_t i;
			for(i=0; i<6; ++i) {
				beta[i] -= JtR[i];

				if(i >=3) {
					beta[i] = fabs(beta[i]);
				}
			}
		}
		clearGNMatrices(JtJ,JtR);
	}

	uint32_t i;
	for (i=0; i<6; i++) {
		calibParams[i] = beta[i];
	}

	clearObservationMatrices();
}

