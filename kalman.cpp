
/***
 * Implemented by : Suryansh Kumar
 * Kalman Filter Routine
 * 
 * Names of variable are taken from
 * Probabilistic Robotics Text Book by Sebastian Thrun, Wolfram Burgard, Dieter Fox.
 * */

#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "kalman.h"

using namespace cv;
using namespace std;


void kalman :: init_kalman(Mat mean, Mat system_matrix, Mat observation_matrix, Mat system_noise, Mat observation_noise, Mat state_covariance)
{
 mu = mean;
 a = system_matrix; c = observation_matrix;
 r = system_noise;  q = observation_noise;
 p = state_covariance;
}

void kalman :: predict()
{
  mu = a*mu;
  p = (a*p)*(a.t()) + r;
}

void kalman :: update(Mat observation, int size)
{
  Mat tmp = (c*p)*(c.t()) + q;
  invert(tmp, tmp, DECOMP_LU);
  Mat K = (p*c.t())*tmp;
  mu = mu + K*(observation - c*mu);
  p = (Mat::eye(size, size, CV_64F) - K*c)*p;
}

void kalman:: set_p(Mat value)
{
  p = value;
}

Mat kalman :: get_p()
{
  return p;
}

void kalman :: set_mean(Mat value)
{
  mu = value;
}

Mat kalman :: get_mean()
{
  return mu;
}


